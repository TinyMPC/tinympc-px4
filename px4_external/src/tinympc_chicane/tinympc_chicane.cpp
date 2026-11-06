/*
 * Native TinyMPC acceleration-level PX4 integration.
 *
 * mpc: PX4 EKF -> TinyMPC position/velocity MPC -> acceleration setpoint ->
 *      PX4 attitude/rate controllers -> PX4 control allocation -> motors.
 * pid/pid_tuned: PX4 EKF -> stock PX4 cascaded position/velocity controller
 *      -> the same PX4 attitude/rate/allocation stack. pid_tuned additionally
 *      verifies the gain set used by the published fair comparison.
 *
 * The monitor and hold modes provide a staged hardware-validation path before
 * the time-parameterized chicane is enabled. The module never publishes motor
 * commands and never replaces PX4 commander, failsafes, or inner loops.
 */

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/tinympc_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "tinympc_chicane_course.hpp"
#include "tinympc_interface.h"
#include "tinympc_build_info.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <time.h>

#if defined(__PX4_NUTTX)
#include <malloc.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#endif

namespace
{

constexpr uint32_t kUpdatePeriodUs = 20000;
constexpr uint64_t kStateFreshnessUs = 100000;
constexpr uint64_t kStatusFreshnessUs = 2000000;
constexpr uint64_t kMaximumFutureTimestampUs = 20000;
constexpr uint64_t kSolveDeadlineUs = 18000;
constexpr float kMaximumCorridorDeparture = 0.60f;
constexpr float kMaximumAltitudeError = 0.60f;
constexpr float kMaximumHorizontalSpeed = 5.0f;
constexpr float kRequiredTiltLimitDegrees = 15.0f;
constexpr float kTunedPositionGain = 0.21f;
constexpr float kTunedVelocityProportionalGain = 5.0f;
constexpr float kTunedVelocityIntegralGain = 0.17f;
constexpr float kTunedVelocityDerivativeGain = 0.13f;
constexpr int kPx4CustomMainModeAuto = 4;
constexpr int kPx4CustomSubModeAutoLoiter = 3;

enum ControllerMode {
	ModeMpc = 0,
	ModePid = 1,
	ModePidTuned = 2,
	ModeHold = 3,
	ModeMonitor = 4,
};

enum FailureReason {
	FailureNone = 0,
	FailureConfiguration = 1,
	FailureStaleState = 2,
	FailureEstimatorReset = 3,
	FailureStateEnvelope = 4,
	FailureSolver = 5,
	FailureDeadline = 6,
};

struct SharedStatus {
	volatile bool running{false};
	volatile bool ready{false};
	volatile bool engaged{false};
	volatile bool failed{false};
	volatile int mode{ModeMonitor};
	volatile int failure_reason{FailureNone};
	volatile float course_time{0.f};
	volatile float corridor_violation{0.f};
	volatile float maximum_corridor_violation{0.f};
	volatile float last_solve_us{0.f};
	volatile float maximum_solve_us{0.f};
	volatile uint64_t solve_count{0};
	volatile uint64_t deadline_miss_count{0};
	volatile int solve_policy{TINY_MPC_SOLVE_INVALID};
	volatile int iterations{0};
	volatile float primal_residual{0.f};
	volatile float dual_residual{0.f};
};

volatile bool g_should_exit{false};
px4_task_t g_task_id{-1};
int g_requested_mode{ModeMonitor};
SharedStatus g_status{};

uint64_t monotonic_wall_time_us()
{
#if defined(__PX4_NUTTX)
	return hrt_absolute_time();
#else
	// SITL's hrt clock can stop while the solver runs. Measure actual host work.
	struct timespec now {};
	return clock_gettime(CLOCK_MONOTONIC, &now) == 0 ?
		static_cast<uint64_t>(now.tv_sec) * 1000000 + now.tv_nsec / 1000 : 0;
#endif
}

uint32_t bounded_us(uint64_t value)
{
	return static_cast<uint32_t>(std::min(value, static_cast<uint64_t>(UINT32_MAX)));
}

uint32_t sample_age(uint64_t now, uint64_t timestamp)
{
	return timestamp == 0 ? UINT32_MAX : bounded_us(now > timestamp ? now - timestamp : 0);
}

bool finite(float value)
{
	return std::isfinite(value);
}

bool read_int_param(const char *name, int32_t &value)
{
	const param_t handle = param_find(name);
	return handle != PARAM_INVALID && param_get(handle, &value) == PX4_OK;
}

bool require_float_param(const char *name, float expected, float tolerance = 1.0e-3f)
{
	const param_t handle = param_find(name);
	float value = NAN;

	if (handle == PARAM_INVALID || param_get(handle, &value) != PX4_OK || !finite(value)) {
		PX4_ERR("required parameter %s is unavailable", name);
		return false;
	}

	if (std::fabs(value - expected) > tolerance) {
		PX4_ERR("%s=%.3f; required %.3f", name, (double)value, (double)expected);
		return false;
	}

	return true;
}

const char *mode_name(int mode)
{
	switch (mode) {
	case ModePid: return "px4_pid";
	case ModePidTuned: return "px4_pid_tuned";
	case ModeHold: return "tinympc_hold";
	case ModeMonitor: return "tinympc_monitor";
	default: return "tinympc";
	}
}

const char *failure_name(int reason)
{
	switch (reason) {
	case FailureNone: return "none";
	case FailureConfiguration: return "configuration";
	case FailureStaleState: return "stale_state";
	case FailureEstimatorReset: return "estimator_reset";
	case FailureStateEnvelope: return "state_envelope";
	case FailureSolver: return "solver";
	case FailureDeadline: return "deadline";
	default: return "unknown";
	}
}

class ChicaneController
{
public:
	explicit ChicaneController(int mode) : _mode(mode)
	{
		_metrics.run_id = hrt_absolute_time();
		_metrics.schema_version = tinympc_status_s::SCHEMA_VERSION;
		std::strncpy(_metrics.source_id, TINYMPC_SOURCE_ID, sizeof(_metrics.source_id) - 1);
		std::strncpy(_metrics.source_commit, TINYMPC_SOURCE_COMMIT, sizeof(_metrics.source_commit) - 1);
		_metrics.system_cpu_load = NAN;
		_metrics.system_ram_usage = NAN;
		_metrics.heap_free_bytes = UINT32_MAX;
		_metrics.heap_min_free_bytes = UINT32_MAX;
		_metrics.stack_free_bytes = UINT32_MAX;
		_metrics.stack_size_bytes = UINT32_MAX;
		_metrics.init_heap_delta_bytes = INT32_MIN;
		reset_sample();
	}

	bool init()
	{
		vehicle_status_s status{};
		if (!_vehicle_status_sub.copy(&status) ||
		    status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			PX4_ERR("module must be initialized while the vehicle is disarmed");
			g_status.failed = true;
			g_status.failure_reason = FailureConfiguration;
			return false;
		}

		int32_t autostart = -1;
		const bool autostart_read = read_int_param("SYS_AUTOSTART", autostart);
#if defined(__PX4_NUTTX)
		const bool airframe_supported = autostart_read &&
			(autostart == 4001 || autostart == 4019);
#else
		const bool airframe_supported = autostart_read &&
			(autostart == 4001 || autostart == 4002 ||
			 autostart == 4005 || autostart == 4010);
#endif

		if (!airframe_supported) {
			PX4_ERR("requires an explicitly supported quadrotor airframe");
			g_status.failed = true;
			g_status.failure_reason = FailureConfiguration;
			return false;
		}

		bool parameters_valid = require_float_param(
			"MPC_TILTMAX_AIR", kRequiredTiltLimitDegrees, 1.0e-2f);

		if (_mode == ModePidTuned) {
			parameters_valid = require_float_param("MPC_XY_P", kTunedPositionGain) &&
					   require_float_param("MPC_XY_VEL_P_ACC", kTunedVelocityProportionalGain) &&
					   require_float_param("MPC_XY_VEL_I_ACC", kTunedVelocityIntegralGain) &&
					   require_float_param("MPC_XY_VEL_D_ACC", kTunedVelocityDerivativeGain) &&
					   parameters_valid;
		}

		if (!parameters_valid) {
			PX4_ERR("controller parameters do not match mode=%s", mode_name(_mode));
			g_status.failed = true;
			g_status.failure_reason = FailureConfiguration;
			return false;
		}

		const uint64_t init_start = monotonic_wall_time_us();
#if defined(__PX4_NUTTX)
		const int32_t heap_before = mallinfo().uordblks;
#endif
		MPC_Init();
		MPC_Reset();
		_metrics.init_time_us = bounded_us(monotonic_wall_time_us() - init_start);
#if defined(__PX4_NUTTX)
		_metrics.init_heap_delta_bytes = static_cast<int32_t>(mallinfo().uordblks) - heap_before;
#endif
		TinyMpcConfig config{};
		MPC_GetConfig(_mode == ModeMpc ? TINY_MPC_SCENARIO_CHICANE_SOC : TINY_MPC_SCENARIO_POSITION_HOLD, &config);
		_metrics.config[0] = config.horizon_knots;
		_metrics.config[1] = config.iteration_cap;
		_metrics.config[2] = config.single_precision;
		_metrics.config[3] = config.sample_time_s;
		_metrics.config[4] = config.primal_tolerance;
		_metrics.config[5] = config.dual_tolerance;
		sample_resources();
		g_status.ready = true;
		if (_mode == ModeMonitor) {
			PX4_INFO("ready: monitor only; no heartbeat or setpoint publication");
		} else {
			PX4_INFO("ready: mode=%s; take off, then switch to Offboard", mode_name(_mode));
		}
		return true;
	}

	void run()
	{
		uint64_t next_run = hrt_absolute_time();

		while (!g_should_exit) {
			const uint64_t now = hrt_absolute_time();

			if (now < next_run) {
				px4_usleep(next_run - now);
			}

			const uint64_t scheduled = next_run;
			next_run += kUpdatePeriodUs;
			const uint64_t actual_now = hrt_absolute_time();
			reset_sample();
			_metrics.start_lateness_us = bounded_us(actual_now > scheduled ? actual_now - scheduled : 0);
			_metrics.cycle_interval_us = _last_cycle_start ? bounded_us(actual_now - _last_cycle_start) : 0;
			_last_cycle_start = actual_now;

			if (actual_now > next_run + kUpdatePeriodUs) {
				_metrics.skipped_releases += bounded_us((actual_now - next_run) / kUpdatePeriodUs + 1);
				next_run = actual_now + kUpdatePeriodUs;
			}

			const uint64_t cycle_start = monotonic_wall_time_us();
			step();
			sample_resources();
			_metrics.cycle_time_us = bounded_us(monotonic_wall_time_us() - cycle_start);
			_metrics.cycle_max_us = std::max(_metrics.cycle_max_us, _metrics.cycle_time_us);
			_metrics.loop_overruns += _metrics.cycle_time_us > kUpdatePeriodUs ? 1 : 0;
			++_metrics.sequence;
			publish_status();
		}

		vehicle_status_s status{};
		_vehicle_status_sub.copy(&status);

		if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			request_loiter(FailureNone);
		}
	}

	void publish_status()
	{
		if (!g_status.running) {
			reset_sample();
			_metrics.cycle_time_us = 0;
			++_metrics.sequence;
		}
		_metrics.timestamp = hrt_absolute_time();
		_metrics.mode = _mode;
		_metrics.running = g_status.running;
		_metrics.ready = g_status.ready;
		_metrics.engaged = g_status.engaged;
		_metrics.failed = g_status.failed;
		_metrics.failure_reason = g_status.failure_reason;
		_metrics.course_time = g_status.course_time;
		_metrics.corridor_violation = g_status.corridor_violation;
		_status_pub.publish(_metrics);
	}

private:
	void reset_sample()
	{
		_metrics.solve_called = false;
		_metrics.command_accepted = false;
		_metrics.tracking_valid = false;
		_metrics.solve_time_us = 0;
		_metrics.iterations = 0;
		_metrics.policy = TINY_MPC_SOLVE_INVALID;
		_metrics.primal_residual = NAN;
		_metrics.dual_residual = NAN;
		_metrics.planned_state_violation = NAN;
		_metrics.planned_input_violation = NAN;
		_metrics.position_error_norm = NAN;
		_metrics.hold_position_margin = NAN;
		_metrics.hold_velocity_margin = NAN;
		for (int i = 0; i < 3; ++i) {
			_metrics.position_error[i] = NAN;
			_metrics.velocity_error[i] = NAN;
		}
		for (float &command : _metrics.command) {
			command = NAN;
		}
	}

	void sample_resources()
	{
		cpuload_s load{};
		if (_cpuload_sub.update(&load)) {
			_metrics.system_cpu_load = load.load;
			_metrics.system_ram_usage = load.ram_usage;
			_metrics.cpuload_timestamp = load.timestamp;
		}
		const uint64_t now = hrt_absolute_time();
		if (_metrics.resource_timestamp && now - _metrics.resource_timestamp < 1000000) {
			return;
		}
#if defined(__PX4_NUTTX)
		const struct mallinfo mem = mallinfo();
		_metrics.heap_free_bytes = mem.fordblks;
		_metrics.heap_min_free_bytes = std::min(_metrics.heap_min_free_bytes, _metrics.heap_free_bytes);
#if defined(CONFIG_STACK_COLORATION)
		_metrics.stack_free_bytes = up_check_tcbstack_remain(nxsched_self());
		_metrics.stack_size_bytes = nxsched_self()->adj_stack_size;
#endif
		_metrics.resource_timestamp = now;
#else
		(void)now;
#endif
	}

	void record_tracking(const vehicle_local_position_s &local)
	{
		const bool course = _mode == ModeMpc || _mode == ModePid || _mode == ModePidTuned;
		const auto reference = course ? tinympc_chicane::sample(static_cast<double>(_course_time)) :
			tinympc_chicane::Sample{};
		const float relative[3] = {local.x - _origin_x, local.y - _origin_y, local.z - _origin_z};
		_metrics.tracking_valid = _metrics.state_valid;
		_metrics.position_error[0] = relative[0] - static_cast<float>(reference.x);
		_metrics.position_error[1] = relative[1] - static_cast<float>(reference.y);
		_metrics.position_error[2] = relative[2];
		_metrics.velocity_error[0] = local.vx - static_cast<float>(reference.vx);
		_metrics.velocity_error[1] = local.vy - static_cast<float>(reference.vy);
		_metrics.velocity_error[2] = local.vz;
		_metrics.position_error_norm = std::sqrt(
			_metrics.position_error[0] * _metrics.position_error[0] +
			_metrics.position_error[1] * _metrics.position_error[1] + relative[2] * relative[2]);
		if (!course) {
			_metrics.hold_position_margin = std::min(0.5f - std::fabs(relative[0]),
				std::min(0.5f - std::fabs(relative[1]), 0.4f - std::fabs(relative[2])));
			_metrics.hold_velocity_margin = std::min(1.f - std::fabs(local.vx),
				std::min(1.f - std::fabs(local.vy), 0.75f - std::fabs(local.vz)));
		}
	}

	static bool timestamp_is_fresh(uint64_t now, uint64_t timestamp, uint64_t maximum_age)
	{
		return timestamp > 0 &&
		       (timestamp >= now ? timestamp - now <= kMaximumFutureTimestampUs :
		        now - timestamp < maximum_age);
	}

	bool state_is_fresh(uint64_t now, const vehicle_local_position_s &local,
			    const vehicle_attitude_s &attitude,
			    const vehicle_status_s &status) const
	{
		return timestamp_is_fresh(now, local.timestamp, kStateFreshnessUs) &&
		       timestamp_is_fresh(now, attitude.timestamp, kStateFreshnessUs) &&
		       timestamp_is_fresh(now, status.timestamp, kStatusFreshnessUs) &&
		       local.xy_valid && local.z_valid && local.v_xy_valid && local.v_z_valid &&
		       finite(local.x) && finite(local.y) && finite(local.z) &&
		       finite(local.vx) && finite(local.vy) && finite(local.vz) &&
		       finite(attitude.q[0]) && finite(attitude.q[1]) &&
		       finite(attitude.q[2]) && finite(attitude.q[3]);
	}

	void publish_offboard_heartbeat(uint64_t now)
	{
		if (_mode == ModeMonitor) {
			return;
		}

		offboard_control_mode_s control_mode{};
		control_mode.timestamp = now;

		if (_mode == ModePid || _mode == ModePidTuned) {
			control_mode.position = true;
		} else {
			control_mode.acceleration = true;
		}

		_offboard_mode_pub.publish(control_mode);
	}

	float yaw_from_attitude(const vehicle_attitude_s &attitude) const
	{
		const float w = attitude.q[0];
		const float x = attitude.q[1];
		const float y = attitude.q[2];
		const float z = attitude.q[3];
		return std::atan2(2.f * (w * z + x * y),
				  1.f - 2.f * (y * y + z * z));
	}

	void engage(const vehicle_local_position_s &local,
		    const vehicle_attitude_s &attitude)
	{
		_origin_x = local.x;
		_origin_y = local.y;
		_origin_z = local.z;
		_origin_yaw = yaw_from_attitude(attitude);
		_xy_reset_counter = local.xy_reset_counter;
		_z_reset_counter = local.z_reset_counter;
		_vxy_reset_counter = local.vxy_reset_counter;
		_vz_reset_counter = local.vz_reset_counter;
		_quat_reset_counter = attitude.quat_reset_counter;
		_course_time = 0.f;
		_maximum_corridor_violation = 0.f;
		_engaged = true;
		_failover_command_sent = false;
		MPC_Reset();
		++_metrics.session;
		g_status.engaged = true;
		g_status.course_time = 0.f;
		g_status.corridor_violation = 0.f;
		g_status.maximum_corridor_violation = 0.f;
		PX4_INFO("engaged %s at NED [%.2f %.2f %.2f] yaw=%.2f",
			 mode_name(_mode), (double)_origin_x, (double)_origin_y,
			 (double)_origin_z, (double)_origin_yaw);
	}

	bool estimator_was_reset(const vehicle_local_position_s &local,
				 const vehicle_attitude_s &attitude) const
	{
		return local.xy_reset_counter != _xy_reset_counter ||
		       local.z_reset_counter != _z_reset_counter ||
		       local.vxy_reset_counter != _vxy_reset_counter ||
		       local.vz_reset_counter != _vz_reset_counter ||
		       attitude.quat_reset_counter != _quat_reset_counter;
	}

	void empty_setpoint(trajectory_setpoint_s &setpoint, uint64_t now) const
	{
		setpoint = trajectory_setpoint_s{};
		setpoint.timestamp = now;

		for (int axis = 0; axis < 3; ++axis) {
			setpoint.position[axis] = NAN;
			setpoint.velocity[axis] = NAN;
			setpoint.acceleration[axis] = NAN;
			setpoint.jerk[axis] = NAN;
		}

		setpoint.yaw = _origin_yaw;
		setpoint.yawspeed = 0.f;
	}

	bool run_mpc(uint64_t now,
		     const vehicle_local_position_s &local,
		     const vehicle_attitude_s &attitude,
		     int scenario,
		     bool publish_setpoint,
		     float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT])
	{
		float state[12]{};
		state[0] = local.x;
		state[1] = local.y;
		state[2] = local.z;
		state[5] = yaw_from_attitude(attitude);
		state[6] = local.vx;
		state[7] = local.vy;
		state[8] = local.vz;
		float control[4]{};
		float plan[12]{};
		const uint64_t solve_start = monotonic_wall_time_us();
		MPC_Step_Scenario(state, scenario,
				  control, plan, diagnostics);
		const uint64_t solve_end = monotonic_wall_time_us();
		const uint64_t elapsed = solve_start > 0 && solve_end >= solve_start ?
					 solve_end - solve_start : UINT64_MAX;
		g_status.last_solve_us = static_cast<float>(elapsed);
		const float elapsed_float = static_cast<float>(elapsed);
		g_status.maximum_solve_us = elapsed_float > g_status.maximum_solve_us ?
					    elapsed_float : g_status.maximum_solve_us;
		++g_status.solve_count;
		g_status.solve_policy = static_cast<int>(diagnostics[TINY_MPC_DIAG_POLICY]);
		g_status.iterations = static_cast<int>(diagnostics[TINY_MPC_DIAG_ITERATIONS]);
		g_status.primal_residual = diagnostics[TINY_MPC_DIAG_PRIMAL_RESIDUAL];
		g_status.dual_residual = diagnostics[TINY_MPC_DIAG_DUAL_RESIDUAL];
		_metrics.solve_called = true;
		_metrics.solve_time_us = bounded_us(elapsed);
		_metrics.solve_max_us = std::max(_metrics.solve_max_us, _metrics.solve_time_us);
		++_metrics.solve_count;
		_metrics.policy = g_status.solve_policy;
		_metrics.iterations = g_status.iterations;
		_metrics.max_iterations = std::max(_metrics.max_iterations, _metrics.iterations);
		_metrics.primal_residual = g_status.primal_residual;
		_metrics.dual_residual = g_status.dual_residual;
		_metrics.planned_state_violation = diagnostics[TINY_MPC_DIAG_STATE_VIOLATION];
		_metrics.planned_input_violation = diagnostics[TINY_MPC_DIAG_INPUT_VIOLATION];
		std::memcpy(_metrics.command, control, sizeof(control));
		_metrics.converged_count += _metrics.policy == TINY_MPC_SOLVE_CONVERGED ? 1 : 0;
		_metrics.best_effort_count += _metrics.policy == TINY_MPC_SOLVE_BEST_EFFORT ? 1 : 0;
		_metrics.fallback_count += _metrics.policy == TINY_MPC_SOLVE_FALLBACK ? 1 : 0;
		_metrics.iteration_limit_count += _metrics.iterations >= _metrics.config[1] ? 1 : 0;

		if (elapsed > kSolveDeadlineUs) {
			++g_status.deadline_miss_count;
			++_metrics.solve_deadline_misses;
			++_metrics.rejected_count;
			g_status.failure_reason = FailureDeadline;
			return false;
		}

		const int policy = static_cast<int>(diagnostics[TINY_MPC_DIAG_POLICY]);
		const bool accepted = policy == TINY_MPC_SOLVE_CONVERGED ||
				      policy == TINY_MPC_SOLVE_BEST_EFFORT;

		if (!accepted || !finite(control[0]) || !finite(control[1]) ||
		    !finite(control[2]) || !finite(control[3]) ||
		    std::hypot(control[0], control[1]) > 3.0f ||
		    std::fabs(control[2]) > 4.0f || std::fabs(control[3]) > 1.1f) {
			PX4_ERR("reject policy=%d u=[%.2f %.2f %.2f %.2f] residual=[%.3g %.3g] violation=[%.3g %.3g]",
				policy, (double)control[0], (double)control[1],
				(double)control[2], (double)control[3],
				(double)diagnostics[TINY_MPC_DIAG_PRIMAL_RESIDUAL],
				(double)diagnostics[TINY_MPC_DIAG_DUAL_RESIDUAL],
				(double)diagnostics[TINY_MPC_DIAG_STATE_VIOLATION],
				(double)diagnostics[TINY_MPC_DIAG_INPUT_VIOLATION]);
			g_status.failure_reason = FailureSolver;
			++_metrics.rejected_count;
			return false;
		}
		_metrics.command_accepted = true;

		if (publish_setpoint) {
			trajectory_setpoint_s setpoint{};
			empty_setpoint(setpoint, now);
			setpoint.acceleration[0] = control[0];
			setpoint.acceleration[1] = control[1];
			setpoint.acceleration[2] = control[2];
			setpoint.yawspeed = control[3];
			_trajectory_setpoint_pub.publish(setpoint);
		}
		return true;
	}

	void publish_pid_setpoint(uint64_t now)
	{
		const tinympc_chicane::Sample reference =
			tinympc_chicane::sample(static_cast<double>(_course_time));
		trajectory_setpoint_s setpoint{};
		empty_setpoint(setpoint, now);
		setpoint.position[0] = _origin_x + static_cast<float>(reference.x);
		setpoint.position[1] = _origin_y + static_cast<float>(reference.y);
		setpoint.position[2] = _origin_z;
		setpoint.velocity[0] = static_cast<float>(reference.vx);
		setpoint.velocity[1] = static_cast<float>(reference.vy);
		setpoint.velocity[2] = 0.f;
		_trajectory_setpoint_pub.publish(setpoint);
	}

	void publish_debug(uint64_t now, float relative_x, float relative_y,
			   const float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT])
	{
		const bool chicane_mode = _mode == ModeMpc || _mode == ModePid ||
			_mode == ModePidTuned;
		const tinympc_chicane::Sample reference = chicane_mode ?
			tinympc_chicane::sample(static_cast<double>(_course_time)) :
			tinympc_chicane::Sample{};
		debug_array_s debug{};
		debug.timestamp = now;
		debug.id = 43;
		std::strncpy(debug.name, "tmch", sizeof(debug.name));
		debug.data[0] = static_cast<float>(_mode);
		debug.data[1] = _course_time;
		debug.data[2] = relative_x;
		debug.data[3] = relative_y;
		debug.data[4] = static_cast<float>(reference.x);
		debug.data[5] = static_cast<float>(reference.y);
		debug.data[6] = g_status.corridor_violation;
		debug.data[7] = _maximum_corridor_violation;
		debug.data[8] = g_status.last_solve_us;
		debug.data[9] = static_cast<float>(g_status.deadline_miss_count);

		for (int i = 0; i < TINY_MPC_DIAGNOSTIC_COUNT; ++i) {
			debug.data[10 + i] = diagnostics[i];
		}

		_debug_pub.publish(debug);
	}

	void request_loiter(int reason)
	{
		if (reason != FailureNone && !g_status.failed) {
			g_status.failed = true;
			g_status.failure_reason = reason;
			PX4_ERR("fail-closed to autonomous Loiter: %s", failure_name(reason));
		}

		if (!_failover_command_sent) {
			vehicle_command_s command{};
			command.timestamp = hrt_absolute_time();
			command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
			command.param1 = 1.f;
			command.param2 = static_cast<float>(kPx4CustomMainModeAuto);
			command.param3 = static_cast<float>(kPx4CustomSubModeAutoLoiter);
			command.target_system = 1;
			command.target_component = 1;
			command.source_system = 1;
			command.source_component = 1;
			command.from_external = false;
			_vehicle_command_pub.publish(command);
			_failover_command_sent = true;
		}

		_engaged = false;
		g_status.engaged = false;
	}

	void step()
	{
		vehicle_local_position_s local{};
		vehicle_attitude_s attitude{};
		vehicle_status_s status{};
		_local_position_sub.copy(&local);
		_attitude_sub.copy(&attitude);
		_vehicle_status_sub.copy(&status);
		const uint64_t now = hrt_absolute_time();
		_metrics.input_age_us[0] = sample_age(now, local.timestamp);
		_metrics.input_age_us[1] = sample_age(now, attitude.timestamp);
		_metrics.input_age_us[2] = sample_age(now, status.timestamp);
		_metrics.state_valid = state_is_fresh(now, local, attitude, status);
		_metrics.position[0] = local.x;
		_metrics.position[1] = local.y;
		_metrics.position[2] = local.z;
		_metrics.velocity[0] = local.vx;
		_metrics.velocity[1] = local.vy;
		_metrics.velocity[2] = local.vz;
		control_allocator_status_s allocator{};
		_allocator_sub.copy(&allocator);
		_metrics.allocator_valid = timestamp_is_fresh(now, allocator.timestamp, kStateFreshnessUs);
		_metrics.actuator_saturated = false;
		for (const auto saturation : allocator.actuator_saturation) {
			_metrics.actuator_saturated |= saturation != control_allocator_status_s::ACTUATOR_SATURATION_OK;
		}
		_metrics.thrust_setpoint_achieved = allocator.thrust_setpoint_achieved;
		_metrics.torque_setpoint_achieved = allocator.torque_setpoint_achieved;
		_metrics.stale_cycles += !_metrics.state_valid ? 1 : 0;

		if (g_status.failed) {
			return;
		}

		if (!state_is_fresh(now, local, attitude, status)) {
			if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				request_loiter(FailureStaleState);
			}
			return;
		}

		g_status.ready = true;

		if (_mode == ModeMonitor) {
			if (!_monitor_initialized) {
				_origin_x = local.x;
				_origin_y = local.y;
				_origin_z = local.z;
				_origin_yaw = yaw_from_attitude(attitude);
				MPC_Reset();
				++_metrics.session;
				_monitor_initialized = true;
			}
			record_tracking(local);

			float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT]{};
			if (!run_mpc(now, local, attitude,
				     TINY_MPC_SCENARIO_POSITION_HOLD, false, diagnostics)) {
				g_status.failed = true;
				PX4_ERR("monitor solve failed: %s", failure_name(g_status.failure_reason));
				return;
			}

			publish_debug(now, local.x - _origin_x, local.y - _origin_y, diagnostics);
			return;
		}

		publish_offboard_heartbeat(now);
		const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		const bool offboard = status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

		if (!armed || !offboard) {
			_engaged = false;
			_failover_command_sent = false;
			g_status.engaged = false;
			return;
		}

		if (!_engaged) {
			engage(local, attitude);
		}
		record_tracking(local);

		if (estimator_was_reset(local, attitude)) {
			++_metrics.estimator_reset_events;
			request_loiter(FailureEstimatorReset);
			return;
		}

		const float relative_x = local.x - _origin_x;
		const float relative_y = local.y - _origin_y;
		const bool chicane_mode = _mode == ModeMpc || _mode == ModePid ||
			_mode == ModePidTuned;
		const float corridor_violation = chicane_mode ? static_cast<float>(
			tinympc_chicane::corridorViolation(relative_x, relative_y)) : 0.f;
		_maximum_corridor_violation = std::max(_maximum_corridor_violation,
							 corridor_violation);
		g_status.course_time = _course_time;
		g_status.corridor_violation = corridor_violation;
		g_status.maximum_corridor_violation = _maximum_corridor_violation;

		const bool hold_position_exceeded = _mode == ModeHold &&
			std::hypot(relative_x, relative_y) > 0.75f;
		if (corridor_violation > kMaximumCorridorDeparture ||
		    hold_position_exceeded ||
		    std::fabs(local.z - _origin_z) > kMaximumAltitudeError ||
		    std::hypot(local.vx, local.vy) > kMaximumHorizontalSpeed) {
			request_loiter(FailureStateEnvelope);
			return;
		}

		float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT]{};

		if (_mode == ModeMpc || _mode == ModeHold) {
			const int scenario = _mode == ModeHold ?
				TINY_MPC_SCENARIO_POSITION_HOLD : TINY_MPC_SCENARIO_CHICANE_SOC;
			if (!run_mpc(now, local, attitude, scenario, true, diagnostics)) {
				request_loiter(g_status.failure_reason);
				return;
			}
		} else {
			publish_pid_setpoint(now);
		}

		publish_debug(now, relative_x, relative_y, diagnostics);
		_course_time += static_cast<float>(kUpdatePeriodUs) * 1.0e-6f;
	}

	int _mode;
	tinympc_status_s _metrics{};
	uint64_t _last_cycle_start{0};
	bool _engaged{false};
	bool _monitor_initialized{false};
	bool _failover_command_sent{false};
	float _origin_x{0.f};
	float _origin_y{0.f};
	float _origin_z{0.f};
	float _origin_yaw{0.f};
	float _course_time{0.f};
	float _maximum_corridor_violation{0.f};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _quat_reset_counter{0};

	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription _allocator_sub{ORB_ID(control_allocator_status)};
	uORB::Publication<offboard_control_mode_s> _offboard_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<debug_array_s> _debug_pub{ORB_ID(debug_array)};
	uORB::Publication<tinympc_status_s> _status_pub{ORB_ID(tinympc_status)};
};

int controller_thread_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	g_status = SharedStatus{};
	g_status.running = true;
	g_status.mode = g_requested_mode;
	ChicaneController controller(g_requested_mode);

	if (controller.init()) {
		controller.run();
	}

	g_status.running = false;
	g_status.ready = false;
	g_status.engaged = false;
	controller.publish_status();
	g_task_id = -1;
	return PX4_OK;
}

void print_usage()
{
	PX4_INFO("usage: tinympc_chicane {start [monitor|hold|mpc|pid|pid_tuned]|stop|status}");
	PX4_INFO("default is monitor (solver runs, no Offboard heartbeat or setpoint)");
	PX4_INFO("PX4 MPC_TILTMAX_AIR must be 15 degrees");
}

} // namespace

extern "C" __EXPORT int tinympc_chicane_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return PX4_ERROR;
	}

	if (std::strcmp(argv[1], "start") == 0) {
		if (g_status.running || g_task_id >= 0) {
			PX4_WARN("already running");
			return PX4_OK;
		}

		g_requested_mode = ModeMonitor;

		if (argc >= 3) {
			if (std::strcmp(argv[2], "monitor") == 0) {
				g_requested_mode = ModeMonitor;
			} else if (std::strcmp(argv[2], "hold") == 0) {
				g_requested_mode = ModeHold;
			} else if (std::strcmp(argv[2], "mpc") == 0) {
				g_requested_mode = ModeMpc;
			} else if (std::strcmp(argv[2], "pid") == 0) {
				g_requested_mode = ModePid;
			} else if (std::strcmp(argv[2], "pid_tuned") == 0) {
				g_requested_mode = ModePidTuned;
			} else {
				PX4_ERR("unknown controller: %s", argv[2]);
				return PX4_ERROR;
			}
		}

#if defined(__PX4_NUTTX)
		// Keep the first hardware image staged: autonomous course modes remain
		// SITL-only until monitor logs and props-off checks pass on the board.
		if (g_requested_mode != ModeMonitor && g_requested_mode != ModeHold) {
			PX4_ERR("hardware build permits only monitor or hold mode");
			return PX4_ERROR;
		}
#endif

		g_should_exit = false;
		g_task_id = px4_task_spawn_cmd("tinympc_chicane",
					       SCHED_DEFAULT,
					       SCHED_PRIORITY_DEFAULT,
					       64000,
					       controller_thread_main,
					       nullptr);
		return g_task_id < 0 ? PX4_ERROR : PX4_OK;
	}

	if (std::strcmp(argv[1], "stop") == 0) {
		if (!g_status.running) {
			PX4_WARN("not running");
			return PX4_OK;
		}

		g_should_exit = true;
		return PX4_OK;
	}

	if (std::strcmp(argv[1], "status") == 0) {
		PX4_INFO("metrics_schema=%u source=%s commit=%s", tinympc_status_s::SCHEMA_VERSION,
			 TINYMPC_SOURCE_ID, TINYMPC_SOURCE_COMMIT);
		PX4_INFO("%s ready=%d engaged=%d failed=%d mode=%s reason=%s",
			 g_status.running ? "running" : "stopped",
			 g_status.ready, g_status.engaged, g_status.failed,
			 mode_name(g_status.mode), failure_name(g_status.failure_reason));
		PX4_INFO("time=%.2f violation=%.3f max=%.3f solves=%llu solve_us=%.0f max_us=%.0f",
			 (double)g_status.course_time, (double)g_status.corridor_violation,
			 (double)g_status.maximum_corridor_violation,
			 static_cast<unsigned long long>(g_status.solve_count),
			 (double)g_status.last_solve_us, (double)g_status.maximum_solve_us);
		PX4_INFO("policy=%d iter=%d residual=[%.3g %.3g] deadline_misses=%llu",
			 g_status.solve_policy, g_status.iterations,
			 (double)g_status.primal_residual, (double)g_status.dual_residual,
			 static_cast<unsigned long long>(g_status.deadline_miss_count));
		uORB::Subscription metrics_sub{ORB_ID(tinympc_status)};
		tinympc_status_s metrics{};
		if (metrics_sub.copy(&metrics)) {
			PX4_INFO("cycle_us=%lu max=%lu late_us=%lu overruns=%lu skipped=%lu stale=%lu",
				 (unsigned long)metrics.cycle_time_us, (unsigned long)metrics.cycle_max_us,
				 (unsigned long)metrics.start_lateness_us, (unsigned long)metrics.loop_overruns,
				 (unsigned long)metrics.skipped_releases, (unsigned long)metrics.stale_cycles);
			PX4_INFO("converged=%lu best_effort=%lu fallback=%lu rejected=%lu cap_hits=%lu",
				 (unsigned long)metrics.converged_count, (unsigned long)metrics.best_effort_count,
				 (unsigned long)metrics.fallback_count, (unsigned long)metrics.rejected_count,
				 (unsigned long)metrics.iteration_limit_count);
			PX4_INFO("cpu=%.1f%% ram=%.1f%% heap_free=%lu min=%lu stack_free=%lu/%lu",
				 (double)(100.f * metrics.system_cpu_load), (double)(100.f * metrics.system_ram_usage),
				 (unsigned long)metrics.heap_free_bytes, (unsigned long)metrics.heap_min_free_bytes,
				 (unsigned long)metrics.stack_free_bytes, (unsigned long)metrics.stack_size_bytes);
			PX4_INFO("horizon=%u cap=%u fp32=%d init_us=%lu heap_delta=%ld pos_err=%.3f margin=%.3f",
				 (unsigned)metrics.config[0], (unsigned)metrics.config[1], (int)metrics.config[2],
				 (unsigned long)metrics.init_time_us, (long)metrics.init_heap_delta_bytes,
				 (double)metrics.position_error_norm, (double)metrics.hold_position_margin);
		}
		return PX4_OK;
	}

	print_usage();
	return PX4_ERROR;
}
