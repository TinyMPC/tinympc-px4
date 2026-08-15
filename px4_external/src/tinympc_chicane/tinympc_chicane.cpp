/*
 * Matched, no-disturbance PX4 SITL comparison.
 *
 * mpc: PX4 EKF -> TinyMPC position/velocity MPC -> acceleration setpoint ->
 *      PX4 attitude/rate controllers -> PX4 control allocation -> motors.
 * pid/pid_tuned: PX4 EKF -> stock PX4 cascaded position/velocity controller
 *      -> the same PX4 attitude/rate/allocation stack. pid_tuned additionally
 *      verifies the gain set used by the published fair comparison.
 *
 * Both modes receive the same time-parameterized chicane and use the same
 * vehicle and 15-degree tilt limit. The module never publishes motor commands.
 */

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "tinympc_chicane_course.hpp"
#include "tinympc_interface.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>

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
	volatile int mode{ModeMpc};
	volatile int failure_reason{FailureNone};
	volatile float course_time{0.f};
	volatile float corridor_violation{0.f};
	volatile float maximum_corridor_violation{0.f};
	volatile float last_solve_us{0.f};
	volatile float maximum_solve_us{0.f};
	volatile uint64_t solve_count{0};
};

volatile bool g_should_exit{false};
px4_task_t g_task_id{-1};
int g_requested_mode{ModeMpc};
SharedStatus g_status{};

uint64_t monotonic_wall_time_us()
{
	struct timespec value {};

	if (::clock_gettime(CLOCK_MONOTONIC_RAW, &value) != 0) {
		return 0;
	}

	return static_cast<uint64_t>(value.tv_sec) * 1000000ULL +
	       static_cast<uint64_t>(value.tv_nsec) / 1000ULL;
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
	explicit ChicaneController(int mode) : _mode(mode) {}

	bool init()
	{
		int32_t autostart = -1;

		if (!read_int_param("SYS_AUTOSTART", autostart) ||
		    (autostart != 4001 && autostart != 4002 &&
		     autostart != 4005 && autostart != 4010)) {
			PX4_ERR("requires a Gazebo X500 multirotor airframe");
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

		MPC_Init();
		MPC_Reset();
		g_status.ready = true;
		PX4_INFO("ready: mode=%s; take off, then switch to Offboard", mode_name(_mode));
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

			next_run += kUpdatePeriodUs;
			const uint64_t actual_now = hrt_absolute_time();

			if (actual_now > next_run + kUpdatePeriodUs) {
				next_run = actual_now + kUpdatePeriodUs;
			}

			step();
		}

		vehicle_status_s status{};
		_vehicle_status_sub.copy(&status);

		if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			request_loiter(FailureNone);
		}
	}

private:
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
		offboard_control_mode_s control_mode{};
		control_mode.timestamp = now;

		if (_mode != ModeMpc) {
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

	bool publish_mpc_setpoint(uint64_t now,
				  const vehicle_local_position_s &local,
				  const vehicle_attitude_s &attitude,
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
		MPC_Step_Scenario(state, TINY_MPC_SCENARIO_CHICANE_SOC,
				  control, plan, diagnostics);
		const uint64_t solve_end = monotonic_wall_time_us();
		const uint64_t elapsed = solve_start > 0 && solve_end >= solve_start ?
					 solve_end - solve_start : UINT64_MAX;
		g_status.last_solve_us = static_cast<float>(elapsed);
		const float elapsed_float = static_cast<float>(elapsed);
		g_status.maximum_solve_us = elapsed_float > g_status.maximum_solve_us ?
					    elapsed_float : g_status.maximum_solve_us;
		++g_status.solve_count;

		if (elapsed > kSolveDeadlineUs) {
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
			return false;
		}

		trajectory_setpoint_s setpoint{};
		empty_setpoint(setpoint, now);
		setpoint.acceleration[0] = control[0];
		setpoint.acceleration[1] = control[1];
		setpoint.acceleration[2] = control[2];
		setpoint.yawspeed = control[3];
		_trajectory_setpoint_pub.publish(setpoint);
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
		const tinympc_chicane::Sample reference =
			tinympc_chicane::sample(static_cast<double>(_course_time));
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

		if (estimator_was_reset(local, attitude)) {
			request_loiter(FailureEstimatorReset);
			return;
		}

		const float relative_x = local.x - _origin_x;
		const float relative_y = local.y - _origin_y;
		const float corridor_violation = static_cast<float>(
			tinympc_chicane::corridorViolation(relative_x, relative_y));
		_maximum_corridor_violation = std::max(_maximum_corridor_violation,
							 corridor_violation);
		g_status.course_time = _course_time;
		g_status.corridor_violation = corridor_violation;
		g_status.maximum_corridor_violation = _maximum_corridor_violation;

		if (corridor_violation > kMaximumCorridorDeparture ||
		    std::fabs(local.z - _origin_z) > kMaximumAltitudeError ||
		    std::hypot(local.vx, local.vy) > kMaximumHorizontalSpeed) {
			request_loiter(FailureStateEnvelope);
			return;
		}

		float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT]{};

		if (_mode == ModeMpc) {
			if (!publish_mpc_setpoint(now, local, attitude, diagnostics)) {
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
	bool _engaged{false};
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
	uORB::Publication<offboard_control_mode_s> _offboard_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<debug_array_s> _debug_pub{ORB_ID(debug_array)};
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
	g_task_id = -1;
	return PX4_OK;
}

void print_usage()
{
	PX4_INFO("usage: tinympc_chicane {start [mpc|pid|pid_tuned]|stop|status}");
	PX4_INFO("SITL only. Same chicane/no wind; PX4 MPC_TILTMAX_AIR must be 15.");
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

		g_requested_mode = ModeMpc;

		if (argc >= 3) {
			if (std::strcmp(argv[2], "mpc") == 0) {
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
		PX4_INFO("%s ready=%d engaged=%d failed=%d mode=%s reason=%s",
			 g_status.running ? "running" : "stopped",
			 g_status.ready, g_status.engaged, g_status.failed,
			 mode_name(g_status.mode), failure_name(g_status.failure_reason));
		PX4_INFO("time=%.2f violation=%.3f max=%.3f solves=%llu solve_us=%.0f max_us=%.0f",
			 (double)g_status.course_time, (double)g_status.corridor_violation,
			 (double)g_status.maximum_corridor_violation,
			 static_cast<unsigned long long>(g_status.solve_count),
			 (double)g_status.last_solve_us, (double)g_status.maximum_solve_us);
		return PX4_OK;
	}

	print_usage();
	return PX4_ERROR;
}
