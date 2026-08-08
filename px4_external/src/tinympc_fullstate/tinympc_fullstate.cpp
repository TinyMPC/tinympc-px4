/*
 * Experimental, SITL-only full-state TinyMPC/PX4 integration.
 *
 * TinyMPC owns the 12-state/four-motor horizon. PX4 still owns commander,
 * arming, failsafes, control allocation, output limiting, and the simulator
 * motor interface. The module publishes torque/thrust setpoints at the
 * control-allocation boundary; it never publishes actuator_motors directly.
 */

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include "tinympc_full_state_model.hpp"
#include "tinympc_interface.h"

#include <Eigen.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

namespace
{

constexpr uint32_t kUpdatePeriodUs = 20000; // The supplied model is 50 Hz.
constexpr uint64_t kStateFreshnessUs = 100000;
/* vehicle_status is intentionally low-rate (nominally about 2 Hz). */
constexpr uint64_t kStatusFreshnessUs = 2000000;
constexpr uint64_t kMaximumFutureTimestampUs = 20000;
constexpr uint64_t kSolveDeadlineUs = 18000;
constexpr float kDefaultActuatorGain = 1.0f;
constexpr float kMaximumAllocatorError = 0.05f;
constexpr int kAllocatorErrorLimit = 4;
constexpr float kMaximumRodriguesTilt = 0.24f;
constexpr float kMaximumBodyRate = 3.2f;
constexpr float kMaximumHoverPositionError = 0.65f;
constexpr float kMinimumActuator = 0.02f;
constexpr float kMaximumActuator = 0.98f;
constexpr float kGeometryTolerance = 0.025f;
constexpr int kPx4CustomMainModeAuto = 4;
constexpr int kPx4CustomSubModeAutoLoiter = 3;

uint64_t monotonic_wall_time_us()
{
	struct timespec value {};

	if (::clock_gettime(CLOCK_MONOTONIC_RAW, &value) != 0) {
		return 0;
	}

	return static_cast<uint64_t>(value.tv_sec) * 1000000ULL +
	       static_cast<uint64_t>(value.tv_nsec) / 1000ULL;
}

enum FailureReason {
	FailureNone = 0,
	FailureConfiguration = 1,
	FailureStaleState = 2,
	FailureEstimatorReset = 3,
	FailureStateEnvelope = 4,
	FailureSolver = 5,
	FailureDeadline = 6,
	FailureMotorMapping = 7,
	FailureAllocatorRoundTrip = 8,
	FailureAllocatorSaturation = 9,
};

struct Quaternion {
	float w;
	float x;
	float y;
	float z;
};

struct Vector3 {
	float x;
	float y;
	float z;
};

struct SharedStatus {
	volatile bool running{false};
	volatile bool ready{false};
	volatile bool engaged{false};
	volatile bool failed{false};
	volatile int scenario{TINY_MPC_FULL_STATE_HOVER};
	volatile int failure_reason{FailureNone};
	volatile float actuator_gain{kDefaultActuatorGain};
	volatile float hover_actuator{0.f};
	volatile float allocator_error{0.f};
	volatile float last_solve_us{0.f};
	volatile float max_solve_us{0.f};
	volatile uint64_t solve_count{0};
};

volatile bool g_should_exit{false};
px4_task_t g_task_id{-1};
int g_requested_scenario{TINY_MPC_FULL_STATE_HOVER};
float g_requested_gain{kDefaultActuatorGain};
SharedStatus g_status{};

bool finite(float value)
{
	return std::isfinite(value);
}

Quaternion normalize(const Quaternion &q)
{
	const float norm_squared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

	if (!finite(norm_squared) || norm_squared < 1.0e-8f) {
		return {NAN, NAN, NAN, NAN};
	}

	const float scale = 1.f / std::sqrt(norm_squared);
	return {q.w * scale, q.x * scale, q.y * scale, q.z * scale};
}

Quaternion conjugate(const Quaternion &q)
{
	return {q.w, -q.x, -q.y, -q.z};
}

Quaternion multiply(const Quaternion &a, const Quaternion &b)
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
	};
}

Vector3 rotate(const Quaternion &q, const Vector3 &v)
{
	const Quaternion vector_q{0.f, v.x, v.y, v.z};
	const Quaternion rotated = multiply(multiply(q, vector_q), conjugate(q));
	return {rotated.x, rotated.y, rotated.z};
}

bool read_float_param(const char *name, float &value)
{
	const param_t handle = param_find(name);
	return handle != PARAM_INVALID && param_get(handle, &value) == PX4_OK && finite(value);
}

bool read_int_param(const char *name, int32_t &value)
{
	const param_t handle = param_find(name);
	return handle != PARAM_INVALID && param_get(handle, &value) == PX4_OK;
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
	case FailureMotorMapping: return "motor_mapping";
	case FailureAllocatorRoundTrip: return "allocator_roundtrip";
	case FailureAllocatorSaturation: return "allocator_saturation";
	default: return "unknown";
	}
}

class FullStateController
{
public:
	FullStateController(int scenario, float actuator_gain) :
		_scenario(scenario),
		_actuator_gain(actuator_gain)
	{
	}

	bool init()
	{
		if (!load_allocator_model()) {
			g_status.failure_reason = FailureConfiguration;
			g_status.failed = true;
			return false;
		}

		MPC_FullState_Init();
		MPC_FullState_Reset();
		g_status.ready = true;
		g_status.hover_actuator = _hover_actuator;
		PX4_INFO("ready: hover=%.3f gain=%.3f scenario=%s", (double)_hover_actuator,
			 (double)_actuator_gain,
			 _scenario == TINY_MPC_FULL_STATE_HOVER ? "hover" : "wall");
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

			step(actual_now);
		}

		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			request_position_control(FailureNone);
		}
	}

private:
	bool load_allocator_model()
	{
		int32_t airframe = -1;
		int32_t rotor_count = -1;
		int32_t autostart = -1;

		if (!read_int_param("CA_AIRFRAME", airframe) || airframe != 0 ||
		    !read_int_param("CA_ROTOR_COUNT", rotor_count) || rotor_count != 4 ||
		    !read_int_param("SYS_AUTOSTART", autostart) ||
		    (autostart != 4001 && autostart != 4002 && autostart != 4005 && autostart != 4010) ||
		    !read_float_param("MPC_THR_HOVER", _hover_actuator) ||
		    _hover_actuator < 0.3f || _hover_actuator > 0.8f) {
			PX4_ERR("requires the Gazebo X500 multirotor geometry");
			return false;
		}

		const float expected_x[4] = {0.13f, -0.13f, 0.13f, -0.13f};
		const float expected_y[4] = {0.22f, -0.20f, -0.22f, 0.20f};
		const float expected_km[4] = {0.05f, 0.05f, -0.05f, -0.05f};
		Eigen::Matrix<float, 4, 4> effectiveness;

		for (int motor = 0; motor < 4; ++motor) {
			char name[18];
			float px = 0.f;
			float py = 0.f;
			float ct = 0.f;
			float km = 0.f;

			std::snprintf(name, sizeof(name), "CA_ROTOR%d_PX", motor);
			const bool got_px = read_float_param(name, px);
			std::snprintf(name, sizeof(name), "CA_ROTOR%d_PY", motor);
			const bool got_py = read_float_param(name, py);
			std::snprintf(name, sizeof(name), "CA_ROTOR%d_CT", motor);
			const bool got_ct = read_float_param(name, ct);
			std::snprintf(name, sizeof(name), "CA_ROTOR%d_KM", motor);
			const bool got_km = read_float_param(name, km);

			if (!got_px || !got_py || !got_ct || !got_km || ct <= 0.f ||
			    std::fabs(px - expected_x[motor]) > kGeometryTolerance ||
			    std::fabs(py - expected_y[motor]) > kGeometryTolerance ||
			    std::fabs(km - expected_km[motor]) > kGeometryTolerance) {
				PX4_ERR("X500 rotor %d parameters do not match", motor);
				return false;
			}

			/* Axis is fixed to (0,0,-1) by PX4's multirotor effectiveness.
			 * Rows are roll, pitch, yaw, and body-Z thrust. */
			effectiveness(0, motor) = -ct * py;
			effectiveness(1, motor) =  ct * px;
			effectiveness(2, motor) =  ct * km;
			effectiveness(3, motor) = -ct;
		}

		if (std::fabs(effectiveness.determinant()) < 1.0e-6f) {
			PX4_ERR("X500 effectiveness matrix is singular");
			return false;
		}

		Eigen::Matrix<float, 4, 4> raw_mix = effectiveness.inverse();
		int roll_count = 0;
		int pitch_count = 0;
		float roll_squared = 0.f;
		float pitch_squared = 0.f;
		float thrust_sum = 0.f;
		int thrust_count = 0;

		for (int motor = 0; motor < 4; ++motor) {
			if (std::fabs(raw_mix(motor, 0)) > 1.0e-3f) {
				++roll_count;
				roll_squared += raw_mix(motor, 0) * raw_mix(motor, 0);
			}

			if (std::fabs(raw_mix(motor, 1)) > 1.0e-3f) {
				++pitch_count;
				pitch_squared += raw_mix(motor, 1) * raw_mix(motor, 1);
			}

			if (std::fabs(raw_mix(motor, 3)) > 1.0e-6f) {
				++thrust_count;
				thrust_sum += std::fabs(raw_mix(motor, 3));
			}
		}

		if (roll_count == 0 || pitch_count == 0 || thrust_count == 0) {
			return false;
		}

		const float roll_scale = std::sqrt(roll_squared / (roll_count / 2.f));
		const float pitch_scale = std::sqrt(pitch_squared / (pitch_count / 2.f));
		const float rp_scale = roll_scale > pitch_scale ? roll_scale : pitch_scale;
		const float yaw_scale = raw_mix.col(2).maxCoeff();
		const float thrust_scale = thrust_sum / thrust_count;

		if (rp_scale <= 0.f || yaw_scale <= 0.f || thrust_scale <= 0.f) {
			return false;
		}

		_allocator_forward = effectiveness;
		_allocator_forward.row(0) *= rp_scale;
		_allocator_forward.row(1) *= rp_scale;
		_allocator_forward.row(2) *= yaw_scale;
		_allocator_forward.row(3) *= thrust_scale;

		/* Verify the forward map against PX4's normalized inverse mix. */
		Eigen::Matrix<float, 4, 4> normalized_mix = raw_mix;
		normalized_mix.col(0) /= rp_scale;
		normalized_mix.col(1) /= rp_scale;
		normalized_mix.col(2) /= yaw_scale;
		normalized_mix.col(3) /= thrust_scale;
		const Eigen::Matrix<float, 4, 4> round_trip = normalized_mix * _allocator_forward;
		const float error = (round_trip - Eigen::Matrix<float, 4, 4>::Identity()).cwiseAbs().maxCoeff();

		if (!finite(error) || error > 2.0e-4f) {
			PX4_ERR("allocator map self-check failed: %.6f", (double)error);
			return false;
		}

		return true;
	}

	bool state_is_fresh(uint64_t now, const vehicle_local_position_s &local,
			    const vehicle_attitude_s &attitude,
			    const vehicle_angular_velocity_s &angular,
			    const vehicle_status_s &status) const
	{
		return timestamp_is_fresh(now, local.timestamp, kStateFreshnessUs) &&
		       timestamp_is_fresh(now, attitude.timestamp, kStateFreshnessUs) &&
		       timestamp_is_fresh(now, angular.timestamp, kStateFreshnessUs) &&
		       timestamp_is_fresh(now, status.timestamp, kStatusFreshnessUs) &&
		       local.xy_valid && local.z_valid && local.v_xy_valid && local.v_z_valid &&
		       finite(local.x) && finite(local.y) && finite(local.z) &&
		       finite(local.vx) && finite(local.vy) && finite(local.vz) &&
		       finite(attitude.q[0]) && finite(attitude.q[1]) &&
		       finite(attitude.q[2]) && finite(attitude.q[3]) &&
		       finite(angular.xyz[0]) && finite(angular.xyz[1]) && finite(angular.xyz[2]);
	}

	static bool timestamp_is_fresh(uint64_t now, uint64_t timestamp, uint64_t maximum_age)
	{
		/* A uORB publisher may run between the caller sampling `now` and this
		 * module copying the topic. Never let that small scheduling race turn
		 * the unsigned subtraction into a multi-century apparent age. */
		return timestamp > 0 &&
		       (timestamp >= now ? timestamp - now <= kMaximumFutureTimestampUs : now - timestamp < maximum_age);
	}

	void publish_offboard_heartbeat(uint64_t now)
	{
		offboard_control_mode_s mode{};
		mode.timestamp = now;
		mode.thrust_and_torque = true;
		_offboard_mode_pub.publish(mode);
	}

	void latch_reference(const vehicle_local_position_s &local,
			     const vehicle_attitude_s &attitude)
	{
		_origin = {local.x, local.y, local.z};
		const Quaternion current = normalize({attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]});
		const float heading = std::atan2(2.f * (current.w * current.z + current.x * current.y),
					  1.f - 2.f * (current.y * current.y + current.z * current.z));
		const float half_heading = 0.5f * heading;
		/* Keep the local model's Z axis vertical. Latching the full hover
		 * attitude here would rotate position/velocity by the vehicle's small
		 * instantaneous roll and pitch and invalidate the gravity linearization. */
		_reference_q = {std::cos(half_heading), 0.f, 0.f, std::sin(half_heading)};
		_xy_reset_counter = local.xy_reset_counter;
		_z_reset_counter = local.z_reset_counter;
		_vxy_reset_counter = local.vxy_reset_counter;
		_vz_reset_counter = local.vz_reset_counter;
		_quat_reset_counter = attitude.quat_reset_counter;
		_engaged = true;
		_has_expected_motors = false;
		_allocator_error_count = 0;
		_allocator_saturation_count = 0;
		MPC_FullState_Reset();
		g_status.engaged = true;
		PX4_INFO("engaged at NED [%.2f %.2f %.2f]", (double)local.x,
			 (double)local.y, (double)local.z);
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

	bool build_model_state(const vehicle_local_position_s &local,
			       const vehicle_attitude_s &attitude,
			       const vehicle_angular_velocity_s &angular,
			       float state[12]) const
	{
		/* PX4 is NED/FRD. The supplied TinyMPC model is Crazyflie-style
		 * X-forward/Y-left/Z-up. Express translation in the latched heading
		 * frame, then apply C=diag(1,-1,-1). */
		const Vector3 ned_delta{local.x - _origin.x, local.y - _origin.y, local.z - _origin.z};
		const Vector3 ned_velocity{local.vx, local.vy, local.vz};
		const Vector3 reference_position = rotate(conjugate(_reference_q), ned_delta);
		const Vector3 reference_velocity = rotate(conjugate(_reference_q), ned_velocity);

		Quaternion current = normalize({attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]});
		Quaternion relative = normalize(multiply(conjugate(_reference_q), current));

		if (!finite(relative.w) || std::fabs(relative.w) < 0.2f) {
			return false;
		}

		if (relative.w < 0.f) {
			relative.w = -relative.w;
			relative.x = -relative.x;
			relative.y = -relative.y;
			relative.z = -relative.z;
		}

		state[0] = reference_position.x;
		state[1] = -reference_position.y;
		state[2] = -reference_position.z;
		/* Conjugating the relative rotation by C maps its vector part to
		 * [qx,-qy,-qz]. The supplied controller uses q_vec/q_w. */
		state[3] = relative.x / relative.w;
		state[4] = -relative.y / relative.w;
		state[5] = -relative.z / relative.w;
		state[6] = reference_velocity.x;
		state[7] = -reference_velocity.y;
		state[8] = -reference_velocity.z;
		state[9] = angular.xyz[0];
		state[10] = -angular.xyz[1];
		state[11] = -angular.xyz[2];

		for (int i = 0; i < 12; ++i) {
			if (!finite(state[i])) {
				return false;
			}
		}

		return true;
	}

	bool state_inside_runtime_envelope(const float state[12]) const
	{
		const float position_norm = std::sqrt(state[0] * state[0] + state[1] * state[1] + state[2] * state[2]);
		const float tilt = std::sqrt(state[3] * state[3] + state[4] * state[4]);
		const float rate = std::sqrt(state[9] * state[9] + state[10] * state[10] + state[11] * state[11]);
		return position_norm <= (_scenario == TINY_MPC_FULL_STATE_HOVER ? kMaximumHoverPositionError : 1.25f) &&
		       tilt <= kMaximumRodriguesTilt && rate <= kMaximumBodyRate &&
		       std::fabs(state[5]) <= 0.35f;
	}

	bool map_motors_to_wrench(const float model_motor[4],
				 float px4_motor[4],
				 Eigen::Matrix<float, 4, 1> &wrench) const
	{
		/* Crazyflie M1..M4 are front-right, rear-right, rear-left,
		 * front-left. PX4 X500 rotors 0..3 are front-right, rear-left,
		 * front-left, rear-right. */
		const int model_for_px4[4] = {0, 2, 3, 1};

		for (int px4_motor_index = 0; px4_motor_index < 4; ++px4_motor_index) {
			const float model_value = model_motor[model_for_px4[px4_motor_index]];
			px4_motor[px4_motor_index] = _hover_actuator + _actuator_gain *
				(model_value - static_cast<float>(tinympc_full_state_model::kHoverCommand));

			if (!finite(px4_motor[px4_motor_index]) ||
			    px4_motor[px4_motor_index] < kMinimumActuator ||
			    px4_motor[px4_motor_index] > kMaximumActuator) {
				return false;
			}
		}

		Eigen::Matrix<float, 4, 1> actuator;
		for (int i = 0; i < 4; ++i) {
			actuator(i) = px4_motor[i];
		}
		wrench = _allocator_forward * actuator;

		return wrench.array().isFinite().all() &&
		       std::fabs(wrench(0)) <= 1.f && std::fabs(wrench(1)) <= 1.f &&
		       std::fabs(wrench(2)) <= 1.f && wrench(3) <= 0.f && wrench(3) >= -1.f;
	}

	void publish_wrench(uint64_t now, uint64_t timestamp_sample,
			   const Eigen::Matrix<float, 4, 1> &wrench)
	{
		vehicle_torque_setpoint_s torque{};
		torque.timestamp = now;
		torque.timestamp_sample = timestamp_sample;
		torque.xyz[0] = wrench(0);
		torque.xyz[1] = wrench(1);
		torque.xyz[2] = wrench(2);
		_torque_pub.publish(torque);

		vehicle_thrust_setpoint_s thrust{};
		thrust.timestamp = now;
		thrust.timestamp_sample = timestamp_sample;
		thrust.xyz[0] = 0.f;
		thrust.xyz[1] = 0.f;
		thrust.xyz[2] = wrench(3);
		_thrust_pub.publish(thrust);
		_last_wrench_publish = now;
	}

	bool check_allocator_round_trip()
	{
		if (!_has_expected_motors) {
			return true;
		}

		actuator_motors_s actual{};

		/* In lockstep SITL the allocator output can legitimately carry the
		 * same simulated timestamp as the wrench it consumed. */
		if (!_actuator_motors_sub.copy(&actual) || actual.timestamp < _last_wrench_publish) {
			return true;
		}

		float maximum_error = 0.f;

		for (int motor = 0; motor < 4; ++motor) {
			if (!finite(actual.control[motor])) {
				maximum_error = 1.f;
				break;
			}

			const float error = std::fabs(actual.control[motor] - _expected_motors[motor]);
			maximum_error = error > maximum_error ? error : maximum_error;
			_actual_motors[motor] = actual.control[motor];
		}

		g_status.allocator_error = maximum_error;
		_allocator_error_count = maximum_error > kMaximumAllocatorError ? _allocator_error_count + 1 : 0;
		return _allocator_error_count < kAllocatorErrorLimit;
	}

	bool check_allocator_saturation()
	{
		control_allocator_status_s allocator{};

		if (!_allocator_status_sub.copy(&allocator)) {
			return true;
		}

		bool bad = !allocator.torque_setpoint_achieved || !allocator.thrust_setpoint_achieved;

		for (int motor = 0; motor < 4; ++motor) {
			bad = bad || allocator.actuator_saturation[motor] != control_allocator_status_s::ACTUATOR_SATURATION_OK;
		}

		_allocator_saturation_count = bad ? _allocator_saturation_count + 1 : 0;
		return _allocator_saturation_count < kAllocatorErrorLimit;
	}

	void publish_debug(uint64_t now, const float state[12], const float model_motor[4],
			   const float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT])
	{
		debug_array_s debug{};
		debug.timestamp = now;
		debug.id = 42;
		std::strncpy(debug.name, "tmfs", sizeof(debug.name));
		debug.data[0] = diagnostics[TINY_MPC_DIAG_POLICY];
		debug.data[1] = g_status.last_solve_us;
		for (int i = 0; i < 4; ++i) {
			debug.data[2 + i] = model_motor[i];
			debug.data[6 + i] = _expected_motors[i];
			debug.data[10 + i] = _actual_motors[i];
		}
		debug.data[14] = g_status.allocator_error;
		debug.data[15] = static_cast<float>(_scenario);
		debug.data[16] = _actuator_gain;
		debug.data[17] = _engaged ? 1.f : 0.f;
		debug.data[18] = g_status.failed ? 1.f : 0.f;
		debug.data[19] = static_cast<float>(g_status.failure_reason);
		for (int i = 0; i < 12; ++i) {
			debug.data[20 + i] = state[i];
		}
		for (int i = 0; i < TINY_MPC_DIAGNOSTIC_COUNT; ++i) {
			debug.data[32 + i] = diagnostics[i];
		}
		_debug_pub.publish(debug);
	}

	void request_position_control(int reason)
	{
		if (reason != FailureNone && !g_status.failed) {
			g_status.failure_reason = reason;
			g_status.failed = true;
			PX4_ERR("fail-closed to autonomous Loiter: %s", failure_name(reason));
		}

		if (!_failover_command_sent) {
			vehicle_command_s command{};
			command.timestamp = hrt_absolute_time();
			command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
			command.param1 = 1.f;
			/* AUTO_LOITER does not require an RC/manual-control setpoint and is
			 * therefore the deterministic autonomous recovery mode in headless
			 * SITL (and on an RC-less companion-computer vehicle). */
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
		g_status.ready = false;
		_has_expected_motors = false;
	}

	void step(uint64_t scheduled_now)
	{
		(void)scheduled_now;
		vehicle_local_position_s local{};
		vehicle_attitude_s attitude{};
		vehicle_angular_velocity_s angular{};
		vehicle_status_s status{};
		_local_position_sub.copy(&local);
		_attitude_sub.copy(&attitude);
		_angular_velocity_sub.copy(&angular);
		_vehicle_status_sub.copy(&status);
		/* Sample time after copying. Topic timestamps can otherwise be a few
		 * microseconds newer than the value passed in by the scheduler loop. */
		const uint64_t now = hrt_absolute_time();

		if (g_status.failed) {
			return;
		}

		if (!state_is_fresh(now, local, attitude, angular, status)) {
			if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				request_position_control(FailureStaleState);
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
			_has_expected_motors = false;
			g_status.engaged = false;
			return;
		}

		if (!_engaged) {
			latch_reference(local, attitude);
		}

		if (estimator_was_reset(local, attitude)) {
			request_position_control(FailureEstimatorReset);
			return;
		}

		if (!check_allocator_round_trip()) {
			request_position_control(FailureAllocatorRoundTrip);
			return;
		}

		if (!check_allocator_saturation()) {
			request_position_control(FailureAllocatorSaturation);
			return;
		}

		float state[12]{};

		if (!build_model_state(local, attitude, angular, state) || !state_inside_runtime_envelope(state)) {
			request_position_control(FailureStateEnvelope);
			return;
		}

		float model_motor[4]{};
		float plan[12]{};
		float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT]{};
		const uint64_t solve_start = monotonic_wall_time_us();
		MPC_FullState_Step(state, _scenario, model_motor, plan, diagnostics);
		const uint64_t solve_end = monotonic_wall_time_us();
		const uint64_t solve_elapsed = solve_start > 0 && solve_end >= solve_start ? solve_end - solve_start : UINT64_MAX;
		g_status.last_solve_us = static_cast<float>(solve_elapsed);
		g_status.max_solve_us = solve_elapsed > g_status.max_solve_us ? static_cast<float>(solve_elapsed) : g_status.max_solve_us;
		++g_status.solve_count;

		if (solve_elapsed > kSolveDeadlineUs) {
			request_position_control(FailureDeadline);
			return;
		}

		const int policy = static_cast<int>(diagnostics[TINY_MPC_DIAG_POLICY]);

		if (policy != TINY_MPC_SOLVE_CONVERGED && policy != TINY_MPC_SOLVE_BEST_EFFORT) {
			request_position_control(FailureSolver);
			return;
		}

		float px4_motor[4]{};
		Eigen::Matrix<float, 4, 1> wrench;

		if (!map_motors_to_wrench(model_motor, px4_motor, wrench)) {
			request_position_control(FailureMotorMapping);
			return;
		}

		for (int i = 0; i < 4; ++i) {
			_expected_motors[i] = px4_motor[i];
		}
		_has_expected_motors = true;
		publish_wrench(now, local.timestamp_sample, wrench);
		publish_debug(now, state, model_motor, diagnostics);
	}

	int _scenario;
	float _actuator_gain;
	float _hover_actuator{0.f};
	Eigen::Matrix<float, 4, 4> _allocator_forward{Eigen::Matrix<float, 4, 4>::Zero()};
	Vector3 _origin{};
	Quaternion _reference_q{};
	bool _engaged{false};
	bool _failover_command_sent{false};
	bool _has_expected_motors{false};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _quat_reset_counter{0};
	uint64_t _last_wrench_publish{0};
	int _allocator_error_count{0};
	int _allocator_saturation_count{0};
	float _expected_motors[4]{};
	float _actual_motors[4]{};

	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Publication<offboard_control_mode_s> _offboard_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<debug_array_s> _debug_pub{ORB_ID(debug_array)};
};

int controller_thread_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	g_status = SharedStatus{};
	g_status.running = true;
	g_status.scenario = g_requested_scenario;
	g_status.actuator_gain = g_requested_gain;
	FullStateController controller(g_requested_scenario, g_requested_gain);

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
	PX4_INFO("usage: tinympc_fullstate {start [hover|wall|degraded] [actuator_gain]|stop|status}");
	PX4_INFO("SITL only. Take off in Position mode, then switch to Offboard.");
}

} // namespace

extern "C" __EXPORT int tinympc_fullstate_main(int argc, char *argv[])
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

		g_requested_scenario = TINY_MPC_FULL_STATE_HOVER;

		if (argc >= 3) {
			if (std::strcmp(argv[2], "hover") == 0) {
				g_requested_scenario = TINY_MPC_FULL_STATE_HOVER;
			} else if (std::strcmp(argv[2], "wall") == 0) {
				g_requested_scenario = TINY_MPC_FULL_STATE_ACTUATOR_WALL;
			} else if (std::strcmp(argv[2], "degraded") == 0) {
				g_requested_scenario = TINY_MPC_FULL_STATE_DEGRADED_ACTUATOR_WALL;
			} else {
				PX4_ERR("unknown scenario: %s", argv[2]);
				return PX4_ERROR;
			}
		}

		g_requested_gain = kDefaultActuatorGain;

		if (argc >= 4) {
			char *end = nullptr;
			const float requested = std::strtof(argv[3], &end);

			if (end == argv[3] || !finite(requested) || requested < 0.1f || requested > 2.f) {
				PX4_ERR("actuator_gain must be within [0.1, 2.0]");
				return PX4_ERROR;
			}

			g_requested_gain = requested;
		}

		g_should_exit = false;
		g_task_id = px4_task_spawn_cmd("tinympc_fullstate",
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
		PX4_INFO("%s, ready=%d engaged=%d failed=%d scenario=%d reason=%s",
			 g_status.running ? "running" : "stopped",
			 g_status.ready, g_status.engaged, g_status.failed,
			 g_status.scenario, failure_name(g_status.failure_reason));
		PX4_INFO("hover=%.3f gain=%.3f solves=%llu solve_us=%.0f max=%.0f allocator_err=%.4f",
			 (double)g_status.hover_actuator, (double)g_status.actuator_gain,
			 static_cast<unsigned long long>(g_status.solve_count),
			 (double)g_status.last_solve_us, (double)g_status.max_solve_us,
			 (double)g_status.allocator_error);
		return PX4_OK;
	}

	print_usage();
	return PX4_ERROR;
}
