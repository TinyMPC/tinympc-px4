# One sample per controller cycle, including idle, stale-input and failed cycles.
# Counters are module-lifetime totals; session increments when the MPC is reset.
uint64 timestamp
uint64 run_id # module construction time; distinguishes restarts even across log gaps
uint64 resource_timestamp
uint64 cpuload_timestamp
uint32 sequence
uint32 session
uint16 schema_version
uint16 SCHEMA_VERSION = 1
char[17] source_id
char[13] source_commit
uint8 mode
uint8 failure_reason
bool running
bool ready
bool engaged
bool failed
bool state_valid
bool tracking_valid
bool solve_called
bool command_accepted
bool allocator_valid

uint32 cycle_time_us # controller work and resource sampling, excluding final status publication
uint32 cycle_max_us
uint32 start_lateness_us
uint32 cycle_interval_us
uint32 loop_overruns # controller work exceeded 20000 us
uint32 skipped_releases # scheduled 20 ms releases skipped after an overrun
uint32 solve_time_us # valid only when solve_called; wrapper-inclusive wall time
uint32 solve_max_us
uint32 solve_deadline_misses # solve exceeded 18000 us
uint32 solve_count
uint32 converged_count
uint32 best_effort_count
uint32 fallback_count
uint32 rejected_count
uint32 iteration_limit_count
uint16 iterations
uint16 max_iterations
int8 policy
float32 primal_residual
float32 dual_residual
float32 planned_state_violation # mixed state units; not a distance
float32 planned_input_violation # mixed input units

uint32[3] input_age_us # position, attitude, vehicle_status; UINT32_MAX missing, zero for future stamp
uint32 stale_cycles
uint32 estimator_reset_events
float32[3] position # NED metres
float32[3] velocity # NED m/s
float32[3] position_error # measured minus reference, metres
float32[3] velocity_error # measured minus reference, m/s
float32[4] command # acceleration NED m/s^2 and yaw rate rad/s, when solve_called
float32 position_error_norm
float32 corridor_violation # actual position, metres; course modes only
float32 hold_position_margin # min(0.5-|dx|, 0.5-|dy|, 0.4-|dz|), metres; hold/monitor only
float32 hold_velocity_margin # min(1-|vx|, 1-|vy|, 0.75-|vz|), m/s; hold/monitor only
float32 course_time
bool actuator_saturated
bool thrust_setpoint_achieved
bool torque_setpoint_achieved

float32 system_cpu_load # 0..1 from cpuload, NaN if unavailable
float32 system_ram_usage # 0..1 from cpuload, NaN if unavailable
uint32 heap_free_bytes # UINT32_MAX if unavailable; system-wide sample at 1 Hz
uint32 heap_min_free_bytes # minimum observed at 1 Hz, not an allocation watermark
uint32 stack_free_bytes # task stack high-water headroom at 1 Hz, UINT32_MAX if unavailable
uint32 stack_size_bytes
uint32 init_time_us
int32 init_heap_delta_bytes # system-wide net heap change across MPC_Init/Reset; INT32_MIN if unavailable
# Compact static metadata keeps the format inside PX4 v1.15.3's ULog limit.
# [horizon_knots, iteration_cap, single_precision (0/1), sample_time_s, primal_tol, dual_tol]
float32[6] config

uint8 ORB_QUEUE_LENGTH = 8
