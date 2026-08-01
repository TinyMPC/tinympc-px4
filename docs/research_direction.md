# TinyMPC-PX4 research direction

## The useful claim

“Nobody runs MPC onboard PX4” is too broad. Prior Pixhawk/PX4 controllers exist.
The defensible target is an open and reproducible constrained TinyMPC
trajectory controller that runs inside modern PX4 on a flight-controller MCU,
without a companion optimizer, with a clearly defined control boundary,
timing/memory evidence, logged solver health, and matched baselines.

The direct acceleration mode establishes the intended boundary:

```text
TinyMPC: position/velocity feedback, trajectory constraints, acceleration plan
PX4:     acceleration-to-attitude/thrust, attitude/rate loops, allocation, safety
```

Guidance mode remains valuable as a conservative integration baseline, but it
does not isolate TinyMPC constraint enforcement because PX4's position loop can
modify the downstream acceleration.

## Benchmark ladder

### 1. Geofence braking with momentum

Use a feasible target near a virtual wall. Apply the same repeatable impulse or
wind disturbance to TinyMPC and tuned PX4 baselines. Measure peak boundary
violation, time outside, recovery time, tracking cost, input saturation,
solver residuals, and deadline misses. Do not manufacture the result by giving
only the baseline an impossible reference.

The checked-in native `virtual_wall` comparison is the deterministic first
step. The next step is the same protocol in SIH, Gazebo, and then a restrained
flight volume with a conservative wall backoff.

### 2. Corridor / obstacle passage

Track through a narrow corridor under a crosswind or lateral impulse. Box
constraints cover an axis-aligned corridor; linear half-spaces can later model
rotated walls. A moving obstacle example should use time-varying constraints
and a stated fallback if the horizon becomes infeasible.

### 3. Reduced thrust or actuator authority

Repeat a matched path after reducing known acceleration/thrust authority. The
MPC should replan using the reduced input set. Compare against a controller
whose unconstrained request is clipped after control computation. Extend this
to asymmetric limits and motor-failure allocation only after the mapping from
motor loss to translational authority is validated.

### 4. Tilt/thrust envelope

Replace independent acceleration boxes with a physically meaningful coupled
tilt/thrust constraint. TinyMPC supports conic constraints, but the chosen
specific-force convention and PX4 hover-thrust mapping must match the vehicle.

### 5. Constraint-aware landing

Add descent-rate, thrust, and touchdown-region constraints with explicit
infeasibility handling. This is a strong application once state-estimation and
ground-effect assumptions are documented.

## Evidence needed for an onboard result

For each solver call, log:

- p50/p95/p99/worst execution time and 20 ms deadline misses;
- iteration count, primal/dual residuals, converged/best-effort/fallback state;
- flight-controller CPU load, stack/high-water mark, static RAM, heap use, and
  firmware flash delta;
- tracking error, constraint margin/violation, time outside, saturation, and
  recovery time.

Run the same reference, disturbances, estimator, inner loops, actuator limits,
and termination rules for all baselines. Include stock PX4, tuned PX4, TinyMPC
guidance, and TinyMPC direct. Report failure cases, not only successful runs.

## Immediate engineering work

1. Publish the eight-value solver diagnostic vector to a logged uORB topic.
2. Measure the generated solver on a Pixhawk-class target; desktop timing is
   only a build/regression signal.
3. Replace runtime TinyMPC setup/allocation with generated static solver data
   for deterministic memory use.
4. Identify translational dynamics and acceleration/thrust mapping for the
   actual airframe.
5. Add estimator-reset handling and an explicit arming/engagement reset.
6. Add coupled tilt/thrust constraints and robust backoffs.
7. Automate SIH/Gazebo matched-baseline runs and ULog metric extraction.

These tasks are more important to the onboard contribution than adding many
unvalidated trajectories.
