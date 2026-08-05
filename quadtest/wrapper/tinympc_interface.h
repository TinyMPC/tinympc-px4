#ifndef TINYMPC_INTERFACE_H
#define TINYMPC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

enum TinyMpcScenario {
    TINY_MPC_SCENARIO_HOVER = 0,
    TINY_MPC_SCENARIO_VIRTUAL_WALL = 1,
    TINY_MPC_SCENARIO_CORRIDOR = 2,
    TINY_MPC_SCENARIO_REDUCED_AUTHORITY = 3,
    TINY_MPC_SCENARIO_WALL_UNCONSTRAINED = 4,
    TINY_MPC_SCENARIO_FIGURE_EIGHT_SOC = 5,
    TINY_MPC_SCENARIO_FIGURE_EIGHT_BOX = 6
};

enum TinyMpcSolvePolicy {
    TINY_MPC_SOLVE_FALLBACK = -1,
    TINY_MPC_SOLVE_INVALID = -2,
    TINY_MPC_SOLVE_CONVERGED = 1,
    TINY_MPC_SOLVE_BEST_EFFORT = 2
};

enum TinyMpcDiagnosticIndex {
    TINY_MPC_DIAG_POLICY = 0,
    TINY_MPC_DIAG_ITERATIONS = 1,
    TINY_MPC_DIAG_PRIMAL_RESIDUAL = 2,
    TINY_MPC_DIAG_DUAL_RESIDUAL = 3,
    TINY_MPC_DIAG_STATE_VIOLATION = 4,
    TINY_MPC_DIAG_INPUT_VIOLATION = 5,
    TINY_MPC_DIAG_FALLBACK_COUNT = 6,
    TINY_MPC_DIAG_SOLVE_COUNT = 7,
    TINY_MPC_DIAGNOSTIC_COUNT = 8
};

/* Experimental full-state controller scenarios. These use the supplied
 * 50 Hz hover-linearized model and normalized motor commands. They are kept
 * separate from TinyMpcScenario because the trajectory-setpoint demo must
 * never select direct motor control accidentally. */
enum TinyMpcFullStateScenario {
    TINY_MPC_FULL_STATE_ACTUATOR_WALL = 0,
    TINY_MPC_FULL_STATE_DEGRADED_ACTUATOR_WALL = 1,
    TINY_MPC_FULL_STATE_REACTIVE_BASELINE = 2
};

void MPC_Init(void);

/* Reset the engagement-relative origin, solver warm start, last-good command,
 * and diagnostic counters. The allocated solver is retained. */
void MPC_Reset(void);

/* Legacy reference-tracking API retained for compatibility. */
void MPC_Step(const float x[12],
              const float xref[12],
              float u[4]);

/* Legacy plan API. xplan is horizon column 10, approximately 200 ms ahead at
 * 50 Hz. This path uses input bounds but no meaningful state bounds. */
void MPC_Step_Plan(const float x[12],
                   const float xref[12],
                   float u[4],
                   float xplan[12]);

/* TinyMPC-focused onboard path. A scenario latches an engagement-relative
 * origin, creates its reference and hard state/input boxes, solves the MPC,
 * and applies the documented bounded/fallback policy. Diagnostics contains
 * TINY_MPC_DIAGNOSTIC_COUNT values indexed by TinyMpcDiagnosticIndex. */
void MPC_Step_Scenario(const float x[12],
                       int scenario,
                       float u[4],
                       float xplan[12],
                       float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT]);

/* Experimental 12-state / four-motor TinyMPC path.
 *
 * State convention:
 *   [position(3), local quaternion-vector/Rodrigues attitude coordinates(3),
 *    velocity(3), body rates(3)]
 *
 * motor_command is an absolute normalized motor command centered on the
 * supplied hover trim (0.297485). Internally, the model is augmented with
 * the previous four motor commands and TinyMPC optimizes command increments.
 * This makes both absolute motor authority and inter-sample slew hard box
 * constraints over the prediction horizon.
 *
 * IMPORTANT: this API is intentionally not connected to the PX4 output
 * model yet. Motor order/scaling and the identified airframe must be
 * validated on the target before flight use. */
void MPC_FullState_Init(void);
void MPC_FullState_Reset(void);
void MPC_FullState_Step(const float x[12],
                        int scenario,
                        float motor_command[4],
                        float xplan[12],
                        float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
