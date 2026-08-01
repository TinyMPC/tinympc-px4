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
    TINY_MPC_SCENARIO_WALL_UNCONSTRAINED = 4
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

#ifdef __cplusplus
}
#endif

#endif
