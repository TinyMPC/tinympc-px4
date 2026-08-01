#include "tinympc_interface.h"
#include "tinympc_full_state_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <tinympc/admm.hpp>
#include <tinympc/tiny_api.hpp>

namespace {

constexpr int kPhysicalStates = tinympc_full_state_model::kPhysicalStates;
constexpr int kMotorCount = tinympc_full_state_model::kMotorCount;
constexpr int kAugmentedStates = kPhysicalStates + kMotorCount;
constexpr int kHorizon = 25;
constexpr int kPlanLookaheadSteps = 10;
constexpr tinytype kRho = 20.0;
constexpr tinytype kHoverCommand = tinympc_full_state_model::kHoverCommand;
constexpr tinytype kMotorMinimum = 0.05;
constexpr tinytype kMotorMaximum = 0.60;
constexpr tinytype kDegradedMotorMaximum = 0.34;
constexpr tinytype kSlewPerSample = 0.045;
constexpr tinytype kUnbounded = 1.0e6;
constexpr tinytype kBoundTolerance = 1.0e-5;
/* Projected boxes alone are not enough: cap ADMM's dynamics/box disagreement
 * before accepting a fixed-budget iterate. The physical wall still keeps a
 * separate 1.5 cm margin because this mixed-unit residual is not a distance
 * guarantee. */
constexpr tinytype kBestEffortPrimalLimit = 2.0e-2;

using PhysicalState = Matrix<tinytype, kPhysicalStates, 1>;
using MotorVector = Matrix<tinytype, kMotorCount, 1>;
using AugmentedState = Matrix<tinytype, kAugmentedStates, 1>;
using AugmentedMatrix = Matrix<tinytype, kAugmentedStates, kAugmentedStates>;
using AugmentedInputMatrix = Matrix<tinytype, kAugmentedStates, kMotorCount>;
using StateTrajectory = Matrix<tinytype, kAugmentedStates, kHorizon>;
using InputTrajectory = Matrix<tinytype, kMotorCount, kHorizon - 1>;

TinySolver* g_full_solver = nullptr;
MotorVector g_motor_delta = MotorVector::Zero();
unsigned long g_full_solve_count = 0;
unsigned long g_full_fallback_count = 0;
int g_full_active_scenario = -1;

AugmentedMatrix makeAugmentedA()
{
    const auto A = tinympc_full_state_model::makePhysicalA();
    const auto B = tinympc_full_state_model::makePhysicalB();
    AugmentedMatrix augmented = AugmentedMatrix::Zero();
    augmented.block<kPhysicalStates, kPhysicalStates>(0, 0) = A;
    augmented.block<kPhysicalStates, kMotorCount>(0, kPhysicalStates) = B;
    augmented.block<kMotorCount, kMotorCount>(kPhysicalStates, kPhysicalStates).setIdentity();
    return augmented;
}

AugmentedInputMatrix makeAugmentedB()
{
    const auto B = tinympc_full_state_model::makePhysicalB();
    AugmentedInputMatrix augmented = AugmentedInputMatrix::Zero();
    augmented.block<kPhysicalStates, kMotorCount>(0, 0) = B;
    augmented.block<kMotorCount, kMotorCount>(kPhysicalStates, 0).setIdentity();
    return augmented;
}

AugmentedState makeWeights()
{
    AugmentedState Q;
    Q << 140.0, 120.0, 180.0,
         35.0, 35.0, 60.0,
         24.0, 24.0, 30.0,
         3.0, 3.0, 5.0,
         0.5, 0.5, 0.5, 0.5;
    return Q;
}

bool finite(const AugmentedState& x)
{
    return x.array().isFinite().all();
}

int sanitizeFullScenario(int scenario)
{
    if (scenario < TINY_MPC_FULL_STATE_ACTUATOR_WALL ||
        scenario > TINY_MPC_FULL_STATE_REACTIVE_BASELINE) {
        return TINY_MPC_FULL_STATE_ACTUATOR_WALL;
    }
    return scenario;
}

void resetFullWarmStart()
{
    if (g_full_solver == nullptr) {
        return;
    }

    TinyWorkspace* work = g_full_solver->work;
    work->x.setZero();
    work->u.setZero();
    work->v.setZero();
    work->vnew.setZero();
    work->z.setZero();
    work->znew.setZero();
    work->g.setZero();
    work->y.setZero();
    work->q.setZero();
    work->r.setZero();
    g_full_solver->solution->x.setZero();
    g_full_solver->solution->u.setZero();
    g_full_solver->solution->iter = 0;
    g_full_solver->solution->solved = 0;
}

void setFutureStateBox(StateTrajectory& lower,
                       StateTrajectory& upper,
                       int state,
                       tinytype minimum,
                       tinytype maximum)
{
    for (int k = 1; k < kHorizon; ++k) {
        lower(state, k) = minimum;
        upper(state, k) = maximum;
    }
}

void configureFullScenario(int scenario,
                           AugmentedState& reference,
                           StateTrajectory& stateLower,
                           StateTrajectory& stateUpper,
                           InputTrajectory& inputLower,
                           InputTrajectory& inputUpper)
{
    reference.setZero();
    /* Intentionally place the requested target beyond the virtual wall. The
     * constrained modes must settle at the closest feasible point; the
     * reactive baseline tracks the unsafe request. */
    reference(0) = 0.95;

    stateLower.setConstant(-kUnbounded);
    stateUpper.setConstant(kUnbounded);

    /* All modes model the same hardware. Motor command is a state so the
     * box applies to every future command, while the optimized input is the
     * command change and receives the slew box. */
    for (int motor = 0; motor < kMotorCount; ++motor) {
        const tinytype maximum =
            (scenario == TINY_MPC_FULL_STATE_DEGRADED_ACTUATOR_WALL && motor == 0)
                ? kDegradedMotorMaximum : kMotorMaximum;
        setFutureStateBox(stateLower, stateUpper,
                          kPhysicalStates + motor,
                          kMotorMinimum - kHoverCommand,
                          maximum - kHoverCommand);
    }

    inputLower.setConstant(-kSlewPerSample);
    inputUpper.setConstant(kSlewPerSample);

    if (scenario != TINY_MPC_FULL_STATE_REACTIVE_BASELINE) {
        /* Predictive flight envelope. These are deliberately simultaneous:
         * the controller must brake before the wall without asking for a
         * motor, attitude, or rate trajectory the airframe cannot deliver. */
        /* The benchmark treats x = 0.87 m as the physical wall and keeps a
         * 1.5 cm planning margin for finite solver/model error. */
        setFutureStateBox(stateLower, stateUpper, 0, -0.75, 0.855);
        setFutureStateBox(stateLower, stateUpper, 1, -0.50, 0.50);
        setFutureStateBox(stateLower, stateUpper, 2, -0.40, 0.40);
        setFutureStateBox(stateLower, stateUpper, 3, -0.18, 0.18);
        setFutureStateBox(stateLower, stateUpper, 4, -0.18, 0.18);
        setFutureStateBox(stateLower, stateUpper, 5, -0.25, 0.25);
        setFutureStateBox(stateLower, stateUpper, 6, -1.20, 1.20);
        setFutureStateBox(stateLower, stateUpper, 7, -1.00, 1.00);
        setFutureStateBox(stateLower, stateUpper, 8, -0.80, 0.80);
        setFutureStateBox(stateLower, stateUpper, 9, -2.80, 2.80);
        setFutureStateBox(stateLower, stateUpper, 10, -2.80, 2.80);
        setFutureStateBox(stateLower, stateUpper, 11, -2.00, 2.00);
    }
}

tinytype maximumViolation(const tinyMatrix& values,
                          const tinyMatrix& lower,
                          const tinyMatrix& upper)
{
    const tinytype below = (lower - values).maxCoeff();
    const tinytype above = (values - upper).maxCoeff();
    return std::max<tinytype>(0.0, std::max(below, above));
}

void copyFullOutput(const MotorVector& motor,
                    const AugmentedState& plan,
                    float motorCommand[kMotorCount],
                    float xplan[kPhysicalStates])
{
    for (int i = 0; i < kMotorCount; ++i) {
        motorCommand[i] = static_cast<float>(motor(i));
    }
    for (int i = 0; i < kPhysicalStates; ++i) {
        xplan[i] = static_cast<float>(plan(i));
    }
}

void fillFullDiagnostics(float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT],
                         int policy,
                         tinytype stateViolation,
                         tinytype inputViolation)
{
    if (diagnostics == nullptr) {
        return;
    }

    const tinytype primal = g_full_solver == nullptr ? kUnbounded :
        std::max(g_full_solver->work->primal_residual_state,
                 g_full_solver->work->primal_residual_input);
    const tinytype dual = g_full_solver == nullptr ? kUnbounded :
        std::max(g_full_solver->work->dual_residual_state,
                 g_full_solver->work->dual_residual_input);
    const int iterations = g_full_solver == nullptr ? 0 :
        g_full_solver->solution->iter;

    diagnostics[TINY_MPC_DIAG_POLICY] = static_cast<float>(policy);
    diagnostics[TINY_MPC_DIAG_ITERATIONS] = static_cast<float>(iterations);
    diagnostics[TINY_MPC_DIAG_PRIMAL_RESIDUAL] = static_cast<float>(primal);
    diagnostics[TINY_MPC_DIAG_DUAL_RESIDUAL] = static_cast<float>(dual);
    diagnostics[TINY_MPC_DIAG_STATE_VIOLATION] = static_cast<float>(stateViolation);
    diagnostics[TINY_MPC_DIAG_INPUT_VIOLATION] = static_cast<float>(inputViolation);
    diagnostics[TINY_MPC_DIAG_FALLBACK_COUNT] = static_cast<float>(g_full_fallback_count);
    diagnostics[TINY_MPC_DIAG_SOLVE_COUNT] = static_cast<float>(g_full_solve_count);
}

MotorVector safeReturnTowardHover()
{
    MotorVector nextDelta;
    for (int i = 0; i < kMotorCount; ++i) {
        nextDelta(i) = g_motor_delta(i) +
            std::clamp(-g_motor_delta(i), -kSlewPerSample, kSlewPerSample);
    }
    g_motor_delta = nextDelta;
    return MotorVector::Constant(kHoverCommand) + g_motor_delta;
}

} // namespace

void MPC_FullState_Init(void)
{
    if (g_full_solver != nullptr) {
        return;
    }

    const AugmentedMatrix A = makeAugmentedA();
    const AugmentedInputMatrix B = makeAugmentedB();
    const AugmentedState f = AugmentedState::Zero();
    const AugmentedState Q = makeWeights();
    MotorVector R;
    R << 2.0, 2.0, 2.0, 2.0;

    const int status = tiny_setup(&g_full_solver,
                                  A,
                                  B,
                                  f,
                                  Q.asDiagonal(),
                                  R.asDiagonal(),
                                  kRho,
                                  kAugmentedStates,
                                  kMotorCount,
                                  kHorizon,
                                  0);
    if (status != 0 || g_full_solver == nullptr) {
        std::fprintf(stderr, "TinyMPC full-state setup failed with status %d\n", status);
        g_full_solver = nullptr;
        return;
    }

    g_full_solver->settings->max_iter = 250;
    g_full_solver->settings->check_termination = 1;
    tiny_set_x_ref(g_full_solver, StateTrajectory::Zero());
    tiny_set_u_ref(g_full_solver, InputTrajectory::Zero());
}

void MPC_FullState_Reset(void)
{
    g_motor_delta.setZero();
    g_full_solve_count = 0;
    g_full_fallback_count = 0;
    g_full_active_scenario = -1;
    resetFullWarmStart();
}

void MPC_FullState_Step(const float x[kPhysicalStates],
                        int scenario,
                        float motorCommand[kMotorCount],
                        float xplan[kPhysicalStates],
                        float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT])
{
    if (g_full_solver == nullptr) {
        MPC_FullState_Init();
    }

    AugmentedState x0;
    for (int i = 0; i < kPhysicalStates; ++i) {
        x0(i) = static_cast<tinytype>(x[i]);
    }
    x0.tail<kMotorCount>() = g_motor_delta;

    if (g_full_solver == nullptr || !finite(x0)) {
        const MotorVector safeMotor = safeReturnTowardHover();
        const AugmentedState safePlan = finite(x0) ? x0 : AugmentedState::Zero();
        copyFullOutput(safeMotor, safePlan, motorCommand, xplan);
        fillFullDiagnostics(diagnostics, TINY_MPC_SOLVE_INVALID,
                            kUnbounded, kUnbounded);
        return;
    }

    scenario = sanitizeFullScenario(scenario);
    if (scenario != g_full_active_scenario) {
        g_full_active_scenario = scenario;
        g_motor_delta.setZero();
        x0.tail<kMotorCount>().setZero();
        resetFullWarmStart();
    }

    AugmentedState reference;
    StateTrajectory stateLower;
    StateTrajectory stateUpper;
    InputTrajectory inputLower;
    InputTrajectory inputUpper;
    configureFullScenario(scenario, reference, stateLower, stateUpper,
                          inputLower, inputUpper);

    /* The augmented motor-state box couples command and slew after ADMM
     * convergence. Also intersect the first slew knot with the measured
     * previous command so every accepted first command is actuator-feasible
     * even for a bounded best-effort iterate. */
    for (int motor = 0; motor < kMotorCount; ++motor) {
        const tinytype maximum =
            (scenario == TINY_MPC_FULL_STATE_DEGRADED_ACTUATOR_WALL && motor == 0)
                ? kDegradedMotorMaximum : kMotorMaximum;
        inputLower(motor, 0) = std::max(
            inputLower(motor, 0),
            kMotorMinimum - kHoverCommand - g_motor_delta(motor));
        inputUpper(motor, 0) = std::min(
            inputUpper(motor, 0),
            maximum - kHoverCommand - g_motor_delta(motor));
    }

    int apiStatus = 0;
    apiStatus |= tiny_set_x0(g_full_solver, x0);
    apiStatus |= tiny_set_x_ref(g_full_solver,
                                reference.replicate<1, kHorizon>());
    apiStatus |= tiny_set_bound_constraints(g_full_solver,
                                            stateLower,
                                            stateUpper,
                                            inputLower,
                                            inputUpper);

    if (apiStatus != 0) {
        ++g_full_fallback_count;
        const MotorVector safeMotor = safeReturnTowardHover();
        copyFullOutput(safeMotor, x0, motorCommand, xplan);
        fillFullDiagnostics(diagnostics, TINY_MPC_SOLVE_INVALID,
                            kUnbounded, kUnbounded);
        return;
    }

    update_linear_cost(g_full_solver);
    const int solveStatus = tiny_solve(g_full_solver);
    ++g_full_solve_count;

    const tinytype stateViolation = maximumViolation(
        g_full_solver->solution->x, stateLower, stateUpper);
    const tinytype inputViolation = maximumViolation(
        g_full_solver->solution->u, inputLower, inputUpper);
    const MotorVector slew = g_full_solver->solution->u.col(0);
    const MotorVector candidateDelta = g_motor_delta + slew;
    const MotorVector candidateMotor =
        MotorVector::Constant(kHoverCommand) + candidateDelta;
    const tinytype motor0Maximum =
        scenario == TINY_MPC_FULL_STATE_DEGRADED_ACTUATOR_WALL
            ? kDegradedMotorMaximum : kMotorMaximum;

    bool actualCommandFeasible = slew.array().isFinite().all() &&
        (slew.array().abs() <= kSlewPerSample + kBoundTolerance).all();
    for (int motor = 0; motor < kMotorCount; ++motor) {
        const tinytype maximum = motor == 0 ? motor0Maximum : kMotorMaximum;
        actualCommandFeasible = actualCommandFeasible &&
            candidateMotor(motor) >= kMotorMinimum - kBoundTolerance &&
            candidateMotor(motor) <= maximum + kBoundTolerance;
    }

    const bool finiteSolution = finite(g_full_solver->solution->x.col(
                                    kPlanLookaheadSteps)) &&
                                candidateMotor.array().isFinite().all();
    const bool projectedFeasible =
        stateViolation <= kBoundTolerance &&
        inputViolation <= kBoundTolerance &&
        actualCommandFeasible;
    const tinytype primalResidual = std::max(
        g_full_solver->work->primal_residual_state,
        g_full_solver->work->primal_residual_input);
    const bool dynamicsCloseEnough =
        primalResidual <= kBestEffortPrimalLimit;

    int policy = TINY_MPC_SOLVE_FALLBACK;
    if (finiteSolution && projectedFeasible && solveStatus == 0) {
        policy = TINY_MPC_SOLVE_CONVERGED;
    } else if (finiteSolution && projectedFeasible && dynamicsCloseEnough) {
        policy = TINY_MPC_SOLVE_BEST_EFFORT;
    }

    AugmentedState plan = x0;
    MotorVector outputMotor;
    if (policy == TINY_MPC_SOLVE_CONVERGED ||
        policy == TINY_MPC_SOLVE_BEST_EFFORT) {
        g_motor_delta = candidateDelta;
        outputMotor = candidateMotor;
        plan = g_full_solver->solution->x.col(kPlanLookaheadSteps);
    } else {
        ++g_full_fallback_count;
        outputMotor = safeReturnTowardHover();
    }

    copyFullOutput(outputMotor, plan, motorCommand, xplan);
    fillFullDiagnostics(diagnostics, policy, stateViolation, inputViolation);
}
