#include "tinympc_interface.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <tinympc/admm.hpp>
#include <tinympc/tiny_api.hpp>

namespace {

constexpr int kNumStates = 12;
constexpr int kNumInputs = 4;
constexpr int kHorizon = 25;
constexpr int kPlanLookaheadSteps = 10;
constexpr tinytype kSampleTime = 0.02;
constexpr tinytype kRho = 5.0;
constexpr tinytype kDefaultAccelLimit = 4.0;
constexpr tinytype kDefaultYawRateLimit = 1.0;
constexpr tinytype kUnbounded = 1.0e6;
constexpr tinytype kBoundTolerance = 1.0e-5;

TinySolver* g_solver = nullptr;
bool g_has_origin = false;
int g_active_scenario = -1;
unsigned long g_solve_count = 0;
unsigned long g_fallback_count = 0;

typedef Matrix<tinytype, kNumStates, 1> StateVector;
typedef Matrix<tinytype, kNumInputs, 1> InputVector;
typedef Matrix<tinytype, kNumStates, kNumStates> StateMatrix;
typedef Matrix<tinytype, kNumStates, kNumInputs> InputMatrix;
typedef Matrix<tinytype, kNumStates, kHorizon> StateTrajectory;
typedef Matrix<tinytype, kNumInputs, kHorizon - 1> InputTrajectory;

StateVector g_origin = StateVector::Zero();
StateVector g_last_plan = StateVector::Zero();
InputVector g_last_input = InputVector::Zero();

StateMatrix makeAdyn()
{
    StateMatrix A = StateMatrix::Identity();
    A(0, 6) = kSampleTime;
    A(1, 7) = kSampleTime;
    A(2, 8) = kSampleTime;
    return A;
}

InputMatrix makeBdyn()
{
    InputMatrix B = InputMatrix::Zero();
    B(0, 0) = 0.5 * kSampleTime * kSampleTime;
    B(1, 1) = 0.5 * kSampleTime * kSampleTime;
    B(2, 2) = 0.5 * kSampleTime * kSampleTime;
    B(6, 0) = kSampleTime;
    B(7, 1) = kSampleTime;
    B(8, 2) = kSampleTime;
    B(5, 3) = kSampleTime;
    return B;
}

StateVector makeStateWeights()
{
    StateVector Q;
    Q << 100.0, 100.0, 150.0,
         0.0, 0.0, 5.0,
         15.0, 15.0, 20.0,
         0.0, 0.0, 0.0;
    return Q;
}

StateVector copyState(const float values[kNumStates])
{
    StateVector out;
    for (int i = 0; i < kNumStates; ++i) {
        out(i) = static_cast<tinytype>(values[i]);
    }
    return out;
}

bool stateIsFinite(const StateVector& x)
{
    return x.array().isFinite().all();
}

void copyOutput(const InputVector& input,
                const StateVector& plan,
                float u[kNumInputs],
                float xplan[kNumStates])
{
    for (int i = 0; i < kNumInputs; ++i) {
        u[i] = static_cast<float>(input(i));
    }
    for (int i = 0; i < kNumStates; ++i) {
        xplan[i] = static_cast<float>(plan(i));
    }
}

void resetWarmStart()
{
    if (g_solver == nullptr) {
        return;
    }

    TinyWorkspace* work = g_solver->work;
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
    g_solver->solution->x.setZero();
    g_solver->solution->u.setZero();
    g_solver->solution->iter = 0;
    g_solver->solution->solved = 0;
}

int sanitizeScenario(int scenario)
{
    if (scenario < TINY_MPC_SCENARIO_HOVER ||
        scenario > TINY_MPC_SCENARIO_WALL_UNCONSTRAINED) {
        return TINY_MPC_SCENARIO_HOVER;
    }
    return scenario;
}

void setStateBox(StateTrajectory& lower,
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

void configureScenario(int scenario,
                       StateVector& reference,
                       StateTrajectory& stateLower,
                       StateTrajectory& stateUpper,
                       InputTrajectory& inputLower,
                       InputTrajectory& inputUpper)
{
    reference = g_origin;
    reference.segment<6>(6).setZero();
    reference(2) = g_origin(2) - 1.0;

    stateLower.setConstant(-kUnbounded);
    stateUpper.setConstant(kUnbounded);

    tinytype accelXY = kDefaultAccelLimit;
    tinytype accelZ = kDefaultAccelLimit;
    tinytype yawRate = kDefaultYawRateLimit;

    if (scenario != TINY_MPC_SCENARIO_WALL_UNCONSTRAINED) {
        setStateBox(stateLower, stateUpper, 2,
                    g_origin(2) - 1.5, g_origin(2) + 0.25);
        setStateBox(stateLower, stateUpper, 8, -1.0, 1.0);
    }

    switch (scenario) {
    case TINY_MPC_SCENARIO_VIRTUAL_WALL:
        reference(0) = g_origin(0) + 0.85;
        setStateBox(stateLower, stateUpper, 0,
                    g_origin(0) - 0.5, g_origin(0) + 0.98);
        setStateBox(stateLower, stateUpper, 1,
                    g_origin(1) - 0.75, g_origin(1) + 0.75);
        setStateBox(stateLower, stateUpper, 6, -1.2, 1.2);
        setStateBox(stateLower, stateUpper, 7, -1.2, 1.2);
        break;

    case TINY_MPC_SCENARIO_CORRIDOR:
        reference(0) = g_origin(0) + 2.0;
        reference(1) = g_origin(1) + 0.25;
        setStateBox(stateLower, stateUpper, 0,
                    g_origin(0) - 0.5, g_origin(0) + 3.0);
        setStateBox(stateLower, stateUpper, 1,
                    g_origin(1) - 0.35, g_origin(1) + 0.35);
        setStateBox(stateLower, stateUpper, 7, -0.8, 0.8);
        break;

    case TINY_MPC_SCENARIO_REDUCED_AUTHORITY:
        reference(0) = g_origin(0) + 1.0;
        setStateBox(stateLower, stateUpper, 0,
                    g_origin(0) - 1.0, g_origin(0) + 2.0);
        setStateBox(stateLower, stateUpper, 1,
                    g_origin(1) - 1.0, g_origin(1) + 1.0);
        setStateBox(stateLower, stateUpper, 6, -0.8, 0.8);
        setStateBox(stateLower, stateUpper, 7, -0.8, 0.8);
        accelXY = 0.75;
        accelZ = 1.5;
        yawRate = 0.5;
        break;

    case TINY_MPC_SCENARIO_WALL_UNCONSTRAINED:
        reference(0) = g_origin(0) + 0.85;
        break;

    case TINY_MPC_SCENARIO_HOVER:
    default:
        setStateBox(stateLower, stateUpper, 0,
                    g_origin(0) - 2.0, g_origin(0) + 2.0);
        setStateBox(stateLower, stateUpper, 1,
                    g_origin(1) - 2.0, g_origin(1) + 2.0);
        setStateBox(stateLower, stateUpper, 6, -1.5, 1.5);
        setStateBox(stateLower, stateUpper, 7, -1.5, 1.5);
        break;
    }

    InputVector minimum;
    minimum << -accelXY, -accelXY, -accelZ, -yawRate;
    InputVector maximum = -minimum;
    inputLower = minimum.replicate<1, kHorizon - 1>();
    inputUpper = maximum.replicate<1, kHorizon - 1>();
}

tinytype maximumViolation(const tinyMatrix& values,
                          const tinyMatrix& lower,
                          const tinyMatrix& upper)
{
    const tinytype lowerViolation = (lower - values).maxCoeff();
    const tinytype upperViolation = (values - upper).maxCoeff();
    return std::max<tinytype>(0.0, std::max(lowerViolation, upperViolation));
}

void fillDiagnostics(float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT],
                     int policy,
                     tinytype stateViolation,
                     tinytype inputViolation)
{
    if (diagnostics == nullptr) {
        return;
    }

    const tinytype primal = g_solver == nullptr ? kUnbounded :
        std::max(g_solver->work->primal_residual_state,
                 g_solver->work->primal_residual_input);
    const tinytype dual = g_solver == nullptr ? kUnbounded :
        std::max(g_solver->work->dual_residual_state,
                 g_solver->work->dual_residual_input);
    const int iterations = g_solver == nullptr ? 0 : g_solver->solution->iter;

    diagnostics[TINY_MPC_DIAG_POLICY] = static_cast<float>(policy);
    diagnostics[TINY_MPC_DIAG_ITERATIONS] = static_cast<float>(iterations);
    diagnostics[TINY_MPC_DIAG_PRIMAL_RESIDUAL] = static_cast<float>(primal);
    diagnostics[TINY_MPC_DIAG_DUAL_RESIDUAL] = static_cast<float>(dual);
    diagnostics[TINY_MPC_DIAG_STATE_VIOLATION] = static_cast<float>(stateViolation);
    diagnostics[TINY_MPC_DIAG_INPUT_VIOLATION] = static_cast<float>(inputViolation);
    diagnostics[TINY_MPC_DIAG_FALLBACK_COUNT] = static_cast<float>(g_fallback_count);
    diagnostics[TINY_MPC_DIAG_SOLVE_COUNT] = static_cast<float>(g_solve_count);
}

bool solveReference(const StateVector& x0,
                    const StateVector& reference,
                    const StateTrajectory& stateLower,
                    const StateTrajectory& stateUpper,
                    const InputTrajectory& inputLower,
                    const InputTrajectory& inputUpper,
                    InputVector& input,
                    StateVector& plan,
                    int& solveStatus,
                    tinytype& stateViolation,
                    tinytype& inputViolation)
{
    const StateTrajectory referenceHorizon = reference.replicate<1, kHorizon>();
    int apiStatus = 0;
    apiStatus |= tiny_set_x0(g_solver, x0);
    apiStatus |= tiny_set_x_ref(g_solver, referenceHorizon);
    apiStatus |= tiny_set_bound_constraints(g_solver,
                                            stateLower,
                                            stateUpper,
                                            inputLower,
                                            inputUpper);
    if (apiStatus != 0) {
        solveStatus = apiStatus;
        stateViolation = kUnbounded;
        inputViolation = kUnbounded;
        return false;
    }

    update_linear_cost(g_solver);
    solveStatus = tiny_solve(g_solver);
    ++g_solve_count;

    input = g_solver->solution->u.col(0);
    plan = g_solver->solution->x.col(kPlanLookaheadSteps);
    stateViolation = maximumViolation(g_solver->solution->x, stateLower, stateUpper);
    inputViolation = maximumViolation(g_solver->solution->u, inputLower, inputUpper);
    return stateIsFinite(plan) && input.array().isFinite().all();
}

} // namespace

void MPC_Init(void)
{
    if (g_solver != nullptr) {
        return;
    }

    const StateMatrix Adyn = makeAdyn();
    const InputMatrix Bdyn = makeBdyn();
    const StateVector fdyn = StateVector::Zero();
    const StateVector Q = makeStateWeights();
    const InputVector R = InputVector::Ones();

    const int status = tiny_setup(&g_solver,
                                  Adyn,
                                  Bdyn,
                                  fdyn,
                                  Q.asDiagonal(),
                                  R.asDiagonal(),
                                  kRho,
                                  kNumStates,
                                  kNumInputs,
                                  kHorizon,
                                  0);

    if (status != 0 || g_solver == nullptr) {
        std::fprintf(stderr, "TinyMPC setup failed with status %d\n", status);
        g_solver = nullptr;
        return;
    }

    g_solver->settings->max_iter = 50;
    g_solver->settings->check_termination = 1;
    tiny_set_x_ref(g_solver, StateTrajectory::Zero());
    tiny_set_u_ref(g_solver, InputTrajectory::Zero());
}

void MPC_Reset(void)
{
    g_has_origin = false;
    g_active_scenario = -1;
    g_solve_count = 0;
    g_fallback_count = 0;
    g_origin.setZero();
    g_last_plan.setZero();
    g_last_input.setZero();
    resetWarmStart();
}

void MPC_Step_Scenario(const float x[12],
                       int scenario,
                       float u[4],
                       float xplan[12],
                       float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT])
{
    if (g_solver == nullptr) {
        MPC_Init();
    }

    const StateVector x0 = copyState(x);
    if (g_solver == nullptr || !stateIsFinite(x0)) {
        InputVector zero = InputVector::Zero();
        const StateVector safePlan = stateIsFinite(x0) ? x0 : StateVector::Zero();
        copyOutput(zero, safePlan, u, xplan);
        fillDiagnostics(diagnostics, TINY_MPC_SOLVE_INVALID, kUnbounded, kUnbounded);
        return;
    }

    scenario = sanitizeScenario(scenario);
    if (scenario != g_active_scenario) {
        g_has_origin = false;
        g_active_scenario = scenario;
        resetWarmStart();
    }

    if (!g_has_origin) {
        g_origin = x0;
        g_last_plan = x0;
        g_last_input.setZero();
        g_has_origin = true;
    }

    StateVector reference;
    StateTrajectory stateLower;
    StateTrajectory stateUpper;
    InputTrajectory inputLower;
    InputTrajectory inputUpper;
    configureScenario(scenario,
                      reference,
                      stateLower,
                      stateUpper,
                      inputLower,
                      inputUpper);

    InputVector candidateInput;
    StateVector candidatePlan;
    int solveStatus = 1;
    tinytype stateViolation = kUnbounded;
    tinytype inputViolation = kUnbounded;
    const bool finite = solveReference(x0,
                                       reference,
                                       stateLower,
                                       stateUpper,
                                       inputLower,
                                       inputUpper,
                                       candidateInput,
                                       candidatePlan,
                                       solveStatus,
                                       stateViolation,
                                       inputViolation);

    const bool boundsSatisfied = stateViolation <= kBoundTolerance &&
                                 inputViolation <= kBoundTolerance;

    int policy = TINY_MPC_SOLVE_FALLBACK;
    if (finite && boundsSatisfied && solveStatus == 0) {
        policy = TINY_MPC_SOLVE_CONVERGED;
    } else if (finite && boundsSatisfied) {
        // ADMM's projected iterate is box-feasible even when the fixed
        // onboard iteration budget expires. Use that bounded best-known
        // command and expose its residuals; only malformed/non-finite output
        // triggers the fallback.
        policy = TINY_MPC_SOLVE_BEST_EFFORT;
    }

    if (policy == TINY_MPC_SOLVE_CONVERGED ||
        policy == TINY_MPC_SOLVE_BEST_EFFORT) {
        g_last_input = candidateInput;
        g_last_plan = candidatePlan;
    } else {
        ++g_fallback_count;
        // Zero acceleration means hover at PX4's acceleration interface. Do
        // not hold a stale nonzero acceleration when this wrapper is used in
        // direct mode; the last plan is retained for legacy guidance mode.
        g_last_input.setZero();
    }

    copyOutput(g_last_input, g_last_plan, u, xplan);
    fillDiagnostics(diagnostics, policy, stateViolation, inputViolation);
}

void MPC_Step_Plan(const float x[12],
                   const float xref[12],
                   float u[4],
                   float xplan[12])
{
    if (g_solver == nullptr) {
        MPC_Init();
    }

    const StateVector x0 = copyState(x);
    const StateVector reference = copyState(xref);
    if (g_solver == nullptr || !stateIsFinite(x0) || !stateIsFinite(reference)) {
        copyOutput(InputVector::Zero(), stateIsFinite(x0) ? x0 : StateVector::Zero(), u, xplan);
        return;
    }

    const StateTrajectory stateLower = StateTrajectory::Constant(-kUnbounded);
    const StateTrajectory stateUpper = StateTrajectory::Constant(kUnbounded);
    InputVector minimum;
    minimum << -kDefaultAccelLimit, -kDefaultAccelLimit,
               -kDefaultAccelLimit, -kDefaultYawRateLimit;
    const InputTrajectory inputLower = minimum.replicate<1, kHorizon - 1>();
    const InputTrajectory inputUpper = (-minimum).replicate<1, kHorizon - 1>();

    InputVector candidateInput;
    StateVector candidatePlan;
    int solveStatus = 1;
    tinytype stateViolation = 0.0;
    tinytype inputViolation = 0.0;
    const bool finite = solveReference(x0,
                                       reference,
                                       stateLower,
                                       stateUpper,
                                       inputLower,
                                       inputUpper,
                                       candidateInput,
                                       candidatePlan,
                                       solveStatus,
                                       stateViolation,
                                       inputViolation);
    if (finite) {
        copyOutput(candidateInput, candidatePlan, u, xplan);
    } else {
        copyOutput(InputVector::Zero(), x0, u, xplan);
    }
}

void MPC_Step(const float x[12],
              const float xref[12],
              float u[4])
{
    float xplan[kNumStates];
    MPC_Step_Plan(x, xref, u, xplan);
}
