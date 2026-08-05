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
constexpr tinytype kGravity = 9.80665;
constexpr tinytype kFigureEightConeSlope = 0.2679491924311227; // tan(15 deg)
constexpr tinytype kFigureEightPeriod = 6.0;
constexpr tinytype kFigureEightOmega = 1.0471975511965976; // 2*pi/6 s
constexpr tinytype kFigureEightBlendDuration = 2.0;
constexpr tinytype kFigureEightXAmplitude = 1.5;
constexpr tinytype kFigureEightYAmplitude = 0.70;
constexpr tinytype kFigureEightZAmplitude = 0.15;
constexpr tinytype kFigureEightHeight = 0.60;
constexpr tinytype kFigureEightTakeoffRate = 0.40;
constexpr int kFigureEightAltitudeHoldSteps = 10;
constexpr tinytype kUnbounded = 1.0e6;
constexpr tinytype kBoundTolerance = 1.0e-5;

TinySolver* g_solver = nullptr;
TinySolver* g_figure_solver = nullptr;
bool g_has_origin = false;
int g_active_scenario = -1;
unsigned long g_solve_count = 0;
unsigned long g_fallback_count = 0;
tinytype g_figure_time = 0.0;
tinytype g_figure_takeoff_time = 0.0;
int g_figure_altitude_hold_steps = 0;

typedef Matrix<tinytype, kNumStates, 1> StateVector;
typedef Matrix<tinytype, kNumInputs, 1> InputVector;
typedef Matrix<tinytype, kNumStates, kNumStates> StateMatrix;
typedef Matrix<tinytype, kNumStates, kNumInputs> InputMatrix;
typedef Matrix<tinytype, kNumStates, kHorizon> StateTrajectory;
typedef Matrix<tinytype, kNumInputs, kHorizon - 1> InputTrajectory;

StateVector g_origin = StateVector::Zero();
StateVector g_last_plan = StateVector::Zero();
InputVector g_last_input = InputVector::Zero();
StateVector g_figure_last_plan = StateVector::Zero();
InputVector g_figure_last_input = InputVector::Zero();

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

InputMatrix makeSpecificThrustBdyn()
{
    InputMatrix B = InputMatrix::Zero();
    B(0, 0) = 0.5 * kSampleTime * kSampleTime;
    B(1, 1) = 0.5 * kSampleTime * kSampleTime;
    B(2, 2) = -0.5 * kSampleTime * kSampleTime;
    B(6, 0) = kSampleTime;
    B(7, 1) = kSampleTime;
    B(8, 2) = -kSampleTime;
    B(5, 3) = kSampleTime;
    return B;
}

StateVector makeSpecificThrustFdyn()
{
    StateVector f = StateVector::Zero();
    f(2) = 0.5 * kGravity * kSampleTime * kSampleTime;
    f(8) = kGravity * kSampleTime;
    return f;
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

void copySpecificThrustOutput(const InputVector& input,
                              const StateVector& plan,
                              float u[kNumInputs],
                              float xplan[kNumStates])
{
    u[0] = static_cast<float>(input(0));
    u[1] = static_cast<float>(input(1));
    u[2] = static_cast<float>(kGravity - input(2));
    u[3] = static_cast<float>(input(3));
    for (int i = 0; i < kNumStates; ++i) {
        xplan[i] = static_cast<float>(plan(i));
    }
}

void resetWarmStart(TinySolver* solver)
{
    if (solver == nullptr) {
        return;
    }

    TinyWorkspace* work = solver->work;
    work->x.setZero();
    work->u.setZero();
    work->v.setZero();
    work->vnew.setZero();
    work->z.setZero();
    work->znew.setZero();
    work->g.setZero();
    work->y.setZero();
    work->vc.setZero();
    work->vcnew.setZero();
    work->zc.setZero();
    work->zcnew.setZero();
    work->gc.setZero();
    work->yc.setZero();
    work->q.setZero();
    work->r.setZero();
    solver->solution->x.setZero();
    solver->solution->u.setZero();
    solver->solution->iter = 0;
    solver->solution->solved = 0;
}

int sanitizeScenario(int scenario)
{
    if (scenario < TINY_MPC_SCENARIO_HOVER ||
        scenario > TINY_MPC_SCENARIO_FIGURE_EIGHT_BOX) {
        return TINY_MPC_SCENARIO_HOVER;
    }
    return scenario;
}

bool isFigureEightScenario(int scenario)
{
    return scenario == TINY_MPC_SCENARIO_FIGURE_EIGHT_SOC ||
           scenario == TINY_MPC_SCENARIO_FIGURE_EIGHT_BOX;
}

tinytype clampValue(tinytype value, tinytype lower, tinytype upper)
{
    return std::max(lower, std::min(value, upper));
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

StateVector figureEightReference(tinytype time)
{
    const tinytype phase = kFigureEightOmega * time;
    const tinytype doublePhase = 2.0 * phase;
    StateVector hold = g_origin;
    hold(2) -= kFigureEightHeight;
    hold.segment<6>(6).setZero();

    StateVector raw = hold;
    raw(0) += kFigureEightXAmplitude * std::sin(phase);
    raw(1) += kFigureEightYAmplitude * std::sin(doublePhase);
    raw(2) -= 0.5 * kFigureEightZAmplitude * (1.0 - std::cos(doublePhase));
    raw(6) = kFigureEightXAmplitude * kFigureEightOmega * std::cos(phase);
    raw(7) = 2.0 * kFigureEightYAmplitude * kFigureEightOmega *
             std::cos(doublePhase);
    raw(8) = -kFigureEightZAmplitude * kFigureEightOmega *
             std::sin(doublePhase);

    const tinytype blendPhase = std::min<tinytype>(
        1.0, std::max<tinytype>(0.0, time / kFigureEightBlendDuration));
    const tinytype blend = blendPhase * blendPhase * (3.0 - 2.0 * blendPhase);
    const tinytype blendRate = blendPhase >= 1.0 ? 0.0 :
        6.0 * blendPhase * (1.0 - blendPhase) / kFigureEightBlendDuration;

    StateVector reference = hold;
    reference.head<6>() = hold.head<6>() +
        blend * (raw.head<6>() - hold.head<6>());
    reference.segment<6>(6) = blend * raw.segment<6>(6);
    reference.segment<3>(6) +=
        blendRate * (raw.head<3>() - hold.head<3>());
    return reference;
}

void configureFigureEight(const StateVector& x0,
                          StateTrajectory& reference,
                          StateTrajectory& stateLower,
                          StateTrajectory& stateUpper,
                          InputTrajectory& inputLower,
                          InputTrajectory& inputUpper)
{
    const bool altitudeReached =
        x0(2) <= g_origin(2) - (kFigureEightHeight - 0.10);
    const bool takeoffWindowComplete = g_figure_takeoff_time >= 4.0;
    if ((altitudeReached || takeoffWindowComplete) &&
        g_figure_altitude_hold_steps < kFigureEightAltitudeHoldSteps) {
        ++g_figure_altitude_hold_steps;
    }
    const bool pathActive = g_figure_altitude_hold_steps >= kFigureEightAltitudeHoldSteps;

    for (int k = 0; k < kHorizon; ++k) {
        if (pathActive) {
            reference.col(k) = figureEightReference(
                g_figure_time + static_cast<tinytype>(k) * kSampleTime);
        } else {
            reference.col(k) = g_origin;
            // Use an engagement-relative absolute climb ramp. A target based
            // on the current altitude becomes a receding reference when the
            // real PX4 vertical loop lags the ideal double-integrator model.
            const tinytype futureTakeoffTime = g_figure_takeoff_time +
                static_cast<tinytype>(k) * kSampleTime;
            const tinytype takeoffHeight = std::min(
                kFigureEightHeight,
                kFigureEightTakeoffRate * futureTakeoffTime);
            reference(2, k) = g_origin(2) - takeoffHeight;
            reference.col(k).segment<6>(6).setZero();
            if (takeoffHeight < kFigureEightHeight) {
                reference(8, k) = -kFigureEightTakeoffRate;
            }
        }
    }

    if (pathActive) {
        g_figure_time += kSampleTime;
    } else {
        g_figure_takeoff_time += kSampleTime;
    }

    stateLower.setConstant(-kUnbounded);
    stateUpper.setConstant(kUnbounded);
    setStateBox(stateLower, stateUpper, 0,
                g_origin(0) - 1.8, g_origin(0) + 1.8);
    setStateBox(stateLower, stateUpper, 1,
                g_origin(1) - 1.0, g_origin(1) + 1.0);
    setStateBox(stateLower, stateUpper, 2,
                g_origin(2) - 1.5, g_origin(2) + 0.25);
    setStateBox(stateLower, stateUpper, 6, -2.2, 2.2);
    setStateBox(stateLower, stateUpper, 7, -2.2, 2.2);
    setStateBox(stateLower, stateUpper, 8, -1.0, 1.0);

    InputVector minimum;
    InputVector maximum;
    minimum << -4.0, -4.0, 0.0, -kDefaultYawRateLimit;
    maximum << 4.0, 4.0, kGravity + 3.0, kDefaultYawRateLimit;
    inputLower = minimum.replicate<1, kHorizon - 1>();
    inputUpper = maximum.replicate<1, kHorizon - 1>();
}

tinytype maximumInputConeViolation(const tinyMatrix& input)
{
    tinytype maximum = 0.0;
    for (int k = 0; k < input.cols(); ++k) {
        const tinytype lateral = input.block(0, k, 2, 1).norm();
        maximum = std::max(maximum,
                           lateral - kFigureEightConeSlope * input(2, k));
    }
    return std::max<tinytype>(0.0, maximum);
}

InputVector figureEightFallback(const StateVector& state)
{
    InputVector fallback = InputVector::Zero();
    const tinytype targetZ = g_origin(2) - kFigureEightHeight;
    fallback(0) = clampValue(
        -1.0 * (state(0) - g_origin(0)) - 1.5 * state(6), -1.5, 1.5);
    fallback(1) = clampValue(
        -1.0 * (state(1) - g_origin(1)) - 1.5 * state(7), -1.5, 1.5);
    const tinytype verticalAcceleration = clampValue(
        -1.0 * (state(2) - targetZ) - 1.5 * state(8), -1.5, 1.5);
    fallback(2) = kGravity - verticalAcceleration;

    const tinytype lateralNorm = fallback.head<2>().norm();
    const tinytype lateralLimit = kFigureEightConeSlope * fallback(2);
    if (lateralNorm > lateralLimit && lateralNorm > 0.0) {
        fallback.head<2>() *= lateralLimit / lateralNorm;
    }
    return fallback;
}

tinytype maximumViolation(const tinyMatrix& values,
                          const tinyMatrix& lower,
                          const tinyMatrix& upper)
{
    const tinytype lowerViolation = (lower - values).maxCoeff();
    const tinytype upperViolation = (values - upper).maxCoeff();
    return std::max<tinytype>(0.0, std::max(lowerViolation, upperViolation));
}

void fillDiagnostics(TinySolver* solver,
                     float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT],
                     int policy,
                     tinytype stateViolation,
                     tinytype inputViolation)
{
    if (diagnostics == nullptr) {
        return;
    }

    const tinytype primal = solver == nullptr ? kUnbounded :
        std::max(solver->work->primal_residual_state,
                 solver->work->primal_residual_input);
    const tinytype dual = solver == nullptr ? kUnbounded :
        std::max(solver->work->dual_residual_state,
                 solver->work->dual_residual_input);
    const int iterations = solver == nullptr ? 0 : solver->solution->iter;

    diagnostics[TINY_MPC_DIAG_POLICY] = static_cast<float>(policy);
    diagnostics[TINY_MPC_DIAG_ITERATIONS] = static_cast<float>(iterations);
    diagnostics[TINY_MPC_DIAG_PRIMAL_RESIDUAL] = static_cast<float>(primal);
    diagnostics[TINY_MPC_DIAG_DUAL_RESIDUAL] = static_cast<float>(dual);
    diagnostics[TINY_MPC_DIAG_STATE_VIOLATION] = static_cast<float>(stateViolation);
    diagnostics[TINY_MPC_DIAG_INPUT_VIOLATION] = static_cast<float>(inputViolation);
    diagnostics[TINY_MPC_DIAG_FALLBACK_COUNT] = static_cast<float>(g_fallback_count);
    diagnostics[TINY_MPC_DIAG_SOLVE_COUNT] = static_cast<float>(g_solve_count);
}

bool solveTrajectory(TinySolver* solver,
                     const StateVector& x0,
                     const StateTrajectory& referenceHorizon,
                     const StateTrajectory& stateLower,
                     const StateTrajectory& stateUpper,
                     const InputTrajectory& inputLower,
                     const InputTrajectory& inputUpper,
                     bool enforceInputCone,
                     InputVector& input,
                     StateVector& plan,
                     int& solveStatus,
                     tinytype& stateViolation,
                     tinytype& inputViolation)
{
    int apiStatus = 0;
    apiStatus |= tiny_set_x0(solver, x0);
    apiStatus |= tiny_set_x_ref(solver, referenceHorizon);
    apiStatus |= tiny_set_bound_constraints(solver,
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

    solver->settings->en_input_bound = enforceInputCone ? 0 : 1;
    solver->settings->en_input_soc = enforceInputCone ? 1 : 0;
    update_linear_cost(solver);
    solveStatus = tiny_solve(solver);
    ++g_solve_count;

    const tinyMatrix& feasibleInput = solver->solution->u;
    input = feasibleInput.col(0);
    plan = solver->solution->x.col(kPlanLookaheadSteps);
    stateViolation = maximumViolation(solver->solution->x, stateLower, stateUpper);
    inputViolation = maximumViolation(feasibleInput, inputLower, inputUpper);
    if (enforceInputCone) {
        inputViolation = std::max(inputViolation,
                                  maximumInputConeViolation(feasibleInput));
    }
    return stateIsFinite(plan) && input.array().isFinite().all();
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
    return solveTrajectory(g_solver, x0, referenceHorizon,
                           stateLower, stateUpper, inputLower, inputUpper,
                           false, input, plan, solveStatus,
                           stateViolation, inputViolation);
}

void initFigureEightSolver()
{
    if (g_figure_solver != nullptr) {
        return;
    }

    const StateMatrix Adyn = makeAdyn();
    const InputMatrix Bdyn = makeSpecificThrustBdyn();
    const StateVector fdyn = makeSpecificThrustFdyn();
    const StateVector Q = makeStateWeights();
    InputVector R;
    R << 1.0, 1.0, 0.5, 1.0;

    const int setupStatus = tiny_setup(&g_figure_solver,
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
    if (setupStatus != 0 || g_figure_solver == nullptr) {
        std::fprintf(stderr,
                     "TinyMPC figure-eight setup failed with status %d\n",
                     setupStatus);
        g_figure_solver = nullptr;
        return;
    }

    VectorXi stateConeStarts(0);
    VectorXi stateConeSizes(0);
    tinyVector stateConeSlopes(0);
    VectorXi inputConeStarts(1);
    VectorXi inputConeSizes(1);
    tinyVector inputConeSlopes(1);
    inputConeStarts << 0;
    inputConeSizes << 3;
    inputConeSlopes << kFigureEightConeSlope;
    const int coneStatus = tiny_set_cone_constraints(
        g_figure_solver,
        stateConeStarts,
        stateConeSizes,
        stateConeSlopes,
        inputConeStarts,
        inputConeSizes,
        inputConeSlopes);
    if (coneStatus != 0) {
        std::fprintf(stderr,
                     "TinyMPC figure-eight cone setup failed with status %d\n",
                     coneStatus);
        g_figure_solver = nullptr;
        return;
    }

    g_figure_solver->settings->max_iter = 100;
    g_figure_solver->settings->check_termination = 1;
    g_figure_solver->settings->en_input_soc = 1;
    tiny_set_x_ref(g_figure_solver, StateTrajectory::Zero());
    InputTrajectory inputReference = InputTrajectory::Zero();
    inputReference.row(2).setConstant(kGravity);
    tiny_set_u_ref(g_figure_solver, inputReference);
    g_figure_last_input << 0.0, 0.0, kGravity, 0.0;
}

} // namespace

void MPC_Init(void)
{
    if (g_solver != nullptr) {
        initFigureEightSolver();
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
    initFigureEightSolver();
}

void MPC_Reset(void)
{
    g_has_origin = false;
    g_active_scenario = -1;
    g_solve_count = 0;
    g_fallback_count = 0;
    g_figure_time = 0.0;
    g_figure_takeoff_time = 0.0;
    g_figure_altitude_hold_steps = 0;
    g_origin.setZero();
    g_last_plan.setZero();
    g_last_input.setZero();
    g_figure_last_plan.setZero();
    g_figure_last_input << 0.0, 0.0, kGravity, 0.0;
    resetWarmStart(g_solver);
    resetWarmStart(g_figure_solver);
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
    scenario = sanitizeScenario(scenario);
    TinySolver* selectedSolver = isFigureEightScenario(scenario)
        ? g_figure_solver : g_solver;
    if (selectedSolver == nullptr || !stateIsFinite(x0)) {
        InputVector safeInput = InputVector::Zero();
        if (isFigureEightScenario(scenario)) {
            safeInput(2) = kGravity;
        }
        const StateVector safePlan = stateIsFinite(x0) ? x0 : StateVector::Zero();
        if (isFigureEightScenario(scenario)) {
            copySpecificThrustOutput(safeInput, safePlan, u, xplan);
        } else {
            copyOutput(safeInput, safePlan, u, xplan);
        }
        fillDiagnostics(selectedSolver, diagnostics, TINY_MPC_SOLVE_INVALID,
                        kUnbounded, kUnbounded);
        return;
    }

    if (scenario != g_active_scenario) {
        g_has_origin = false;
        g_active_scenario = scenario;
        g_figure_time = 0.0;
        g_figure_takeoff_time = 0.0;
        g_figure_altitude_hold_steps = 0;
        resetWarmStart(g_solver);
        resetWarmStart(g_figure_solver);
    }

    if (!g_has_origin) {
        g_origin = x0;
        g_last_plan = x0;
        g_last_input.setZero();
        g_figure_last_plan = x0;
        g_figure_last_input << 0.0, 0.0, kGravity, 0.0;
        g_has_origin = true;
    }

    if (isFigureEightScenario(scenario)) {
        StateTrajectory referenceHorizon;
        StateTrajectory stateLower;
        StateTrajectory stateUpper;
        InputTrajectory inputLower;
        InputTrajectory inputUpper;
        configureFigureEight(x0, referenceHorizon,
                             stateLower, stateUpper,
                             inputLower, inputUpper);

        InputVector candidateInput;
        StateVector candidatePlan;
        int solveStatus = 1;
        tinytype stateViolation = kUnbounded;
        tinytype inputViolation = kUnbounded;
        const bool enforceCone =
            scenario == TINY_MPC_SCENARIO_FIGURE_EIGHT_SOC;
        const bool finite = solveTrajectory(g_figure_solver,
                                            x0,
                                            referenceHorizon,
                                            stateLower,
                                            stateUpper,
                                            inputLower,
                                            inputUpper,
                                            enforceCone,
                                            candidateInput,
                                            candidatePlan,
                                            solveStatus,
                                            stateViolation,
                                            inputViolation);
        const bool constraintsSatisfied =
            stateViolation <= kBoundTolerance &&
            inputViolation <= kBoundTolerance;
        int policy = TINY_MPC_SOLVE_FALLBACK;
        if (finite && constraintsSatisfied && solveStatus == 0) {
            policy = TINY_MPC_SOLVE_CONVERGED;
        } else if (finite && constraintsSatisfied) {
            policy = TINY_MPC_SOLVE_BEST_EFFORT;
        }

        if (policy == TINY_MPC_SOLVE_CONVERGED ||
            policy == TINY_MPC_SOLVE_BEST_EFFORT) {
            g_figure_last_input = candidateInput;
            g_figure_last_plan = candidatePlan;
        } else {
            ++g_fallback_count;
            g_figure_last_input = figureEightFallback(x0);
        }

        copySpecificThrustOutput(g_figure_last_input,
                                 g_figure_last_plan,
                                 u,
                                 xplan);
        fillDiagnostics(g_figure_solver, diagnostics, policy,
                        stateViolation, inputViolation);
        return;
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
    fillDiagnostics(g_solver, diagnostics, policy,
                    stateViolation, inputViolation);
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
