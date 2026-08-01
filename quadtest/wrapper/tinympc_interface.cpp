#include "tinympc_interface.h"

#include <cstdio>

#include <tinympc/admm.hpp>
#include <tinympc/tiny_api.hpp>

namespace {

constexpr int kNumStates = 12;
constexpr int kNumInputs = 4;
constexpr int kHorizon = 25;
constexpr tinytype kSampleTime = 0.02;
constexpr tinytype kRho = 5.0;
constexpr tinytype kAccelLimit = 4.0;
constexpr tinytype kYawRateLimit = 1.0;
constexpr tinytype kStateBound = 1.0e6;

TinySolver* g_solver = nullptr;

typedef Matrix<tinytype, kNumStates, 1> StateVector;
typedef Matrix<tinytype, kNumInputs, 1> InputVector;
typedef Matrix<tinytype, kNumStates, kNumStates> StateMatrix;
typedef Matrix<tinytype, kNumStates, kNumInputs> InputMatrix;
typedef Matrix<tinytype, kNumStates, kHorizon> StateTrajectory;
typedef Matrix<tinytype, kNumInputs, kHorizon - 1> InputTrajectory;

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

InputVector makeInputWeights()
{
    return InputVector::Ones();
}

StateVector copyState(const float values[kNumStates])
{
    StateVector out;
    for (int i = 0; i < kNumStates; ++i) {
        out(i) = static_cast<tinytype>(values[i]);
    }
    return out;
}

void zeroOutput(float u[kNumInputs])
{
    for (int i = 0; i < kNumInputs; ++i) {
        u[i] = 0.0f;
    }
}

InputTrajectory makeInputLowerBounds()
{
    InputVector lower;
    lower << -kAccelLimit, -kAccelLimit, -kAccelLimit, -kYawRateLimit;
    return lower.replicate<1, kHorizon - 1>();
}

InputTrajectory makeInputUpperBounds()
{
    InputVector upper;
    upper << kAccelLimit, kAccelLimit, kAccelLimit, kYawRateLimit;
    return upper.replicate<1, kHorizon - 1>();
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
    const InputVector R = makeInputWeights();

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

    tiny_set_bound_constraints(g_solver,
                               StateTrajectory::Constant(-kStateBound),
                               StateTrajectory::Constant(kStateBound),
                               makeInputLowerBounds(),
                               makeInputUpperBounds());
    tiny_set_x_ref(g_solver, StateTrajectory::Zero());
    tiny_set_u_ref(g_solver, InputTrajectory::Zero());
}

void MPC_Step_Plan(const float x[12],
                   const float xref[12],
                   float u[4],
                   float xnext[12])
{
    if (g_solver == nullptr) {
        MPC_Init();
    }

    if (g_solver == nullptr) {
        zeroOutput(u);
        for (int i = 0; i < kNumStates; ++i) {
            xnext[i] = x[i];
        }
        return;
    }

    const StateVector x0 = copyState(x);
    const StateVector xr = copyState(xref);
    const StateTrajectory xRefHorizon = xr.replicate<1, kHorizon>();

    int status = 0;
    status |= tiny_set_x0(g_solver, x0);
    status |= tiny_set_x_ref(g_solver, xRefHorizon);
    if (status != 0) {
        zeroOutput(u);
        for (int i = 0; i < kNumStates; ++i) {
            xnext[i] = x[i];
        }
        return;
    }

    update_linear_cost(g_solver);
    tiny_solve(g_solver);

    for (int i = 0; i < kNumInputs; ++i) {
        u[i] = static_cast<float>(g_solver->solution->u(i, 0));
    }
    // Publish a plan point ~200 ms ahead rather than one step (20 ms):
    // a one-step lead collapses onto the current state, so the tracking
    // loops get no drive and convergence crawls on feed-forward alone.
    constexpr int kPlanLookaheadSteps = 10;
    for (int i = 0; i < kNumStates; ++i) {
        xnext[i] = static_cast<float>(g_solver->solution->x(i, kPlanLookaheadSteps));
    }
}

void MPC_Step(const float x[12],
              const float xref[12],
              float u[4])
{
    float xnext[kNumStates];
    MPC_Step_Plan(x, xref, u, xnext);
}
