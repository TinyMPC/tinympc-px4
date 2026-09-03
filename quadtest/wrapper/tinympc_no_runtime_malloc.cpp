#include "tinympc_interface.h"

#include <Eigen.h>
#ifdef TINY_MPC_USE_STATIC_CACHE
#include <tinympc/tiny_api.hpp>
#endif

#include <cmath>
#include <cstdio>

#ifdef TINY_MPC_USE_STATIC_CACHE
namespace {

bool static_cache_matches_model()
{
    constexpr int nx = 12;
    constexpr int nu = 4;
    constexpr tinytype dt = 0.02f;
    constexpr tinytype rho = 5.0f;

    tinyMatrix A = tinyMatrix::Identity(nx, nx);
    A(0, 6) = dt;
    A(1, 7) = dt;
    A(2, 8) = dt;
    tinyMatrix B = tinyMatrix::Zero(nx, nu);
    B(0, 0) = 0.5f * dt * dt;
    B(1, 1) = 0.5f * dt * dt;
    B(2, 2) = 0.5f * dt * dt;
    B(6, 0) = dt;
    B(7, 1) = dt;
    B(8, 2) = dt;
    B(5, 3) = dt;
    const tinyVector f = tinyVector::Zero(nx);
    tinyVector q(nx);
    q << 100.0f, 100.0f, 150.0f, 0.0f, 0.0f, 5.0f,
         15.0f, 15.0f, 20.0f, 0.0f, 0.0f, 0.0f;
    const tinyVector r = tinyVector::Ones(nu);
    tinyMatrix augmented_q = q.asDiagonal();
    tinyMatrix augmented_r = r.asDiagonal();
    augmented_q.diagonal().array() += rho;
    augmented_r.diagonal().array() += rho;

    TinyCache generated{};
    TinyCache embedded{};
    if (tiny_precompute_and_set_cache(&generated, A, B, f, augmented_q,
                                      augmented_r, nx, nu, rho, 0) != 0 ||
        tiny_load_static_cache(&embedded) != 0) {
        return false;
    }

    constexpr tinytype tolerance = 1.0e-3f;
    return (generated.Kinf - embedded.Kinf).cwiseAbs().maxCoeff() <= tolerance &&
           (generated.Pinf - embedded.Pinf).cwiseAbs().maxCoeff() <= tolerance &&
           (generated.Quu_inv - embedded.Quu_inv).cwiseAbs().maxCoeff() <= tolerance &&
           (generated.AmBKt - embedded.AmBKt).cwiseAbs().maxCoeff() <= tolerance;
}

} // namespace
#endif

int main()
{
    float state[12]{};
    float control[4]{};
    float plan[12]{};
    float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT]{};

#ifdef TINY_MPC_USE_STATIC_CACHE
    if (!static_cache_matches_model()) {
        std::fprintf(stderr, "Embedded cache does not match the controller model.\n");
        return 1;
    }
#endif

    // Solver construction is a pre-arm initialization operation. Once it is
    // complete, Eigen's runtime guard turns every subsequent heap allocation
    // into a test failure.
    MPC_Init();
    MPC_Reset();
    Eigen::internal::set_is_malloc_allowed(false);
    MPC_Reset();

    for (int step = 0; step < 500; ++step) {
        if (step == 100) {
            state[0] += 0.20f;
            state[6] += 0.15f;
        }

        MPC_Step_Scenario(state, TINY_MPC_SCENARIO_POSITION_HOLD,
                          control, plan, diagnostics);
        for (int axis = 0; axis < 3; ++axis) {
            state[axis] += 0.02f * state[axis + 6] +
                           0.5f * 0.02f * 0.02f * control[axis];
            state[axis + 6] += 0.02f * control[axis];
        }

        if (diagnostics[TINY_MPC_DIAG_POLICY] < TINY_MPC_SOLVE_CONVERGED ||
            !std::isfinite(control[0]) || !std::isfinite(control[1]) ||
            !std::isfinite(control[2]) || !std::isfinite(control[3])) {
            Eigen::internal::set_is_malloc_allowed(true);
            std::fprintf(stderr, "Runtime-allocation test received an invalid solve.\n");
            return 1;
        }
    }

    if (std::hypot(state[0], state[1]) > 0.02f || std::fabs(state[2]) > 0.02f) {
        Eigen::internal::set_is_malloc_allowed(true);
        std::fprintf(stderr, "Position-hold recovery did not converge.\n");
        return 1;
    }

#ifndef TINY_MPC_HARDWARE_HOLD_ONLY
    MPC_Reset();
    for (float &value : state) {
        value = 0.0f;
    }

    for (int step = 0; step < 300; ++step) {
        MPC_Step_Scenario(state, TINY_MPC_SCENARIO_CHICANE_SOC,
                          control, plan, diagnostics);
        for (int axis = 0; axis < 3; ++axis) {
            state[axis] += 0.02f * state[axis + 6] +
                           0.5f * 0.02f * 0.02f * control[axis];
            state[axis + 6] += 0.02f * control[axis];
        }

        if (diagnostics[TINY_MPC_DIAG_POLICY] < TINY_MPC_SOLVE_CONVERGED) {
            Eigen::internal::set_is_malloc_allowed(true);
            std::fprintf(stderr, "Chicane allocation test received an invalid solve.\n");
            return 1;
        }
    }
#endif

    Eigen::internal::set_is_malloc_allowed(true);
#ifdef TINY_MPC_HARDWARE_HOLD_ONLY
    std::puts("No Eigen heap allocation occurred in 500 embedded hold solves.");
#else
    std::puts("No Eigen heap allocation occurred in 500 hold and 300 chicane solves.");
#endif
    return 0;
}
