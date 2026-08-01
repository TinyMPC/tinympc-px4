#include "tinympc_interface.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {

constexpr float kDt = 0.02f;
constexpr int kSteps = 400;

struct Result {
    int scenario;
    const char* name;
    float maxX;
    float maxAbsY;
    float maxSpeed;
    float maxAccel;
    float maxPrimal;
    float maxDual;
    float maxPlannedStateViolation;
    float maxPlannedInputViolation;
    double solveP50Us;
    double solveP95Us;
    double solveP99Us;
    double solveMaxUs;
    int maxIterations;
    int converged;
    int bestEffort;
    int fallbacks;
};

void integrate(float x[12], const float u[4])
{
    x[0] += kDt * x[6] + 0.5f * kDt * kDt * u[0];
    x[1] += kDt * x[7] + 0.5f * kDt * kDt * u[1];
    x[2] += kDt * x[8] + 0.5f * kDt * kDt * u[2];
    x[6] += kDt * u[0];
    x[7] += kDt * u[1];
    x[8] += kDt * u[2];
    x[5] += kDt * u[3];
}

Result runScenario(int scenario, const char* name)
{
    float x[12] = {0.0f};
    float u[4] = {0.0f};
    float plan[12] = {0.0f};
    float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT] = {0.0f};

    MPC_Reset();
    Result result{scenario, name, x[0], 0.0f, 0.0f, 0.0f,
                  0.0f, 0.0f, 0.0f, 0.0f,
                  0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0};
    std::array<double, kSteps> solveTimesUs{};

    for (int step = 0; step < kSteps; ++step) {
        // A repeatable lateral impulse after the nominal trajectory settles.
        // It is applied to both the constrained and matched baseline cases.
        if (step == 220 &&
            (scenario == TINY_MPC_SCENARIO_VIRTUAL_WALL ||
             scenario == TINY_MPC_SCENARIO_WALL_UNCONSTRAINED)) {
            x[6] += 0.9f;
        }
        if (step == 220 && scenario == TINY_MPC_SCENARIO_CORRIDOR) {
            x[7] += 0.5f;
        }

        const auto solveStart = std::chrono::steady_clock::now();
        MPC_Step_Scenario(x, scenario, u, plan, diagnostics);
        const auto solveEnd = std::chrono::steady_clock::now();
        solveTimesUs[step] = std::chrono::duration<double, std::micro>(
            solveEnd - solveStart).count();
        integrate(x, u);

        result.maxX = std::max(result.maxX, x[0]);
        result.maxAbsY = std::max(result.maxAbsY, std::fabs(x[1]));
        result.maxSpeed = std::max(result.maxSpeed,
                                   std::sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8]));
        result.maxAccel = std::max(result.maxAccel,
                                   std::max({std::fabs(u[0]), std::fabs(u[1]), std::fabs(u[2])}));
        result.maxPrimal = std::max(result.maxPrimal,
                                    diagnostics[TINY_MPC_DIAG_PRIMAL_RESIDUAL]);
        result.maxDual = std::max(result.maxDual,
                                  diagnostics[TINY_MPC_DIAG_DUAL_RESIDUAL]);
        result.maxPlannedStateViolation = std::max(
            result.maxPlannedStateViolation,
            diagnostics[TINY_MPC_DIAG_STATE_VIOLATION]);
        result.maxPlannedInputViolation = std::max(
            result.maxPlannedInputViolation,
            diagnostics[TINY_MPC_DIAG_INPUT_VIOLATION]);
        result.maxIterations = std::max(result.maxIterations,
                                        static_cast<int>(diagnostics[TINY_MPC_DIAG_ITERATIONS]));
        const int policy = static_cast<int>(diagnostics[TINY_MPC_DIAG_POLICY]);
        result.converged += policy == TINY_MPC_SOLVE_CONVERGED;
        result.bestEffort += policy == TINY_MPC_SOLVE_BEST_EFFORT;
    }

    std::sort(solveTimesUs.begin(), solveTimesUs.end());
    result.solveP50Us = solveTimesUs[kSteps * 50 / 100];
    result.solveP95Us = solveTimesUs[kSteps * 95 / 100];
    result.solveP99Us = solveTimesUs[kSteps * 99 / 100];
    result.solveMaxUs = solveTimesUs.back();
    result.fallbacks = static_cast<int>(diagnostics[TINY_MPC_DIAG_FALLBACK_COUNT]);
    return result;
}

} // namespace

int main()
{
    MPC_Init();
    const Result results[] = {
        runScenario(TINY_MPC_SCENARIO_HOVER, "hover_box"),
        runScenario(TINY_MPC_SCENARIO_VIRTUAL_WALL, "virtual_wall"),
        runScenario(TINY_MPC_SCENARIO_WALL_UNCONSTRAINED, "wall_baseline"),
        runScenario(TINY_MPC_SCENARIO_CORRIDOR, "corridor"),
        runScenario(TINY_MPC_SCENARIO_REDUCED_AUTHORITY, "reduced_authority")
    };

    std::puts("scenario,name,max_x,max_abs_y,max_speed,max_accel,max_primal,max_dual,max_planned_state_violation,max_planned_input_violation,solve_p50_us,solve_p95_us,solve_p99_us,solve_max_us,max_iterations,converged,best_effort,fallbacks");
    for (const Result& result : results) {
        std::printf("%d,%s,%.6f,%.6f,%.6f,%.6f,%.6g,%.6g,%.6g,%.6g,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
                    result.scenario, result.name, result.maxX, result.maxAbsY,
                    result.maxSpeed, result.maxAccel, result.maxPrimal,
                    result.maxDual, result.maxPlannedStateViolation,
                    result.maxPlannedInputViolation, result.solveP50Us,
                    result.solveP95Us, result.solveP99Us, result.solveMaxUs,
                    result.maxIterations, result.converged,
                    result.bestEffort, result.fallbacks);
    }

    const Result& wall = results[1];
    const Result& baseline = results[2];
    const Result& corridor = results[3];
    const Result& reduced = results[4];
    bool ok = true;
    ok = ok && wall.maxX <= 1.01f;
    ok = ok && baseline.maxX > wall.maxX + 0.001f;
    ok = ok && corridor.maxAbsY <= 0.36f;
    ok = ok && reduced.maxAccel <= 1.501f;
    for (const Result& result : results) {
        ok = ok && result.fallbacks == 0;
        ok = ok && result.maxPlannedStateViolation <= 1.0e-5f;
        ok = ok && result.maxPlannedInputViolation <= 1.0e-5f;
    }

    if (!ok) {
        std::fputs("Constraint benchmark assertions failed.\n", stderr);
        return 1;
    }

    std::puts("Constraint benchmark assertions passed.");
    return 0;
}
