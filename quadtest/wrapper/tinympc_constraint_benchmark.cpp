#include "tinympc_interface.h"
#include "tinympc_chicane_course.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {

constexpr float kDt = 0.02f;
constexpr int kSteps = 700;
constexpr float kGravity = 9.80665f;
constexpr float kFigureEightConeSlope = 0.26794919f;

struct Result {
    int scenario;
    const char* name;
    float maxX;
    float minX;
    float maxAbsY;
    float maxSpeed;
    float maxAccel;
    float maxHorizontalAccel;
    float maxEquivalentTiltDeg;
    float maxFigureEightConeViolation;
    float maxChicaneCorridorViolation;
    float finalChicanePositionError;
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
    Result result{scenario, name, x[0], x[0], 0.0f, 0.0f, 0.0f, 0.0f,
                  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
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
        result.minX = std::min(result.minX, x[0]);
        result.maxAbsY = std::max(result.maxAbsY, std::fabs(x[1]));
        result.maxSpeed = std::max(result.maxSpeed,
                                   std::sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8]));
        result.maxAccel = std::max(result.maxAccel,
                                   std::max({std::fabs(u[0]), std::fabs(u[1]), std::fabs(u[2])}));
        const float horizontalAccel = std::sqrt(u[0] * u[0] + u[1] * u[1]);
        result.maxHorizontalAccel = std::max(result.maxHorizontalAccel,
                                             horizontalAccel);
        const float verticalSpecificThrust = kGravity - u[2];
        result.maxEquivalentTiltDeg = std::max(
            result.maxEquivalentTiltDeg,
            std::atan2(horizontalAccel, std::max(0.0f, verticalSpecificThrust)) *
                180.0f / 3.14159265358979323846f);
        result.maxFigureEightConeViolation = std::max(
            result.maxFigureEightConeViolation,
            std::max(0.0f,
                     horizontalAccel -
                     kFigureEightConeSlope * verticalSpecificThrust));
        if (scenario == TINY_MPC_SCENARIO_CHICANE_SOC) {
            result.maxChicaneCorridorViolation = std::max(
                result.maxChicaneCorridorViolation,
                static_cast<float>(tinympc_chicane::corridorViolation(x[0], x[1])));
        }
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
    if (scenario == TINY_MPC_SCENARIO_CHICANE_SOC) {
        result.finalChicanePositionError = std::hypot(
            x[0] - static_cast<float>(tinympc_chicane::kFinishX),
            x[1] - static_cast<float>(tinympc_chicane::kSecondCornerY));
    }
    return result;
}

Result runPx4CascadedPidChicane()
{
    // Best zero-departure PX4 gain set retained from the SITL tuning sweep.
    constexpr float kPositionP = 0.21f;
    constexpr float kVelocityP = 5.0f;
    constexpr float kVelocityI = 0.17f;
    constexpr float kVelocityD = 0.13f;
    constexpr float kHorizontalVelocityLimit = 12.0f;
    constexpr float kHorizontalAccelerationLimit =
        kFigureEightConeSlope * kGravity;

    float x[12] = {0.0f};
    float u[4] = {0.0f};
    float velocityIntegral[2] = {0.0f, 0.0f};
    float measuredAcceleration[2] = {0.0f, 0.0f};
    Result result{-1, "tuned_px4_cascaded_pid_chicane", x[0], x[0], 0.0f,
                  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                  0.0f, 0.0f, 0.0f, 0.0f, 0.0, 0.0, 0.0, 0.0,
                  0, 0, 0, 0};

    for (int step = 0; step < kSteps; ++step) {
        const tinympc_chicane::Sample reference =
            tinympc_chicane::sample(step * kDt);
        float velocitySetpoint[2] = {
            static_cast<float>(reference.vx) +
                kPositionP * (static_cast<float>(reference.x) - x[0]),
            static_cast<float>(reference.vy) +
                kPositionP * (static_cast<float>(reference.y) - x[1])
        };
        const float velocitySetpointNorm = std::hypot(
            velocitySetpoint[0], velocitySetpoint[1]);
        if (velocitySetpointNorm > kHorizontalVelocityLimit) {
            const float scale = kHorizontalVelocityLimit / velocitySetpointNorm;
            velocitySetpoint[0] *= scale;
            velocitySetpoint[1] *= scale;
        }

        float velocityError[2] = {
            velocitySetpoint[0] - x[6],
            velocitySetpoint[1] - x[7]
        };
        float requestedAcceleration[2] = {
            kVelocityP * velocityError[0] + velocityIntegral[0] -
                kVelocityD * measuredAcceleration[0],
            kVelocityP * velocityError[1] + velocityIntegral[1] -
                kVelocityD * measuredAcceleration[1]
        };
        u[0] = requestedAcceleration[0];
        u[1] = requestedAcceleration[1];
        const float requestedNorm = std::hypot(u[0], u[1]);
        if (requestedNorm > kHorizontalAccelerationLimit) {
            const float scale = kHorizontalAccelerationLimit / requestedNorm;
            u[0] *= scale;
            u[1] *= scale;
        }

        // PX4-style tracking anti-windup: unwind the velocity integrator in
        // the direction removed by the downstream tilt/thrust saturation.
        const float antiResetWindupGain = 2.0f / kVelocityP;
        velocityError[0] -= antiResetWindupGain *
            (requestedAcceleration[0] - u[0]);
        velocityError[1] -= antiResetWindupGain *
            (requestedAcceleration[1] - u[1]);
        velocityIntegral[0] += kVelocityI * velocityError[0] * kDt;
        velocityIntegral[1] += kVelocityI * velocityError[1] * kDt;
        measuredAcceleration[0] = u[0];
        measuredAcceleration[1] = u[1];
        integrate(x, u);

        result.maxX = std::max(result.maxX, x[0]);
        result.minX = std::min(result.minX, x[0]);
        result.maxAbsY = std::max(result.maxAbsY, std::fabs(x[1]));
        result.maxSpeed = std::max(result.maxSpeed, std::hypot(x[6], x[7]));
        const float horizontalAcceleration = std::hypot(u[0], u[1]);
        result.maxAccel = std::max(result.maxAccel, horizontalAcceleration);
        result.maxHorizontalAccel = std::max(
            result.maxHorizontalAccel, horizontalAcceleration);
        result.maxEquivalentTiltDeg = std::max(
            result.maxEquivalentTiltDeg,
            std::atan2(horizontalAcceleration, kGravity) *
                180.0f / 3.14159265358979323846f);
        result.maxFigureEightConeViolation = std::max(
            result.maxFigureEightConeViolation,
            std::max(0.0f, horizontalAcceleration -
                     kFigureEightConeSlope * kGravity));
        result.maxChicaneCorridorViolation = std::max(
            result.maxChicaneCorridorViolation,
            static_cast<float>(tinympc_chicane::corridorViolation(x[0], x[1])));
    }

    result.finalChicanePositionError = std::hypot(
        x[0] - static_cast<float>(tinympc_chicane::kFinishX),
        x[1] - static_cast<float>(tinympc_chicane::kSecondCornerY));
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
        runScenario(TINY_MPC_SCENARIO_REDUCED_AUTHORITY, "reduced_authority"),
        runScenario(TINY_MPC_SCENARIO_FIGURE_EIGHT_SOC, "figure_eight_soc"),
        runScenario(TINY_MPC_SCENARIO_FIGURE_EIGHT_BOX, "figure_eight_box"),
        runScenario(TINY_MPC_SCENARIO_CHICANE_SOC, "chicane_soc"),
        runPx4CascadedPidChicane()
    };

    std::puts("scenario,name,max_x,min_x,max_abs_y,max_speed,max_accel,max_horizontal_accel,max_equivalent_tilt_deg,max_figure_eight_cone_violation,max_chicane_corridor_violation,final_chicane_position_error,max_primal,max_dual,max_planned_state_violation,max_planned_input_violation,solve_p50_us,solve_p95_us,solve_p99_us,solve_max_us,max_iterations,converged,best_effort,fallbacks");
    for (const Result& result : results) {
        std::printf("%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6g,%.6g,%.6g,%.6g,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
                    result.scenario, result.name, result.maxX, result.minX,
                    result.maxAbsY,
                    result.maxSpeed, result.maxAccel,
                    result.maxHorizontalAccel,
                    result.maxEquivalentTiltDeg,
                    result.maxFigureEightConeViolation,
                    result.maxChicaneCorridorViolation,
                    result.finalChicanePositionError,
                    result.maxPrimal,
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
    const Result& figureEightSoc = results[5];
    const Result& figureEightBox = results[6];
    const Result& chicaneSoc = results[7];
    const Result& cascadedPid = results[8];
    bool ok = true;
    ok = ok && wall.maxX <= 1.01f;
    ok = ok && baseline.maxX > wall.maxX + 0.001f;
    ok = ok && corridor.maxAbsY <= 0.36f;
    ok = ok && reduced.maxAccel <= 1.501f;
    ok = ok && figureEightSoc.maxX > 1.3f;
    ok = ok && figureEightSoc.minX < -1.3f;
    ok = ok && figureEightSoc.maxAbsY > 0.5f;
    ok = ok && figureEightSoc.maxEquivalentTiltDeg <= 15.001f;
    ok = ok && figureEightSoc.maxFigureEightConeViolation <= 1.0e-4f;
    ok = ok && figureEightBox.maxEquivalentTiltDeg > 15.5f;
    ok = ok && figureEightBox.maxFigureEightConeViolation > 0.05f;
    ok = ok && chicaneSoc.maxEquivalentTiltDeg <= 15.001f;
    ok = ok && cascadedPid.maxEquivalentTiltDeg <= 15.001f;
    ok = ok && chicaneSoc.maxChicaneCorridorViolation <= 1.0e-5f;
    ok = ok && cascadedPid.maxChicaneCorridorViolation <= 1.0e-5f;
    ok = ok && chicaneSoc.finalChicanePositionError < 0.03f;
    ok = ok && cascadedPid.finalChicanePositionError < 0.03f;
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
