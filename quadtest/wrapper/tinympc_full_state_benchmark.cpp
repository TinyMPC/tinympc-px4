#include "tinympc_interface.h"
#include "tinympc_full_state_model.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

namespace {

constexpr int kSteps = 500;

using PhysicalState = Matrix<tinytype,
    tinympc_full_state_model::kPhysicalStates, 1>;
using MotorVector = Matrix<tinytype,
    tinympc_full_state_model::kMotorCount, 1>;

struct Result {
    int scenario;
    const char* name;
    float maxX;
    float finalX;
    float maxAttitudeCoordinate;
    float maxBodyRate;
    float minimumMotor;
    float maximumMotor;
    float maximumMotor0;
    float maximumSlew;
    float maxPlannedStateViolation;
    float maxPlannedInputViolation;
    float maxPrimal;
    float maxDual;
    double solveP50Us;
    double solveP95Us;
    double solveP99Us;
    double solveMaxUs;
    int maxIterations;
    int converged;
    int bestEffort;
    int fallbacks;
};

void propagate(float x[12], const float motorCommand[4])
{
    const auto A = tinympc_full_state_model::makePhysicalA();
    const auto B = tinympc_full_state_model::makePhysicalB();
    PhysicalState state;
    MotorVector motorDelta;
    for (int i = 0; i < 12; ++i) {
        state(i) = x[i];
    }
    for (int i = 0; i < 4; ++i) {
        motorDelta(i) = motorCommand[i] -
            tinympc_full_state_model::kHoverCommand;
    }
    state = A * state + B * motorDelta;
    for (int i = 0; i < 12; ++i) {
        x[i] = static_cast<float>(state(i));
    }
}

Result runScenario(int scenario, const char* name)
{
    float x[12] = {0.0f};
    float motor[4] = {
        static_cast<float>(tinympc_full_state_model::kHoverCommand),
        static_cast<float>(tinympc_full_state_model::kHoverCommand),
        static_cast<float>(tinympc_full_state_model::kHoverCommand),
        static_cast<float>(tinympc_full_state_model::kHoverCommand)};
    float previousMotor[4] = {motor[0], motor[1], motor[2], motor[3]};
    float plan[12] = {0.0f};
    float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT] = {0.0f};

    MPC_FullState_Reset();
    Result result{};
    result.scenario = scenario;
    result.name = name;
    result.minimumMotor = 1.0f;
    std::array<double, kSteps> solveTimesUs{};

    for (int step = 0; step < kSteps; ++step) {
        const auto start = std::chrono::steady_clock::now();
        MPC_FullState_Step(x, scenario, motor, plan, diagnostics);
        const auto end = std::chrono::steady_clock::now();
        solveTimesUs[step] = std::chrono::duration<double, std::micro>(
            end - start).count();

        for (int i = 0; i < 4; ++i) {
            result.minimumMotor = std::min(result.minimumMotor, motor[i]);
            result.maximumMotor = std::max(result.maximumMotor, motor[i]);
            result.maximumSlew = std::max(result.maximumSlew,
                                          std::fabs(motor[i] - previousMotor[i]));
            previousMotor[i] = motor[i];
        }
        result.maximumMotor0 = std::max(result.maximumMotor0, motor[0]);

        propagate(x, motor);
        result.maxX = std::max(result.maxX, x[0]);
        result.maxAttitudeCoordinate = std::max(
            result.maxAttitudeCoordinate,
            std::max({std::fabs(x[3]), std::fabs(x[4]), std::fabs(x[5])}));
        result.maxBodyRate = std::max(
            result.maxBodyRate,
            std::max({std::fabs(x[9]), std::fabs(x[10]), std::fabs(x[11])}));
        result.maxPlannedStateViolation = std::max(
            result.maxPlannedStateViolation,
            diagnostics[TINY_MPC_DIAG_STATE_VIOLATION]);
        result.maxPlannedInputViolation = std::max(
            result.maxPlannedInputViolation,
            diagnostics[TINY_MPC_DIAG_INPUT_VIOLATION]);
        result.maxPrimal = std::max(result.maxPrimal,
                                    diagnostics[TINY_MPC_DIAG_PRIMAL_RESIDUAL]);
        result.maxDual = std::max(result.maxDual,
                                  diagnostics[TINY_MPC_DIAG_DUAL_RESIDUAL]);
        const int policy = static_cast<int>(diagnostics[TINY_MPC_DIAG_POLICY]);
        result.maxIterations = std::max(
            result.maxIterations,
            static_cast<int>(diagnostics[TINY_MPC_DIAG_ITERATIONS]));
        result.converged += policy == TINY_MPC_SOLVE_CONVERGED;
        result.bestEffort += policy == TINY_MPC_SOLVE_BEST_EFFORT;
    }

    std::sort(solveTimesUs.begin(), solveTimesUs.end());
    result.finalX = x[0];
    result.solveP50Us = solveTimesUs[kSteps * 50 / 100];
    result.solveP95Us = solveTimesUs[kSteps * 95 / 100];
    result.solveP99Us = solveTimesUs[kSteps * 99 / 100];
    result.solveMaxUs = solveTimesUs.back();
    result.fallbacks = static_cast<int>(
        diagnostics[TINY_MPC_DIAG_FALLBACK_COUNT]);
    return result;
}

bool invalidStateCheck()
{
    float x[12] = {0.0f};
    float motor[4] = {0.0f};
    float plan[12] = {0.0f};
    float diagnostics[TINY_MPC_DIAGNOSTIC_COUNT] = {0.0f};
    x[4] = std::numeric_limits<float>::quiet_NaN();

    MPC_FullState_Reset();
    MPC_FullState_Step(x, TINY_MPC_FULL_STATE_ACTUATOR_WALL,
                       motor, plan, diagnostics);
    if (static_cast<int>(diagnostics[TINY_MPC_DIAG_POLICY]) !=
        TINY_MPC_SOLVE_INVALID) {
        return false;
    }
    for (float value : motor) {
        if (!std::isfinite(value) || value < 0.05f || value > 0.60f) {
            return false;
        }
    }
    for (float value : plan) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

} // namespace

int main()
{
    MPC_FullState_Init();
    if (!invalidStateCheck()) {
        std::fputs("Full-state invalid-state fallback check failed.\n", stderr);
        return 1;
    }
    const Result results[] = {
        runScenario(TINY_MPC_FULL_STATE_HOVER,
                    "hover_integration_reference"),
        runScenario(TINY_MPC_FULL_STATE_ACTUATOR_WALL,
                    "predictive_envelope"),
        runScenario(TINY_MPC_FULL_STATE_REACTIVE_BASELINE,
                    "reactive_baseline"),
        runScenario(TINY_MPC_FULL_STATE_DEGRADED_ACTUATOR_WALL,
                    "degraded_motor_0")
    };

    std::puts("scenario,name,max_x,final_x,max_attitude_coordinate,max_body_rate,min_motor,max_motor,max_motor0,max_slew,max_planned_state_violation,max_planned_input_violation,max_primal,max_dual,solve_p50_us,solve_p95_us,solve_p99_us,solve_max_us,max_iterations,converged,best_effort,fallbacks");
    for (const Result& result : results) {
        std::printf("%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6g,%.6g,%.6g,%.6g,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
                    result.scenario, result.name, result.maxX, result.finalX,
                    result.maxAttitudeCoordinate, result.maxBodyRate,
                    result.minimumMotor, result.maximumMotor,
                    result.maximumMotor0, result.maximumSlew,
                    result.maxPlannedStateViolation,
                    result.maxPlannedInputViolation, result.maxPrimal,
                    result.maxDual, result.solveP50Us, result.solveP95Us,
                    result.solveP99Us, result.solveMaxUs,
                    result.maxIterations, result.converged,
                    result.bestEffort, result.fallbacks);
    }

    const Result& hover = results[0];
    const Result& envelope = results[1];
    const Result& baseline = results[2];
    const Result& degraded = results[3];
    bool ok = true;
    ok = ok && std::fabs(hover.maxX) <= 1.0e-6f;
    ok = ok && std::fabs(hover.finalX) <= 1.0e-6f;
    ok = ok && std::fabs(hover.minimumMotor -
                         static_cast<float>(tinympc_full_state_model::kHoverCommand)) <= 1.0e-6f;
    ok = ok && std::fabs(hover.maximumMotor -
                         static_cast<float>(tinympc_full_state_model::kHoverCommand)) <= 1.0e-6f;
    ok = ok && envelope.maxX <= 0.870f;
    ok = ok && baseline.maxX > envelope.maxX + 0.005f;
    ok = ok && envelope.maxAttitudeCoordinate <= 0.185f;
    ok = ok && envelope.maxBodyRate <= 3.01f;
    ok = ok && degraded.maximumMotor0 <= 0.34001f;
    for (const Result& result : results) {
        ok = ok && result.minimumMotor >= 0.04999f;
        ok = ok && result.maximumMotor <= 0.60001f;
        ok = ok && result.maximumSlew <= 0.04501f;
        ok = ok && result.maxPlannedStateViolation <= 1.0e-5f;
        ok = ok && result.maxPlannedInputViolation <= 1.0e-5f;
        ok = ok && result.maxIterations <= 250;
        ok = ok && result.fallbacks == 0;
    }

    if (!ok) {
        std::fputs("Full-state constraint benchmark assertions failed.\n", stderr);
        return 1;
    }

    std::puts("Full-state constraint benchmark assertions passed.");
    return 0;
}
