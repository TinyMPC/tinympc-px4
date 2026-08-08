#pragma once

#include <tinympc/types.hpp>

namespace tinympc_full_state_model {

constexpr int kPhysicalStates = 12;
constexpr int kMotorCount = 4;
constexpr tinytype kHoverCommand = 0.297485;

using PhysicalMatrix = Matrix<tinytype, kPhysicalStates, kPhysicalStates>;
using MotorMatrix = Matrix<tinytype, kPhysicalStates, kMotorCount>;

/* 50 Hz hover-linearized model supplied for this PX4/TinyMPC experiment.
 * State: [position(3), Rodrigues attitude(3), velocity(3), body rates(3)].
 * Input: normalized motor-command deviation from kHoverCommand. */
inline PhysicalMatrix makePhysicalA()
{
    PhysicalMatrix A;
    A <<
        1.0, 0.0, 0.0, 0.0, 0.003924, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0000131, 0.0,
        0.0, 1.0, 0.0, -0.003924, 0.0, 0.0, 0.0, 0.02, 0.0, -0.0000131, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        0.0, 0.0, 0.0, 0.0, 0.3924, 0.0, 1.0, 0.0, 0.0, 0.0, 0.001962, 0.0,
        0.0, 0.0, 0.0, -0.3924, 0.0, 0.0, 0.0, 1.0, 0.0, -0.001962, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    return A;
}

inline MotorMatrix makePhysicalB()
{
    MotorMatrix B;
    B <<
        -0.0000075,  0.0000075,  0.0000075, -0.0000075,
         0.0000081,  0.0000081, -0.0000081, -0.0000081,
         0.0016488,  0.0016488,  0.0016488,  0.0016488,
        -0.0123947, -0.0123947,  0.0123947,  0.0123947,
        -0.0115338,  0.0115338,  0.0115338, -0.0115338,
        -0.0006456,  0.0006456, -0.0006456,  0.0006456,
        -0.0015086,  0.0015086,  0.0015086, -0.0015086,
         0.0016212,  0.0016212, -0.0016212, -0.0016212,
         0.1648823,  0.1648823,  0.1648823,  0.1648823,
        -2.4789302, -2.4789302,  2.4789302,  2.4789302,
        -2.3067553,  2.3067553,  2.3067553, -2.3067553,
        -0.1291248,  0.1291248, -0.1291248,  0.1291248;
    return B;
}

} // namespace tinympc_full_state_model
