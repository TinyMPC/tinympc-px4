#include "tinympc_interface.h"

#include <cmath>
#include <cstdio>

int main()
{
    float x[12] = {0.0f};
    float xref[12] = {0.0f};
    float u[4] = {0.0f};
    float xplan[12] = {0.0f};

    xref[2] = -1.0f;

    MPC_Init();
    MPC_Step_Plan(x, xref, u, xplan);

    for (float value : u) {
        if (!std::isfinite(value)) {
            std::fprintf(stderr, "Smoke test failed: non-finite control output.\n");
            return 1;
        }
    }
    for (float value : xplan) {
        if (!std::isfinite(value)) {
            std::fprintf(stderr, "Smoke test failed: non-finite planned state.\n");
            return 1;
        }
    }

    if (std::fabs(u[0]) > 4.001f || std::fabs(u[1]) > 4.001f ||
        std::fabs(u[2]) > 4.001f || std::fabs(u[3]) > 1.001f) {
        std::fprintf(stderr, "Smoke test failed: control bounds were violated.\n");
        return 1;
    }

    if (u[2] >= 0.0f || xplan[2] >= 0.0f) {
        std::fprintf(stderr, "Smoke test failed: hover reference did not produce upward NED motion.\n");
        return 1;
    }

    std::printf("TinyMPC wrapper smoke test passed.\n");
    std::printf("u = [ax=%f, ay=%f, az=%f, yawspeed=%f], planned_z=%f\n",
                u[0], u[1], u[2], u[3], xplan[2]);
    return 0;
}
