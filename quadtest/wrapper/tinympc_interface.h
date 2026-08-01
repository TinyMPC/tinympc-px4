#ifndef TINYMPC_INTERFACE_H
#define TINYMPC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

void MPC_Init(void);

void MPC_Step(const float x[12],
              const float xref[12],
              float u[4]);

/* Like MPC_Step, but also returns the predicted state approximately 200 ms
 * ahead (horizon column 10 at 50 Hz) so the caller can publish the MPC plan
 * as a trackable position/velocity setpoint. */
void MPC_Step_Plan(const float x[12],
                   const float xref[12],
                   float u[4],
                   float xnext[12]);

#ifdef __cplusplus
}
#endif

#endif
