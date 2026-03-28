#include <stdio.h>
#include "Control.h"

#define DSHOT_MAX_THROTTLE 700
#define DSHOT_MIN_THROTTLE 50

static int16_t constrain_throttle(int16_t calculated_throttle)
{
    if (calculated_throttle < DSHOT_MIN_THROTTLE)
        return 0;
    if (calculated_throttle > DSHOT_MAX_THROTTLE)
        return DSHOT_MAX_THROTTLE;
    return (int16_t)calculated_throttle;
}

void Control_Run(uint16_t base_throttle, vector_t *PID_output)
{
    uint16_t throttle1, throttle2, throttle3, throttle4;
    throttle1 = constrain_throttle(base_throttle - PID_output->x - PID_output->y - PID_output->z);
    throttle2 = constrain_throttle(base_throttle - PID_output->x + PID_output->y + PID_output->z);
    throttle3 = constrain_throttle(base_throttle + PID_output->x - PID_output->y + PID_output->z);
    throttle4 = constrain_throttle(base_throttle + PID_output->x + PID_output->y - PID_output->z);
    ESC_RunAllPropeller(throttle1, throttle2, throttle3, throttle4);
}