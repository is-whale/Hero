#include "chassis_task.h"

static Rc_Ctrl_t *rc_data;

void StartChassisTask(void const *argument)
{
    static float chassis_motor_speed[4] = {0.0, 0.0, 0.0, 0.0};
    rc_data = Get_Rc_Parsed_RemoteData_Pointer();

    osDelay(100);

    for (;;)
    {

        osDelay(10);
    }
}
