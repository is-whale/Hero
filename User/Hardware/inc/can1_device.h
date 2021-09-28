#ifndef __CAN1_DEVICE_H_
#define __CAN1_DEVICE_H_

#include "stm32f4xx.h"

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    SUPER_CAPACITOR_ID = 0x211,
    SUPER_CAPACITOR_SEND_ID = 0x210,
} Can1_RxMsg_ID;

#endif // __CAN1_DEVICE_H_
