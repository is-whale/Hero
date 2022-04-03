#ifndef __SUPER_CAPACITOR_TASK_H_
#define __SUPER_CAPACITOR_TASK_H_

#include "can1_device.h"
#include "cmsis_os.h"

typedef struct
{
    float input_voltage; ///< �����ѹ
    float cap_voltage;   ///< ���ݵ�ѹ
    float input_current; ///< �������
    float target_power;  ///< Ŀ�깦��
} Super_Capacitor_t;


void Info_Super_Capacitor_Parse_Data(void);
void Parse_Super_Capacitor_RxdData(CAN_RxHeaderTypeDef *can1_rx_header_t, Super_Capacitor_t *cap_data, uint8_t *can_message);
const Super_Capacitor_t * Get_SuperCapacitor_Date(void);
#endif //__SUPER_CAPACITOR_TASK_H_
