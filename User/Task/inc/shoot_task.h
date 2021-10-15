#ifndef __SHOOT_TASK_H_
#define __SHOOT_TASK_H_

#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "remote_task.h"
#include "can2_device.h"
#include "stdlib.h"
#include "console_print.h"

void Parse_Friction_Wave_Motor_Feedback_Data(CAN_RxHeaderTypeDef *p_can_rx_header,uint8_t *can2_rxd_data_buffer);
int8_t Updata_Wave_Ch_Value(int16_t *last_wave_ch_value,int16_t *this_wave_ch_value);
void Emission_Once_Time(void);
int8_t *Get_Is_OK_Fire(void);
const int32_t* Get_Fire_One_Bullet_Signal(void);
const int32_t* Get_Fire_Five_Bullet_Signal(void);

#endif //__SHOOT_TASK_H_
