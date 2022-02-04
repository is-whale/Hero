#ifndef __GIMBAL_TASK_H_
#define __GIMBAL_TASK_H_

#include "cmsis_os.h"
#include "can2.h"
#include "can2_device.h"
#include "remote_task.h"
#include "imu_mpu6050.h"
#include "console_print.h"
#include "math2.h"

/*yaw*/
#define YAW_GM6020_HEAD_ANGLE   5450    ///<��ȡ��̨����̶���ʱ��GM6020�����е��
#define YAW_LIMIT_SPEED         150     ///<YAW���ٶ��޷�
#define YAW_SPEED_OUTPUT_LIMIT(data, limit)     Float_Constraion(data, -limit, limit)


/*pitch*/
/*��������λ�õĵ���Ƕȷ���ֵ�õ���Щ���ݡ�����̨���&����ܵ����λ��*/

#define PITCH_UP_LIMIT      6960    ///<pitch����λ
#define PITCH_MID_ANGLE     7490    ///<pitch�м丽��
#define PITCH_DOWN_LIMIT    8010    ///<pitch����λ


#define __OPEN_CAN2_RX_FIFO0_IT__ HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); 
/*���⺯������*/
void Parse_Can2_Gimbal_Rxd_Data(CAN_RxHeaderTypeDef *p_can_rx_header, uint8_t data[], Motor_Measure_t *motor);
Motor_Measure_t *Get_Gimbal_Parsed_FeedBack_Data(void);
const uint8_t* Get_Pitch_Motor_Index(void);
const uint8_t* Get_Yaw_Motor_Index(void);
float GM6020_YAW_Angle_To_360(uint16_t gm6020_angle);///<������ػ�е������ת��Ϊ������ʽ

#endif //__GIMBAL_TASK_H_
