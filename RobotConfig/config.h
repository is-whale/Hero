/**
 * @file config.h
 * @brief   ȫ�������ļ����������ֵ��ԣ����á�������ֲ���в�������
 * @version 0.1
 * @date    2022.3.5
 * @author  whale
 * @note    ���ļ������궨���������ã������������÷�����ԣ������ʵ���߼�Ӧ������������ɡ�
 * @copyright Copyright (c) 2022
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* ���԰�ȫ���� */
#define LASER_OFF 0          ///<�򿪼��⣨����ʱ����رշ�ֹ���ˣ�
#define CHASSIS_SPEED_ZERO 0 ///<��������ģʽ,��ֹ���̷�ת
#define GIMBAL_SPEED_ZERO 0  ///<��̨����ģʽ,��ֹ���̷�ת
#define LOSS_REMOTE_SIGNAL 1 ///<ң�����źŶ�ʧ����Լ���������ʱδʵ�֣�

/* ���������Բ��� */
/* ��е�� */
// #define PITCH_UP_LIMIT 6960///<pitch����λ
// #define PITCH_MID_ANGLE 7490///<pitch�м丽��
// #define PITCH_DOWN_LIMIT 8010///<pitch����λ
/* Yaw */
#define YAW_GM6020_HEAD_ANGLE 6640 ///< ά��֮��ǶȻ��    2468
#define YAW_LIMIT_SPEED 150        ///<YAW���ٶ��޷� ԭ��150

/* PID */

/* ����ģʽ */
#define INFORMATION 1
#define DEBUG 0           ///< debug_printf()����ʹ��
#define ERROE_FOR_SHELL 0 ///<�ļ�������Ϣ������أ���δ��ɣ�

/* ��ӻ����˷������ݺ��������б� */

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif
