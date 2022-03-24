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
#define LOSS_REMOTE_SIGNAL 1 ///<ң�����źŶ�ʧ����Լ���������ʱδʵ�֣�

/* ���������Բ��� */
/* ��е�� */
// #define PITCH_UP_LIMIT 6960///<pitch����λ
// #define PITCH_MID_ANGLE 7490///<pitch�м丽��
// #define PITCH_DOWN_LIMIT 8010///<pitch����λ

/* PID */

/* ����ģʽ */
#define INFORMATION 1
#define DEBUG 0           ///< debug_printf()����ʹ��
#define ERROE_FOR_SHELL 1 ///<�ļ�������Ϣ������أ���δ��ɣ�

/* ��ӻ����˷������ݺ��������б� */

#endif
