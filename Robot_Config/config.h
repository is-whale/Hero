/**
 * @file config.h
 * @brief   全局配置文件，包含各种调试，配置。后期移植所有参数调节
 * @version 0.1
 * @date    2022.3.5
 * @author  whale
 * @note    此文件仅做宏定义或参数设置，集中所有设置方便调试，具体的实现逻辑应在其他部分完成。
 * @copyright Copyright (c) 2022
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* 调试安全配置 */
#define LASER_OFF 0          ///<打开激光（调试时建议关闭防止伤人）
#define CHASSIS_SPEED_ZERO 0 ///<底盘无力模式,防止底盘疯转
#define LOSS_REMOTE_SIGNAL 1 ///<遥控器信号丢失检测以及错误处理（暂时未实现）

/* 机器人属性参数 */
/* 机械角 */
// #define PITCH_UP_LIMIT 6960///<pitch上限位
// #define PITCH_MID_ANGLE 7490///<pitch中间附近
// #define PITCH_DOWN_LIMIT 8010///<pitch下限位

/* PID */

/* 调试模式 */
#define INFORMATION 1
#define DEBUG 0           ///< debug_printf()函数使能
#define ERROE_FOR_SHELL 1 ///<文件错误信息输出开关（暂未完成）

/* 添加机器人返回数据函数定义列表 */

#endif
