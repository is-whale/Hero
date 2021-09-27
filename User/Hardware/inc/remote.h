
#ifndef __REMOTE_H_
#define __REMOTE_H_

#include "stm32f4xx.h"
#include "stdio.h"

///< 按键定义
#define KEY_W       0x01
#define KEY_A       0x04
#define KEY_S       0x02
#define KEY_D       0x08

#define KEY_CTRL    0x20
#define KEY_SHIFT   0x10

#define KEY_Z       0x800
#define KEY_X       0x1000
#define KEY_C       0x2000
#define KEY_V       0x4000
#define KEY_B       0x8000
#define KEY_F       0x200
#define KEY_G       0x400
#define KEY_Q       0x40
#define KEY_E       0x80
#define KEY_R       0x100

#define RC_KEY_PRESSED(rc_key, key)     (rc_key & key)

#define RC_CH_VALUE_MIN                 ((uint16_t)364)
#define RC_CH_VALUE_OFFSET              ((uint16_t)1024)
#define RC_CH_VALUE_MAX                 ((uint16_t)1684)

#define RC_SW_UP                        ((uint8_t)1)
#define RC_SW_MID                       ((uint8_t)3)
#define RC_SW_DOWN                      ((uint8_t)2)

#define ROCKER_DATA_CHECK(data)         ( (data > 660) || (data < -660))
#define SWITCH_DATA_CHECK(data)         (!((data == 1) || (data == 2) || (data == 3)))

#define VIRTUAL_ROCKER_STEP1            20
#define VIRTUAL_ROCKER_STEP2            10


/**
 * @brief           遥控器数据结构 
 * @details         通过串口 1 接收到的数据对遥控器结构体成员进行解析赋值
 */
typedef __packed struct
{
    __packed struct
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t ch4;
        uint8_t s1;
        uint8_t s2;
    } rc;

    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    __packed struct
    {
        uint16_t value;
    } key;

    __packed struct
    {
        int16_t ch2; ///< 左右
        int16_t ch3; ///< 前后

    } virtual_rocker;

} Rc_Ctrl_t;

void Parse_Remoter_Data(volatile const uint8_t *sbus_buf, Rc_Ctrl_t *rc_ctrl);
int16_t Mouse_Coordnite_Value_Limit(int16_t data, int16_t min_value, int16_t max_value);
void Parse_Wasd_Key_To_Virtual_Rocker(Rc_Ctrl_t *rc);
uint8_t Remoter_Data_Check(Rc_Ctrl_t *rc);
void Rc_Data_Copy(Rc_Ctrl_t* rc1, Rc_Ctrl_t* rc2);
void Rc_Data_Reset(Rc_Ctrl_t* rc);

#endif //__REMOTE_H_

