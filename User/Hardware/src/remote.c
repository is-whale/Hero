
#include "remote.h"

void Parse_Remoter_Data(volatile const uint8_t *sbus_buf, Rc_Ctrl_t *rc_ctrl)
{
    rc_ctrl->rc.ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        ///< Channel 0
    rc_ctrl->rc.ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; ///< Channel 1
    rc_ctrl->rc.ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          ///< Channel 2
                       (sbus_buf[4] << 10)) &
                      0x07ff;
    rc_ctrl->rc.ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; ///< Channel 3
    rc_ctrl->rc.s1 = ((sbus_buf[5] >> 4) & 0x0003);                       ///< Switch left
    rc_ctrl->rc.s2 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  ///< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                  ///< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                  ///< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                ///< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                ///< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                ///< Mouse Right Is Press ?
    rc_ctrl->key.value = sbus_buf[14] | (sbus_buf[15] << 8);              ///< KeyBoard value
    rc_ctrl->rc.ch4 = sbus_buf[16] | (sbus_buf[17] << 8);                 ///< Channel 4

    rc_ctrl->rc.ch0 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch1 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch2 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch3 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch4 -= RC_CH_VALUE_OFFSET;

    ///< 将WASD按键数据解析为虚拟遥感
    Parse_Wasd_Key_To_Virtual_Rocker(rc_ctrl);

    ///< 鼠标x、y轴限幅
    rc_ctrl->mouse.x = Mouse_Coordnite_Value_Limit(rc_ctrl->mouse.x, -5000, 5000);
    rc_ctrl->mouse.y = Mouse_Coordnite_Value_Limit(rc_ctrl->mouse.y, -5000, 5000);
}

int16_t Mouse_Coordnite_Value_Limit(int16_t data, int16_t min_value, int16_t max_value)
{
    if (data > max_value)
        return max_value;
    else if (data < min_value)
        return min_value;
    return data;
}

void Parse_Wasd_Key_To_Virtual_Rocker(Rc_Ctrl_t *rc)
{
    #define RC_KEY rc->key.value
    #define KEY_PRESSED(key) (RC_KEY & key)

    ///< 只按下W时，不按下S
    if (KEY_PRESSED(KEY_W) && !(KEY_PRESSED(KEY_S)))
    {
        if (rc->virtual_rocker.ch3 < 0)
        {
            rc->virtual_rocker.ch3 += VIRTUAL_ROCKER_STEP2;
        }
        if (rc->virtual_rocker.ch3 < 600)
        {
            rc->virtual_rocker.ch3 += VIRTUAL_ROCKER_STEP1;
        }
    }
    ///< 只按下S时，不按下W
    else if (KEY_PRESSED(KEY_S) && !(KEY_PRESSED(KEY_W)))
    {
        if (rc->virtual_rocker.ch3 > 0)
        {
            rc->virtual_rocker.ch3 -= VIRTUAL_ROCKER_STEP2;
        }
        if (rc->virtual_rocker.ch3 > -600)
        {
            rc->virtual_rocker.ch3 -= VIRTUAL_ROCKER_STEP1;
        }
    }
    ///< W、S都未按下时
    else
    {
        if (rc->virtual_rocker.ch3 < 0)
        {
            rc->virtual_rocker.ch3 += VIRTUAL_ROCKER_STEP2;
            if (rc->virtual_rocker.ch3 > 0)
                rc->virtual_rocker.ch3 = 0;
        }
        else if (rc->virtual_rocker.ch3 > 0)
        {
            rc->virtual_rocker.ch3 -= VIRTUAL_ROCKER_STEP2;
            if (rc->virtual_rocker.ch3 < 0)
                rc->virtual_rocker.ch3 = 0;
        }
    }

    ///< 只按下D时
    if (KEY_PRESSED(KEY_D) && !(KEY_PRESSED(KEY_A)))
    {
        if (rc->virtual_rocker.ch2 < 0)
        {
            rc->virtual_rocker.ch2 += VIRTUAL_ROCKER_STEP2;
        }
        if (rc->virtual_rocker.ch2 < 600)
        {
            rc->virtual_rocker.ch2 += VIRTUAL_ROCKER_STEP1;
        }
    }
    ///< 只按下A时
    else if (KEY_PRESSED(KEY_A) && !(KEY_PRESSED(KEY_D)))
    {
        if (rc->virtual_rocker.ch2 > 0)
        {
            rc->virtual_rocker.ch2 -= VIRTUAL_ROCKER_STEP2;
        }
        if (rc->virtual_rocker.ch2 > -600)
        {
            rc->virtual_rocker.ch2 -= VIRTUAL_ROCKER_STEP1;
        }
    }
    ///< A、D都未按下时
    else
    {
        if (rc->virtual_rocker.ch2 < 0)
        {
            rc->virtual_rocker.ch2 += VIRTUAL_ROCKER_STEP2;
            if (rc->virtual_rocker.ch2 > 0)
                rc->virtual_rocker.ch2 = 0;
        }
        else if (rc->virtual_rocker.ch2 > 0)
        {
            rc->virtual_rocker.ch2 -= VIRTUAL_ROCKER_STEP2;
            if (rc->virtual_rocker.ch2 < 0)
                rc->virtual_rocker.ch2 = 0;
        }
    }
    #undef RC_KEY
    #undef KEY_PRESSED
}

uint8_t Remoter_Data_Check(Rc_Ctrl_t *rc)
{
	if (ROCKER_DATA_CHECK(rc->rc.ch0))
	{
		return 1;
	}
	if (ROCKER_DATA_CHECK(rc->rc.ch1))
	{
		return 1;
	}
	if (ROCKER_DATA_CHECK(rc->rc.ch2))
	{
		return 1;
	}
	if (ROCKER_DATA_CHECK(rc->rc.ch3))
	{
		return 1;
	}
	if (SWITCH_DATA_CHECK(rc->rc.s1))
	{
		return 1;
	}
	if (SWITCH_DATA_CHECK(rc->rc.s2))
	{
		return 1;
	}
	return 0;
}


/**
 * @brief               将 rc2 赋值给 rc1 
 * @param {Rc_Ctrl_t}   遥控器数据 1
 * @param {Rc_Ctrl_t}   遥控器数据 2
 * @retval              void
 */
void Rc_Data_Copy(Rc_Ctrl_t* rc1, Rc_Ctrl_t* rc2)
{
	rc1->key.value = rc2->key.value;
	
	rc1->mouse.press_l = rc2->mouse.press_l;
	rc1->mouse.press_r = rc2->mouse.press_r;
	rc1->mouse.x = rc2->mouse.x;
	rc1->mouse.y = rc2->mouse.y;
	rc1->mouse.z = rc2->mouse.z;
	
	rc1->rc.ch0 = rc2->rc.ch0;
	rc1->rc.ch1 = rc2->rc.ch1;
	rc1->rc.ch2 = rc2->rc.ch2;
	rc1->rc.ch3 = rc2->rc.ch3;
	rc1->rc.ch4 = rc2->rc.ch4;
	rc1->rc.s1 = rc2->rc.s1;
	rc1->rc.s2 = rc2->rc.s2;
}

/**
 * @brief               重置遥控器数据 
 * @param {Rc_Ctrl_t}   遥控器数据
 * @retval              void
 */
void Rc_Data_Reset(Rc_Ctrl_t* rc)
{
	rc->key.value = 0;
	
	rc->mouse.press_l = 0;
	rc->mouse.press_r = 0;
	rc->mouse.x = 0;
	rc->mouse.y = 0;
	rc->mouse.z = 0;
	
	rc->rc.ch0 = 0;
	rc->rc.ch1 = 0;
	rc->rc.ch2 = 0;
	rc->rc.ch3 = 0;
	rc->rc.ch4 = 0;
	rc->rc.s1 = 3;
	rc->rc.s2 = 3;
}
