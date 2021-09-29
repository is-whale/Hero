#ifndef __REMOTE_TASK_H_
#define __REMOTE_TASK_H_

#include "cmsis_os.h"
#include "usart1.h"
#include "remote.h"
#include "control_data.h"
#include "monitor_task.h"

void Info_RemoteTask_Parse_Data(void);
Rc_Ctrl_t *Get_Rc_Parsed_RemoteData(void);

#endif //__REMOTE_TASK_H_
