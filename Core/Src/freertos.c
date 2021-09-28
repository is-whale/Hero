/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId shellTaskHandle;
uint32_t shellTaskBuffer[ 1024 ];
osStaticThreadDef_t shellTaskControlBlock;
osThreadId remoteTaskHandle;
uint32_t remoteTaskBuffer[ 512 ];
osStaticThreadDef_t remoteTaskControlBlock;
osThreadId monitorTaskHandle;
uint32_t monitorTaskBuffer[ 512 ];
osStaticThreadDef_t monitorTaskControlBlock;
osThreadId canSendTaskHandle;
uint32_t canSendTaskBuffer[ 512 ];
osStaticThreadDef_t canSendTaskControlBlock;
osThreadId chassisTaskHandle;
uint32_t chassisTaskBuffer[ 512 ];
osStaticThreadDef_t chassisTaskControlBlock;
osThreadId parseCan1RxDataHandle;
uint32_t parseCanRxDataTBuffer[ 512 ];
osStaticThreadDef_t parseCanRxDataTControlBlock;
osThreadId parseCan2RxDataHandle;
uint32_t parseCan2RxDataBuffer[ 512 ];
osStaticThreadDef_t parseCan2RxDataControlBlock;
osSemaphoreId shellGetDataBinarySemHandle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
osSemaphoreId remoteGetDataBinarySemHandle;
osStaticSemaphoreDef_t remoteGetDataBinarySemControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartShellTask(void const * argument);
void StartRemoteTask(void const * argument);
void StartMonitorTask(void const * argument);
void StartCanSendTask(void const * argument);
void StartChassisTask(void const * argument);
void StartParseCan1RxDataTask(void const * argument);
void StartParseCan2RxDataTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of shellGetDataBinarySem */
  osSemaphoreStaticDef(shellGetDataBinarySem, &myBinarySem01ControlBlock);
  shellGetDataBinarySemHandle = osSemaphoreCreate(osSemaphore(shellGetDataBinarySem), 1);

  /* definition and creation of remoteGetDataBinarySem */
  osSemaphoreStaticDef(remoteGetDataBinarySem, &remoteGetDataBinarySemControlBlock);
  remoteGetDataBinarySemHandle = osSemaphoreCreate(osSemaphore(remoteGetDataBinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 512, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of shellTask */
  osThreadStaticDef(shellTask, StartShellTask, osPriorityBelowNormal, 0, 1024, shellTaskBuffer, &shellTaskControlBlock);
  shellTaskHandle = osThreadCreate(osThread(shellTask), NULL);

  /* definition and creation of remoteTask */
  osThreadStaticDef(remoteTask, StartRemoteTask, osPriorityHigh, 0, 512, remoteTaskBuffer, &remoteTaskControlBlock);
  remoteTaskHandle = osThreadCreate(osThread(remoteTask), NULL);

  /* definition and creation of monitorTask */
  osThreadStaticDef(monitorTask, StartMonitorTask, osPriorityLow, 0, 512, monitorTaskBuffer, &monitorTaskControlBlock);
  monitorTaskHandle = osThreadCreate(osThread(monitorTask), NULL);

  /* definition and creation of canSendTask */
  osThreadStaticDef(canSendTask, StartCanSendTask, osPriorityAboveNormal, 0, 512, canSendTaskBuffer, &canSendTaskControlBlock);
  canSendTaskHandle = osThreadCreate(osThread(canSendTask), NULL);

  /* definition and creation of chassisTask */
  osThreadStaticDef(chassisTask, StartChassisTask, osPriorityAboveNormal, 0, 512, chassisTaskBuffer, &chassisTaskControlBlock);
  chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

  /* definition and creation of parseCan1RxData */
  osThreadStaticDef(parseCan1RxData, StartParseCan1RxDataTask, osPriorityAboveNormal, 0, 512, parseCanRxDataTBuffer, &parseCanRxDataTControlBlock);
  parseCan1RxDataHandle = osThreadCreate(osThread(parseCan1RxData), NULL);

  /* definition and creation of parseCan2RxData */
  osThreadStaticDef(parseCan2RxData, StartParseCan2RxDataTask, osPriorityIdle, 0, 512, parseCan2RxDataBuffer, &parseCan2RxDataControlBlock);
  parseCan2RxDataHandle = osThreadCreate(osThread(parseCan2RxData), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartShellTask */
/**
* @brief Function implementing the shellTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShellTask */
__weak void StartShellTask(void const * argument)
{
  /* USER CODE BEGIN StartShellTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartShellTask */
}

/* USER CODE BEGIN Header_StartRemoteTask */
/**
* @brief Function implementing the remoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemoteTask */
__weak void StartRemoteTask(void const * argument)
{
  /* USER CODE BEGIN StartRemoteTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRemoteTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the monitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
__weak void StartMonitorTask(void const * argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartCanSendTask */
/**
* @brief Function implementing the canSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanSendTask */
__weak void StartCanSendTask(void const * argument)
{
  /* USER CODE BEGIN StartCanSendTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCanSendTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the chassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
__weak void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartParseCan1RxDataTask */
/**
* @brief Function implementing the parseCan1RxData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParseCan1RxDataTask */
__weak void StartParseCan1RxDataTask(void const * argument)
{
  /* USER CODE BEGIN StartParseCan1RxDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartParseCan1RxDataTask */
}

/* USER CODE BEGIN Header_StartParseCan2RxDataTask */
/**
* @brief Function implementing the parseCan2RxData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParseCan2RxDataTask */
__weak void StartParseCan2RxDataTask(void const * argument)
{
  /* USER CODE BEGIN StartParseCan2RxDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartParseCan2RxDataTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
