/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_LENGTH 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t rx_buffer[RX_BUFFER_LENGTH];
extern UART_HandleTypeDef huart3;

/* USER CODE END Variables */
/* Definitions for Listener */
osThreadId_t ListenerHandle;
const osThreadAttr_t Listener_attributes = {
  .name = "Listener",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Speaker */
osThreadId_t SpeakerHandle;
const osThreadAttr_t Speaker_attributes = {
  .name = "Speaker",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartListener(void *argument);
void StartSpeaker(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of Listener */
  ListenerHandle = osThreadNew(StartListener, NULL, &Listener_attributes);

  /* creation of Speaker */
  SpeakerHandle = osThreadNew(StartSpeaker, NULL, &Speaker_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartListener */
/**
  * @brief  Function implementing the Listener thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartListener */
void StartListener(void *argument)
{
  /* USER CODE BEGIN StartListener */
	char local_buffer[RX_BUFFER_LENGTH];
	uint8_t local_pointer = 0;
	/* Infinite loop */
  for(;;)
  {
	 HAL_StatusTypeDef ret = HAL_UART_Receive(&huart3,rx_buffer ,1, 100);
	 if(ret == HAL_OK){
		 local_buffer[local_pointer] = rx_buffer[0];
		 if(local_buffer[local_pointer] == '\n' || local_buffer[local_pointer] == '\r'){

		 }else{
			 local_pointer++;
			 if(local_pointer > RX_BUFFER_LENGTH){
				 local_pointer = 0;
			 }
		 }
	 }
    osDelay(100);
  }
  /* USER CODE END StartListener */
}

/* USER CODE BEGIN Header_StartSpeaker */
/**
* @brief Function implementing the Speaker thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSpeaker */
void StartSpeaker(void *argument)
{
  /* USER CODE BEGIN StartSpeaker */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSpeaker */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
