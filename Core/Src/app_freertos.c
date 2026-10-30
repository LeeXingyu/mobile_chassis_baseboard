/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DYPA22.h"
#include "usb_cdc_result.h"
#include "zlac8015d.h"
#include "cdc_can_command.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DYP_STARTUP_CONFIG_ENABLE (0U)
#define DYP_DEFAULT_ALGORITHM_ENABLE (1U)
#define DYP_DEFAULT_ANGLE_LEVEL (4U)
#define DYP_DEFAULT_RANGE_LEVEL (4U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* Static allocation preserves the small (3072 byte) FreeRTOS heap. */
static StaticTask_t motorTaskControl;
static StackType_t motorTaskStack[256];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void StartMotorTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (xTaskCreateStatic(StartMotorTask, "motorCAN", 256U, NULL,
                        (UBaseType_t)osPriorityAboveNormal, motorTaskStack,
                        &motorTaskControl) == NULL) {
    Error_Handler();
  }
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN StartDefaultTask */
#if DYP_STARTUP_CONFIG_ENABLE
  /* Optional one-time sensor setup; enable only when sensor configuration is required. */
  for (uint8_t sensor = 0U; sensor < SENSOR_COUNT; ++sensor)
  {
    uint8_t sensor_id = (uint8_t)(sensor + 1U);
    (void)Ultrasonic_SetAlgorithm(sensor_id, DYP_DEFAULT_ALGORITHM_ENABLE);
    (void)Ultrasonic_ConfigSensorLevels(sensor_id,
                                        DYP_DEFAULT_ANGLE_LEVEL,
                                        DYP_DEFAULT_RANGE_LEVEL);
  }
#endif
  /* Infinite loop */
  for(;;)
  {
    (void)UsbCdc_SendCanReply();
    /* Keep protocol parsing and sensor polling outside interrupt context. */
    if (Ultrasonic_ServiceTick() != 0U)
    {
      (void)UsbCdc_SendUltrasonicResults(g_sensors, SENSOR_COUNT);
    }
    /* Keep the existing 1-second DYP round delay while draining command replies.
     * DYP's own blocking reads may still delay a reply by one polling round. */
    for (uint32_t wait = 0U; wait < 50U; ++wait) {
      (void)UsbCdc_SendCanReply();
      osDelay(20U);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* The motor task is the only CANopen sender. USB IRQ only queues command text;
 * no periodic probe competes with externally requested SDO transactions. */
static void StartMotorTask(void *argument)
{
  (void)argument;
  /* Keep command service alive after init failure so the host gets an error. */
  (void)Zlac_Init();
  for (;;) {
    Zlac_Service();
    CdcCan_Service();
    vTaskDelay(pdMS_TO_TICKS(5U));
  }
}

/* USER CODE END Application */

