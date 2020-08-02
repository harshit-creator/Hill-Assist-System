
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/*CAN Message IDs to be used*/
#define MSG_TILT_ANGLE 0x65D
#define MSG_GEAR   0x651
#define MSG_WHEEL  0x652
/* For Wheel and Gear Position(Forward and Reverse)*/
#define WFWD 1
#define WREV 0
#define GFWD 1
#define GREV 0

/* signals used in Task Notification*/

#define ACTIVATE 0x01
#define DEACTIVATE 0x02
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

osThreadId defaultTaskHandle;
osThreadId vHeartBeatHandle;
osThreadId vDataProcessHandle;
osThreadId vTaskActionHandle;
osMutexId dataMutexHandle;
osSemaphoreId dataSemaphoreHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_RxHeaderTypeDef RxHeader;
uint8_t rcvd_msg[1],recv_Data;
uint8_t dataTiltAngle=0,dataGear=0,dataWheel=0;
int32_t sigNotify=0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* Clock and Peripheral Initialization Functions*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* user defined functions declaration*/
void StartDefaultTask(void const * argument);
void tHeartBeat(void const * argument);
void tDataProcess(void const * argument);
void tTaskAction(void const * argument);
void CAN_START();

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN_START();           // Start the CAN  Module
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of dataMutex */
  osMutexDef(dataMutex);
  dataMutexHandle = osMutexCreate(osMutex(dataMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of dataSemaphore */
 /* osSemaphoreDef(dataSemaphore);
  dataSemaphoreHandle = osSemaphoreCreate(osSemaphore(dataSemaphore), 1);*/

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of vHeartBeat */
  osThreadDef(vHeartBeat, tHeartBeat, osPriorityAboveNormal, 0, 128);
  vHeartBeatHandle = osThreadCreate(osThread(vHeartBeat), NULL);

  /* definition and creation of vDataProcess */
  osThreadDef(vDataProcess, tDataProcess, osPriorityAboveNormal, 0, 128);
  vDataProcessHandle = osThreadCreate(osThread(vDataProcess), NULL);

  /* definition and creation of vTaskAction */
  osThreadDef(vTaskAction, tTaskAction, osPriorityAboveNormal, 0, 128);
  vTaskActionHandle = osThreadCreate(osThread(vTaskAction), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD9 PD10 PD11 PD12 
                           PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief CAN functions
 *Functions used --->
 *CAN1_Tx()     --        To transmit data over CAN BUS
* @param1      data to be transimitted
*@param2       CAN Message ID
 */

void CAN1_Tx(uint8_t data,uint32_t id)
{
	CAN_TxHeaderTypeDef TxHeader;

		uint32_t TxMailbox;

		uint8_t message;

		TxHeader.DLC = 1;
		TxHeader.StdId = id;
		TxHeader.IDE   = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;

          message=data;

		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);

		if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&message,&TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}
}
/*@brief    CAN Message Filter Configuration 
Config used: No Filter apllied(accept all CAN Messages)
                   Select Filter Bank 0,Select CAN Receiver FIFO0,FilterScale 32 Bit  
*/
void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef can1_filter_init;

	can1_filter_init.FilterActivation = ENABLE;
	can1_filter_init.FilterBank  = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0x0000;
	can1_filter_init.FilterIdLow = 0x0000;
	can1_filter_init.FilterMaskIdHigh = 0X0000;
	can1_filter_init.FilterMaskIdLow = 0x0000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

	if( HAL_CAN_ConfigFilter(&hcan1,&can1_filter_init) != HAL_OK)              //HAL layer API to config CAN Filter
	{
		Error_Handler();
	}

}
/*@brief   Function used to enablethe CAN Module
must be called before using CAN functionality
*/
void CAN_START(){
	   CAN_Filter_Config();                     //call filter config function
	  	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK) //HAL Layer API to start CAN Module
	  	{
	  			Error_Handler();
	  	}
	  	if( HAL_CAN_Start(&hcan1) != HAL_OK)
	  	{
	  		Error_Handler();
	  	}
}
/*
 * @brief Interrupt CallBacks
 *   3 Transmit Mailboxes interrupt callbacks
*    1 Receive interrupt FIFO0 callback 
 * */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
}
/* Messgae is filter based on message ID and  data is copied to respective sensor variables */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    uint32_t ID;
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK)
	{
		Error_Handler();
	}
      // ID=RxHeader.StdId;

	if(RxHeader.StdId == 0x65D && RxHeader.RTR == 0 )
		{
			//This is data frame sent by n1 to n2
		 dataTiltAngle=rcvd_msg[0];

		}
	if(RxHeader.StdId == 0x651 && RxHeader.RTR == 0 )
		{
		   dataGear=rcvd_msg[0];
		}
	if(RxHeader.StdId == 0x652 && RxHeader.RTR == 0 )
		{
			//This is data frame sent by n1 to n2
		  dataWheel=rcvd_msg[0];

		}
               /*if(RxHeader.RTR == 0 && RxHeader.StdId>0){
            	   switch(ID){

            	   case MSG_TILT_ANGLE:
            		   dataTiltAngle=rcvd_msg[0];
            		   break;
            	   case MSG_GEAR:
            		   dataGear=rcvd_msg[0];
            		   break;
            	   case MSG_WHEEL:
            		   dataWheel=rcvd_msg[0];
            		   break;
            	   default:
            		   break;
            	   }
               }*/


}
/* CAN Functions End*/

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* tHeartBeat function */
/* Task Handle used to indicate whether our systen is running or not
It is periodic task having period of 250 ms*/
void tHeartBeat(void const * argument)
{
  /* USER CODE BEGIN tHeartBeat */
	            TickType_t xPreviousWakeTime=xTaskGetTickCount();
				TickType_t ticksToIncrement=pdMS_TO_TICKS(250);
			  for(;;)
			  {
				          HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
							vTaskDelayUntil(&xPreviousWakeTime,ticksToIncrement);

			  }
}

/* tDataProcess function */
/*@brief    Data received from slave node is processed in this function to check whether critical occurs or not
*periodic task of 100ms
*Here Task Notification mechanism is used to notify ACTION task if Critical action occurs notify the Action Task
 */
void tDataProcess(void const * argument)
{
  /* USER CODE BEGIN tDataProcess */
  /* Infinite loop */
	 	 	 	 	TickType_t xPreviousWakeTime=xTaskGetTickCount();
					TickType_t ticksToIncrement=pdMS_TO_TICKS(100);
				  for(;;)
				  {
					        if(dataTiltAngle<150)
					        {
					        	if(dataGear==GFWD && dataWheel==WREV){
					        		osSignalSet(vTaskActionHandle,(int32_t)ACTIVATE);         //  Notify the Action Task to Activate the System


					        	}
					        	else if(dataGear==GFWD && dataWheel==WFWD){              // Notify the Action to task to deacitivate the system
					        		osSignalSet(vTaskActionHandle,(int32_t)DEACTIVATE);


					        	}
					        }
					            HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
								vTaskDelayUntil(&xPreviousWakeTime,ticksToIncrement);

				  }

  /* USER CODE END tDataProcess */
}

/* tTaskAction function */
/*If Critical action occurs activate the System
*/
void tTaskAction(void const * argument)
{
  /* USER CODE BEGIN tTaskAction */
  /* Infinite loop */
	osEvent event;
  for(;;)
  {
	         HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);


	 	 	  event=osSignalWait(sigNotify,osWaitForever);     //wait for the Signal event to occurred
	 	 	  if(event.status==osEventSignal){
	 	 		  if(event.value.signals==ACTIVATE){

	 	 			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET);       //Indicates System is activated
                    }
	 	 		  else if(event.value.signals==DEACTIVATE){
	 	 			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET);  // indicates system is deactivated

	 	 		  }
	 	 	  }
    osDelay(1);
  }
  /* USER CODE END tTaskAction */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
