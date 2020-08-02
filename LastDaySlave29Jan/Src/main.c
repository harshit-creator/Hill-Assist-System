
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
#include "math.h"
/*  Macro Definitions */
/* for 3 axes data*/
#define Xaxis 1
#define Yaxis 2
#define Zaxis 3
/*CAN Message IDs to be used*/
#define MSG_TILT_ANGLE 0x65D
#define MSG_GEAR   0x651
#define MSG_WHEEL  0x652
/* For Wheel and Gear Position(Forward and Reverse)*/
#define WFWD 1
#define WREV 0
#define GFWD 1
#define GREV 0
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

osThreadId defaultTaskHandle;
osThreadId vHeartBeatHandle;
osThreadId vAngleofTiltHandle;
osThreadId vTaskWheelPosHandle;
osMutexId mutexCANHandle;
osSemaphoreId semaCANHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t TxBuf[2],RxBuf_X[2],RxBuf_Y[2],RxBuf_Z[2],ReturnVal[2],readValX,readValY,readValZ,recv_Data;   // Accelerometer data
CAN_RxHeaderTypeDef RxHeader;
uint8_t rcvd_msg[1];
float resultX,resultY;
uint8_t angleData[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*----Peripheral Initialization------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
/*----Task Function Declarations-------*/
void StartDefaultTask(void const * argument);
void tHeartBeat(void const * argument);
void tMeasureTilt(void const * argument);
void tWheelPosition(void const * argument);
void accln_tilt(void); 

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SPI_start();         
void spiRead(int choice); 
void CAN1_Tx(uint8_t data,uint32_t id);
void CAN_START();
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  CAN_START();                  //start the CAN Module
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of mutexCAN */
  osMutexDef(mutexCAN);
  mutexCANHandle = osMutexCreate(osMutex(mutexCAN));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of semaCAN */
  /*osSemaphoreDef(semaCAN);
  semaCANHandle = osSemaphoreCreate(osSemaphore(semaCAN), 1);*/

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

  /* definition and creation of vAngleofTilt */
  osThreadDef(vAngleofTilt, tMeasureTilt, osPriorityAboveNormal, 0, 128);
  vAngleofTiltHandle = osThreadCreate(osThread(vAngleofTilt), NULL);

  /* definition and creation of vTaskWheelPos */
  osThreadDef(vTaskWheelPos, tWheelPosition, osPriorityAboveNormal, 0, 128);
  vTaskWheelPosHandle = osThreadCreate(osThread(vTaskWheelPos), NULL);

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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void SPI_start(){

	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	  TxBuf[0]=0x20;
	  TxBuf[1]=0x037;
	  HAL_SPI_Transmit(&hspi1,TxBuf,2,50);
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
}
void spiRead(int choice){


	switch(choice){

	case 1:
		         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
			     TxBuf[0]=0x29|0x80;
			     HAL_SPI_Transmit(&hspi1,TxBuf,1,50);
			     HAL_SPI_Receive(&hspi1,RxBuf_X,1,50);  //Read X
			     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
			     readValX=RxBuf_X[0];
		         break;
	case 2:
		 	 	 	 	 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
			    	     TxBuf[0]=0x2B|0x80;
			    	     HAL_SPI_Transmit(&hspi1,TxBuf,1,50);
			    	     HAL_SPI_Receive(&hspi1,RxBuf_Y,1,50);  //Read Y
			    	     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
			    	     readValY=RxBuf_Y[0];
			    	     break;
	case 3:
		                                 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
			    	    	    	     TxBuf[0]=0x2D|0x80;
			    	    	    	     HAL_SPI_Transmit(&hspi1,TxBuf,1,50);
			    	    	    	     HAL_SPI_Receive(&hspi1,RxBuf_Z,1,50);  //Read Z
			    	    	    	     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
			    	    	    	     readValZ=RxBuf_Z[0];
			    	    	    	     break;
	default:
		break;
	}


}
/* SPI functions END*/

/* @brief calculate angle of tilt and pitch
 *
 * **/
void accln_tilt(void){
	uint8_t x2,y2,z2,temp1,temp2,tempR;
	float accln_angleX,accln_angleY;
	x2=readValX*readValX;
	y2=readValY*readValY;
	z2=readValZ*readValZ;
	temp1=(uint8_t)(x2+y2);
    temp2=(uint8_t)(x2+z2);
   resultX=sqrtf(temp1);
   resultY=sqrtf(temp2);

   resultX=readValX/resultX;
   resultY=readValY/resultY;

   /*accln_angleX=tanf(resultX);
   accln_angleY=tanf(resultY);*/

   angleData[0]=100*tanf(resultX);   //roll angle
   //*(angleDataptr+1)=tanf(resultY);   //pitch angle

   //temp4=(float)angleData[0]


}
/*
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
		osStatus status;

		TxHeader.DLC = 1;
		TxHeader.StdId = id;
		TxHeader.IDE   = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;

          message=data;

		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
		status=osMutexWait(mutexCANHandle,50);
		if(status!=osOK){
			Error_Handler();
		}

		if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&message,&TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}
		status=osMutexRelease(mutexCANHandle);
		if(status!=osOK){
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

	if( HAL_CAN_ConfigFilter(&hcan1,&can1_filter_init) != HAL_OK)
	{
		Error_Handler();
	}

}
/*@brief   Function used to enablethe CAN Module
must be called before using CAN functionality
*/
void CAN_START(){
	   CAN_Filter_Config();                     //call filter config function
	  	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK)
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
 * */void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
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


	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK)
	{
		Error_Handler();
	}

	if(RxHeader.StdId == 0x65D && RxHeader.RTR == 0 )
	{
		//This is data frame sent by n1 to n2
		//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
		recv_Data=rcvd_msg[0];

	}

}
/* CAN Functions End*/

/**
 * @brief External Interrrupt for Gear Position:
 *   pin PA0 is used.
 *   logic 1=gear forward
 *   logic 0=gear Reverse
 *
 * */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	static uint8_t dataGear=1;
	if(dataGear){
		CAN1_Tx(GREV,(uint32_t)MSG_GEAR);
		dataGear=0;
	}else{

		CAN1_Tx(GFWD,(uint32_t)MSG_GEAR);
	   dataGear=1;
	}

}
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
		  /* Infinite loop */
			TickType_t xPreviousWakeTime=xTaskGetTickCount();
			TickType_t ticksToIncrement=pdMS_TO_TICKS(250);
		  for(;;)
		  {
			          HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
						vTaskDelayUntil(&xPreviousWakeTime,ticksToIncrement);

		  }
		  /* USER CODE END tHeartBeat */
}

/* tMeasureTilt function */
/*Task to measure the angle of tilt using Acclerometer based on data received by X,Y and Z axis*/
void tMeasureTilt(void const * argument)
{
  /* USER CODE BEGIN tMeasureTilt */
  /* Infinite loop */

            //uint8_t angleData[2];
			TickType_t xPreviousWakeTime=xTaskGetTickCount();
			TickType_t ticksToIncrement=pdMS_TO_TICKS(200);
			SPI_start();   //Initialize Accelerometer Registers
		  for(;;)
		  {

			  spiRead(Xaxis);                             // function to read Axes data
			  spiRead(Yaxis);
			  spiRead(Zaxis);

			  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
			  accln_tilt();
			  CAN1_Tx(angleData[0],(uint32_t)MSG_TILT_ANGLE); //Transmit the CAN message for angle of tilt

				vTaskDelayUntil(&xPreviousWakeTime,ticksToIncrement);
		  }

}

/* tWheelPosition function */
/*task to check gear position */
void tWheelPosition(void const * argument)
{
  /* USER CODE BEGIN tWheelPosition */
	uint8_t dataWheel=0;
	TickType_t xPreviousWakeTime=xTaskGetTickCount();
	TickType_t ticksToIncrement=pdMS_TO_TICKS(200);
  /* Infinite loop */
  for(;;)
  {

	  if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11)){
		  dataWheel=WREV;       //if Switch is pressed--> Wheel Direction---Reverse
	      HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
	  }
	  else {
		  dataWheel=WFWD;     //if switch is not pressed--> Wheel Direction---Forward
	  }

      CAN1_Tx(dataWheel,(uint32_t)MSG_WHEEL);

   vTaskDelayUntil(&xPreviousWakeTime,ticksToIncrement);
  }
  /* USER CODE END tWheelPosition */
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
