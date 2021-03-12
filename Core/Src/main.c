/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include "Delay.h"
#include <stdio.h>
#include <math.h>
#include "ads1115.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t rxbuffer[255];
uint8_t ADS1115_ADDRESS = 0x48;
unsigned char ADSwrite[6];
unsigned char ADS1115_ADDRES[4];
int16_t reading;
float voltageT[4];
const float voltageConv = 6.114 / 32768.0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum
{
  Switch_Off    = 0x00U,
  Switch_On       = 0x01U,
} SwitchStatus;

typedef enum
{
  differentialMode    = 0x00U,
  singleEndMode       = 0x01U,
} ads1115QeuryMode;




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

osThreadId ModbusTransTaskHandle;
osThreadId ModbusSetTaskHandle;
osThreadId ADS1115TaskHandle;
/* USER CODE BEGIN PV */
uint8_t SLAVEID[8];
uint8_t ADCVALUE[16];
uint8_t slaveID = 0;
uint8_t SwitchValue[8];

float voltage = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
void ModbusTransThread(void const * argument);
void ModbusSetThread(void const * argument);
void ADS1115Thread(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  delay_init(10);


  //TODO: read DIP switch
  SwitchValue[0] = HAL_GPIO_ReadPin(GPIOE, Switch1_Pin);
  SwitchValue[1] = HAL_GPIO_ReadPin(GPIOE, Switch2_Pin);
  SwitchValue[2] = HAL_GPIO_ReadPin(GPIOE, Switch3_Pin);
  SwitchValue[3] = HAL_GPIO_ReadPin(GPIOE, Switch4_Pin);
  SwitchValue[4] = HAL_GPIO_ReadPin(GPIOE, Switch5_Pin);
  SwitchValue[5] = HAL_GPIO_ReadPin(GPIOE, Switch6_Pin);
  SwitchValue[6] = HAL_GPIO_ReadPin(GPIOE, Switch7_Pin);
  SwitchValue[7] = HAL_GPIO_ReadPin(GPIOE, Switch8_Pin);


  //TODO: converter DIP switch to modbus slave using binary
  for(uint8_t i = 0; i <8; i++)
  {
	  slaveID += SwitchValue[i] * pow(2,i);
  }

  delay_ms(1);

 //ADCstatus =  HAL_ADC_Stop_DMA(&hadc1);

  //HAL_UART_Receive_DMA(&huart6, rxbuffer, 8);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  //set DMA receive, transmit 255 word every time.
  HAL_UART_Receive_DMA(&huart6, (uint8_t*)rxbuffer, 255);

  ADS1115_ADDRES[0] = ADS1115_ADDRESS_ADDR_GND;
  ADS1115_ADDRES[1] = ADS1115_ADDRESS_ADDR_VDD;
  ADS1115_ADDRES[2] = ADS1115_ADDRESS_ADDR_SDA;
  ADS1115_ADDRES[3] = ADS1115_ADDRESS_ADDR_SCL;
  /* USER CODE END 2 */

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
  /* definition and creation of ModbusTransTask */
  osThreadDef(ModbusTransTask, ModbusTransThread, osPriorityAboveNormal, 0, 128);
  ModbusTransTaskHandle = osThreadCreate(osThread(ModbusTransTask), NULL);

  /* definition and creation of ModbusSetTask */
  osThreadDef(ModbusSetTask, ModbusSetThread, osPriorityAboveNormal, 0, 128);
  ModbusSetTaskHandle = osThreadCreate(osThread(ModbusSetTask), NULL);

  /* definition and creation of ADS1115Task */
  osThreadDef(ADS1115Task, ADS1115Thread, osPriorityNormal, 0, 128);
  ADS1115TaskHandle = osThreadCreate(osThread(ADS1115Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  ModBus_Init();
  ModBus_SetAddress(slaveID);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : Switch1_Pin Switch2_Pin Switch3_Pin Switch4_Pin 
                           Switch5_Pin Switch6_Pin Switch7_Pin Switch8_Pin */
  GPIO_InitStruct.Pin = Switch1_Pin|Switch2_Pin|Switch3_Pin|Switch4_Pin 
                          |Switch5_Pin|Switch6_Pin|Switch7_Pin|Switch8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))   //Idle interrupt case.
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart6);                     //clear idle flag, or process will trigger idle interrupt process.
            //printf("\r\nUART6 Idle IQR Detected\r\n");
            USAR_UART_IDLECallback(huart);                          //Idle interrupt callback
        }
    }
}

void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	//STOP DMA
    HAL_UART_DMAStop(&huart6);

    uint8_t data_length  = 255 - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);   //计算接收到的数据长度

    if (rxbuffer[0]==slaveID)
	{
		// if slaveID match, put rxbuffer to ModBusInHandle
		for(int i=0;i<data_length;i++)
			osMessagePut(ModBusInHandle,rxbuffer[i], data_length);
	}else{
		// if uart RX_ID no match slaveID(switch on board), reset rxbuffer
		memset(rxbuffer,0,data_length);
		data_length = 0;
		HAL_Delay(10);
	}

    //restart DMA and transmit 255 word per times.
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)rxbuffer, 255);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ModbusTransThread */
/**
  * @brief  Function implementing the ModbusTransTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_ModbusTransThread */
void ModbusTransThread(void const * argument)
{
  /* USER CODE BEGIN 5 */
	  uint8_t buf[256]; // buffer, where we collect output data
	  uint8_t c = 0; // counter for buffer fill
	  uint8_t count = 0;
  /* Infinite loop */
  for(;;)
  {
	 // ModBus_SetRegister(0,5+1);

    osEvent evt = osMessageGet(ModBusOutHandle,200); // wait here 200 tick
    if (evt.status == osEventMessage)
      {

        buf[c++]=(uint8_t) evt.value.v;
      }
    if (evt.status == osEventTimeout)
      {
        if( (c > 0) && (c < 254) ) // ok, something in buffer exist, lets send it
        {
        	HAL_StatusTypeDef AA = HAL_UART_Transmit(&huart6, buf, c, 50); // by USB-CDC
        }
      c=0;
      }

    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_ModbusSetThread */
/**
* @brief Function implementing the ModbusSetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModbusSetThread */
void ModbusSetThread(void const * argument)
{
  /* USER CODE BEGIN ModbusSetThread */
  /* Infinite loop */
  for(;;)
  {

		   ModBus_SetRegister(0,(int)Convert2Modbus(ADS1115_ADDRESS_ADDR_GND_BOARD.ADS1115_CH1.data));
		   ModBus_SetRegister(1,(int)Convert2Modbus(ADS1115_ADDRESS_ADDR_GND_BOARD.ADS1115_CH2.data));
		   ModBus_SetRegister(2,(int)Convert2Modbus(ADS1115_ADDRESS_ADDR_VDD_BOARD.ADS1115_CH1.data));
		   ModBus_SetRegister(3,(int)Convert2Modbus(ADS1115_ADDRESS_ADDR_VDD_BOARD.ADS1115_CH2.data));
		   ModBus_SetRegister(4,(int)Convert2Modbus(ADS1115_ADDRESS_ADDR_SDA_BOARD.ADS1115_CH1.data));
		   ModBus_SetRegister(5,(int)Convert2Modbus(ADS1115_ADDRESS_ADDR_SDA_BOARD.ADS1115_CH2.data));
		   ModBus_SetRegister(6,(int)Convert2Modbus(ADS1115_ADDRESS_ADDR_SCL_BOARD.ADS1115_CH1.data));
		   ModBus_SetRegister(7,(int)Convert2Modbus(ADS1115_ADDRESS_ADDR_SCL_BOARD.ADS1115_CH2.data));

    osDelay(1);
  }
  /* USER CODE END ModbusSetThread */
}

/* USER CODE BEGIN Header_ADS1115Thread */
/**
* @brief Function implementing the ADS1115Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADS1115Thread */
void ADS1115Thread(void const * argument)
{
  /* USER CODE BEGIN ADS1115Thread */
  /* Infinite loop */
  for(;;)
  {

	  uint8_t ADS1115_ADDRESS_ADDR = 0;
	  for( int j = ADS1115_ADDRESS_ADDR_GND; j <= ADS1115_ADDRESS_ADDR_SCL; j++) {
		  	  switch(j)
			  {
				  case(ADS1115_ADDRESS_ADDR_GND):
					ADS1115_ADDRESS_ADDR = ADS1115_ADDRESS_ADDR_GND; // 10000001 CH0+ CH1
					Read_ads1115(&ADS1115_ADDRESS_ADDR_GND_BOARD, ADS1115_ADDRESS_ADDR);
					break;
				  case(ADS1115_ADDRESS_ADDR_VDD):
					ADS1115_ADDRESS_ADDR = ADS1115_ADDRESS_ADDR_VDD; // 10000001 CH1+ CH2
					Read_ads1115(&ADS1115_ADDRESS_ADDR_VDD_BOARD, ADS1115_ADDRESS_ADDR);
					break;
				  case(ADS1115_ADDRESS_ADDR_SDA):
					ADS1115_ADDRESS_ADDR = ADS1115_ADDRESS_ADDR_SDA; // 10000001 CH3+ CH4
					Read_ads1115(&ADS1115_ADDRESS_ADDR_SDA_BOARD, ADS1115_ADDRESS_ADDR);
					break;
				  case(ADS1115_ADDRESS_ADDR_SCL):
					ADS1115_ADDRESS_ADDR = ADS1115_ADDRESS_ADDR_SCL; // 10000001 CH5+ CH6
					Read_ads1115(&ADS1115_ADDRESS_ADDR_SCL_BOARD, ADS1115_ADDRESS_ADDR);
					break;
		  }
	  }
	  osDelay(1);
  }
  /* USER CODE END ADS1115Thread */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
