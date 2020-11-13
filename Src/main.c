/* USER CODE BEGIN Header */
/**
     ******************************************************************************
     * @file					 : main.c
     * @brief					: Main program body
     ******************************************************************************
     * @attention
     *
     * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
     * All rights reserved.</center></h2>
     *
     * This software component is licensed by ST under Ultimate Liberty license
     * SLA0044, the "License"; You may not use this file except in compliance with
     * the License. You may obtain a copy of the License at:
     *														 www.st.com/SLA0044
     *
     ******************************************************************************
     */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sht3x.h"
#include "STM_MY_LCD16X2.h"
#include "bmp085.h"
#include "BH1750.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define sensorsLot 8
#define circularBufferLength 10
#define circularBufferLot 10

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*
     ++++++++++Pines para el Display 16x2 en la STM32F4 con la librer�a HAL+++++++++

                     E --> PE1
                     RS --> PE0
                     D7 --> PD11
                     D6 --> PD10
                     D5 --> PD9
                     D4 --> PD8
     */

/*
     ++++++++++Pines para la SD_Card en la STM32F4 con la librer�a HAL++++++++++++++
                     SDCS --> -
                     MOSI --> PD2
                     SCK --> PC12
                     MISO --> PC8
     */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float temperature = 0.0;
float humidity = 0.0;
int lumen = 0;
uint32_t adc1 = 0;
uint32_t adc2 = 0;
uint32_t adc3 = 0;
float BMP085_temp_K;
long BMP085_press_K;
float BH1750_lux;
FATFS myFATAFS;
FIL myFILE;
UINT testByte;
char myData[] = "Hello Worldasdas\0";
char myPath[] = "WRITE1.TXT\0";
int btncounter = 0;
int lcdcounter = 7;
int lcdcounterCallback = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Read_Adc(ADC_Channel_t channel);
void USART_Puts(USART_TypeDef *USARTx, char *str);

//BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

long map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float circularBuffer[circularBufferLot][circularBufferLength];
int circularBufferIndex[circularBufferLot] = {0};
int circularBufferFlag[circularBufferLot] = {0};
float minValue[circularBufferLot] = {999999999};
float maxValue[circularBufferLot] = {-999999999};


void initMinMax () {
	    for (int i = 0; i < circularBufferLot; i++)
      {
         minValue[i] = 999999999;
				maxValue[i] = -999999999;
      }
}

void appendToBuffer(int bufferID, float value)
{

   if (circularBufferIndex[bufferID] >= circularBufferLength)
   {
      circularBufferIndex[bufferID] = 0;
      if (!circularBufferFlag[bufferID])
         circularBufferFlag[bufferID] = 1;
   }
   circularBufferIndex[bufferID]++;

   circularBuffer[bufferID][circularBufferIndex[bufferID]] = value;
}

// MM =  moving mean -> moving average
float appendToBufferAndGetMM(int bufferID, float value)
{

   if (circularBufferIndex[bufferID] >= circularBufferLength)
   {
      circularBufferIndex[bufferID] = 0;
      if (!circularBufferFlag[bufferID])
         circularBufferFlag[bufferID] = 1;
   }
   circularBuffer[bufferID][circularBufferIndex[bufferID]] = value;

   float sum = 0;
   float result = 0;
   if (circularBufferFlag[bufferID] == 1)
   {
      for (int i = 0; i < circularBufferLength; i++)
      {
         sum += circularBuffer[bufferID][i];
      }
      circularBufferIndex[bufferID]++;
      result = sum / circularBufferLength;
   }
   else
   {
      for (int i = 0; i < circularBufferIndex[bufferID] + 1; i++)
      {
         sum += circularBuffer[bufferID][i];
      }
      circularBufferIndex[bufferID]++;
      result = sum / (circularBufferIndex[bufferID]);
   }
   if (result > maxValue[bufferID])
      maxValue[bufferID] = result;
   if (result < minValue[bufferID])
      minValue[bufferID] = result;
   return result;
}



float sensorValue[sensorsLot] = {0};
char *sensorName[] = {
    "Temperature",     //0 - Temperature SHT3X
    "Humidity",        //1 - Humidity SHT3X
    "Temperature 2",   //2 - BMP180 Temperature
    "Pressure",        //3 - BMP180 Pressure
    "Altitude",        //4 - BMP180 Altitude
    "Light Intensity", //5 - BH1750  Light Intensity
    "MQ7",             //6 - MQ7
    "MQ135"            //7 - MQ135
};

int errorCode = 0;
char *errorString[] = {"All Ok", "Not SD"};

/* USER CODE END 0 */

/**
   * @brief	The application entry point.
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
   MX_SDIO_SD_Init();
   MX_FATFS_Init();
   MX_ADC1_Init();
   MX_I2C1_Init();
   MX_I2C2_Init();
   MX_I2C3_Init();
   MX_USART2_UART_Init();
   MX_TIM2_Init();
   BH1750_Init(&hi2c2);
   BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);
   /* USER CODE BEGIN 2 */

   //Timer2
   HAL_TIM_Base_Start_IT(&htim2);

   //BMP085Calibration(&hi2c1);
   bmp_t bmp;
   bmp_init(&bmp);
   float altitud = 0.0;
   float temp = 0.0;
	initMinMax ();
   //Inicialización del Display 16x2
   LCD1602_Begin4BIT(RS_GPIO_Port, RS_Pin, E_Pin, D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);

   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   char buff[250];
   //Guardar datos en un .txt dentro de la SD_Card
   if (f_mount(&myFATAFS, SDPath, 1) == FR_OK)
   {
     HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);     
     f_open(&myFILE, myPath, FA_WRITE | FA_CREATE_ALWAYS);
		 	memset(buff, 0, sizeof buff);		
		 sprintf(buff, "Min\t%s\tMax\tMin\t%s\tMax\tMin\t%s\tMax\tMin\t%s\tMax\tMin\t%s\tMax\tMin\t%s\tMax\tMin\t%s\tMax\r\n", sensorName[0], sensorName[1],  sensorName[3], sensorName[4], sensorName[5],  sensorName[6], sensorName[7]);		

     f_write(&myFILE, buff, sizeof(buff), &testByte);
     f_close(&myFILE);
   }
   else
   {
      errorCode = 1;
   }


   int count = 0;
	 int flag = 0;
	 int flag_comPlotter = 1;
   LCD1602_noBlink();
   while (1)
   {

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

      if (count % 100 == 0 && flag == 1)
      {
         if (f_mount(&myFATAFS, SDPath, 1) == FR_OK)
         {
            f_open(&myFILE, myPath, FA_OPEN_APPEND | FA_WRITE);       
						memset(buff, 0, sizeof buff);					 
            
						sprintf(buff, "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n",  minValue[0],sensorValue[0],maxValue[0],minValue[1],sensorValue[1],maxValue[1],minValue[3],sensorValue[3],maxValue[3],minValue[4],sensorValue[4],maxValue[4],minValue[5],sensorValue[5],maxValue[5], minValue[6],sensorValue[6],maxValue[6],minValue[7],sensorValue[7],maxValue[7]);
            f_write(&myFILE, buff, sizeof(buff), &testByte);
            f_close(&myFILE);
         }
      }

      if (count % 10 == 0 || lcdcounterCallback == 1)
      {
				 int id=0;
				 lcdcounterCallback = 0;
         char displaybuffer[18];
				 LCD1602_clear();
				
				 if (lcdcounter == 0) 	{
					flag_comPlotter = 1;
					sprintf(displaybuffer, "TDII - UTN FRBB ");
					LCD1602_1stLine();
					LCD1602_print(displaybuffer);

					sprintf(displaybuffer, "Diaz - Kremer ");
					LCD1602_2ndLine();
					LCD1602_print(displaybuffer);						 
				 } else {
					 flag_comPlotter = 0;
					sprintf(displaybuffer, "%s :  ", sensorName[lcdcounter-1]);
					LCD1602_1stLine();
					LCD1602_print(displaybuffer);

					sprintf(displaybuffer, "%0.4lf ", sensorValue[lcdcounter-1]);
					LCD1602_2ndLine();
					LCD1602_print(displaybuffer);			
				 }	
				 if(flag==0)flag=1;
      }
      else
      {

         //Imprimir en el Display

         /************ SHT3X ********************/
         SHT3X_Update();
				 sensorValue[0] = appendToBufferAndGetMM(0,SHT3X_getTemperature());
				 sensorValue[1] = appendToBufferAndGetMM(1,SHT3X_getHumidity());
       

         /************ BMP180 ********************/
         bmp.uncomp.temp = get_ut();
         bmp.data.temp = get_temp(&bmp);
         bmp.uncomp.press = get_up(bmp.oss);
         bmp.data.press = get_pressure(bmp);
         bmp.data.altitude = get_altitude(&bmp);

				 sensorValue[2] = appendToBufferAndGetMM(2, bmp.data.temp);
					float temp =  bmp.data.press;
					temp /= 100;
				 sensorValue[3] = appendToBufferAndGetMM(3, temp);
				 sensorValue[4] = appendToBufferAndGetMM(4,bmp.data.altitude);

         /************ BH1750_lux ********************/
         if (BH1750_OK == BH1750_ReadLight(&BH1750_lux))
         {
  					 
					 			sensorValue[5] = appendToBufferAndGetMM(5,BH1750_lux *2);
         }

         /************ MQs ********************/
         //adc1 = Read_Adc(ADC_Channel_11);
/*
				 temp =  Read_Adc(ADC_Channel_12)*3.0/4096;
				 temp = (3.0-temp)/temp;
				 temp =  (19.32  * pow(temp, -0.64));*/
				 //temp = map ( Read_Adc(ADC_Channel_13) -55 ,0,4096,400,5000);
		
				 	sensorValue[6] = appendToBufferAndGetMM(6, Read_Adc(ADC_Channel_12)); //MQ-7
			
				 	sensorValue[7] = appendToBufferAndGetMM(7,Read_Adc(ADC_Channel_11)); //MQ-135

			if ( lcdcounter == 0)
				sprintf(buff, "\r%.2f \t %.2f \t %.2f \t%.2f \t%.2f \t%.2f \t%.2f \n", sensorValue[0], sensorValue[1],  sensorValue[3]/1000, sensorValue[5], sensorValue[6], sensorValue[7]);
			else{
				if( (lcdcounter-1) ==3)
						sprintf(buff, "\r %.2f\t \n", sensorValue[lcdcounter-1]/1000);
				else {
							sprintf(buff, "\r %.2f\t \n", sensorValue[lcdcounter-1]);
				}
			}
				
				
        
				USART_Puts(USART2, buff);
      }

      count++;
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
   RCC_OscInitStruct.PLL.PLLQ = 7;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
      Error_Handler();
   }
   /** Initializes the CPU, AHB and APB busses clocks
   */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
   * @brief ADC1 Initialization Function
   * @param None
   * @retval None
   */
static void MX_ADC1_Init(void)
{

   /* USER CODE BEGIN ADC1_Init 0 */

   /* USER CODE END ADC1_Init 0 */

   ADC_ChannelConfTypeDef sConfig = {0};

   /* USER CODE BEGIN ADC1_Init 1 */

   /* USER CODE END ADC1_Init 1 */
   /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
   hadc1.Instance = ADC1;
   hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
   hadc1.Init.Resolution = ADC_RESOLUTION_12B;
   hadc1.Init.ScanConvMode = DISABLE;
   hadc1.Init.ContinuousConvMode = DISABLE;
   hadc1.Init.DiscontinuousConvMode = DISABLE;
   hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
   hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
   hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
   hadc1.Init.NbrOfConversion = 1;
   hadc1.Init.DMAContinuousRequests = DISABLE;
   hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
   if (HAL_ADC_Init(&hadc1) != HAL_OK)
   {
      Error_Handler();
   }
   /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
   sConfig.Channel = ADC_CHANNEL_11;
   sConfig.Rank = 1;
   sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN ADC1_Init 2 */

   /* USER CODE END ADC1_Init 2 */
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
   * @brief I2C2 Initialization Function
   * @param None
   * @retval None
   */
static void MX_I2C2_Init(void)
{

   /* USER CODE BEGIN I2C2_Init 0 */

   /* USER CODE END I2C2_Init 0 */

   /* USER CODE BEGIN I2C2_Init 1 */

   /* USER CODE END I2C2_Init 1 */
   hi2c2.Instance = I2C2;
   hi2c2.Init.ClockSpeed = 100000;
   hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
   hi2c2.Init.OwnAddress1 = 0;
   hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
   hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
   hi2c2.Init.OwnAddress2 = 0;
   hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
   hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
   if (HAL_I2C_Init(&hi2c2) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN I2C2_Init 2 */

   /* USER CODE END I2C2_Init 2 */
}

/**
   * @brief I2C3 Initialization Function
   * @param None
   * @retval None
   */
static void MX_I2C3_Init(void)
{

   /* USER CODE BEGIN I2C3_Init 0 */

   /* USER CODE END I2C3_Init 0 */

   /* USER CODE BEGIN I2C3_Init 1 */

   /* USER CODE END I2C3_Init 1 */
   hi2c3.Instance = I2C3;
   hi2c3.Init.ClockSpeed = 100000;
   hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
   hi2c3.Init.OwnAddress1 = 0;
   hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
   hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
   hi2c3.Init.OwnAddress2 = 0;
   hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
   hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
   if (HAL_I2C_Init(&hi2c3) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN I2C3_Init 2 */

   /* USER CODE END I2C3_Init 2 */
}

/**
   * @brief SDIO Initialization Function
   * @param None
   * @retval None
   */
static void MX_SDIO_SD_Init(void)
{

   /* USER CODE BEGIN SDIO_Init 0 */

   /* USER CODE END SDIO_Init 0 */

   /* USER CODE BEGIN SDIO_Init 1 */

   /* USER CODE END SDIO_Init 1 */
   hsd.Instance = SDIO;
   hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
   hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
   hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
   hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
   hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
   hsd.Init.ClockDiv = 0;
   /* USER CODE BEGIN SDIO_Init 2 */

   /* USER CODE END SDIO_Init 2 */
}

/**
   * @brief TIM2 Initialization Function
   * @param None
   * @retval None
   */
static void MX_TIM2_Init(void)
{

   /* USER CODE BEGIN TIM2_Init 0 */

   /* USER CODE END TIM2_Init 0 */

   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};

   /* USER CODE BEGIN TIM2_Init 1 */

   /* USER CODE END TIM2_Init 1 */
   htim2.Instance = TIM2;
   htim2.Init.Prescaler = 16000;
   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim2.Init.Period = 1000;
   htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
   {
      Error_Handler();
   }
   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
   {
      Error_Handler();
   }
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN TIM2_Init 2 */

   /* USER CODE END TIM2_Init 2 */
}

/**
   * @brief USART2 Initialization Function
   * @param None
   * @retval None
   */
static void MX_USART2_UART_Init(void)
{

   /* USER CODE BEGIN USART2_Init 0 */

   /* USER CODE END USART2_Init 0 */

   /* USER CODE BEGIN USART2_Init 1 */

   /* USER CODE END USART2_Init 1 */
   huart2.Instance = USART2;
   huart2.Init.BaudRate = 9600;
   huart2.Init.WordLength = UART_WORDLENGTH_8B;
   huart2.Init.StopBits = UART_STOPBITS_1;
   huart2.Init.Parity = UART_PARITY_NONE;
   huart2.Init.Mode = UART_MODE_TX_RX;
   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
   if (HAL_UART_Init(&huart2) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN USART2_Init 2 */

   /* USER CODE END USART2_Init 2 */
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
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOE_CLK_ENABLE();

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOD, D4_Pin | D5_Pin | D6_Pin | D7_Pin | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOE, RS_Pin | E_Pin, GPIO_PIN_RESET);

   /*Configure GPIO pins : PA0 PA1 */
   GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*Configure GPIO pin : PA4 */
   GPIO_InitStruct.Pin = GPIO_PIN_4;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin
                PD12 PD13 PD14 PD15 */
   GPIO_InitStruct.Pin = D4_Pin | D5_Pin | D6_Pin | D7_Pin | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

   /*Configure GPIO pins : RS_Pin E_Pin */
   GPIO_InitStruct.Pin = RS_Pin | E_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

   /* EXTI interrupt init*/
   HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/* USER CODE BEGIN 4 */

//Interrupción Externa PA4 - Pulsador
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   //Prevent unused argument(s) compilation warning
   UNUSED(GPIO_Pin);

   //Delay para evitar el rebote del pulsador
   //for(uint32_t i=0; i<1000000; i++);
   //HAL_Delay(20);

   if (btncounter % 2 == 0)
   {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
      lcdcounter++;
      lcdcounterCallback = 1;

      if (lcdcounter > sensorsLot)
         lcdcounter = 0;
   }
   for (uint32_t i = 0; i < 20000000; i++);
	 
   btncounter++;
}

//Timer2
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

   /* Prevent unused argument(s) compilation warning */
   UNUSED(htim);

   /* NOTE : This function should not be modified, when the callback is needed,
        the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
    */
   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
}

uint16_t Read_Adc(ADC_Channel_t channel)
{
   ADC_ChannelConfTypeDef sConfig;

   /* Configure ADC regular channel */
   sConfig.Channel = (uint8_t)channel;
   sConfig.Rank = 1;

   sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
   sConfig.Offset = 0;

   /* Set handle */
   hadc1.Instance = ADC1;

   /* Return zero */
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
   {
      return 0;
   }

   /* Start conversion */
   if (HAL_ADC_Start(&hadc1) != HAL_OK)
   {
      return 0;
   }

   /* Poll for end */
   HAL_ADC_PollForConversion(&hadc1, 1000);

   /* Check if the continous conversion of regular channel is finished */
   if (HAL_ADC_GetState(&hadc1) == 0x201)
   {

      // Get the converted value of regular channel
      return HAL_ADC_GetValue(&hadc1);
   }

   /* Return zero */
   return 0;
}

void USART_Puts(USART_TypeDef *USARTx, char *str)
{
   /* Go through entire string */
   while (*str)
   {
      /* Wait to be ready, buffer empty */
      USART_WAIT(USARTx);
      /* Send data */
      USART_WRITE_DATA(USARTx, (uint16_t)(*str++));
      /* Wait to be ready, buffer empty */
      USART_WAIT(USARTx);
   }
}

/* USER CODE END 4 */

/**
   * @brief	This function is executed in case of error occurrence.
   * @retval None
   */
void Error_Handler(void)
{
   /* USER CODE BEGIN Error_Handler_Debug */
   /* User can add his own implementation to report the HAL error return state */

   /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
   * @brief	Reports the name of the source file and the source line number
   *				 where the assert_param error has occurred.
   * @param	file: pointer to the source file name
   * @param	line: assert_param error line source number
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
