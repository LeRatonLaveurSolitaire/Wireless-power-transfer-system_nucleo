/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include "../../Drivers/kiss_fft/kiss_fft.h"
// #include "../../Drivers/kiss_fft/kiss_fftr.h"
#define ARM_MATH_CM4
#include "arm_math.h"

// #include <math.h>
#include "../../X-CUBE-AI/App/network.h"
#include "../../X-CUBE-AI/App/network_data.h"
#include "../../Drivers/u8g2/u8g2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define PI 3.14159265358979323846264338327
#define SIG_BUFF_LEN 4096
#define FFT_BUFF_LEN (SIG_BUFF_LEN / 2)
#define OFFSET 6        // offset between I and V (6e-6 in this case)
#define Te 0.000001     // Sampling period
#define L1 0.000024     // Primary coil impedance in H
#define R1 0.075        // Coil ESR in Ohm
#define C1 0.000000146  // Capacitor value in F
#define SQRT_L1L2_uH 24 // sqrt of L1 * L2 in µH
#define Vcc 10          // V bus voltage in volt

#define Qi 6.6 / 494 / 0.001 * 1.42 * 32768 / (4095) // Current quantization factor in ampere per bit
#define Qv Vcc * 32768 / (2000)                      // Voltage quantization factor in volt per bit

#define DEVICE_ADDRESS 0b0111100
#define TX_TIMEOUT 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t PHASE_SHIFT = 0;
volatile uint8_t PRBS_ACTIVE = 0;
volatile uint16_t PRBS_period = (1 << 10) - 1;
volatile uint8_t BP_STATE_OLD = 0;
volatile uint8_t PRBS_AMPLITUDE = 20;
volatile uint8_t NOISE_ACTIVE = 0;
volatile uint8_t COMPUTE_ACTIVE = 0;

int16_t *raw_current_sig;
int16_t *raw_voltage_sig;

uint16_t index_list[] = {205, 221, 238, 257, 277, 299, 323, 348, 376, 405, 437, 471, 509, 549, 592};
uint8_t filtering_low_index_list[] = {11, 12, 13, 14, 15, 16, 18, 19, 21, 22, 24, 26, 28, 31, 33};
uint8_t filtering_high_index_list[] = {12, 13, 14, 15, 16, 17, 19, 20, 22, 24, 26, 28, 30, 32, 35};

arm_rfft_fast_instance_f32 fft_handler; // 32 floating point fft

/*################# AI declaration ################### */
/* Global handle to reference the instantiated C-model */
static ai_handle network = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
static ai_float in_data[AI_NETWORK_IN_1_SIZE];
/* or static ai_u8 in_data[AI_NETWORK_IN_1_SIZE_BYTES]; */

/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
static ai_float out_data[AI_NETWORK_OUT_1_SIZE];
/*or static ai_u8 out_data[AI_NETWORK_OUT_1_SIZE_BYTES]; */

/* Array of pointer to manage the model's input/output tensors */
static ai_buffer *ai_input;
static ai_buffer *ai_output;

static u8g2_t u8g2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

static int8_t PRBS();

int aiInit(void);
int aiRun(const void *in_data, void *out_data);

float_t delinearize_R(float_t);
float_t delinearize_M(float_t);

u8x8_msg_cb u8x8_stm32_gpio_and_delay(u8x8_t *, uint8_t, uint8_t, void *);
u8x8_msg_cb u8x8_byte_stm32_hw_i2c(u8x8_t *, uint8_t, uint8_t, void *);

void display_oled(float_t, float_t);

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc3, POTENTIOMETER_Pin);

  __HAL_RCC_CRC_CLK_ENABLE();

  arm_rfft_fast_init_f32(&fft_handler, SIG_BUFF_LEN);

  aiInit();

  u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, (u8x8_msg_cb)u8x8_byte_stm32_hw_i2c, (u8x8_msg_cb)u8x8_stm32_gpio_and_delay); // init u8g2 structure
  u8g2_InitDisplay(&u8g2);                                                                                                             // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&u8g2, 0);                                                                                                         // wake up display
  u8g2_ClearDisplay(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_6x13_mf);
  do
  {
    u8g2_DrawStr(&u8g2, 6, 14, "Press USER to start");
    u8g2_DrawFrame(&u8g2, 50 - 12, 35 - 12, 25, 25);
    u8g2_DrawCircle(&u8g2, 50, 35, 8, U8G2_DRAW_ALL);
    u8g2_DrawLine(&u8g2, 68, 35, 95, 35);
    u8g2_DrawLine(&u8g2, 68, 35, 78, 30);
    u8g2_DrawLine(&u8g2, 68, 35, 78, 40);
    u8g2_DrawStr(&u8g2, 39, 60, "USER");
  } while (u8g2_NextPage(&u8g2));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // State machine to control the identification prcedure
    if (!PRBS_ACTIVE)
    {
      // Read Potentiometer to set phase shift
      HAL_ADC_Start(&hadc3);
      HAL_ADC_PollForConversion(&hadc3, 1);
      uint16_t ADC_VAL = HAL_ADC_GetValue(&hadc3);
      PHASE_SHIFT = ADC_VAL * 100 / (1 << 12);
      HAL_Delay(1);
    }

    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) & !BP_STATE_OLD & !COMPUTE_ACTIVE)
    {

      u8g2_ClearDisplay(&u8g2);
      u8g2_SetFont(&u8g2, u8g2_font_6x13_mf);
      do
      {
        u8g2_DrawStr(&u8g2, 5, 30, "Computing parameters");
        u8g2_DrawStr(&u8g2, 24, 50, "Please wait...");
      } while (u8g2_NextPage(&u8g2));

      raw_current_sig = (int16_t *)malloc((SIG_BUFF_LEN + OFFSET + 15) * sizeof(int16_t));
      raw_voltage_sig = (int16_t *)malloc((SIG_BUFF_LEN + OFFSET + 15) * sizeof(int16_t));
      PRBS_ACTIVE = 1;
      BP_STATE_OLD = 1;
    }
    else if ((!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) & (BP_STATE_OLD))
    {

      BP_STATE_OLD = 0;
    }
    if (COMPUTE_ACTIVE)
    {

      /* 			Signals	Scaling			*/
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);

      float_t *current_sig = (float_t *)malloc((SIG_BUFF_LEN + OFFSET + 15) * sizeof(float_t));
      float_t *voltage_sig = (float_t *)malloc((SIG_BUFF_LEN + OFFSET + 15) * sizeof(float_t));

      arm_q15_to_float(raw_current_sig, current_sig, SIG_BUFF_LEN + OFFSET + 15);
      free(raw_current_sig);
      arm_q15_to_float(raw_voltage_sig, voltage_sig, SIG_BUFF_LEN + OFFSET + 15);
      free(raw_current_sig);
      arm_scale_f32(current_sig, Qi, current_sig, SIG_BUFF_LEN + OFFSET + 15);
      arm_scale_f32(voltage_sig, Qv, voltage_sig, SIG_BUFF_LEN + OFFSET + 15);

      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);

      /* 			Send the signals via UART		*/

//		char message[100];
//		sprintf(message, "index,nc,nv\n");
//		HAL_UART_Transmit(&huart2,(uint8_t *) message, strlen(message), 100);
//		for(uint16_t i = 0; i <SIG_BUFF_LEN; i++){
//			sprintf(message, "%d,%f,%f\n",i, current_sig[i+OFFSET+5],voltage_sig[i+5]);
//			// sprintf(message, "%d,%ld,%ld\n",i, raw_current_sig[i+OFFSET+5],raw_voltage_sig[i+5]);
//			HAL_UART_Transmit(&huart2, (uint8_t *) message, strlen(message), 100);
//		}

      /* 			Compute FFT of the signals			*/

      float_t *fft_current = (float_t *)malloc((SIG_BUFF_LEN) * sizeof(float_t));
      arm_rfft_fast_f32(&fft_handler, (float_t *)current_sig + OFFSET + 5, fft_current, 0);
      free(current_sig);

      float_t *fft_voltage = (float_t *)malloc((SIG_BUFF_LEN) * sizeof(float_t));
      arm_rfft_fast_f32(&fft_handler, (float_t *)voltage_sig + 5, fft_voltage, 0);
      free(voltage_sig);

      /* 			Computing of the raw impedance			*/

      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);

      float_t *impedance = (float_t *)malloc((2 * FFT_BUFF_LEN) * sizeof(float_t));

      for (int i = 0; i < FFT_BUFF_LEN; i++)
      {
        float_t v_r = fft_voltage[2 * i];
        float_t v_i = fft_voltage[2 * i + 1];
        float_t c_r = fft_current[2 * i];
        float_t c_i = fft_current[2 * i + 1];
        float_t norm = c_r * c_r + c_i * c_i;
        impedance[2 * i] = (v_r * c_r + v_i * c_i) / norm;
        impedance[2 * i + 1] = (v_i * c_r - v_r * c_i) / norm;
      }

      free(fft_current);
      free(fft_voltage);

      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);

      /* 				Send the impedance via UART			*/

//		HAL_Delay(500);
//		sprintf(message, "index,impedance_r,impedance_i\n");
//		HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);
//		for (uint16_t i = 0; i < FFT_BUFF_LEN; i++)
//		{
//		  sprintf(message, "%d,%f,%f\n", i, impedance[2*i], impedance[2*i+1]);
//		  HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);
//		}

      /* 				Compute the input tensor				*/

      for (uint16_t i = 0; i < 15; i++)
      {
        in_data[i * 2] = 0;
        in_data[i * 2 + 1] = 0;
        for (int j = index_list[i] - filtering_low_index_list[i]; j < index_list[i] + filtering_high_index_list[i] + 1; j++)
        {
          float_t mag;
          arm_sqrt_f32(impedance[2 * j] * impedance[2 * j] + impedance[2 * j + 1] * impedance[2 * j + 1], &mag);
          in_data[i * 2] += mag;
          in_data[i * 2 + 1] += (impedance[2 * j] == 0) ? PI * (signbit((float_t)impedance[2 * j + 1]) * -1) : atan2(impedance[2 * j + 1], impedance[2 * j]);
        }
        in_data[i * 2] /= (filtering_low_index_list[i] + filtering_high_index_list[i] + 1);
        in_data[i * 2 + 1] /= (filtering_low_index_list[i] + filtering_high_index_list[i] + 1);
      }
      free(impedance);

      /* 			Send the input tensor via UART				*/

//		HAL_Delay(500);
//		sprintf(message, "index,value\n");
//		HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);
//		for (uint16_t i = 0; i < 30; i++)
//		{
//		  sprintf(message, "%d,%f\n", i , in_data[i]);
//		  HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);
//
//		}

      /* 			Compute neural network inference			*/
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
      aiRun(in_data, out_data);

      float_t R_lin = out_data[0];
      float_t M_lin = out_data[1];

      float_t R = delinearize_R(R_lin);
      float_t M = delinearize_M(M_lin);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
      display_oled(R, M);

      COMPUTE_ACTIVE = 0;
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_CC1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_1;
  sConfig.Offset = 2325;
  sConfig.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
  sConfig.OffsetSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_CC1;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_1;
  sConfig.Offset = 2040;
  sConfig.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
  sConfig.OffsetSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
   */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
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
  hi2c1.Init.Timing = 0x108031A0;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 169;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
  // HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (PRBS_ACTIVE)
    {
      int8_t noise = PRBS();
      htim->Instance->CCR2 = PHASE_SHIFT + noise * PRBS_AMPLITUDE;
    }
    else
      htim->Instance->CCR2 = PHASE_SHIFT;
  }
}

int8_t PRBS()
{
  static uint16_t current_val = 1;
  static uint16_t counter = 0;
  current_val = ((current_val << 1) | (((current_val >> 9) ^ (current_val >> 6)) & 0x1)) & 0x3FF;
  counter += 1;

  if (counter == PRBS_period - 2)
  {
    //		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
    NOISE_ACTIVE = 1;
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
    HAL_ADC_Start_DMA(&hadc1, (int16_t *)raw_current_sig, SIG_BUFF_LEN + OFFSET + 15);
    HAL_ADC_Start_DMA(&hadc2, (int16_t *)raw_voltage_sig, SIG_BUFF_LEN + OFFSET + 15);
  }
  if (counter == 2 * PRBS_period)
  {
    counter = 0;
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim1);
    PRBS_ACTIVE = 0;
    NOISE_ACTIVE = 0;
    //		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
    COMPUTE_ACTIVE = 1;
  }

  return (current_val & 0x1) * 2 - 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  HAL_ADC_Stop_DMA(hadc);
}

int aiInit(void)
{
  ai_error err;

  /* Create and initialize the c-model */
  const ai_handle acts[] = {activations};
  err = ai_network_create_and_init(&network, acts, NULL);
  if (err.type != AI_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Reteive pointers to the model's input/output tensors */
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);

  return 0;
}

int aiRun(const void *in_data, void *out_data)
{
  ai_i32 n_batch;
  ai_error err;

  /* 1 - Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(in_data);
  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* 2 - Perform the inference */
  n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
  if (n_batch != 1)
  {
    err = ai_network_get_error(network);
    return err.code;
  };

  return 0;
}

float_t delinearize_R(float_t R_lin)
{
  float_t R = pow(10, R_lin * 0.1);
  return R;
}

float_t delinearize_M(float_t M_lin)
{
  float_t M = pow(10, (M_lin * 0.1)) * (0.1 * SQRT_L1L2_uH);
  return M;
}

u8x8_msg_cb u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    HAL_Delay(1);
    break;
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    break;
  case U8X8_MSG_GPIO_DC:
    break;
  case U8X8_MSG_GPIO_RESET:
    break;
  }
  return (u8x8_msg_cb)1;
}

u8x8_msg_cb u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  /* add extra byte for the i2c address */
  static uint8_t buffer[34];
  static uint8_t buf_idx;
  uint8_t *data;
  switch (msg)
  {
  case U8X8_MSG_BYTE_SEND:
    data = (uint8_t *)arg_ptr;
    while (arg_int > 0)
    {
      buffer[buf_idx++] = *data;
      data++;
      arg_int--;
    }

    break;
  case U8X8_MSG_BYTE_INIT:
    /* add your custom code to init i2c subsystem */
    break;
  case U8X8_MSG_BYTE_SET_DC:
    /* ignored for i2c */
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    buf_idx = 0;
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    while (HAL_I2C_STATE_READY != hi2c1.State)
      ;
    HAL_I2C_Master_Transmit(&hi2c1, DEVICE_ADDRESS << 1, (uint8_t *)buffer, buf_idx, HAL_TIMEOUT);

    break;
  default:
    return 0;
  }
  return (u8x8_msg_cb)1;
}

void display_oled(float_t R, float_t M)
{
  if ((R < 999) & (R > 0) & (M > 0) & (M < 999))
  {
    char line_1[20];
    char line_2[20];
    sprintf(line_1, "R_l : %.2f Ohm", R);
    sprintf(line_2, "M   : %.2f uH", M);
    u8g2_ClearDisplay(&u8g2);

    do
    {
      u8g2_SetFont(&u8g2, u8g2_font_7x14_mf);
      u8g2_DrawStr(&u8g2, 0, 30, "Estimation :");
      u8g2_DrawStr(&u8g2, 0, 50 - 6, line_1);
      u8g2_DrawStr(&u8g2, 0, 64 - 6, line_2);
      u8g2_SetFont(&u8g2, u8g2_font_6x13_mf);
      u8g2_DrawStr(&u8g2, 0, 14, "Press USER to restart");

    } while (u8g2_NextPage(&u8g2));
  }
  else
  {
    u8g2_ClearDisplay(&u8g2);
    do
    {
      u8g2_SetFont(&u8g2, u8g2_font_7x14_mf);
      u8g2_DrawStr(&u8g2, 40, 30, "Error...");
      u8g2_DrawStr(&u8g2, 10, 50 - 6, "Estimated values");
      u8g2_DrawStr(&u8g2, 25, 64 - 6, "out of range");
      u8g2_SetFont(&u8g2, u8g2_font_6x13_mf);
      u8g2_DrawStr(&u8g2, 0, 14, "Press USER to restart");

    } while (u8g2_NextPage(&u8g2));
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(500);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(250);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ENABLE);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, DISABLE);
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
