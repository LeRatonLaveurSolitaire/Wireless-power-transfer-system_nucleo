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
#include  <stdint.h>
#include  <stdio.h>
#include  <string.h>
#include "../../Drivers/kiss_fft/kiss_fft.h"
#include "../../Drivers/kiss_fft/kiss_fftr.h"
#include  <math.h>
//#include "../../X-CUBE-AI/App/network.h"
//#include "../../X-CUBE-AI/App/network_data.h"
//#include "../../Drivers/u8g2/u8g2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265358979323846264338327
#define SIG_BUFF_LEN 4000
#define FFT_BUFF_LEN (SIG_BUFF_LEN/2 + 1)
#define TAU 0.000006 				// Time delay impacting the phase (6e-6 in this case)
#define Te 0.000001					// Sampling period
#define smoothing_factor 1.04		// Filter coefficient
#define L1 0.000024					// Coil impedance in H
#define R1 0.075					// Coil ESR in Ohm
#define C1 0.000000146				// Capacitor value in F
#define Vcc	10						// V bus voltage in volt
#define Q 1.65/1023 /500 /0.001		// Quantization factor in ampere per step
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
DMA_HandleTypeDef hdma_i2c1_tx;

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

int16_t* noisy_current_sig;
int16_t* noisy_voltage_sig;


///*################# AI declaration ################### */
///* Global handle to reference the instantiated C-model */
//static ai_handle network = AI_HANDLE_NULL;
//
///* Global c-array to handle the activations buffer */
//AI_ALIGNED(32)
//static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
//
///* Array to store the data of the input tensor */
//AI_ALIGNED(32)
//static ai_float in_data[AI_NETWORK_IN_1_SIZE];
///* or static ai_u8 in_data[AI_NETWORK_IN_1_SIZE_BYTES]; */
//
///* c-array to store the data of the output tensor */
//AI_ALIGNED(32)
//static ai_float out_data[AI_NETWORK_OUT_1_SIZE];
///* static ai_u8 out_data[AI_NETWORK_OUT_1_SIZE_BYTES]; */
//
///* Array of pointer to manage the model's input/output tensors */
//static ai_buffer *ai_input;
//static ai_buffer *ai_output;
//
//
//static u8g2_t u8g2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static int8_t PRBS();

kiss_fft_cpx add_cpx(kiss_fft_cpx, kiss_fft_cpx);
kiss_fft_cpx sub_cpx(kiss_fft_cpx, kiss_fft_cpx);
kiss_fft_cpx mul_cpx(kiss_fft_cpx, kiss_fft_cpx);
kiss_fft_cpx div_cpx(kiss_fft_cpx, kiss_fft_cpx);
float_t mag_cpx(kiss_fft_cpx);
float_t ang_cpx(kiss_fft_cpx);

//int aiInit(void);
//int aiRun(const void *in_data, void *out_data);

//static u8x8_msg_cb u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
//static u8x8_msg_cb u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	//  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0,(u8x8_msg_cb) u8x8_byte_hw_i2c,(u8x8_msg_cb) u8x8_gpio_and_delay);  // init u8g2 structure
	//  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	//  u8g2_SetPowerSave(&u8g2, 0); // wake up display
	//  u8g2_SetFont(&u8g2, u8g2_font_7x14_mf);
	//  u8g2_ClearDisplay(&u8g2);
	//  u8g2_FirstPage(&u8g2);
	//  u8g2_DrawStr(&u8g2, 32, 32, "Hello World!");
	//  u8g2_send_buffer(&u8g2);
	//  do {
	//	  u8g2_SetFont(&u8g2, u8g2_font_u8glib_4_tf);
	//	  u8g2_DrawStr(&u8g2, 0, 15, "Hello World!");
	//	  u8g2_SendBuffer(&u8g2);
	//	  //u8g2_DrawCircle(&u8g2, 64, 40, 10, U8G2_DRAW_ALL);
	//  } while (u8g2_NextPage(&u8g2));

  HAL_ADCEx_Calibration_Start(&hadc3,POTENTIOMETER_Pin);

  __HAL_RCC_CRC_CLK_ENABLE();
//  aiInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		// State machine to control the identification prcedure
		if (!PRBS_ACTIVE){
			HAL_ADC_Start(&hadc3);
			// Poll ADC1 Perihperal & TimeOut = 1mSec
			HAL_ADC_PollForConversion(&hadc3, 1);
			// Read The ADC Conversion Result & Map It To PWM DutyCycle
			uint16_t ADC_VAL = HAL_ADC_GetValue(&hadc3);
			PHASE_SHIFT = ADC_VAL * 100 / (1 << 12);
			HAL_Delay(1);
		}

		if (HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) & !BP_STATE_OLD & !COMPUTE_ACTIVE){
			noisy_current_sig = (int16_t *)malloc(SIG_BUFF_LEN * sizeof(int16_t));
			noisy_voltage_sig = (int16_t *)malloc(SIG_BUFF_LEN * sizeof(int16_t));
			PRBS_ACTIVE = 1;
			BP_STATE_OLD = 1;
		}
		else if((!HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin)) & (BP_STATE_OLD)){
			// HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
			BP_STATE_OLD = 0;
		}
		if (COMPUTE_ACTIVE){

			/* 			Process the signals				*/

			int32_t noisy_mean = 0;
			int16_t low_threshold = -16;
			int16_t high_threshold = 16;
			for(uint16_t i=0;i < SIG_BUFF_LEN;i++){
				noisy_mean+=noisy_current_sig[i];

				if (noisy_voltage_sig[i] < low_threshold){
					noisy_voltage_sig[i]= Vcc;
				}
				else if (noisy_voltage_sig[i] > high_threshold){
					noisy_voltage_sig[i]= -Vcc;
				}
				else{
					noisy_voltage_sig[i]= 0;
				}
			}
			noisy_mean/=SIG_BUFF_LEN;
			for(uint16_t i=0;i < SIG_BUFF_LEN;i++){
				noisy_current_sig[i]-=noisy_mean;
			}

			/* 			Send the signals via UART		*/
			char message[100];
			sprintf(message, "index,nc,nv\n");
			HAL_UART_Transmit(&huart2,(uint8_t *) message, strlen(message), 100);
			for(uint16_t i = 0; i <SIG_BUFF_LEN; i++){
				sprintf(message, "%d,%d,%d\n",i,noisy_current_sig[i],noisy_voltage_sig[i]);
				HAL_UART_Transmit(&huart2, (uint8_t *) message, strlen(message), 100);
			}

			/*		Transform signals into float32		*/

			float_t *noisy_current_sig_float = (float_t *) malloc(SIG_BUFF_LEN * sizeof(float_t));

			if (noisy_current_sig_float == NULL) {
				Error_Handler();
			}

			for(uint16_t i=0;i <SIG_BUFF_LEN;i++){
				noisy_current_sig_float[i] = noisy_current_sig[i] * Q;
			}
			free(noisy_current_sig);

			float_t *noisy_voltage_sig_float = (float_t *) malloc(SIG_BUFF_LEN * sizeof(float_t));

			if (noisy_voltage_sig_float == NULL) {
				Error_Handler();
			}

			for(uint16_t i=0;i <SIG_BUFF_LEN;i++){
				noisy_voltage_sig_float[i] = noisy_voltage_sig[i];
			}
			free(noisy_voltage_sig);

			/* 			Compute FFT of the signals			*/

			size_t memlen = 40278;
			void* mem = malloc(memlen * sizeof(char));

			kiss_fftr_cfg cfg = kiss_fftr_alloc(SIG_BUFF_LEN, 0, mem, (size_t *)&memlen);

			kiss_fft_cpx * noisy_current_fft = (kiss_fft_cpx *) malloc(FFT_BUFF_LEN * sizeof(kiss_fft_cpx));

			if (noisy_current_fft == NULL) {
				Error_Handler();
			}

			kiss_fftr(cfg, noisy_current_sig_float, noisy_current_fft);

			free(noisy_current_sig_float);

			kiss_fft_cpx * noisy_voltage_fft = (kiss_fft_cpx *) malloc(FFT_BUFF_LEN * sizeof(kiss_fft_cpx));

			if (noisy_voltage_fft == NULL) {
				Error_Handler();
			}

			kiss_fftr(cfg, noisy_voltage_sig_float, noisy_voltage_fft);

			free(noisy_voltage_sig_float);

			free(cfg);

			/* 			Send the FFT signals via UART		*/

			HAL_Delay(500);
			sprintf(message, "index,cr,ci,vr,vi\n");
			HAL_UART_Transmit(&huart2,(uint8_t *) message, strlen(message), 100);
			for(uint16_t i = 0; i <FFT_BUFF_LEN; i++){
				sprintf(message, "%d,%f,%f,%f,%f\n",i,noisy_current_fft[i].r,noisy_current_fft[i].i,noisy_voltage_fft[i].r,noisy_voltage_fft[i].i);
				HAL_UART_Transmit(&huart2, (uint8_t *) message, strlen(message), 100);
			}

			/* 			Computing of the raw impedance			*/

			kiss_fft_cpx * sys_impedance = (kiss_fft_cpx *) malloc((FFT_BUFF_LEN-1) * sizeof(kiss_fft_cpx)); // -1 to avoid the 0Hz data

			for(uint16_t i=0;i <FFT_BUFF_LEN-1;i++){
				sys_impedance[i] = div_cpx(noisy_voltage_fft[i+1],noisy_current_fft[i+1]); // +1 for the 0Hz offset
			}

			free(noisy_current_fft);
			free(noisy_voltage_fft);

			/* 			Filtering of the impedance			*/

			for(uint16_t i=0;i <FFT_BUFF_LEN-1;i++){ //np.pi * 2 * sys_frequencies[i] * tau + 1j*np.pi
				kiss_fft_cpx phase_correction;
				float_t freq = (i+1) / (2 * Te * (FFT_BUFF_LEN - 1));
				phase_correction.r = cos(-2 * PI * freq * TAU + PI);
				phase_correction.i = sin(-2 * PI * freq * TAU + PI);
				sys_impedance[i] = mul_cpx(sys_impedance[i],phase_correction); // +1 for the 0Hz offset
			}

			/* 			Smoothing of the impedance			*/

			kiss_fft_cpx * smooth_sys_impedance = (kiss_fft_cpx *) malloc((FFT_BUFF_LEN-1) * sizeof(kiss_fft_cpx));

		    float_t *amplitudes = (float_t *) malloc((FFT_BUFF_LEN) * sizeof(float_t));
		    float_t *phases = (float_t *) malloc((FFT_BUFF_LEN) * sizeof(float_t));

		    amplitudes[0] = 0;
		    phases[0] = 0;

		    for (uint16_t i = 1; i < FFT_BUFF_LEN; i++) {
		    	amplitudes[i]=amplitudes[i-1] + mag_cpx(sys_impedance[i-1]);
		    	phases[i] = phases[i-1] + ang_cpx(sys_impedance[i-1]);
		    }

		    float_t phase_offset = 0;
		    uint16_t nbr_of_phases_added = 0;

			for (uint16_t i = 1; i < FFT_BUFF_LEN; i++) {
			    uint32_t low_bound = i;
			    uint32_t high_bound = i;

			    float_t current_freq = (i) / (2 * Te * (FFT_BUFF_LEN - 1));

			    while (((low_bound) / (2 * Te * (FFT_BUFF_LEN - 1)) >
			    		(current_freq / smoothing_factor)) &&
			    		(low_bound > 0)) {
			      low_bound--;
			    }
			    low_bound++;

			    while (((high_bound) / (2 * Te * (FFT_BUFF_LEN - 1)) <
			            (current_freq * smoothing_factor)) &&
			           (high_bound < FFT_BUFF_LEN-1)) {
			      high_bound++;
			    }
			    high_bound--;

			    float_t amplitude = 0;
			    float_t phase = 0;

			    amplitude = amplitudes[high_bound] - amplitudes[low_bound-1];
			    phase += phases[high_bound] - phases[low_bound-1];

			    amplitude /= (high_bound - low_bound + 1);
			    phase /= (high_bound - low_bound + 1);

			    smooth_sys_impedance[i-1].r = amplitude*cos(phase);
			    smooth_sys_impedance[i-1].i = amplitude*sin(phase);

			    if (current_freq > 75000 && current_freq < 95000){ // Used to center the phase plot in the 75kHz to 95kHz region
			    	phase_offset -= phase;
			    	nbr_of_phases_added ++;
			    }

			  }

			free(amplitudes);
			free(phases);

			/* 					Centring of the phase				*/
			phase_offset /= nbr_of_phases_added;

			for (uint16_t i = 0; i < FFT_BUFF_LEN-1; i++) {
				kiss_fft_cpx phase_correction;
				phase_correction.r = cos(phase_offset);
				phase_correction.i = sin(phase_offset);
				smooth_sys_impedance[i] = mul_cpx(smooth_sys_impedance[i],phase_correction);
			}


			/* 			Send the smoothed impedance via UART		*/

			HAL_Delay(500);
			sprintf(message, "index,impedance_r,impedance_i\n");
			HAL_UART_Transmit(&huart2,(uint8_t *) message, strlen(message), 100);
			for(uint16_t i = 0; i <FFT_BUFF_LEN-1; i++){
				sprintf(message, "%d,%f,%f\n",i,smooth_sys_impedance[i].r,smooth_sys_impedance[i].i);
				HAL_UART_Transmit(&huart2, (uint8_t *) message, strlen(message), 100);
			}


			free(sys_impedance);

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hadc2.Init.Resolution = ADC_RESOLUTION_6B;
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
  sConfig.Offset = 32;
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
  hi2c1.Init.Timing = 0x30A0A7FB;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
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
	HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim3,TIM_CHANNEL_2);
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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
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
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		if (PRBS_ACTIVE){
			int8_t State = PRBS();
			htim->Instance->CCR2 = PHASE_SHIFT + State*PRBS_AMPLITUDE;
		}
		else htim->Instance->CCR2 = PHASE_SHIFT;
	}
}


int8_t PRBS(){
	static uint16_t current_val = 1;
	static uint16_t counter = 0;
	current_val = ((current_val << 1) | (((current_val >> 9) ^ (current_val >> 6)) & 0x1)) & 0x3FF;
	counter +=1;

	if (counter == PRBS_period-2){
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		NOISE_ACTIVE =1;
		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_1);
		HAL_ADC_Start_DMA(&hadc1,(int16_t *) noisy_current_sig ,SIG_BUFF_LEN);
		HAL_ADC_Start_DMA(&hadc2,(int16_t *) noisy_voltage_sig ,SIG_BUFF_LEN);
	}
	if (counter == 2*PRBS_period){
		counter = 0;
		HAL_TIM_OC_Stop(&htim1,TIM_CHANNEL_1);
		HAL_TIM_Base_Stop(&htim1);
		PRBS_ACTIVE = 0;
		NOISE_ACTIVE = 0;
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		COMPUTE_ACTIVE =1;
	}

	return (current_val & 0x1) * 2 - 1;

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	HAL_ADC_Stop_DMA(hadc);
}

// Complex addition
kiss_fft_cpx add_cpx(kiss_fft_cpx c1, kiss_fft_cpx c2) {
  kiss_fft_cpx c_out;
  c_out.r = c1.r + c2.r;
  c_out.i = c1.i + c2.i;
  return c_out;
}

// Complex substraction
kiss_fft_cpx sub_cpx(kiss_fft_cpx c1, kiss_fft_cpx c2) {
  kiss_fft_cpx c_out;
  c_out.r = c1.r - c2.r;
  c_out.i = c1.i - c2.i;
  return c_out;
}

// Complex multiplication
kiss_fft_cpx mul_cpx(kiss_fft_cpx c1, kiss_fft_cpx c2) {
  kiss_fft_cpx c_out;
  c_out.r = c1.r * c2.r - c1.i * c2.i;
  c_out.i = c1.i * c2.r + c1.r * c2.i;
  return c_out;
}

// Complex division
kiss_fft_cpx div_cpx(kiss_fft_cpx c1, kiss_fft_cpx c2) {
  kiss_fft_cpx c_out;
  c_out.r = (c1.r * c2.r + c1.i * c2.i) / (c2.r * c2.r + c2.i * c2.i);
  c_out.i = (c1.i * c2.r - c1.r * c2.i) / (c2.r * c2.r + c2.i * c2.i);
  return c_out;
}

// Magnitude
float_t mag_cpx(kiss_fft_cpx c) { return sqrt(pow(c.r, 2) + pow(c.i, 2)); }

// Angle
float_t ang_cpx(kiss_fft_cpx c) {
  float_t angle;
  if (c.r == 0) {
    angle = M_PI * (signbit(c.i) * -1);
  } else {
    angle = atan2(c.i, c.r);
  }
  return angle;
}

//int aiInit(void) {
//  ai_error err;
//
//  /* Create and initialize the c-model */
//  const ai_handle acts[] = { activations };
//  err = ai_network_create_and_init(&network, acts, NULL);
//  if (err.type != AI_ERROR_NONE) { Error_Handler(); }
//
//  /* Reteive pointers to the model's input/output tensors */
//  ai_input = ai_network_inputs_get(network, NULL);
//  ai_output = ai_network_outputs_get(network, NULL);
//
//  return 0;
//}
//
//int aiRun(const void *in_data, void *out_data) {
//  ai_i32 n_batch;
//  ai_error err;
//
//  /* 1 - Update IO handlers with the data payload */
//  ai_input[0].data = AI_HANDLE_PTR(in_data);
//  ai_output[0].data = AI_HANDLE_PTR(out_data);
//
//  /* 2 - Perform the inference */
//  n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
//  if (n_batch != 1) {
//      err = ai_network_get_error(network);
//  };
//
//  return 0;
//}
//
//
//u8x8_msg_cb u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
//{
//	return (u8x8_msg_cb)1;
//}
//
//u8x8_msg_cb u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
//	static uint8_t buffer[32];  // Buffer for data (max 32 bytes)
//	static uint8_t buf_idx;    // Index for buffer position
//	uint8_t *data;
//	switch (msg) {
//	case U8X8_MSG_BYTE_SEND:
//		data = (uint8_t *)arg_ptr;  // Get pointer to data
//
//		while (arg_int > 0) {
//			buffer[buf_idx++] = *data;  // Store data in buffer
//			data++;
//			arg_int--;
//		}
//		uint8_t device_address = u8x8_GetI2CAddress(u8x8);  // Get OLED address (shifted for I2C slave address)
//
//		HAL_StatusTypeDef  status = HAL_I2C_Master_Transmit(&hi2c1, device_address, buffer, buf_idx, 100);
//
//		return (u8x8_msg_cb)(status == HAL_OK);  // Return success (1) or error (0)
//
//	case U8X8_MSG_BYTE_INIT:
//		break;
//
//	case U8X8_MSG_BYTE_SET_DC:
//		// Ignored for I2C (DC line control not applicable)
//		break;
//
//	case U8X8_MSG_BYTE_START_TRANSFER:
//		buf_idx = 0;  // Reset buffer index
//		break;
//
//	case U8X8_MSG_BYTE_END_TRANSFER:
//		// Handled in U8X8_MSG_BYTE_SEND for efficiency
//		break;
//
//	default:
//		return 0;
//	}
//
//	return (u8x8_msg_cb)1;
//}

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
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(500);

		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(250);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(250);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(250);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(250);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(250);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(250);

		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,ENABLE);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,DISABLE);
		HAL_Delay(500);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
