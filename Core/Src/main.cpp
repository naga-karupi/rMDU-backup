/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define STM32_NAGA_LIB 1
#include "rotary-encoder.hpp"
#include <cmath>
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
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t recv_data[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float kp = 1500;
float ki = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart1, recv_data, 8);

	if(recv_data[0] == 0x10 && recv_data[1] == 0x0f) {
		if(recv_data[2] == 0x01) kp = *(float*) (recv_data+4);
		if(recv_data[2] == 0x02) ki = *(float*) (recv_data+4);
	}
}

class MD_t{
	  int32_t val;
	  struct GPIO{
		  GPIO_TypeDef* port;
		  uint16_t pin;
	  }enable;

	  TIM_HandleTypeDef* tim;
	  uint32_t channel_a, channel_b;

public:
	  MD_t(GPIO_TypeDef* _port, uint16_t _pin, TIM_HandleTypeDef* _tim,uint32_t _channel_a, uint32_t _channel_b)
	  : tim(_tim), channel_a(_channel_a), channel_b(_channel_b){
		  enable.port = _port;
		  enable.pin  = _pin;
		  HAL_GPIO_WritePin(enable.port, enable.pin, GPIO_PIN_SET);
		  HAL_TIM_PWM_Start(tim, channel_a);
		  HAL_TIM_PWM_Start(tim, channel_b);
	  }

	  ~MD_t(){
		  HAL_GPIO_WritePin(enable.port, enable.pin, GPIO_PIN_RESET);
		  HAL_TIM_PWM_Stop(tim, TIM_CHANNEL_ALL);
	  }

	  void operator ()(int32_t pwm){
		  if(pwm){
			  __HAL_TIM_SET_COMPARE(tim, (pwm > 0) ? channel_a : channel_b , std::abs(pwm));
			  __HAL_TIM_SET_COMPARE(tim, (pwm < 0) ? channel_a : channel_b, 0);
		  } else {
			  __HAL_TIM_SET_COMPARE(tim, channel_a, 0);
			  __HAL_TIM_SET_COMPARE(tim, channel_b, 0);
		  }
	  }
};
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  size_t down_counter = 0;
  constexpr float resolution = 5120 ;
  constexpr float one_resolution = 1/resolution;
  constexpr float rate       = 100;
  constexpr float dt         = 0.01;

  HAL_TIM_Base_Start_IT(&htim2);
  using namespace naga_libs;

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

  rotary_encoder_stm32 enc(&htim3);

  union uart_protocol {
	  struct{
		  uint16_t ID;   //ID: 0x10
		  uint16_t cmd;  //cmd: duty: 0x00, current: 0x01, spd_PID: 0x02, pos_PID: 0x03
		  float value;
	  };
	  uint8_t bin[8];
  }msg;

  constexpr uint16_t ID_val = 0x10;
  enum {
	  DUTY,
	  CURRENT,
	  SPD_PID,
	  POS_PID,
	  SET_P = 0xf0,
	  SET_I = 0xf1,
	  SET_D = 0xf2
  };

//  HAL_GPIO_WritePin(MD_EN_GPIO_Port, MD_EN_Pin, GPIO_PIN_SET);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10000);




  MD_t MD(MD_EN_GPIO_Port, MD_EN_Pin, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);

  MD(0);

  HAL_UART_Receive_IT(&huart1, recv_data, 8);

  auto PI_spd = [=](float target_spd, float now_spd){
	  static float integral_error = 0;

	  auto clamp = [](float input, float limit){
		  if(input > limit) return limit;
		  if(input < -limit) return -limit;
		  return input;
	  };

	  auto error = target_spd - now_spd;
	  integral_error += error * dt;
	  integral_error = clamp(integral_error, 30000.0/ki);

	  if(target_spd == 0.0f)
		  integral_error = 0.0f;

	  return error * kp + integral_error * ki;
  };

  static uint32_t last_ms = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

#define ENC 0
#if ENC
	  static int32_t s = 0;
	  s += (int16_t)enc();
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (s > 0) ? GPIO_PIN_SET: GPIO_PIN_RESET);
#endif
	  for(int i = 0; i < 8; i++) msg.bin[i] = recv_data[i];
	  if(msg.ID != ID_val) continue;
#define MD_TEST 0
#if MD_TEST
	  if(msg.cmd != DUTY) continue;
	  MD(10000*msg.value);

#endif

	  constexpr int32_t target_rotate = 14.0/(10.0*M_PI) * 5120.0 * 2;//TODO: NEED TO CHANGE
	  constexpr int32_t one_target_rotate = 3.0/(10.0*M_PI) * 5120.0 * 2;//todo need to change
#define EASY_POS 0
#if EASY_POS
	  if(msg.cmd != DUTY) continue;
	  if(!msg.value) continue;

	  int32_t now_rotate = 0;
	  while(now_rotate < target_rotate) {
		  MD(10000);// NEED TO CHANGE VALUE
		  now_rotate += (int16_t)enc();

		  while(last_ms + 10 > HAL_GetTick());
		  last_ms = HAL_GetTick();
		  now_rotate += (int16_t)enc();
	  }
	  MD(0);
#endif

#define PI 01
#if PI
	  if(msg.cmd != SPD_PID) continue;
	  int32_t now_rotate = 0;
	  if(msg.bin[5] != 0){
		  if((int8_t)msg.bin[5] < 0){
//			  if(down_counter == 0){
//				  while(now_rotate < init_target_rotate){
//
//					  auto enc_val = (int16_t)enc();
//					  now_rotate += enc_val;
//
//					  MD(PI_spd(4.0*M_PI, (float)enc_val * rate * one_resolution));
//
//					  while(last_ms + 10 > HAL_GetTick());
//					  last_ms = HAL_GetTick();
//				  }
//			  } else
			  {
				  while(now_rotate < target_rotate + ((down_counter == 0) ? one_target_rotate : 0)){

					  auto enc_val = (int16_t)enc();
					  now_rotate += enc_val;

					  MD(PI_spd(4.0*M_PI, (float)enc_val * rate * one_resolution));

					  while(last_ms + 10 > HAL_GetTick());
					  last_ms = HAL_GetTick();
				  }
			  }

			  down_counter++;
		  }


		  if((int8_t)msg.bin[5] > 0){
			  for(size_t i = 0; i < down_counter; i++){
				  int32_t now_rotate = 0;
				  while(now_rotate > -target_rotate - ((i == 0) ? one_target_rotate : 0)){

					  auto enc_val = (int16_t)enc();
					  now_rotate += enc_val;

					  MD(PI_spd(-4.0*M_PI, (float)enc_val * rate * one_resolution));

					  while(last_ms + 10 > HAL_GetTick());
					  last_ms = HAL_GetTick();
				  }
			  }
//
//			  int32_t now_rotate = 0;
//			  while(now_rotate > -init_target_rotate && down_counter==1) {
//
//				  auto enc_val = (int16_t)enc();
//				  now_rotate += enc_val;
//
//				  MD(PI_spd(4.0*M_PI, (float)enc_val * rate * one_resolution));
//
//				  while(last_ms + 10 > HAL_GetTick());
//				  last_ms = HAL_GetTick();
//			  }
			  down_counter = 0;
		  }

		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, down_counter ? GPIO_PIN_SET : GPIO_PIN_RESET);

	  }
	  MD(0);
#endif

#define WHILE 0
#if WHILE

	  static int32_t counter;

	  if((int8_t)msg.bin[5] > 0) {
		  counter = 0;
	  }

#endif


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MD_EN_Pin|LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MD_EN_Pin LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = MD_EN_Pin|LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
