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
#include "mokhw_MPU6050.h"

#include<stdio.h>
#include<string.h>
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 1024
#define RADIAN_TO_DEGREE (180 / 3.14159)
#define ALPHA 0.99
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
typedef struct {
	uint8_t buffer[UART_RX_BUFFER_SIZE];
	uint8_t temp;
	uint8_t rxd;
	volatile uint16_t input_p;
	volatile uint16_t output_p;
} uart_hal_rx_type;
typedef struct {
	float target_angle;
	float angle;
	float rate;
	float prev_rate;
	uint32_t prev_time;
	float stabilize_kp;
	float stabilize_ki;
	float rate_kp;
	float rate_ki;
	float rate_kd;
	float kd;
	float stabilize_iterm;
	float rate_iterm;
	float output;
} PID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
int _write(int, char*, int);
int __io_putchar(int);
void uart_hal_rx_buffer_init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t uart_hal_getchar(void);
void uart_hal_rx_monitor(void);
void mpu6050_init(mpu6050*);
float get_dt(uint32_t*);
void update_mpu6050();
long map(long val, long in_min, long in_max, long out_min, long out_max);
float map_f(float val, float in_min, float in_max, float out_min, float out_max);
void set_std_PID(PID*);
void set_dual_PID(PID*);
float constrain(float value, float min, float max);
void emergency_stop();
void kalman_filter(float accel_angle, float gyro_rate, float* angle, float* bias, float dt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uart_hal_rx_type uart_hal_rx;
uint16_t in = 0;
uint8_t buffer[100];
uint32_t control_last_time;
mpu6050 my_mpu6050;
float base_gy_x, base_gy_y, base_gy_z;
uint32_t mpu_prev_time;
float angle_pitch = 0, angle_roll = 0, angle_yaw = 0;
float base_roll = 0;
float base_pitch = -6;
float base_yaw = 0;
int average[3] = {0};
long throttle = 1000;
float count;
float control_angle[3];
int offset[3] = {-176, 219, -254};
float LPF_Output[3];
float P[2][2] = {
		{1, 0},
		{0, 1}
};
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;
float k[3] = {0};
float bias[3] = {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uart_hal_rx_buffer_init();
	PID pitch = {0};
	pitch.target_angle = base_pitch;
	pitch.stabilize_kp = 1.7;
	pitch.stabilize_ki = 0;
	pitch.rate_kp = 0.8;
	pitch.rate_ki = 0;
	pitch.rate_kd = 1.5;
	PID roll = {0};
	roll.target_angle = base_roll;
	roll.stabilize_kp = 1.7;
	roll.stabilize_ki = 0;
	roll.rate_kp = 0.8;
	roll.rate_ki = 0;
	roll.rate_kd = 1.5;
	PID yaw = {0};
	yaw.target_angle = base_yaw;
	yaw.stabilize_kp = 2;
	yaw.rate_kp = 1;
	yaw.rate_kd = 1;
	uint16_t m1_speed = 0, m2_speed = 0, m3_speed = 0, m4_speed = 0;



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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &uart_hal_rx.temp, 1);
  WHO_AM_I(&hi2c1);
  wake_up(&hi2c1);
  set_sample_rate(&hi2c1, 1000);
  set_sensitivity(&hi2c1, &my_mpu6050, gyro_full_scale_range_250, accel_full_scale_range_2g);
  set_DLPF(&hi2c1, 1);


  mpu6050_init(&my_mpu6050);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uart_hal_rx_monitor();
	  update_mpu6050();

	  if ((HAL_GetTick() - control_last_time) > 5000) {
		  emergency_stop();
	  }
	  if (throttle == 1000) {
		  pitch.stabilize_iterm = 0;
		  pitch.rate_iterm = 0;
		  pitch.output = 0;
		  roll.stabilize_iterm = 0;
		  roll.rate_iterm = 0;
		  roll.output = 0;
		  control_angle[2] = 0;
		  count = 0;
		  base_yaw = angle_yaw;
	  }

	  count += control_angle[2];
	  yaw.target_angle = base_yaw + count / 800;

	  pitch.angle = angle_pitch;
	  pitch.rate = my_mpu6050.gy_x_dps;
	  pitch.target_angle = base_pitch + control_angle[0];
	  roll.angle = angle_roll;
	  roll.rate = my_mpu6050.gy_y_dps;
	  roll.target_angle = base_roll + control_angle[1];
	  yaw.angle = angle_yaw;
	  yaw.rate = my_mpu6050.gy_z_dps;

	  set_dual_PID(&pitch);
	  set_dual_PID(&roll);
	  set_dual_PID(&yaw);

	  pitch.output = constrain(pitch.output, -200, 200);
	  roll.output = constrain(roll.output, -200, 200);
	  yaw.output = constrain(yaw.output, -200, 200);


	  /*m1_speed = (throttle == 1000) ? 1000 : throttle;
	  m2_speed = (throttle == 1000) ? 1000 : throttle;
	  m3_speed = (throttle == 1000) ? 1000 : throttle;
	  m4_speed = (throttle == 1000) ? 1000 : throttle;*/

	  m1_speed = (throttle == 1000) ? 1000 : throttle + pitch.output + roll.output + yaw.output;
	  m2_speed = (throttle == 1000) ? 1000 : throttle + pitch.output - roll.output - yaw.output;
	  m3_speed = (throttle == 1000) ? 1000 : throttle - pitch.output + roll.output - yaw.output;
	  m4_speed = (throttle == 1000) ? 1000 : throttle - pitch.output - roll.output + yaw.output;




	  if (m1_speed < 1000) {
		  m1_speed = 1000;
	  } else if(m1_speed > 2000) {
		  m1_speed = 2000;
	  }
	  if (m2_speed < 1000) {
		  m2_speed = 1000;
	  } else if(m2_speed > 2000) {
		  m2_speed = 2000;
	  }
	  if (m3_speed < 1000) {
		  m3_speed = 1000;
	  } else if(m3_speed > 2000) {
		  m3_speed = 2000;
	  }
	  if (m4_speed < 1000) {
		  m4_speed = 1000;
	  } else if(m4_speed > 2000) {
		  m4_speed = 2000;
	  }

	  htim1.Instance->CCR1 = m1_speed;
	  htim1.Instance->CCR2 = m2_speed;
	  htim1.Instance->CCR3 = m3_speed;
	  htim1.Instance->CCR4 = m4_speed;

	  printf("%f %f %f %f %d %d %d %d %d\r\n", angle_pitch, angle_roll, angle_yaw, yaw.target_angle, throttle, (int)htim1.Instance->CCR1, (int)htim1.Instance->CCR2, (int)htim1.Instance->CCR3, (int)htim1.Instance->CCR4);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	if(HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10) == HAL_OK)
		return len;
	return -1;
}
int __io_putchar(int ch)
{
	if(HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10) != HAL_OK)
		return -1;
	return ch;
}

void uart_hal_rx_buffer_init(void) {
	uart_hal_rx.input_p = 0;
	uart_hal_rx.output_p = 0;
}
void uart_hal_rx_monitor(void) {

	while(uart_hal_getchar() != 0) {

	}

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		uart_hal_rx.buffer[uart_hal_rx.input_p++] = uart_hal_rx.temp;
		if (uart_hal_rx.input_p >= UART_RX_BUFFER_SIZE) {
			uart_hal_rx.input_p = 0;
		}
		HAL_UART_Receive_IT(&huart3, &uart_hal_rx.temp, 1);
	}

}

uint8_t uart_hal_getchar(void) {
	uint32_t reg = READ_REG(huart3.Instance->CR1);

	__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
	if (uart_hal_rx.input_p == uart_hal_rx.output_p) {
		WRITE_REG(huart3.Instance->CR1, reg);
		return 0;
	}
	WRITE_REG(huart3.Instance->CR1, reg);
	uart_hal_rx.rxd = uart_hal_rx.buffer[uart_hal_rx.output_p++];
	if (uart_hal_rx.output_p >= UART_RX_BUFFER_SIZE) {
		uart_hal_rx.output_p = 0;
	}
	buffer[in++] = uart_hal_rx.rxd;
	if (in > 4) {
		if (buffer[in - 1] == 'c') {
			control_last_time = HAL_GetTick();
			throttle = map(buffer[2], 0, 100, 1000, 2000);
			control_angle[0] = -map_f(buffer[0], 0, 100, -30, 30);
			control_angle[1] = -map_f(buffer[1], 0, 100, -30, 30);
			control_angle[2] = map_f(buffer[3], 0, 100, -100, 100);

		}
		in = 0;
	}

	return 1;
}
void mpu6050_init(mpu6050* my_mpu6050) {
	if (my_mpu6050 == NULL) {
		return;
	}
	float sum_gy_x = 0, sum_gy_y = 0, sum_gy_z = 0;
	int sum[3] = {0};
	for (int i = 0; i < 10; i++) {
		read_gyro(&hi2c1, my_mpu6050, raw_data	);

		sum_gy_x += (int16_t) ((my_mpu6050->gy_x - offset[0]) / my_mpu6050->gyro_change_unit_factor);
		sum_gy_y += (int16_t) ((my_mpu6050->gy_y - offset[1]) / my_mpu6050->gyro_change_unit_factor);
		sum_gy_z += (int16_t) ((my_mpu6050->gy_z - offset[2]) / my_mpu6050->gyro_change_unit_factor);
		sum[0] += my_mpu6050->gy_x;
		sum[1] += my_mpu6050->gy_y;
		sum[2] += my_mpu6050->gy_z;
		HAL_Delay(100);
	}
	base_gy_x = sum_gy_x / 10;
	base_gy_y = sum_gy_y / 10;
	base_gy_z = sum_gy_z / 10;
	average[0] = sum[0] / 10;
	average[1] = sum[1] / 10;
	average[2] = sum[2] / 10;

}
float get_dt(uint32_t* prev) {
	float now = HAL_GetTick();
	float dt = (now - *prev) / 1000;
	*prev = now;
	return dt;
}
void update_mpu6050() {
	float angle_ac_x, angle_ac_y, angle_ac_z;
	float angle_gy_x = 0, angle_gy_y = 0, angle_gy_z = 0;
	float f_ac[3];
	float angle_tmp_x, angle_tmp_y, angle_tmp_z;
	float dt;
	read_gyro(&hi2c1, &my_mpu6050, raw_data);
	read_accel(&hi2c1, &my_mpu6050, gravity_acceleration);
	dt = get_dt(&mpu_prev_time);
	f_ac[0] = my_mpu6050.ac_x_g;
	f_ac[1] = my_mpu6050.ac_y_g;
	f_ac[2] = my_mpu6050.ac_z_g;

	for (int i = 0; i < 3; i++) {
		LPF_Output[i] = LPF_Output[i] * 0.9 + f_ac[i] * 0.1;
	}

	f_ac[0] = LPF_Output[0];
	f_ac[1] = LPF_Output[1];
	f_ac[2] = LPF_Output[2];

	my_mpu6050.gy_x_dps = (int16_t) ((my_mpu6050.gy_x - offset[0]) / my_mpu6050.gyro_change_unit_factor);
	my_mpu6050.gy_y_dps = (int16_t) ((my_mpu6050.gy_y - offset[1]) / my_mpu6050.gyro_change_unit_factor);
	my_mpu6050.gy_z_dps = (int16_t) ((my_mpu6050.gy_z - offset[2]) / my_mpu6050.gyro_change_unit_factor);
	angle_ac_x = atan(f_ac[1] / sqrt((pow(f_ac[0], 2) + pow(f_ac[2], 2))));
	angle_ac_x *= RADIAN_TO_DEGREE;
	angle_ac_y = atan(-f_ac[0] / sqrt((pow(f_ac[1], 2) + pow(f_ac[2], 2))));
	angle_ac_y *= RADIAN_TO_DEGREE;
	angle_ac_z = atan(sqrt(pow(f_ac[0], 2) + pow(f_ac[1], 2) / f_ac[2]));
	angle_ac_z *= RADIAN_TO_DEGREE;
	angle_gy_x += (my_mpu6050.gy_x_dps - base_gy_x) * dt;
	angle_gy_y += (my_mpu6050.gy_y_dps - base_gy_y) * dt;
	angle_gy_z += (my_mpu6050.gy_z_dps) * dt;
	angle_tmp_x = angle_pitch + angle_gy_x;
	angle_tmp_y = angle_roll + angle_gy_y;
	angle_tmp_z = angle_yaw + angle_gy_z;
	kalman_filter(f_ac[0], my_mpu6050.gy_x_dps, &k[0], &bias[0], dt);
	kalman_filter(f_ac[1], my_mpu6050.gy_y_dps, &k[1], &bias[1], dt);
	//angle_pitch = ALPHA * angle_tmp_x + (1.0 - ALPHA) * angle_ac_x;
	//angle_roll = ALPHA * angle_tmp_y + (1.0 - ALPHA) * angle_ac_y;
	angle_pitch = k[0];
	angle_roll = k[1];
	angle_yaw = angle_tmp_z;
}

long map(long val, long in_min, long in_max, long out_min, long out_max) {
	return (long)(((val - in_min)/ (double)(in_max - in_min)) * (out_max - out_min)  + out_min);
}
float map_f(float val, float in_min, float in_max, float out_min, float out_max) {
	return ((val - in_min)/ (in_max - in_min)) * (out_max - out_min)  + out_min;
}
void set_std_PID(PID* PID) {
	float error = PID->target_angle - PID->angle;
	float dInput = error - PID->prev_rate;
	float dt = get_dt(&PID->prev_time);
	float pterm, dterm;

	pterm = PID->stabilize_kp * error;
	PID->stabilize_iterm += PID->stabilize_ki * error * dt;
	dterm = PID->kd * (dInput);

	PID->output = pterm + PID->stabilize_iterm + dterm;
	PID->prev_rate = error;

}
void set_dual_PID(PID* PID) {
	float angle_error = PID->target_angle - PID->angle;
	float rate_error;
	float desired_rate;
	float dInput;
	float stabilize_pterm;
	float rate_pterm;
	float rate_dterm;
	float dt = get_dt(&PID->prev_time);
	stabilize_pterm = PID->stabilize_kp * angle_error;
	PID->stabilize_iterm = PID->stabilize_iterm + PID->stabilize_ki * angle_error * dt;
	desired_rate = stabilize_pterm;
	rate_error = desired_rate - PID->rate;
	dInput = rate_error - PID->prev_rate;
	PID->prev_rate = rate_error;
	rate_pterm = PID->rate_kp * rate_error;
	PID->rate_iterm = PID->rate_iterm + PID->rate_ki * rate_error * dt;
	rate_dterm = PID->rate_kd *(dInput);
	PID->output = rate_pterm + PID->rate_iterm + rate_dterm;
}
float constrain(float value, float min, float max) {
	if (value > max) {
		value = max;
	} else if (value < min) {
		value = min;
	}
	return value;
}
void emergency_stop() {
	control_angle[0] = 0;
	control_angle[1] = 0;
	control_angle[2] = 0;
	if (throttle > 1000) {
		throttle--;
	}
}
void kalman_filter(float accel_angle, float gyro_rate, float* angle, float* bias, float dt) {
	float rate = gyro_rate - *bias;
	*angle += rate * dt;

	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;

	float S = P[0][0] + R_measure;
	float K[2] = {P[0][0] / S, P[1][0] / S};

	float y = accel_angle - *angle;
	*angle += K[0] * y;
	*bias += K[1] * y;

	float temp_P00 = P[0][0];
	float temp_P01 = P[0][1];

	P[0][0] -= K[0] * temp_P00;
	P[0][1] -= K[0] * temp_P01;
	P[1][0] -= K[1] * temp_P00;
	P[1][1] -= K[1] * temp_P01;
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
