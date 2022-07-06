/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "LiquidCrystal.h"
//#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define L1      HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)
#define L2      HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)
#define M       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)
#define R2      HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)
#define R1      HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)
#define Display
#define Scroll
#define Print_Path //AutoScroll
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
const int ARR = 10;
float Kp = 10;                  //set up the constants value
float Ki = 0.001;
float Kd = 0.5;
int P = 0 , I = 0, D = 0 ;
const uint8_t Max = 100;
const uint8_t basespeed = 36;
unsigned int sensor[5] = {0 , 0 , 0 , 0 ,0 }; // an array to hold sensor values
int error = 0 ;
int lastError = 0;

bool solving_mode = false; /// when robot strar to run the definatly unsolved
//string line_maze = "", solve_maze = ""; // Memory string
char path[100];  //array for storing path data of dry run
int pathLength=0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
int read_sensor();
float PID_control(float err);
void motorPIDcontrol(int base);
void Follow_The_Rules();
void Save_Path(char state);  //saves dry run data
void Go_one_step();
void TAKE_U_TURN();
void GO_RIGHT();
void GO_LEFT();
void End_Point();
void maze_solving();
void Decession(char Command);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//----------------------------------------------------------delay_us-------------------------------------------------
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

//----------------------------------------------------------motor_control-------------------------------------------------------
void motor_control (double pos_left, double pos_right)
{
	if (pos_left < 0 )             // left
	{
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_3, ARR*0);
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, ARR*pos_left);
	}
	else
	{
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_3, ARR*pos_left);
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, ARR*0);
	}
	if (pos_right < 0 )          // right
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*0);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*pos_right);
	}
	else
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*pos_right);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*0);
	}
}

/*-----------------------------------------------------------Forward-------------------------------------------------------*/
void Forward(double pos_left,double pos_right)
{
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_3, ARR*pos_left);
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, ARR*0);

	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*pos_right);
	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*0);
}
/*-----------------------------------------------------------Backward-------------------------------------------------------*/
void Backward(double pos_left,double pos_right)
{
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_3, ARR*0);
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, ARR*pos_left);

	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*0);
	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*pos_right);
}
/*---------------------------------------------------------------stop--------------------------------------------------------------*/
void stop()
{
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_3, ARR*0);    // left
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, ARR*0);

	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*0);   // right
	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*0);
}
/*-------------------------------------------------------------------Turn_Right------------------------------------------------------------*/
void Turn_Right(double pos_left,double pos_right)
{
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_3, ARR*pos_left);   // left
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, ARR*0);

	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*0);
	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*pos_right);  // right
}
/*-------------------------------------------------------------------Turn_Left----------------------------------------------------------*/
void Turn_Left(double pos_left,double pos_right)
{
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_3, ARR*0);   // left
	__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, ARR*pos_left);

	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*pos_right);
	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*0);  // right
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);                       //starts the PMW of left motor
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);                       //starts the PMW of right motor
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/*
#ifdef AutoScroll
  char buffer[10];
  LiquidCrystal lcd;
  begin(&lcd,16,2,LCD_5x8DOTS);
#endif
*/

  unsigned char smiley[8] = {
    0b00000,
    0b00000,
    0b01010,
    0b00000,
    0b00000,
    0b10001,
    0b01110,
    0b00000
  };


#ifdef Display
LiquidCrystal lcd;
begin(&lcd,16,2,LCD_5x8DOTS);
// create a new character
createChar(&lcd, 1, smiley);
// set the cursor to the top left
setCursor(&lcd, 0, 0);
print(&lcd, "MICROMOUSE PR0" );
write(&lcd,(unsigned char)1);
HAL_Delay(3000);
#endif



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    if((L1 == 1 && R1 == 1) && (L2 == 0 ||  M == 0  || R2 == 0))  //10001
      	  {

    	  motorPIDcontrol(basespeed);

      	  }
          else
       	  {
        	  Follow_The_Rules();
       	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
                          |LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin|LCD_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin
                           LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
                          |LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_EN_Pin LCD_RW_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int read_sensor()
{
sensor[0]= L1 ; //HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
sensor[1]= L2 ; //HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
sensor[2]= M ; //HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
sensor[3]= R2 ; //HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
sensor[4]= R1 ; // HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);

    if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)          //10111
	  {
    	    error= -2;

	  }
    else if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)     //10011
 	  {
 	  	    error= -1;

 	  }
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)     //11011
	  {
	  	    error= 0;

	  }
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1)     //11001
 	  {
 	  	    error= 1;

 	  }
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 1)   //11101
	  {
			error= 2;
	  }
    else if(sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)   //01111
	  {
			error= -4;
	  }
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0)   //11110
	  {
			error= 4;
	  }
    else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)   //00111
	  {
			error= -3;
	  }
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0)   //11100
	  {
			error= 3;
	  }
    else               //if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)    //11111
	  {
			stop();
	  }
		return error;
}
/* ------------------------------------------------------------------------------------------------------- */
float PID_control(float Error)
{
	Error = read_sensor();
	 D = Error - lastError;

	 float PID = (Kp * Error) + ( D * Kd) ;
	 lastError = Error;
	return PID;
}

/* ------------------------------------------------------------------------------------------------------- */
void motorPIDcontrol(int base)
{

	 Forward( base + (PID_control(read_sensor())), base - (PID_control(read_sensor())));
}
// -------------------------------------------------------------------------------------------------------
//if (far right -R1- and far left -L1- got line), so robot encountered "T" or "+" type or "end of maze" line,
//then go one step and read sensor,
//1) if we again get (far right -R1- and far left -L1- both got line), sure it is "end of maze".
//2) if we get (far right -R1- and mid right -R2- both line missing and any line sensor get the line),
//sure it is "+" line then go left , as though it is junction , save "L" to the memory.
//3) if we get (all of the sensor missing line) then sure it is "T" line,
//then go left and save "L" to the memory.
//Note : no matter it is "T" or "+" robot must go to left and this is "the left hand rule" ,
//so here i just check , (1) and (3) number conditions.

void Follow_The_Rules()
{
	if(L1 == 0 &&  R1 == 0) //00000
	{
		stop();
		HAL_Delay(500);
		Go_one_step();
		if(L1 == 0 &&  R1 == 0)  //00000
		{
		stop();           //Finish_Point();
		End_Point();
		}
		if(L1 == 1 &&  R1 == 1)
		{
			GO_LEFT();
			if (!solving_mode) {
				Save_Path('L');          //value ‘L’ saved for left turn ** line_maze += "L";
				LiquidCrystal lcd;
				begin(&lcd,16,2,LCD_5x8DOTS);
				print(&lcd,"Left");

			}
		}
	}
// --------------------------------------------------------------------------------------------------------
//if far left got the line , so robot encountered "left turn" or "-|" type of line, go one step and read sensor,
//1) if (any line sensors (R2,M,L2) got the line), then sure it is junction and "-|" line it is then,
// go left and save "L"  to the memory.
//2) if (all of the sensors missing line) , then sure it is "left turn"
//then go left and doesn't save anything.

	if(L1 == 0 ) //00011
	{
		Go_one_step();
		if(L1 == 0 &&  R1 == 0)  //00000
		{
		End_Point();//	Finish_Point();
		}
		if( L2 == 0 || M == 0 || R2 == 0 ) //00011
		{
			Save_Path('L');
			LiquidCrystal lcd;
			begin(&lcd,16,2,LCD_5x8DOTS);
			print(&lcd,"Left");
			GO_LEFT();
		}
		else
		{
			LiquidCrystal lcd;
			begin(&lcd,16,2,LCD_5x8DOTS);
			print(&lcd,"Left");
			GO_LEFT();

		}
	}
// ------------------------------------------------------------------------------------------------------------------
//if far right got the line , so robot encountered "right turn" or "|-"  type of line, go one step and read sensor,
//1) if (any line sensor got the line),  sure it is "|-"  line  then, go forward and save "S"  to the memory.
//2) if (all of the sensor missing line) , sure it is "right turn" then go right and doesn't save anything.

	if(R1 == 0 ) //00011
	{
		Go_one_step();
		if( L2 == 0 || M == 0 || R2 == 0 ) //00011
		{
			Save_Path('S');
			 motorPIDcontrol(basespeed);
				LiquidCrystal lcd;
				begin(&lcd,16,2,LCD_5x8DOTS);
				print(&lcd,"GO");

		}
		else
		{
			LiquidCrystal lcd;
			begin(&lcd,16,2,LCD_5x8DOTS);
			print(&lcd,"Right");
			GO_RIGHT();
		}
	}

//-----------------------------------------------------------------------------------------------------
 //if (suddenly all of the sensor missing line) ,sure we reached the "dead end", then take U turn. */
	if(L1 == 1 && L2 == 1 && M == 1 && R2 == 1 && R1 == 1)
	{
		 TAKE_U_TURN();
	}
}
//------------------------------------------------------------------------------------------------------

void Save_Path(char state)
{
	 path[pathLength] = state;
	 pathLength++;
	 HAL_Delay(1);
}

//---------------------------------------------------------Go_one_step
void Go_one_step()
{
	Forward(35,35);
	HAL_Delay(150);
	stop();
	HAL_Delay(500);
	read_sensor();
}
//--------------------------------------------------------TAKE_U_TURN
  //IF ROBOT DOESN'T SOLVE THE LINE YET, SAVE "B" IF HE TAKE U TURN
void TAKE_U_TURN()
{
	 if (!solving_mode)
	 {
		 Save_Path('B');
			LiquidCrystal lcd;
			begin(&lcd,16,2,LCD_5x8DOTS);
			print(&lcd,"Back");
	 }
	    Go_one_step();  // GO ONE STEP FOR MAKE SURE THAT LINE IS MISSING UNDER THE SENSORS

	   // WHEN ROBOT TURNNING AROUND, IT REVOLVING UNTIL THE 1TH NUMBER (2ND SENSOR) GET THE LINE, AND FOR (i) LOW INERTIA OF MOTORS (ii) VELOCITY
	       // ROBOT STOP ABOUT THE 4TH OR 3RD NUMBER SENSOR (this is lack of Accuracy of the motors)
	    while ( M != 0)
	    {
	      read_sensor();
	      Turn_Left(40,40);  //180 degree
	    }
	    stop();
	    HAL_Delay(1000);

}
//--------------------------------------------------------TURN RIGHT
void GO_RIGHT()
{
	 Go_one_step();
	  while ( R1 != 0) {
		  read_sensor();
		Turn_Right(60,0);
	  }
	    stop();
	    HAL_Delay(1000);
}
//-------------------------------------------------------TURN LEFT
void GO_LEFT()
{
	 Go_one_step();
	  while ( L2 != 0) {
		  read_sensor();
		  Turn_Left(0,60);
	  }
	    stop();
	    HAL_Delay(1000);

}
//---------------------------------------------------------------------------End_Point
void End_Point()
{
	if (!solving_mode)
	{



#ifdef Print_Path
		LiquidCrystal lcd;
		begin(&lcd,16,2,LCD_5x8DOTS);
		print(&lcd,"Reading Path :");
		HAL_Delay(1000);
	  clear(&lcd);
	setCursor(&lcd, 0, 0);
	for ( pathLength = 0; pathLength < 100; pathLength++) {
		write(&lcd, path[pathLength]);
		HAL_Delay(500);
	}
	// turn off automatic scrolling
	noAutoscroll(&lcd);

	// clear screen for the next loop:
	clear(&lcd);
#endif
   // solve_the_maze();
	Save_Path ('O');
	//msg
    solving_mode = true;
    //REPLY();

	}
	else stop();

}
//---------------------------------------Maze Solving------------------------------------------
void maze_solving()
{
	  bool maze_solved = false;
	  while (!maze_solved)
	  {

	  }
}
//----------------------------------------------------------

//---------------------------------------------------------------------------------Decession
void Decession(char Command)
{
  switch (Command) {
    case 'S':
      Go_one_step();
      break;

    case 'R':
      GO_RIGHT();
      break;

    case 'L':
      GO_LEFT();
      break;

    case 'O':
    	End_Point();
      break;
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
