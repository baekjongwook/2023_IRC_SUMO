/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "JControl.h"
#include "JSensing.h"
#include "Protocol2.h"
#include "iBus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//PIXY
#define UP_PAN_MAX 3700
#define UP_PAN_MIN 600
#define UP_NO_PAN 200

#define DOWN_PAN_MAX 3300
#define DOWN_PAN_MIN 800
#define DOWN_NO_PAN 350

//QTI
#define QTI_JUDGE_1 2500 //2950
#define QTI_JUDGE_2 1500 //1950

//VELOCITY
#define FRONT 300
#define BOOST 850
#define BACK 500
#define TURN 400

//PSD
#define PSD_HIT_1 700
#define PSD_HIT_2 700

#define PSD_BOOST_1 1900
#define PSD_BOOST_2 1900

#define PSD_ESCAPE 1000
#define PSD_RUN 1650


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

//direction
extern DIRECTION Direction1;
extern DIRECTION Direction2;

//duty
extern DUTY MotorDutyVelocity1;
extern DUTY MotorDutyVelocity2;
extern int DutyTarget1;
extern int DutyTarget2;

//adc
extern uint32_t adc_rawdata[11]; //right_back: 0, left_back: 1, bottom: 2, pixy1: 3 right_front: 4, pixy2: 5, left_front: 6
extern MAF PSD_front_left;
extern MAF PSD_front_right;
extern MAF PSD_back_left;
extern MAF PSD_back_right;
extern MAF PSD_bottom;
extern MAF IR1;
extern MAF IR2;
extern MAF PSD_Side1;
extern MAF PSD_Side2;

//gpio
long QTI_front_left = 0; //left_front:7, left_back:5, right_back:4, right_front:6
long QTI_front_right = 0;
long QTI_back_left = 0;
long QTI_back_right = 0;

//qti
int frontQTIflag = 0;

//pixy
int cameraflag = 0;
int subcameraflag = 0;
int pastPixyValue = 0;
int presentPixyValue = 0;
int DIRpixy = 0;

//skill
int hitflag = 0;
int hitcount = 0;

int startcount = 0;
int startflag = 0;

//escape
int stopcnt = 0;
int stopflag = 0;

extern int firstleft;
extern int secondfront;
extern int thirdleft;
extern int fourthfront;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void Duty_Velocity()
{
	Duty_Control_Velocity(&MotorDutyVelocity1, &Direction1, GPIOC, LL_GPIO_PIN_8, TIM1, 1, DutyTarget1);
	Duty_Control_Velocity(&MotorDutyVelocity2, &Direction2, GPIOC, LL_GPIO_PIN_9, TIM1, 2, DutyTarget2);
}

void Stop()
{
	DutyTarget1 = 0;
	DutyTarget2 = 0;
}

void GoFront()
{
	DutyTarget1 = FRONT;
	DutyTarget2 = FRONT;
}

void GoFrontBoost()
{
	DutyTarget1 = BOOST;
	DutyTarget2 = BOOST;
}

void GoStartBoost()
{
	DutyTarget1 = 800;
	DutyTarget2 = 800;
}

void GoBack()
{
	DutyTarget1 = -BACK;
	DutyTarget2 = -BACK;
}

void GoBackSlow()
{
	DutyTarget1 = -150;
	DutyTarget2 = -150;
}

void TurnLeft()
{
	DutyTarget1 = -TURN;
	DutyTarget2 = TURN;
}

void TurnRight()
{
	DutyTarget1 = TURN;
	DutyTarget2 = -TURN;
}

void HitFloor()
{
	hitflag == 1 ? Stop() : GoFrontBoost();
}

long QTI(GPIO_TypeDef* GPIOx, uint16_t PINx)
{
	long qti_value = 0;

	LL_GPIO_SetPinMode(GPIOx, PINx, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetOutputPin(GPIOx, PINx);

	LL_mDelay(1);

	LL_GPIO_SetPinMode(GPIOx, PINx, LL_GPIO_MODE_INPUT);
	LL_GPIO_ResetOutputPin(GPIOx, PINx);

	while(LL_GPIO_IsInputPinSet(GPIOx, PINx))
	{
		qti_value++;
	}

	return qti_value;
}

void JudgeFrontQTI()
{
	if((QTI_front_left < 1400 || QTI_front_right < 1300) || (IR1.output < 3900 || IR2.output < 2000)) //2800 1700 , 1400 1200 , 1300 1100
	{
		frontQTIflag = 1;
	}
	else
	{
		frontQTIflag = 0;
	}
}


void JudgeCamera() //flag : 1 = front, 2 = left, 3 = right, 4 = fail
{
	if(adc_rawdata[5] > UP_PAN_MIN && adc_rawdata[5] <= UP_PAN_MAX)
	{
		cameraflag = 1;
		DIRpixy = 1;
	}
	else if(adc_rawdata[5] > UP_PAN_MAX)
	{
		cameraflag = 2;
		DIRpixy = 2;
	}
	else if(adc_rawdata[5] > UP_NO_PAN && adc_rawdata[5] <= UP_PAN_MIN)
	{
		cameraflag = 3;
		DIRpixy = 3;
	}

	else
	{
		cameraflag = 0;
	}
}

void AlgorithmCamera()
{
	switch(cameraflag)
	{
	case 1:
	{
		GoFront();

		break;
	}

	case 2:
	{
		TurnRight();

		break;
	}

	case 3:
	{
		TurnLeft();

		break;
	}

	default:
	{
		if(DIRpixy == 2)
		{
			TurnRight();
		}
		else
		{
			TurnLeft();
		}
		break;
	}
	}
}

void Algorithm()
{
	JudgeFrontQTI();
	JudgeCamera();

	if(((PSD_back_left.output > 900 || PSD_back_right.output > 900) && PSD_Side1.output > 1200) || ((PSD_back_left.output > 900 || PSD_back_right.output > 900) && PSD_Side2.output > 1000)) //++ side psd
	{
		stopcnt++;

		if(stopcnt > 1000 && stopcnt <= 2000)
		{
			Stop();
		}
		else if(stopcnt > 2000 && stopcnt <= 3500)
		{
			GoBackSlow();
		}
		else if(stopcnt > 3500 && stopcnt <= 9000)
		{
			GoFrontBoost();
		}
		else if(stopcnt > 9000)
		{
			stopcnt = 0;
		}
	}

	else if(PSD_back_left.output > 800 && PSD_back_right.output > 800) //++ side psd
	{
		GoFrontBoost();
	}

	else if(frontQTIflag == 1)
	{
		GoBack();
	}

	else if((PSD_front_left.output > PSD_BOOST_1 || PSD_front_right.output > PSD_BOOST_2) && !(cameraflag == 0))
	{
		GoFrontBoost();
	}

	else if(((PSD_front_left.output > PSD_HIT_1 || PSD_front_right.output > PSD_HIT_2) && (PSD_front_left.output <= PSD_BOOST_1 && PSD_front_right.output <= PSD_BOOST_2)) && !(cameraflag == 0))
	{
		HitFloor();
	}

	else
	{
		AlgorithmCamera();
	}
}

void Start_Algorithm()
{
	if(startflag == 1)
	{
		Algorithm();
	}

	else if(startcount > fourthfront)
	{
		startflag = 1;
	}

	else if(startcount > thirdleft && startcount <= fourthfront)
	{
		startcount++;

		HitFloor();
	}

	else if(startcount > secondfront && startcount <= thirdleft)
	{
		startcount++;

		TurnLeft();
	}

	else if(startcount > firstleft && startcount <= secondfront)
	{
		startcount++;

		GoStartBoost();
	}

	else
	{
		startcount++;

		TurnLeft();
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM8))
	{
		//right_back: 0, left_back: 1, bottom: 2, pixy1: 3 right_front: 4, pixy2: 5, left_front: 6
		MAF_Filter(&PSD_front_left, adc_rawdata[6]);
		MAF_Filter(&PSD_front_right, adc_rawdata[4]);
		MAF_Filter(&PSD_back_left, adc_rawdata[1]);
		MAF_Filter(&PSD_back_right, adc_rawdata[0]);
		MAF_Filter(&PSD_bottom, adc_rawdata[2]);
		MAF_Filter(&IR1, adc_rawdata[7]);
		MAF_Filter(&IR2, adc_rawdata[8]);
		MAF_Filter(&PSD_Side1, adc_rawdata[9]);
		MAF_Filter(&PSD_Side2, adc_rawdata[10]);

		LL_TIM_ClearFlag_UPDATE(TIM8);
	}
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

	if(LL_TIM_IsActiveFlag_UPDATE(TIM6))
	{
		Duty_Velocity();

		Start_Algorithm();

		LL_TIM_ClearFlag_UPDATE(TIM6);
	}

  /* USER CODE END TIM6_DAC_IRQn 0 */

  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

	QTI_front_left = QTI(GPIOA, LL_GPIO_PIN_7);
	QTI_front_right = QTI(GPIOA, LL_GPIO_PIN_6);

	hitcount++;
	if(hitcount == 7)
	{
		hitflag = hitflag == 0 ? 1 : 0;

		hitcount = 0;
	}

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
