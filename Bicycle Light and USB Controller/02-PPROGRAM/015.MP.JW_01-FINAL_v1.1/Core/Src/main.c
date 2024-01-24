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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "ftoa.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//ad voltage divider
#define n_dyn 3.14f //dynamo
#define n_batt 3.14f//battery
#define n_U_USB_converter 3.14f//USB
//adc and pwm coeficients
#define i_current_out 350.0f //adc current output after converting
#define u_voltage_in 0.19f //adc voltage inbut before converting V
#define i_current_add 20.0f //correction factor for adc current measure mA
#define i_usb_current_max 2500.0f	//usb max current mA
#define i_led_current_max 1000.0f	//led max current mA
#define i_noise 30.0f	//noise current value mA
#define u_max_d_b_voltage 10.f	//max voltage for battery and dynamo
#define u_voltage_bridge_d 0.7f	//voltage losses
#define u_d_voltage_load 4.0f	//dynamo load voltage
#define u_low_batt 3.4f	//low battery voltage
#define u_usb_min_out 4.0f	//min usb voltage output V
#define u_usb_max_out 5.5f	//max usb voltage output V
#define i_usb_min_out	10 //minimal usb current output to work mA - to confirm connected device to usb port
#define usb_pwm_min 20		//min value pwm to boost converter usb %
#define usb_pwm_max	50  	//max value pwm to boost converter usb %
//power mode coeficients
//batt ok dynamo ok
#define u_batt_ok 3.8f	//batt voltage value ok
#define u_dynamo_ok 5.0f	//dynamo voltage value ok
#define led_power_mode_1 100	//%
#define usb_power_mode_1 100	//%
//batt ok dynamo nok
#define led_power_mode_2 70
#define usb_power_mode_2 50
//batt nok dynamo ok
#define led_power_mode_3 90
#define usb_power_mode_3 40

//batt nok dynamo nok
#define led_power_mode_4 70
#define usb_power_mode_4 30


//others
#define I_LED1_max 700 //led current max mA
#define mes_size 5	//number adc channels
#define U_reff 3.27f //ref voltage, uC supply voltage V
#define res_adc 4096.0f//adc resolution


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//general variables
uint8_t mode_short = 0; //switch
uint8_t mode_vcc_led = 0; //led max power coeffcient
uint8_t mode_vcc_usb = 0; //usb max power coefficient

//text buffer
char snum[5];	// OLED
char OLED_txt [4];	//mode
char E_L_txt[1]; //led error
char E_B_txt[1]; //usb error
char ERROR_txt[2];	//led and usb error
//buffer for data from adc
uint32_t value_conv[mes_size]; //fresh ADC
float value_voltage[mes_size]; //after converting ADC
//timer
uint8_t timer_counter_1 = 0;	//timer

//USB
//current
uint16_t bb_pwm_set = 0; //boost converter set pwm value
uint16_t bb_current_set = 0; //boost converter set current value
uint16_t bb_current_mes = 0; //boost converter measure current value
//voltage
float bb_voltage_set = 0.0f; ///boost converter set voltage value
float bb_voltage_mes = 0.0f; ///boost converter measure voltage value
float bb_power_mes = 0.0f; //boost converter measure power
//errors
uint8_t bb_error = 0;	//error code
uint8_t bb_error_critical = 0;	//critical error code

//LED
uint16_t led_power_pwm= 0;	//led pwm
uint16_t led_power_mes= 0;	//led current measure
uint16_t led_power_set= 0;	//led current set
//errors
uint8_t led_error =0;	//error code
uint8_t led_error_critical =0;	//critical error code

//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//useful PFP
//map
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//float map
float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//constrain
long constrain(long x,long  a,long b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

//float constrain
float constrain_f (float x,float  a,float b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

//ADC read and converting
void adc_read (void)
{

	//USB current- OK
	  value_voltage[0]=(U_reff/res_adc)*value_conv[0];
	  value_voltage[0]=map_f(value_voltage[0], 0.0f ,u_voltage_in,0.0f,i_current_out)+i_current_add;
	  if(value_voltage[0]<i_noise)
	 {
		  value_voltage[0]=0;
	  }
	  value_voltage[0] = constrain_f( value_voltage[0], 0.0f, i_usb_current_max);
	  bb_current_mes= value_voltage[0];

	  //USB voltage - OK
	  value_voltage[1]=(U_reff/res_adc)*value_conv[1];
	  value_voltage[1]=value_voltage[1]*n_U_USB_converter;
	  bb_voltage_mes = value_voltage[1];

	  //LED current - OK
	  /*measured value
	   * 0,35A - load current
	   * 0,19V - pin adc voltage
	   */
	  value_voltage[2]=(U_reff/res_adc)*value_conv[2]; //pomiar napiecia z bocznika
	  value_voltage[2]=map_f(value_voltage[2], 0.0f ,u_voltage_in,0.0f,i_current_out)+i_current_add;
	  if(value_voltage[2]<i_noise)
	  {
		  value_voltage[2]=0;
	  }
	  value_voltage[2] = constrain_f( value_voltage[2], 0.0f, i_led_current_max);
	  led_power_mes = value_voltage[2];

	  //battery voltage - OK
	  value_voltage[3]=(U_reff/res_adc)*value_conv[3];
	  value_voltage[3]=(value_voltage[3]*n_batt);
	  value_voltage[3] = constrain_f(value_voltage[3], 0.0f ,u_max_d_b_voltage);

	  //dynamo voltage - OK
	  value_voltage[4]=(U_reff/res_adc)*value_conv[4];
	  value_voltage[4]=(value_voltage[4]*n_dyn);
	  value_voltage[4] = constrain_f(value_voltage[4], 0.0f ,u_max_d_b_voltage);

	  //diode voltage adding with load
	  if(value_voltage[4]>u_d_voltage_load)
	  {
		  value_voltage[4] = value_voltage[4] + u_voltage_bridge_d;
	  }

	  //boost converter calculation mA*V/1000
	  bb_power_mes = (bb_voltage_mes * bb_current_mes)/1000.0f;

}

//USB boost converter
void USB_charger (void)
{
	 //set the voltage in proportion to the current
	  if(bb_current_set<bb_current_mes)
	  {
	  bb_voltage_set=bb_voltage_set-0.01f;
	  }
	  else if(bb_current_set>bb_current_mes)
	  {
	  bb_voltage_set=bb_voltage_set+0.01f;
	  }
	  else
	  {
	  }
	  bb_voltage_set = constrain_f(bb_voltage_set, u_usb_min_out, u_usb_max_out);

	  //set pwm
		if((bb_current_set!=0)&&(bb_error==0)&&(bb_error_critical==0)&&(bb_current_mes>=i_usb_min_out))
		{

			if(bb_voltage_mes<bb_voltage_set)
			{
				bb_pwm_set++;
			}
			else if(bb_voltage_mes>bb_voltage_set)
			{
				bb_pwm_set--;
			}
			else
			{

			}
			bb_pwm_set = constrain(bb_pwm_set, ((160*usb_pwm_min)/100), ((160*usb_pwm_max)/100));	//set pwm range as additional condition
		}

		else
		{
			bb_pwm_set = 0;
		}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, bb_pwm_set); //set pwm
}

//led work
void LED_work (void)
{
		if((led_power_set!=0)&&(led_error==0)&&(led_error_critical==0))
		{
				led_power_mes = map( led_power_mes, 0, I_LED1_max, 0, 100 ); //change current range to pwm range, dependet upon max led current

				if(led_power_mes<led_power_set)
				{
					led_power_pwm++;
				}
				else if(led_power_mes>led_power_set)
				{
					led_power_pwm-- ;
				}
				else
				{
				}
		}
		else
		{
			led_power_pwm  = 0;
		}

	led_power_pwm = constrain ( led_power_pwm, 0, 100); //adjust pwm range to arr range
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, led_power_pwm); //set pwm
}

//vcc control system
void vcc_managment (void)
{

	if((value_voltage[4]>u_dynamo_ok)&&(value_voltage[3]>u_batt_ok)) //dynamo ok and batt ok
	{
		mode_vcc_led=led_power_mode_1;
		mode_vcc_usb=usb_power_mode_1;
	}
	else if ((value_voltage[4]<=u_dynamo_ok)&&(value_voltage[3]>u_batt_ok))	//dynamo nok and batt ok
	{
		mode_vcc_led=led_power_mode_2;
		mode_vcc_usb=usb_power_mode_2;
	}
	else if ((value_voltage[4]>u_dynamo_ok)&&(value_voltage[3]<=u_batt_ok))	// dynamo ok and batt nok
	{
		mode_vcc_led=led_power_mode_3;
		mode_vcc_usb=usb_power_mode_3;
	}
	else	//dynamo nok and batt nok
	{
		mode_vcc_led=led_power_mode_4;
		mode_vcc_usb=usb_power_mode_4;
	}
}


//main screen with parameters
void OLED_main (void)
{
	  //batt voltage
	  SSD1306_GotoXY (0,0);
	  SSD1306_Puts ("U[V] ", &Font_6x8, 1);
	  SSD1306_Puts ("Bat ", &Font_6x8, 1);
	  ftoa(snum, value_voltage[3], 1);
	  SSD1306_Puts (snum, &Font_6x8, 1);

	  //dynamo voltage
	  SSD1306_Puts (" Dyn ", &Font_6x8, 1);
	  ftoa(snum, value_voltage[4], 1);
	  SSD1306_Puts (snum, &Font_6x8, 1);

	  //current mode
	  SSD1306_GotoXY (0,8);
	  SSD1306_Puts ("MODE: ", &Font_6x8, 1);
	  SSD1306_Puts (OLED_txt, &Font_6x8, 1);
	  //error code
	  if((led_error!=0)||(led_error_critical!=0)||(bb_error!=0)||(bb_error_critical!=0))
	  {
	  SSD1306_GotoXY (70,8);
	  SSD1306_Puts ("ErrorLB", &Font_6x8, 1);
	  SSD1306_Puts (ERROR_txt, &Font_6x8, 1);
	  }

	  //led power/current
	  SSD1306_GotoXY (0,16);
	  SSD1306_Puts ("LED[%]: ", &Font_6x8, 1);
	  itoa(led_power_set,snum,10);
	  SSD1306_Puts (snum, &Font_6x8, 1);
	  SSD1306_Puts (" |mA ", &Font_6x8, 1);
	  itoa(led_power_mes,snum,10);
	  SSD1306_Puts (snum, &Font_6x8, 1);
	  SSD1306_Puts ("   ", &Font_6x8, 1);

	  //USB currents
	  SSD1306_GotoXY (0,24);
	  SSD1306_Puts ("USB[mA]: ", &Font_6x8, 1);
	  itoa(bb_current_mes,snum,10);
	  SSD1306_Puts (snum, &Font_6x8, 1);
	  SSD1306_Puts (" | ", &Font_6x8, 1);
	  itoa(bb_current_set,snum,10);
	  SSD1306_Puts (snum, &Font_6x8, 1);
	  SSD1306_Puts ("   ", &Font_6x8, 1);

	  SSD1306_UpdateScreen();
}

//diagnostic
void diagnostic (void)
{
	//led - value in mA in 0-100 range
	if(led_power_set!=0)
	{
		if(value_voltage[2]<5)
		{
			if(timer_counter_1>5)	//timer delay
			{
			led_error = 1;	//led circuit is open
			}
		}
		else if(led_power_mes<((led_power_set/2)-10))
		{
			if(timer_counter_1>5) //timer delay
			{
			led_error = 2;	//one led doesnt work
			}
		}
		else if(led_power_mes>(led_power_mes+10))
		{
			led_error = 3;	//short circuit
			led_error_critical = 0;
		}
		else
		{
			led_error = 0;
			timer_counter_1=0;
		}
	}
	else
	{
		led_error = 0;
		timer_counter_1=0;
	}
	//set error code in text
	if(led_error>0)
	{
	itoa(led_error,E_L_txt,10);
	}
	else
	{
	E_L_txt[0]='0';
	}

	//usb/boost converter errors errors
	if((bb_current_set!=0)&&(bb_current_mes>(bb_current_set*2)))
	{
		bb_error = 3;	//zwarcie
		bb_error_critical = 0;
	}
	if((bb_current_set!=0)&&(bb_voltage_mes>6.0f))
	{
		bb_error = 4;	//zwarcie
		bb_error_critical = 0;
	}
	else
	{
		bb_error = 0;
	}
	//set error code in text
	if(bb_error>0)
	{
	itoa(bb_error,E_B_txt,10);
	}
	else
	{
	E_B_txt[0]='0';
	}

	//copy errors to main errors buffer
	ERROR_txt[0]=E_L_txt[0];
	ERROR_txt[1]=E_B_txt[0];
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//switch int
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SW_Pin)
	{
		mode_short++;
		}
}

//timer int 1 s
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM14) {
		timer_counter_1++;	//timer
	}
}
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  //inits
   HAL_ADCEx_Calibration_Start(&hadc1);	//adc calibration
   HAL_ADC_Start_DMA(&hadc1, value_conv, mes_size); //adc dma
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); 	//active pwm 100 kHz
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); 	//active pwm 10 kHz
   HAL_TIM_Base_Start_IT(&htim14);	//activetimer 1 Hz
   SSD1306_Init(); //OLED

   //hello word
   SSD1306_GotoXY (0,0);
   SSD1306_Puts ("INICJALIZACJA",  &Font_6x8 , 1);
   SSD1306_UpdateScreen();
   HAL_Delay (100);
   SSD1306_Clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  adc_read(); //adc read and converting
	  vcc_managment(); //supply managment

	  //mode selection
	  if(mode_short==0)
	  {
		  strcpy(OLED_txt,"OFF");
		  bb_current_set = 0;
		  led_power_set = 0;
	  }
	  else if(mode_short==1)
	  {
		  strcpy(OLED_txt,"LED");
		  bb_current_set = 0;
		  led_power_set = 100;
		  led_power_set = (led_power_set * mode_vcc_led)/100;

	  }
	  else if(mode_short==2)
	  {
		  strcpy(OLED_txt,"USB");
		  bb_current_set = 1000;
		  bb_current_set = (bb_current_set * mode_vcc_usb)/100;
		  led_power_set = 0;

	  }
	  else if(mode_short==3)
	  {
		  strcpy(OLED_txt,"COM");
		  bb_current_set = 600;
		  bb_current_set = (bb_current_set * mode_vcc_usb)/100;
		  led_power_set = 70;
		  led_power_set = (led_power_set * mode_vcc_led)/100;
	  }
	  else
	  {
		  mode_short=0;
	  }

		 OLED_main(); //show

	 //low battery protect
	  if(value_voltage[3]<u_low_batt)
	  {
		  	  //set oled
			   SSD1306_Clear();
			   SSD1306_GotoXY (0,0);
			   //turn off loads
			  bb_current_set = 0;
			  led_power_set = 0;
			  USB_charger(); //usb managment
			  LED_work();	//led managment
			  //show info
			  SSD1306_Puts ("Niski poziom baterii", &Font_6x8, 1);
			  HAL_Delay(1000);
	  }

	  	  //regular work
		  USB_charger(); //usb managment
		  LED_work();	//led managment
		  diagnostic();	//diagnostic

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV32;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
