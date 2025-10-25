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
#include "Anglas_INA226.h"
#include "fonts.h"
#include "ssd1306.h"
#include "ftoa.h"
#include "stdlib.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mes_size 4	//size of adc buffer
#define conv_pwm_arr 2000	//pwm max
#define soldering_error_value 0	//solder station error value (based on PTC)

int16_t pwm = 0;	//pwm
uint32_t adc_raw_value[mes_size]; //raw ADC

//interface
//variable to control encoder puschbuttons
uint32_t pushbutton_encoder_counter = 0;
uint32_t pushbutton_encoder = 0;
uint32_t sw_encoder_last_time = 0;
uint32_t current_time = 0;
const uint16_t switch_time[] =
    {
    100, 1000
    };
int16_t encoder_rot = -1;
int16_t encoder_rot_prev = 0;


//variable to control
uint8_t screen_indicator = 0;
uint8_t main_screen = 0;
uint8_t main_screen_old = 0;
uint8_t work_mode = 0;
uint8_t mode_data = 0;

//solder station
uint16_t set_temp = 365;
uint16_t mes_temp = 0;
uint8_t solder_station_error = 0;

//converter
float set_voltage = 5.0f;	//in V
int16_t set_current = 1000;	//in mA
float mes_voltage = 0.0f;	//in V
int16_t mes_current = 0;	//in mA
float mes_power =0.0f;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
//map int
long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
    /*
     * map function or linear transformation
     * x - value to scale
     * in_min - min input value to be scaled
     * in_max - max input value to be scaled
     * out_min - min output value after scaled
     * out_max - max output value after scaled
     */
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
//map float
float map_f(float x, float in_min, float in_max, float out_min, float out_max)
    {
    /*
     *  map function or linear transformation
     * x - value to scale
     * in_min - min input value to be scaled
     * in_max - max input value to be scaled
     * out_min - min output value after scaled
     * out_max - max output value after scaled
     */
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

//constrain int
int64_t constrain(int64_t x, int64_t a, int64_t b)
    {
    /*
     * function to limit the input values
     * x - input value
     * a - lowest value of the range
     * b - highest value of the range
     */

    if (x < a)
	{
	return a;
	}
    else if (x > b)
	{
	return b;
	}
    else
	return x;
    }

//constrain float
float constrain_f(float x, float a, float b)
    {
    /*
     * function to limit the input values
     * x - input value
     * a - lowest value of the range
     * b - highest value of the range
     */

    if (x < a)
	{
	return a;
	}
    else if (x > b)
	{
	return b;
	}
    else
	return x;
    }

//function to scrolling values int
int32_t scroll_value(int64_t input, int64_t min, int64_t max)
    {
    /*
     * function to scrolling the input values
     * input - input value
     * min - lowest value of the range
     * max - highest value of the range
     */

    if (input > max)
	{
	input = min;
	}
    else if (input < min)
	{
	input = max;
	}
    else
	{

	}
    return input;
    }

//function to scrolling values float
float scroll_value_f(float input, float min, float max)
    {
    /*
     * function to scrolling the input values
     * input - input value
     * min - lowest value of the range
     * max - highest value of the range
     */

    if (input > max)
	{
	input = min;
	}
    else if (input < min)
	{
	input = max;
	}
    else
	{

	}
    return input;
    }

//randomize a number from the range
int64_t random_number(int64_t min_rand, int64_t max_rand)
    {
    /*
     * function to randomize a number from the range
     * min_rand - lowest value of the range
     * max_rand - highest value of the range
     */

    int32_t tmp;
    if (max_rand >= min_rand)
	{
	max_rand -= min_rand;
	}
    else
	{
	tmp = min_rand - max_rand;
	min_rand = max_rand;
	max_rand = tmp;
	}
    return max_rand ? (rand() % max_rand + min_rand) : min_rand;
    }

//moving_average
void moving_average(const float *input, float *output, int length,
	int window_size)
    {
    if (window_size <= 0 || window_size > length)
	{
	return;
	}

    for (int i = 0; i <= length - window_size; i++)
	{
	float sum = 0.0f;
	for (int j = 0; j < window_size; j++)
	    {
	    sum += input[i + j];
	    }
	output[i] = sum / window_size;
	}
    }

//adc processing
float adc_read(uint8_t mode_out)
    {

    float U_reff = 3.3f;
    uint16_t adc_read_value[mes_size];
    float adc_ready_value[mes_size];

    float U_soc_batt = 0.0f;
    uint8_t soc_batt = 0;
    uint16_t T_solder = 0;
    float U_conv_out = 0.0f;
    float I_conv_out = 0.0f;

    float R_1 = 465.0f;
    float R_PTC = 0.0f;

    for (uint8_t i = 0; i <= (mes_size - 1); i++)
	{
	adc_read_value[i] = adc_raw_value[i];
	}

    for (uint8_t i = 0; i <= (mes_size - 1); i++)
	{
	adc_ready_value[i] = (U_reff / 4096) * adc_read_value[i];
	}

    //converter out coltage
    U_conv_out = adc_ready_value[0] * 11;

    // PTC temperature
    if (adc_ready_value[1] < 3.2f)
	{
	R_PTC = -((R_1 * adc_ready_value[1]) / (adc_ready_value[1] - U_reff));
	T_solder = (uint16_t) map_f(R_PTC, 50.8f, 110.0f, 22.0f, 400.0f);	//alternatywne nastawy 27C 53.3 ohm, 525C 223.7 ohm
	T_solder = T_solder * 0.50f;

	}
    else
	{
	T_solder = soldering_error_value;
	}

    //battery calculation
    U_soc_batt = (adc_ready_value[2] * 11) - 0.1f;
    soc_batt = (uint8_t) map_f(U_soc_batt, 18.0f, 21.0f, 0.0f, 100.0f);
    soc_batt = constrain(soc_batt, 0, 100);

    if (mode_out == 1)
	{
	return U_soc_batt;
	}
    else if (mode_out == 2)
	{
	return (float) soc_batt;
	}
    else if (mode_out == 3)
	{
	return (float) T_solder;
	}
    else if (mode_out == 4)
	{
	return U_conv_out;
	}
    else if (mode_out == 5)
	{
	return I_conv_out;
	}
    else
	{
	return 0;
	}

    }

//function to scrolling values from encoder to int
int32_t encoder_managment(int32_t input, int32_t min, int32_t max, int32_t step,
	uint8_t mode)
    {
    /*
     * function scroll values from encoder on the screen depending on the specified parameters
     * input - value to scroll
     * min - min scroll value
     * max - max scroll value
     * step - step to scroll
     * mode - if 1 is set the maximum value is not scrolled to the minimum value , if 0 after passing the maximum value returns the minimum value
     */

    encoder_rot = ( __HAL_TIM_GET_COUNTER(&htim3) >> 1); //encoder counter

    if (encoder_rot_prev > encoder_rot)
	{
	input = input - step;
	encoder_rot_prev = encoder_rot;
	}
    else if (encoder_rot_prev < encoder_rot)
	{
	input = input + step;
	encoder_rot_prev = encoder_rot;
	}
    else
	{

	}

    if (mode == 1)
	{
	input = constrain(input, min, max);
	}
    else
	{
	input = scroll_value(input, min, max);
	}

    return input;
    }

//function to scrolling values from encoder to float
float encoder_managment_f(float input, float min, float max, float step,
	uint8_t mode)
    {
    /*
     * function scroll values from encoder on the screen depending on the specified parameters
     * input - value to scroll
     * min - min scroll value
     * max - max scroll value
     * step - step to scroll
     * mode - if 1 is set the maximum value is not scrolled to the minimum value , if 0 after passing the maximum value returns the minimum value
     */

    encoder_rot = ( __HAL_TIM_GET_COUNTER(&htim3) >> 1); //encoder counter


    if (encoder_rot_prev > encoder_rot)
	{
	input = input - step;
	encoder_rot_prev = encoder_rot;
	}
    else if (encoder_rot_prev < encoder_rot)
	{
	input = input + step;
	encoder_rot_prev = encoder_rot;
	}
    else
	{

	}

    if (mode == 1)
	{
	input = constrain_f(input, min, max);
	}
    else
	{
	input = scroll_value_f(input, min, max);
	}

    return input;
    }

//screen converter
void oled_converter(void)
    {

    char screen_text_buff[5];
    uint8_t high_init = 0;
    uint8_t high = high_init;
    uint8_t data_step = 10;
    uint8_t on_off_sw = 0;
    uint8_t u_set_sw = 0;
    uint8_t i_set_sw = 0;

    SSD1306_GotoXY(1, high);
    SSD1306_Puts("-SUPPLY- ", &Font_7x10, 1);

    if (screen_indicator == 0)
	{
	on_off_sw = 0;
	u_set_sw = 1;
	i_set_sw = 1;
	}
    else if (screen_indicator == 1)
	{
	on_off_sw = 1;
	u_set_sw = 0;
	i_set_sw = 1;
	}
    else
	{
	on_off_sw = 1;
	u_set_sw = 1;
	i_set_sw = 0;
	}

    if (work_mode == 1)
	{
	SSD1306_Puts("ON ", &Font_7x10, on_off_sw);
	}
    else if (work_mode == 0)
	{
	SSD1306_Puts("OFF", &Font_7x10, on_off_sw);
	}
    else
	{
	SSD1306_Puts("ERR", &Font_7x10, on_off_sw);
	}

    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("U Set [V]: ", &Font_7x10, u_set_sw);
    ftoa(screen_text_buff, set_voltage, 1);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("I Set [mA]: ", &Font_7x10, i_set_sw);
    itoa(set_current, screen_text_buff, 10);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("U Mes [V]: ", &Font_7x10, 1);
    ftoa(screen_text_buff, mes_voltage, 1);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("I Mes [mA]: ", &Font_7x10, 1);
    itoa(mes_current, screen_text_buff, 10);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("   ", &Font_7x10, 1);

    //battery level
    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("BATT [V]: ", &Font_7x10, 1);
    ftoa(screen_text_buff, adc_read(1), 1);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    //show data
    SSD1306_UpdateScreen();
    }


//screen solder station
void oled_solder_station(void)
    {

    //data to set cursor
    char screen_text_buff[5];
    uint8_t high_init = 0;
    uint8_t high = high_init;
    uint8_t data_step = 10;
    uint8_t on_off_sw = 0;
    uint8_t t_set_sw = 0;

    if (screen_indicator == 0)
	{
	on_off_sw = 0;
	t_set_sw = 1;
	}
    else
	{
	on_off_sw = 1;
	t_set_sw = 0;
	}

    //show title
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("-SOLDER- ", &Font_7x10, 1);


    solder_station_error = 0;
    if ((work_mode == 1) && (adc_read(3) != soldering_error_value))
	{
	SSD1306_Puts("ON ", &Font_7x10, on_off_sw);
	}
    else if ((work_mode == 0) && (adc_read(3) != 0.0f))
	{
	SSD1306_Puts("OFF", &Font_7x10, on_off_sw);
	}
    else
	{
	solder_station_error = 1;
	SSD1306_Puts("ERR", &Font_7x10, on_off_sw);
	}

    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("Temp Set[C]: ", &Font_7x10, t_set_sw);
    itoa(set_temp, screen_text_buff, 10);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("Temp Mes[C]: ", &Font_7x10, 1);
    itoa(mes_temp, screen_text_buff, 10);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    //solder power
    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("POW [W]: ", &Font_7x10, 1);
    ftoa(screen_text_buff, mes_power, 1);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    //battery level
    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("BATT [V]: ", &Font_7x10, 1);
    ftoa(screen_text_buff, adc_read(1), 1);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    //show data
    SSD1306_UpdateScreen();
    }

//welcome screen
void oled_hello(void)
    {
    //show start screen
    HAL_Delay(100);
    SSD1306_GotoXY(10, 10);
    SSD1306_Puts("INIT OK", &Font_7x10, 1);
    SSD1306_UpdateScreen();
    HAL_Delay(500);
    SSD1306_Clear();
    }

//debug screen
void oled_debug(uint8_t x, uint8_t y, char name[], int32_t data,
	uint32_t time)
    {
    char label[6];
    SSD1306_Clear();
    SSD1306_GotoXY(x, y);
    SSD1306_Puts(name, &Font_7x10, 1);
    SSD1306_Puts(" ", &Font_7x10, 1);
    itoa(data, label, 10);
    SSD1306_Puts(label, &Font_7x10, 1);
    SSD1306_Puts(" ", &Font_7x10, 1);
    SSD1306_UpdateScreen();
    HAL_Delay(time);
    SSD1306_Clear();
    }

//main task
void interface_task(void)
    {

    //switch function
    if (HAL_GPIO_ReadPin(ENCODER_SW_GPIO_Port, ENCODER_SW_Pin)
	    == GPIO_PIN_RESET)
	{

	}
    else
	{
	if ((current_time - sw_encoder_last_time) >= switch_time[1])
	    {
	    main_screen++;
	    }
	else if (((current_time - sw_encoder_last_time) > switch_time[0])
		&& ((current_time - sw_encoder_last_time) < switch_time[1]))
	    {
	    screen_indicator++;
	    }
	else
	    {

	    }
	sw_encoder_last_time = current_time;
	}

    //if solder iron is unconected switch screen
    if(adc_read(3) == soldering_error_value)
	{
	if (main_screen == 0)
	    {
	    main_screen = 1;
	    work_mode = 0;
	    }
	}

    if (main_screen == 0)	//solder station mode
	{
	mes_temp = (uint16_t) adc_read(3);
	mes_power = INA226_Power();

	if (screen_indicator == 1)
	    {
	    set_temp = encoder_managment(set_temp, 30, 500, 5, 1);
	    }
	else if (screen_indicator == 0)
	    {
	    work_mode = encoder_managment(work_mode, 0, 1, 1, 1);
	    }
	else
	    {
	    screen_indicator = 0;
	    }
	}
    else if (main_screen == 1)	//supply source mode
	{
	if (screen_indicator == 1)
	    {
	    set_voltage = encoder_managment_f(set_voltage, 0.0f, 32.0f, 0.1f,
		    1);
	    }
	else if (screen_indicator == 2)
	    {
	    set_current = encoder_managment(set_current, 0, 3000, 50 , 1);
	    }
	else if (screen_indicator == 0)
	    {
	    work_mode = encoder_managment(work_mode, 0, 1, 1, 1);
	    }
	else
	    {
	    screen_indicator = 0;
	    }
	}
    else	//reset state
	{
	main_screen = 0;
	work_mode = 0;
	screen_indicator = 0;
	}

    //settings for screen update
    current_time = HAL_GetTick();
    static uint32_t last_oled_time = 0;
    const uint32_t oled_interval = 300;

    //screen update procedure
	if (current_time - last_oled_time > oled_interval)
	    {

	    if (main_screen == 1)
		{
		oled_converter();
		}
	    else if (main_screen == 0)
		{
		oled_solder_station();
		}

	    //clear screen if is new
	    if (main_screen != main_screen_old)
		{
		SSD1306_Clear();
		main_screen_old = main_screen;
		}

	    last_oled_time = current_time;
	    }


    //read adc
    mes_voltage = INA226_Vbus();
    mes_current = (uint16_t) INA226_Current();

    //work
    if (work_mode == 0)	//off
	{
	HAL_GPIO_WritePin(OUT_RELAY_GPIO_Port, OUT_RELAY_Pin, RESET);

	HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, RESET);

	if(main_screen == 1)
	    {
	    //proximity range of pwm in off mode if device is active
	    pwm =(int16_t) map_f(fabs(set_voltage - mes_voltage), 0.0f, 32.0f, 1990.0f, 40.0f);
	    }
	else
	    {

	    pwm = conv_pwm_arr;
	    }

	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm);	//off converter

	HAL_GPIO_WritePin(OUT_CONV_ACTIVE_GPIO_Port, OUT_CONV_ACTIVE_Pin, RESET);
	}
    else	//on
	{
	if ((main_screen == 0) && (adc_read(3) != soldering_error_value))//solder station
	    {
	    HAL_GPIO_WritePin(OUT_RELAY_GPIO_Port, OUT_RELAY_Pin, RESET);
	    HAL_GPIO_WritePin(OUT_CONV_ACTIVE_GPIO_Port, OUT_CONV_ACTIVE_Pin, SET);

	    uint16_t temp_diff = abs(set_temp-mes_temp);
	    if(temp_diff>20)
		{
		    temp_diff = map(temp_diff, 1, 100, 1, 400);
		    temp_diff = constrain(temp_diff, 1, 400);
		}
	    else
		{
		   temp_diff = 1;
		}

		     if(set_temp>mes_temp)
		      {

		      if( (mes_voltage<21.0f) && (mes_power<50.0f) && (mes_current<3000.0f))
			  {
			      pwm = pwm - (1*temp_diff);
			      HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, SET);
			  }
		      else
			  {

			  }
		      }
		      else if((set_temp<mes_temp) || (mes_voltage>20.0f+1.0f) || (mes_power>47.5f+5.0f) || (mes_current>3000+100))	//decrease powem
		      {
		      HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, RESET);
		      pwm = pwm + (1*temp_diff);
		      }
	    }
	else if (main_screen == 1) //converter
	    {
	    HAL_GPIO_WritePin(OUT_RELAY_GPIO_Port, OUT_RELAY_Pin, SET);
	    HAL_GPIO_WritePin(OUT_CONV_ACTIVE_GPIO_Port, OUT_CONV_ACTIVE_Pin, SET);

	    float voltage_diff = fabs(set_voltage - mes_voltage);
	    static float prev_voltage_diff = 0;

	    //factors
	    const float voltage_lim_1 = 0.1f;
	    const float voltage_lim_2 = 2.5f;
	    float voltage_multiple_factor = 2.5f;
	    float prev_voltage_multiple_factor = 1.5f;

	    //factors limiting voltage fluctuations
	    if(mes_current<=2)
		{
		   voltage_multiple_factor = 1.0f;
		   prev_voltage_multiple_factor = 0.0f;
		}

	    // PWM logic
	    int16_t pwm_delta = 0;


	    if (set_voltage < mes_voltage)//decrease power - voltage
		{
		if ((voltage_diff <= voltage_lim_2)
			&& (voltage_diff >= voltage_lim_1))
		    {
		    pwm_delta = 1;
		    }
		else if (voltage_diff > voltage_lim_2)
		    {
		    pwm_delta = (int16_t) (voltage_diff
			    * voltage_multiple_factor
			    + prev_voltage_diff * prev_voltage_multiple_factor);
		    }
		}
	    else if  (mes_current > set_current)//decrease power - current
		{
		    pwm_delta = 1;

		    HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, RESET);
		}
	    else if ((set_voltage > mes_voltage) && (mes_current < set_current))//increase power
		{
		 HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, SET);

		if ((voltage_diff <= voltage_lim_2)
			&& (voltage_diff >= voltage_lim_1))
		    {
		    pwm_delta = -1;
		    }
		else if (voltage_diff > voltage_lim_2)
		    {
		    pwm_delta = -(int16_t) (voltage_diff
			    * voltage_multiple_factor
			    + prev_voltage_diff * prev_voltage_multiple_factor);
		    }
		}

	    //limiting pwm per one step
	    const int16_t pwm_step_limit = 150;

	    if (pwm_delta > pwm_step_limit)
		{
		pwm_delta = pwm_step_limit;
		}
	    if (pwm_delta < -pwm_step_limit)
		{
		pwm_delta = -pwm_step_limit;
		}

	    pwm += pwm_delta;

	    prev_voltage_diff = voltage_diff;
	    }

	//set pwm
	pwm = constrain(pwm, 0, conv_pwm_arr);
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm);
	}
    }

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC1_Init(void);
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
    HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

    HAL_Delay(100);

    SSD1306_Init();

//    INA226_Init(3000, 19, AVG_16, T_Vbus_204us, T_Vshunt_204us,
//	    MODE_SHUNT_BUS_CONTINUOUS);	//settings for XL6009

    INA226_Init(3000, 16, AVG_16, T_Vbus_140us, T_Vbus_140us,
	    MODE_SHUNT_BUS_CONTINUOUS);	//settings for XL6019

    //others modules
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);	//encoder
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
    HAL_ADC_Start_DMA(&hadc1, adc_raw_value, mes_size);

    oled_hello();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
	{
	interface_task();

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  hi2c1.Init.Timing = 0x0060112F;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 9;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_CONV_ACTIVE_GPIO_Port, OUT_CONV_ACTIVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_RELAY_GPIO_Port, OUT_RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OUT_CONV_ACTIVE_Pin */
  GPIO_InitStruct.Pin = OUT_CONV_ACTIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_CONV_ACTIVE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_LED_Pin */
  GPIO_InitStruct.Pin = OUT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_SW_Pin */
  GPIO_InitStruct.Pin = ENCODER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_RELAY_Pin */
  GPIO_InitStruct.Pin = OUT_RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_RELAY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
