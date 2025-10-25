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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "Anglas_INA226.h"
#include "fonts.h"
#include "ssd1306.h"
#include "ftoa.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mes_size 4	//adc buffer
#define conv_pwm_arr 2000	//max pwm value
#define soldering_error_value 0	//PTC error value

//variable to control
uint8_t screen_indicator = 0;
uint8_t main_screen = 0;
uint8_t main_screen_old = 0;
int16_t encoder_rot = -1; //variable to use encoder
int16_t encoder_rot_prev = 0; //variable to use encoder

//solder station
uint16_t set_temp = 365;
uint16_t mes_temp = 0;

//converter
int16_t pwm = 0;	//pwm
float set_voltage = 5.0f;	//in V
int16_t set_current = 1000;	//in mA
float mes_voltage = 0.0f;	//in V
int16_t mes_current = 0;	//in mA
int16_t mes_power =0;	//in W

//adc
volatile uint8_t adc_ready =0;	//interupt adc flag
float batt_voltage = 0.0f;	//batt voltage value
uint8_t mode_CC =0;	//CC mode flag
uint16_t adc_raw_value[mes_size]; //raw ADC
uint8_t work_mode =0;	// work mode flag

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

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

// ADC calculation
void adc_read(void) {

	float U_reff = 3.3f; // ADC reference voltage
	uint16_t adc_read_value[mes_size]; // raw ADC readings
	float adc_ready_value[mes_size];   // converted ADC readings to voltage

	uint16_t T_solder = 0; // soldering temperature
	float R_1 = 465.0f;    // resistor value for voltage divider
	float R_PTC = 0.0f;    // PTC resistor value

	// --- FILTERING PARAMETERS ---
	// Increase alpha_* → faster response, less smoothing.
	// Decrease alpha_* → slower response, more stable value.
	// Adjust max_delta_* to match expected system dynamics.
	static float U_conv_filtered = 0.0f;  // filtered converter voltage
//	static float I_conv_filtered = 0.0f; // filtered converter current (disabled)
	const float alpha_U = 0.8f;           // smoothing factor for voltage
//	const float alpha_I = 0.75f;         // smoothing factor for current
	const float max_delta_U = 2.5f;       // max voltage change per sample (V)
//	const float max_delta_I = 100.0f;    // max current change per sample (mA)

	// --- RAW DATA CONVERSION ---
	for (uint8_t i = 0; i < mes_size; i++) {
		adc_read_value[i] = adc_raw_value[i]; // get raw ADC value
		adc_ready_value[i] = (U_reff / 4095.0f) * adc_read_value[i]; // convert ADC value to voltage
	}

	// --- CONVERTER OUTPUT VOLTAGE ---
	float U_conv_raw = adc_ready_value[0] * 11.0f; // raw converter voltage before filtering
	float deltaU = U_conv_raw - U_conv_filtered;   // voltage difference

	// outlier clipping (reject extreme changes)
	if (deltaU > max_delta_U) deltaU = max_delta_U;
	if (deltaU < -max_delta_U) deltaU = -max_delta_U;

	// IIR filter for voltage
	U_conv_filtered += alpha_U * deltaU;
	mes_voltage = U_conv_filtered;
//	mes_voltage = U_conv_raw; // unfiltered version
	mes_voltage = (-0.0003f * mes_voltage * mes_voltage) + (0.9804f * mes_voltage) + 0.0255f; // calibration correction

	// --- PTC TEMPERATURE MEASUREMENT ---
	if (adc_ready_value[1] < 3.2f) {
		R_PTC = -((R_1 * adc_ready_value[1]) / (adc_ready_value[1] - U_reff)); // calculate PTC resistance
		T_solder = (uint16_t) map_f(R_PTC, 50.8f, 110.0f, 22.0f, 400.0f);      // map resistance to temperature
		T_solder = T_solder * 0.50f;                                           // scaling adjustment
	} else {
		T_solder = soldering_error_value; // error value when voltage is too high
	}
	mes_temp = (int16_t)T_solder; // store measured temperature

	// --- BATTERY VOLTAGE ---
	batt_voltage = (adc_ready_value[2] * 11.0f) - 0.1f; // battery voltage after scaling and offset
//	soc_batt = (uint8_t) map_f(U_soc_batt, 18.0f, 21.0f, 0.0f, 100.0f); // state of charge (optional)
//	soc_batt = constrain(soc_batt, 0, 100);

	// --- CURRENT MEASUREMENT ---
	#define INA169_RL 10000  // load resistor of INA169
	#define INA169_RS 0.1    // shunt resistor of INA169

	float I_conv_raw = ((adc_ready_value[3] * 1000.0f) / (INA169_RL * INA169_RS)) * 1000.0f; // current [mA]
	I_conv_raw = constrain_f(I_conv_raw, 0.0f, 5000.0f); // limit current range
//	I_conv_out = roundf(I_conv_raw); // optional rounding

//	float deltaI = I_conv_raw - I_conv_filtered; // current delta
//
//	// reject outliers
//	if (deltaI > max_delta_I) deltaI = max_delta_I;
//	if (deltaI < -max_delta_I) deltaI = -max_delta_I;
//
//	// IIR filter for current
//	I_conv_filtered += alpha_I * deltaI;
//	mes_current = (int16_t)I_conv_filtered; // filtered current
	mes_current = (int16_t)I_conv_raw; // unfiltered current

	// protection against false readings
	if((work_mode == 0) && (mes_current <= 100))
	{
		mes_current = 0;
	}

	// --- POWER CALCULATION ---
	mes_power =  (int16_t)((mes_current / 1000.0f) * mes_voltage); // power = current * voltage
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


//converter screen
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

	if(main_screen == 1)
	{
	SSD1306_Puts("-CONV PID- ", &Font_7x10, 1);
	}
	else
	{
	SSD1306_Puts("-CONV TAB- ", &Font_7x10, 1);
	}

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
    else
	{
	SSD1306_Puts("OFF", &Font_7x10, on_off_sw);
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
    SSD1306_Puts("     ", &Font_7x10, 1);

    //battery level
    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("BATT [V]: ", &Font_7x10, 1);
    ftoa(screen_text_buff, batt_voltage, 1);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("    ", &Font_7x10, 1);

    SSD1306_GotoXY(105, high);
    //wyswietlanie trybu dzialania
    if(mode_CC == 1)
    {
    	SSD1306_Puts("|CC", &Font_7x10, 1);
    }
    else
    {
    	SSD1306_Puts("|CV", &Font_7x10, 1);
    }

    //show data
    SSD1306_UpdateScreen();
    }

// solder station screen
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

    if ((work_mode == 1) && (mes_temp != soldering_error_value))
	{
	SSD1306_Puts("ON ", &Font_7x10, on_off_sw);
	}
    else if ((work_mode == 0) && (mes_temp != 0.0f))
	{
	SSD1306_Puts("OFF", &Font_7x10, on_off_sw);
	}
    else
	{
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
    itoa(mes_power, screen_text_buff, 10);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    //battery level
    high = high + data_step;
    SSD1306_GotoXY(1, high);
    SSD1306_Puts("BATT [V]: ", &Font_7x10, 1);
    ftoa(screen_text_buff, batt_voltage, 1);
    SSD1306_Puts(screen_text_buff, &Font_7x10, 1);
    SSD1306_Puts("  ", &Font_7x10, 1);

    //show data
    SSD1306_UpdateScreen();
    }

//welcome screen
void oled_hello(void)
    {
    //show start screen
    SSD1306_Clear();
    HAL_Delay(10);
    SSD1306_GotoXY(10, 10);
    SSD1306_Puts("INIT OK", &Font_7x10, 1);
    SSD1306_UpdateScreen();
    HAL_Delay(200);
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

//switch manage
void switch_manage (void)
    {
	const uint16_t switch_time[] =
	    {
	    100, 800
	    };

	static uint32_t sw_encoder_last_time = 0;
	uint32_t current_time = HAL_GetTick();

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
    }

// screen manage
void screen_manage (void)
    {
    //change screen if PTC is unconnecteed
    if(mes_temp== soldering_error_value)
	{
	if (main_screen == 0)
	    {
	    main_screen = 1;
	    work_mode = 0;
	    }
	}

    if (main_screen == 0)	//solder station
	{
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
    else if ((main_screen == 1)|| (main_screen == 2))	//converter
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

    //variable for time update the screen
    uint32_t current_time = HAL_GetTick();
    static uint32_t last_oled_time = 0;
    const uint16_t oled_interval = 400;

    //oled update
	if (current_time - last_oled_time > oled_interval)
	    {

	    if ((main_screen == 1)|| (main_screen == 2))
		{
		oled_converter();
		}
	    else if (main_screen == 0)
		{
		oled_solder_station();
		}

	    //czyszczenie ekranu
	    if (main_screen != main_screen_old)
		{
		SSD1306_Clear();
		main_screen_old = main_screen;
		}

	    last_oled_time = current_time;
	    }
    }

//check CC CV mode
uint8_t update_CC_mode(void)
{
    static uint8_t CC_mode = 0;                 // 0 = CV mode, 1 = CC mode
    static uint32_t state_change_timer = 0;     // time when transition counting started
    static uint8_t desired_CC_mode = 0;         // target mode (based on measurement)
    uint32_t current_time = HAL_GetTick();

	#define currest_hist 1
	#define time_CC 50
	#define time_CV 5

    // --- determine which mode we want to have ---
    if (mes_current > set_current)
    {
        desired_CC_mode = 1;   // current exceeded
    }
    else if (mes_current < (set_current - currest_hist))
    {
        desired_CC_mode = 0;   // current dropped by 100 mA
    }
    else
    {
        // dead zone — do not change anything
        return CC_mode;
    }

    // --- if the desired mode differs from the current one ---
    if (desired_CC_mode != CC_mode)
    {
        // start timing if not already running
        if (state_change_timer == 0)
        {
            state_change_timer = current_time;
        }
        else
        {
            uint32_t elapsed = current_time - state_change_timer;

            // logic depending on the direction of change
            if (desired_CC_mode == 1 && elapsed >= time_CC)
            {
                // switch to CC after 0.5 s
                CC_mode = 1;
                state_change_timer = 0;
            }
            else if (desired_CC_mode == 0 && elapsed >= time_CV)
            {
                // return to CV after 3 s
                CC_mode = 0;
                state_change_timer = 0;
            }
        }
    }
    else
    {
        // stable state — reset timer
        state_change_timer = 0;
    }

    return CC_mode;
}



//correction values to r divider
#define N 13
const float PWM_table[N]  = {443, 562, 687, 812, 937, 1062, 1187, 1312, 1437, 1562, 1687, 1812, 1937};
const float Umes_table[N] = {0.42, 0.52, 1.69, 4.33, 7.75, 12.72, 17.77, 21.26, 23.96, 25.99, 27.72, 29.07, 30.32};


// correction function for pwm and voltage output from converter
double getPWM(double targetU) {
    if(targetU <= Umes_table[0]) return PWM_table[0];
    if(targetU >= Umes_table[N-1]) return PWM_table[N-1];

    for(int i = 0; i < N-1; i++) {
        if(targetU >= Umes_table[i] && targetU <= Umes_table[i+1]) {
            double slope = (PWM_table[i+1] - PWM_table[i]) / (Umes_table[i+1] - Umes_table[i]);
            return PWM_table[i] + slope * (targetU - Umes_table[i]);
        }
    }

    return PWM_table[N-1];
}


//loop for converter and soldering station
void work_task(void)
    {
    //work loop
      if (work_mode == 0)	//off mode
		{
		HAL_GPIO_WritePin(OUT_RELAY_GPIO_Port, OUT_RELAY_Pin, RESET);
		HAL_GPIO_WritePin(OUT_CONV_ACTIVE_GPIO_Port, OUT_CONV_ACTIVE_Pin, RESET);
		pwm = 0;
		}
      else	//work mode
  	{
		if ((main_screen == 0) && (mes_temp != soldering_error_value)) // solder station
			{
				HAL_GPIO_WritePin(OUT_RELAY_GPIO_Port, OUT_RELAY_Pin, RESET);
				HAL_GPIO_WritePin(OUT_CONV_ACTIVE_GPIO_Port, OUT_CONV_ACTIVE_Pin,
						SET);

				// --- Parametry PID ---T
				float Kp_temp = 10.0;
				float Ki_temp = 2.7;
				float Kd_temp = 0.8;

				//previous data
				static float last_p_temp_error = 0;
				static float i_temp_error = 0;
				static float d_temp_error = 0;

				//const value
				#define I_temp_error_max 700
				#define Boost_temp_diff_value 150
				#define Boost_pwm_value 1850
				#define Slow_work_diff_temp_value 2
				#define Slow_work_factor 0.98

				//compute PID
				float p_temp_error = set_temp - mes_temp;//accual error

				if ((p_temp_error < Boost_temp_diff_value)
						&& (abs(p_temp_error) >= Slow_work_diff_temp_value))//pid
						{

					i_temp_error = i_temp_error + p_temp_error;	//integral
					i_temp_error = constrain_f(i_temp_error, -I_temp_error_max,
					I_temp_error_max);	//anty windup

					d_temp_error = p_temp_error - last_p_temp_error;

					last_p_temp_error = p_temp_error;	//save previosu error

					float f_pid = (Kp_temp * p_temp_error)
							+ (Ki_temp * i_temp_error) + (Kd_temp * d_temp_error);

					pwm = (int16_t) roundf(f_pid);
				}

				else if (p_temp_error >= Boost_temp_diff_value)	//boost
				{
					pwm = Boost_pwm_value;
					i_temp_error = 0.0f; // reset integral
				}

				else	//dont charge settings
				{
					pwm = (int16_t) roundf((float) pwm * Slow_work_factor);
				}

				// --- protection code---
				static uint32_t overcurrent_start_time = 0; // overcurrent time
				#define Overpower_delay 2000
				#define Overpower_current_value 3000
				#define Overpower_voltage_value 20.0f
				#define Overpower_power_value 50
				#define Overpower_decrease_factor 0.5f

				if ((mes_voltage > Overpower_voltage_value)
						|| (mes_power > Overpower_power_value)
						|| (mes_current > Overpower_current_value)) {
					// record the time during overload
					if (overcurrent_start_time == 0) {
						overcurrent_start_time = HAL_GetTick();
					} else {
						// if the overload still occurs, secure the device
						if (HAL_GetTick()
								- overcurrent_start_time>= Overpower_delay) {
							i_temp_error = 0.0f; // reset integral
							pwm =
									pwm
											- (int16_t) roundf(
													(float) pwm
															* Overpower_decrease_factor); // reduce power
						}
					}
				}

				else {
					// if power is ok, reset time
					overcurrent_start_time = 0;
				}
			}

    else if ((main_screen == 1)|| (main_screen == 2)) //converter
	{
	    HAL_GPIO_WritePin(OUT_RELAY_GPIO_Port, OUT_RELAY_Pin, SET);
	    HAL_GPIO_WritePin(OUT_CONV_ACTIVE_GPIO_Port, OUT_CONV_ACTIVE_Pin, SET);

	    static float new_set_voltage =0.0f;
	    static int16_t last_set_current =0;
	    static float last_set_voltage =0.0f;
	    static uint8_t start_clean_cc =0;
	    static float reduce_voltage =0.0f;

		#define VOLTAGE_STEP 0.0001f  //decrease or increase voltage step

		// --- logic for voltage reduction when overcurrent protection is triggered ---
			if ( update_CC_mode() == 1)   // overcurrent detected
			{
				// gradually reduce voltage until current drops below the threshold
				reduce_voltage += VOLTAGE_STEP;
				reduce_voltage = constrain_f(reduce_voltage, 0.0f, set_voltage);
				mode_CC = 1;
				start_clean_cc = 0;
			}
			else	// overcurrent not active
			{
				if((last_set_current!=set_current)|| (last_set_voltage!=set_voltage)) // pulsation occurring
				{
					start_clean_cc = 1;
				}

			    // if the voltage recovery process is in progress
			    if (start_clean_cc == 1 && reduce_voltage > 0.0f)
			    {
			        reduce_voltage -=  VOLTAGE_STEP; // gradual restoration
			        reduce_voltage = constrain_f(reduce_voltage, 0.0f, set_voltage);
			    }

			    // if voltage has been fully restored – end the process
			    if (reduce_voltage <= 0.0f)
			    {
			        reduce_voltage = 0.0f;
			        start_clean_cc = 0;
			        mode_CC = 0;
			    }

			    if(last_set_voltage!=set_voltage)
			    {
			    	mode_CC = 0;
			    }
			}

			new_set_voltage = set_voltage - reduce_voltage;
			last_set_current = set_current;
			last_set_voltage = set_voltage;

			if (main_screen == 1)
			{
				 // --- PID Parameters ---
		      float Kp_voltage = 5.0f;    // reduced for stability
		      const float Ki_voltage = 0.01;    // smaller integral term (slower growth, fewer oscillations)
		      float Kd_voltage = (1.0f*Kp_voltage)/5;   // stronger D term to dampen oscillations

		      // calculate PID parameters
		      float p_voltage_error = new_set_voltage - mes_voltage;	// calculate current error

		      // PID gain correction
		      if(fabs(p_voltage_error)<0.5f)
		      {
		    	  Kp_voltage = 0.1f;
		    	  Kd_voltage = (1.0f*Kp_voltage)/10.0f;
		      }

		      // previous values
		      static float last_p_voltage_error =0;
		      static float  i_voltage_error =0;
		      static float d_voltage_error = 0;

		      // calculate PID terms
		      if(fabs(p_voltage_error)>=0.01f)
			  {
			      i_voltage_error = i_voltage_error + p_voltage_error;	// calculate integral
			      i_voltage_error = constrain_f(i_voltage_error, -10000.0f, 10000.0f);	// limit integral

			      d_voltage_error = p_voltage_error - last_p_voltage_error;

			      last_p_voltage_error = p_voltage_error;	// store previous error

			      float f_pid = (Kp_voltage*p_voltage_error) + (Ki_voltage*i_voltage_error) + (Kd_voltage*d_voltage_error);

			      pwm = (int16_t)roundf(getPWM(new_set_voltage)) + (int16_t)roundf(f_pid);
			  }
			}
			else if (main_screen == 2)
			{

				pwm = (int16_t)roundf(getPWM(new_set_voltage)); 	// preset imitation mode

			}

	}

  }
	//set pwm
	pwm = constrain(pwm, 0, conv_pwm_arr);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (conv_pwm_arr - pwm));	//ustawienie mocy pwm
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm/4);
    }

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(OUT_CONV_ACTIVE_GPIO_Port, OUT_CONV_ACTIVE_Pin, RESET);
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_Delay(100);

    SSD1306_Init();
    //others modules

    //init
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);	//encoder
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);	//led
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	//converter
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_value, mes_size);

    oled_hello();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
	{
    	//compute adc if is ready
    	if(adc_ready == 1)
    	{
    		adc_read();
			adc_ready = 0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_value, mes_size);
    	}

	switch_manage();
	screen_manage();

	work_task();

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//adc interupt (conversion is done)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adc_ready = 1;
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
