/* USER CODE BEGIN Header */
/**

 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include <adc.h>
#include <AD9833.h>
#include <cmsis_gcc.h>
#include <dma.h>
#include <fonts.h>
#include <ftoa.h>
#include <gpio.h>
#include <i2c.h>
#include <moving_average.h>
#include <ssd1306.h>
#include <stm32f103xb.h>
#include <stm32f1xx.h>
#include <stm32f1xx_hal_adc.h>
#include <stm32f1xx_hal_adc_ex.h>
#include <stm32f1xx_hal_def.h>
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_rcc.h>
#include <stm32f1xx_hal_rcc_ex.h>
#include <stm32f1xx_hal_tim.h>
#include <tim.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//constants values
#define mes_size 8 //number of ADC channels

//ADC factors
#define n_batt 2.02f	//battery voltage divider multiplier
#define f_batt 0.1f	 //battery voltage factor (adding to value)
#define n_pos_vdc 11.15f	//positive voltage divider (to opamp) multiplier
#define f_pos_vdc 0.1f	//positive voltage divider (to opamp) factor (adding to value)
#define n_conv_voltage_out 11.01f	//converter voltage divider multiplier
#define f_gain 0.05f //factor to opamp gain

//voltage values (references)
#define U_reff 3.33f //reference voltage to converting voltages value
#define U_bb_max 25.0f //max voltage [in V] value possible to set in converter
#define I_bb_max 2000.0f //max current [in mA] value possible to set in converter
#define U_I_mes_bb_max 1.00f// max voltage [in V] in the shunt for current measurements (at the max possible current)
#define U_batt_min 2.7f// min voltage in battery
#define U_batt_max 4.2f// max voltage in battery

/*
 * UNITS:
 * VOLTAGE -  [V]
 * CURRENT - [mA]
 * FREQUENCY - [Hz]
 * PWM - [-]
 *
 * CODE IS DEDICATED TO WORK WITH HARDWARE IN 2.3.1 VERSION
 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//NECESSARY VALUES

//converter
int16_t conv_pwm_control = 0;	//PWM value to converter transistor
float conv_voltage_set = 5.0f; //voltage set on the converter - with initial value
float conv_voltage_mes = 0.0f; //voltage read from ADC (after processing)
float conv_current_set =500.0f ;//current set on the converter - with initial value
float conv_current_mes = 0.0f; //current read from ADC (after processing)
uint8_t conv_sw_start = 0; //converter start stop variable
uint8_t conv_sw_start_prev = 0; //converter start stop variable - previous state
uint8_t conv_led_red = 0; //red LED (overcurrent protection) state variable
uint8_t conv_led_green = 0; //green (on/off) LED state variable
float conv_prev_voltage_diff = 0.0f;	//previous voltage difference between set and measure voltage
float conv_actual_voltage_diff = 0.0f;	//actual voltage difference between set and measure voltage
const uint16_t conv_pwm_min=0; //min PWM filling threshold
const uint16_t conv_pwm_max=80; //max PWM filling threshold (to protect circuit about short)
const uint16_t conv_pwm_arr_value=360; //max PWM filling - ARR value

//wave generator
float wg_offset_set = 0.0f; //voltage offset output value
float wg_offset_raw = 0.0f; //raw offset value - before converting on real value of offset
uint8_t wg_sw_start = 0; //on off wave generator state
uint8_t wg_led_blue = 0; //blue(on/off) LED state variable
uint32_t dac_freq = 1000; //frequency set on the wg - with initial value
uint8_t wg_wave_mode = 0; //type of generated wave (sine, square, triangle)
const uint32_t wg_max_frequency = 300000; //max wave frequency possible to set
float wg_gain = 0.0f; //gain output signal value
float wg_raw_amplitude = 0.0f;	//raw amplitude value (without gain)
uint8_t wg_offset_ovr = 0;	//overrange of supply voltage for offset
uint8_t wg_out_ovr = 0;	//overrange of supply voltage for wg output
uint8_t wg_out_state = 0;	//wg start stop variable
float wg_cal_out = 0.0f; //calculated output wg value

//ADC and supply
float vcc_voltage_batt_mes = 0.00f; //battery voltage
uint8_t vcc_batt_soc = 0; //battery soc in percent
const uint8_t batt_low_soc = 20; //low battery level in percent
uint8_t batt_low_state = 0;	//low battery state variable
uint8_t error_screen_delay = 0;	//variable to turn on supply error screens after delay (to stabilize the power supply in the opamp section)
uint8_t power_error_state = 0;	//if =1 error screen was show
float pos_vdc_voltage_mes = 0; //positive voltage value to opamp section
float neg_vdc_voltage_mes = 0; //negative voltage value to opamp section
const float pos_vdc_voltage_low = 8.8f; //positive voltage value to set supply trouble
const float neg_vdc_voltage_low = -8.0f; //negative voltage value to set supply trouble
uint32_t adc_raw_value[mes_size]; //ADC raw values buffer
uint16_t adc_value_conv[mes_size]; //ADC filtered values buffer
volatile float value_voltage[mes_size]; //buffer for voltage values from ADC (after conversion on voltage)

//encoder - optimization is needed
volatile uint8_t encoder_switch_short = 0; //short hold variable
uint8_t encoder_switch_short_1 = 0; //short hold variable
uint8_t encoder_switch_short_2 = 0; //short hold variable
volatile uint8_t encoder_switch_long = 0; //long hold variable
uint8_t encoder_switch_long_1 = 0; //long hold variable
uint8_t encoder_switch_long_2 = 0; //long hold variable
volatile uint8_t encoder_switch_double_long = 1; //very long hold variable
uint8_t encoder_switch_double_long_1 = 1; //very long hold variable
uint8_t encoder_switch_double_long_2 = 1; //very long hold variable
uint64_t encoder_switch_timer = 0; //variable to save hold encoder pushbutton moment
volatile uint16_t encoder_rot =0; //ncoder rotation value
uint16_t encoder_rot_prev =0; //previous encoder rotation value
const uint8_t endoer_hold_times[]= {1,5,10,15};	//hold times for encoder pushbutton

//timers
volatile uint8_t timer_3_counter_main = 0;	//main timer counter to generate time base and delays
const uint8_t timer_3_counter_main_max_fac = 10; //main timer counter - value to generate 1 s
volatile uint8_t timer_3_counter_encoder_sw = 0;	//timer counter to encoder pushbutton hold mode
volatile uint8_t timer_3_counter_conv_sync = 0;	//timer for converter reset after asynchronization
volatile uint8_t timer_3_counter_error_check_delay = 0;	//timer for generate delay to error screens
const uint8_t timer_3_counter_error_check_delay_fac = 20; //value of delay to timer for generate delay to error screens
volatile uint8_t timer_3_counter_graph = 0;	//timer to draw x value on a graph
const uint8_t timer_3_counter_graph_max_fac = 5; //value for generate time base for graph (multiplication factor to 10 ms)

//screen
char screen_text_buff[6];	//text buffer
uint8_t screen_cursor_config[3];	//table with cursor configuration for any screen
/*
 * pos 0 - 1=on, 0=0ff
 * pos 1 - number of first line with cursor
 * pos 2 - number of cursor step (in line)
 */
uint8_t screen_new =1;	//new screen was set flag
uint8_t screen_old_num = 0;	//previous screen number
uint16_t screen_multiplier = 10;	//multiplier of the selected value on the screen
const uint16_t screen_multiplier_value[] = {1, 10, 100, 1000}; //possible multipliers for selected values
const uint8_t screen_indicator_step_vertical = 9;	//number of pixels to be jumped by the cursor (during selecting value)
const uint8_t indicator_screens_number = 2;	//number of screens with cursor - view in code

//graph
uint8_t x_value_graph = 0;	//graph value in horizontal (in this code is a time on graphs)
const uint8_t x_value_graph_max_fac = 100; //max value for above
double ox_last_value_graph = 0;	//last value x on a graph
double oy_last_value_graph = 0; //last value y on a graph
uint8_t init_new_graph = 1;	//variable to set new coordinate system on the graph
uint8_t graph_set = 0;	//variable to select graph
uint8_t graph_set_prev = 0;	//previous selected graph number
const uint8_t graph_screen_max = 2;	//number of all screens with a graph

//filters
FilterTypeDef_0 filterStruct_slow[mes_size]; //moving average structure - slow
FilterTypeDef_1 filterStruct_fast[mes_size]; //moving average structure - fast

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//map int
long map(long x, long in_min, long in_max, long out_min, long out_max) {
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
float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
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
long constrain(long x, long a, long b) {
	/*
	 * function to limit the input values
	 * x - input value
	 * a - lowest value of the range
	 * b - highest value of the range
	 */

	if (x < a) {
		return a;
	} else if (x > b) {
		return b;
	} else
		return x;
}

//constrain float
float constrain_f(float x, float a, float b) {
	/*
	 * function to limit the input values
	 * x - input value
	 * a - lowest value of the range
	 * b - highest value of the range
	 */

	if (x < a) {
		return a;
	} else if (x > b) {
		return b;
	} else
		return x;
}

//function to scrolling values int
int32_t scroll_value (int64_t input, int64_t min, int64_t max )
{
	/*
	 * function to scrolling the input values
	 * input - input value
	 * min - lowest value of the range
	 * max - highest value of the range
	 */

	if(input>max)
	{
		input = min;
	}
	else
	{

	}
	return input;
}

//function to scrolling values float
float scroll_value_f (float input,float min, float max )
{
	/*
	 * function to scrolling the input values
	 * input - input value
	 * min - lowest value of the range
	 * max - highest value of the range
	 */

	if(input>max)
	{
		input = min;
	}
	else
	{

	}
	return input;
}

//randomize a number from the range
int64_t random_number (int64_t min_rand, int64_t max_rand)
{
	/*
	 * function to randomize a number from the range
	 * min_rand - lowest value of the range
	 * max_rand - highest value of the range
	 */

	int32_t tmp;
    if (max_rand>=min_rand)
    {
        max_rand-= min_rand;
    }
    else
    {
        tmp= min_rand - max_rand;
        min_rand= max_rand;
        max_rand= tmp;
    }
    return max_rand ? (rand() % max_rand + min_rand) : min_rand;
}

//graph function
void draw_graph(double x, double y, double graph_orgin_x, double graph_orgin_y,
		double graph_w, double graph_h, double xlo_value, double xhi_value,
		double ylo__value, double yhi__value, char title[], uint8_t data_type,
		char time_step[]) {
	/*
	 * function to make graph y=f(x), adapted to work with a library to ssd1306
	 * x - input x value (horizontal), like as time
	 * y - input y value (vertical), lie as voltage
	 * graph_orgin_x - origin point of the graph on the x axis (origin of screen is in high, left corner)
	 * graph_orgin_y - origin point of the graph on the y axis (origin of screen is in high, left corner)
	 * graph_w - graph width
	 * graph_h - graph height
	 * xlo_value - lowest value on the x axis
	 * xhi_value - highest value on the x axis
	 * ylo_value - lowest value on the y axis
	 * yhi_value - highest value on the y axis
	 * title - title of the graph
	 * data_type - data type, 1 if int is used, 0 if float is used,
	 * time_step - step of x value, on the label of the graph
	 */

	//text offset values (near axis)
	uint8_t x_addicional_space = 25;
	uint8_t y_addicional_space = 2;

	if (init_new_graph == 1) {

		init_new_graph = 0;
		SSD1306_DrawFilledRectangle(0, 0, 127, 10, 1);

		SSD1306_GotoXY(1, 2);

		SSD1306_Puts(title, &Font_6x8, 0);
		SSD1306_Puts(": ", &Font_6x8, 0);
		if (data_type == 1) {
			itoa((int16_t) y, screen_text_buff, 10);
		} else {
			ftoa(screen_text_buff, y, 1);
		}
		SSD1306_Puts(screen_text_buff, &Font_6x8, 0);
		SSD1306_Puts(time_step, &Font_6x8, 0);

		ox_last_value_graph = (x - xlo_value) * (graph_w)
				/ (xhi_value - xlo_value) + graph_orgin_x;
		oy_last_value_graph = (y - ylo__value)
				* (graph_orgin_y - graph_h - graph_orgin_y)
				/ (yhi__value - ylo__value) + graph_orgin_y;

		// draw y scale
		SSD1306_DrawLine(graph_orgin_x, graph_orgin_y, graph_orgin_x,
				graph_orgin_y - graph_h, 1);

		SSD1306_GotoXY(graph_orgin_x - x_addicional_space, graph_orgin_y);

		if (data_type == 1) {
			itoa((int16_t) ylo__value, screen_text_buff, 10);
		} else {
			ftoa(screen_text_buff, ylo__value, 1);
		}
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);

		SSD1306_GotoXY(graph_orgin_x - x_addicional_space, graph_orgin_y - graph_h);

		if (data_type == 1) {
			itoa((int16_t) yhi__value, screen_text_buff, 10);
		} else {
			ftoa(screen_text_buff, yhi__value, 1);
		}
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);

		// draw x scale
		SSD1306_DrawLine(graph_orgin_x, graph_orgin_y, graph_orgin_x + graph_w,
				graph_orgin_y, 1);

		SSD1306_GotoXY(graph_orgin_x, graph_orgin_y + y_addicional_space);

		if (data_type == 1) {
			itoa((int16_t) xlo_value, screen_text_buff, 10);
		} else {
			ftoa(screen_text_buff, xlo_value, 1);
		}
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);

		SSD1306_GotoXY(graph_orgin_x + graph_w, graph_orgin_y + y_addicional_space);

		if (data_type == 1) {
			itoa((int16_t) xhi_value, screen_text_buff, 10);
		} else {
			ftoa(screen_text_buff, xhi_value, 1);
		}
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	}

	// graph drawn now plot the data
	x = (x - xlo_value) * (graph_w) / (xhi_value - xlo_value) + graph_orgin_x;
	y = (y - ylo__value) * (graph_orgin_y - graph_h - graph_orgin_y)
			/ (yhi__value - ylo__value) + graph_orgin_y;
	SSD1306_DrawLine(ox_last_value_graph, oy_last_value_graph, x, y, 1);
	SSD1306_DrawLine(ox_last_value_graph, oy_last_value_graph - 1, x, y - 1, 1);
	ox_last_value_graph = x;
	oy_last_value_graph = y;

	// this call sends the data to the screen
	SSD1306_UpdateScreen();

}

//function to calculate voltage inputing to ADC from voltage divider with negative level
float adc_signal_shifter (float r1, float r2, float r3, float Vcc, float Vx)
{
	/*
	 *more about function in calculation sheet
	 * r1 - resistor 1 (Vin)
	 * r2 - resistor 2 (Vcc)
	 * r3 - resistor 3 (ground or the lowest voltage)
	 * Vcc - divider supply voltage
	 * Vx - input voltage (to divider)
	 */
 return ((((Vx*((r2*r3)+(r1*r3)+(r1*r2)))/r3) - (Vcc*r1))/r2);
}

//function with correct factors to offset calculation (obtained using the signal measured by the uC and a multimeter)
float final_offset_correction(float input_offset) {
	/*
	 * input_offset - raw value of ADC voltage, without correction
	 */
	if (input_offset <= -5.0f) {
		input_offset = input_offset * 0.77;
	} else if ((input_offset > -5.0f) && (input_offset <= -4.0f)) {
		input_offset = input_offset * 0.74;
	} else if ((input_offset > -4.0f) && (input_offset <= -3.0f)) {
		input_offset = input_offset * 0.72;
	} else if ((input_offset > -3.0f) && (input_offset <= -2.0f)) {
		input_offset = input_offset * 0.73f;
	} else if ((input_offset > -2.0f) && (input_offset <= -1.0f)) {
		input_offset = input_offset * 0.67f;
	} else if ((input_offset > -1.0f) && (input_offset < 0.0f)) {
		input_offset = input_offset * 0.62f;
	} else if ((input_offset > 0.0f) && (input_offset <= 1.0f)) {
		input_offset = input_offset * 1.01f;
	} else if ((input_offset > 1.0f) && (input_offset <= 2.0f)) {
		input_offset = input_offset * 0.95;
	} else if ((input_offset > 2.0f) && (input_offset <= 3.0f)) {
		input_offset = input_offset * 0.88;
	} else if ((input_offset > 3.0f) && (input_offset <= 4.0f)) {
		input_offset = input_offset * 0.82;
	} else if ((input_offset > 4.0f) && (input_offset <= 5.0f)) {
		input_offset = input_offset * 0.83;
	} else if ((input_offset > 5.0f) && (input_offset <= 6.0f)) {
		input_offset = input_offset * 0.82;
	} else if ((input_offset > 6.0f) && (input_offset <= 7.0f)) {
		input_offset = input_offset * 0.80f;
	} else if ((input_offset > 7.0f) && (input_offset <= 8.0f)) {
		input_offset = input_offset * 0.79;
	} else {
		input_offset = input_offset;
	}

	return input_offset;
}

//function to read ADC from multiple channels and processing the values
void adc_read(void) {

	HAL_ADC_Start_DMA(&hadc1, adc_raw_value, mes_size); //read ADC values

	//use the moving average if is needed
	for (uint8_t i = 0; i <= mes_size - 1; i++) {

		if ((i == 0) || (i == 1) || (i == 7)) {	//slow moving average - to voltages like as supply, pos and neg to op amp

			adc_value_conv[i] = Moving_Average_Compute_0(adc_raw_value[i],
					&filterStruct_slow[i]);

		}

		else if ((i == 2) || (i == 6) || (i == 5) || (i == 4)) {//fast moving average - to potentiometers and converter current

			adc_value_conv[i] = Moving_Average_Compute_1(adc_raw_value[i],
					&filterStruct_fast[i]);
		}

		else {
			adc_value_conv[i] = adc_raw_value[i]; // without moving average - for other signals
		}

		value_voltage[i] = (U_reff / 4095.0f) * adc_value_conv[i];//conversion to voltage
	}

	//further voltage processing

	//channel 9 - li-ion battery voltage - conversion to SOC
	vcc_voltage_batt_mes = (value_voltage[0] * n_batt) + f_batt;
	vcc_batt_soc = map_f(vcc_voltage_batt_mes, U_batt_min, U_batt_max, 0, 100);
	vcc_batt_soc = constrain_f(vcc_batt_soc, 0, 100);

	//channel 8 - step up converter voltage to supply op amp circuit (with required corrections)
	pos_vdc_voltage_mes = (value_voltage[1] * n_pos_vdc) + f_pos_vdc;

	/*
	 *channel 7 - voltage from offset potentiometer,
	 *voltage from the offset potentiometer, between the positive and negative values to ground
	 *is needed shift voltage to ADC level (0-3.3 V)
	 */
	wg_offset_raw = adc_signal_shifter(10.0f, 2.2f, 4.7f, U_reff,
			value_voltage[2]);

	//channel 7 - value correction to the actual level of the voltage value coming out of the potentiometer - raw voltage value
	if ((wg_offset_raw > 4.0f) && (wg_offset_raw < 6.0f)) {
		wg_offset_raw = wg_offset_raw * 0.98f;
	} else if ((wg_offset_raw < 0.0f) && (wg_offset_raw > -4.0f)) {
		wg_offset_raw = wg_offset_raw * 0.95f;
	} else if (wg_offset_raw <= -4.0f) {
		wg_offset_raw = wg_offset_raw * 0.97f;
	} else {
		wg_offset_raw = wg_offset_raw;
	}

	//channel 6 - sepic converter voltage out - to user and feedback
	conv_voltage_mes = value_voltage[3] * n_conv_voltage_out;

	//channel 6 - value correction for channel 6
	if (conv_voltage_mes <= 5.0f) {
		conv_voltage_mes = conv_voltage_mes - 0.1f;
	} else if ((conv_voltage_mes > 5.0f) && (conv_voltage_mes <= 10.0f)) {
		conv_voltage_mes = conv_voltage_mes - 0.25f;
	} else if ((conv_voltage_mes > 10.0f) && (conv_voltage_mes <= 20.0f)) {
		conv_voltage_mes = conv_voltage_mes - 0.5f;
	} else {
		conv_voltage_mes = conv_voltage_mes - 1.0f;
	}

	//channel 5 - shunt voltage to measure current from the sepic converter and proccesing of them
	conv_current_mes = map_f(value_voltage[4], 0.0f, U_I_mes_bb_max, 0.0f,
			I_bb_max);

	//channel 4 - old voltage from offset potentiometer (from second wire of potentiometer) - not used now, free channel to future application
//	dac_offset_set = value_voltage[5];
//	dac_offset_set = map_f(dac_offset_set, 0.0f, U_reff, neg_vdc_voltage_mes ,pos_vdc_voltage_mes);
//	dac_offset_set= constrain_f(dac_offset_set, neg_vdc_voltage_mes, pos_vdc_voltage_mes) ;

	//channel 3 -  voltage from the second circuit of the signal gain potentiometer and converting to potentiometer resistance
	wg_gain = value_voltage[6] + f_gain;
	wg_gain = map_f(wg_gain, 0.0f, U_reff, 0.0f, 10.0f);
	wg_gain = constrain_f(wg_gain, 0.0f, 10.0f);

	//variable with resistance value of gain potentiometer
	float dac_amplitude_pot = wg_gain;

	//calculation of true signal gain on the opamp circuit output
	wg_gain = ((wg_gain / 1) * wg_raw_amplitude);

	//channel 3 - value correction for channel 3
	if ((wg_gain > 0.50f) && (wg_gain < 1.0f)) {
		wg_gain = wg_gain - 0.1f;
	} else if ((wg_gain >= 1.0f) && (wg_gain < 2.0f)) {
		wg_gain = wg_gain * 0.84f;
	} else if ((wg_gain >= 2.0f) && (wg_gain < 3.0f)) {
		wg_gain = wg_gain * 0.84f;
	} else if ((wg_gain >= 3.0f) && (wg_gain < 4.0f)) {
		wg_gain = wg_gain * 0.87f;
	} else if (wg_gain >= 4.0f) {
		wg_gain = wg_gain * 0.91;
	} else {
		wg_gain = wg_gain;
	}

	/*
	 *channel 2 - negative voltage to supply op amp circuit
	 *is needed shift voltage to ADC level (0-3.3 V)
	 */
	neg_vdc_voltage_mes = adc_signal_shifter(10.0f, 2.2f, 4.7f, U_reff,
			value_voltage[7]);

	//calculation of true voltage offset on the output of the opamp circuit
	wg_offset_set = wg_offset_raw * (1 + (dac_amplitude_pot));

	//voltage offset value correction
	wg_offset_set = final_offset_correction(wg_offset_set);	//korekta offsetu

	//average output voltage from wave generator
	wg_cal_out = wg_offset_set - (wg_gain / 2.0f);

	//checking the max offset and output voltage values (is in opamp circuit supply voltage range)
	if (((wg_cal_out - 0.5f) > pos_vdc_voltage_mes)
			|| ((wg_cal_out - 0.5f) < neg_vdc_voltage_mes)) {
		wg_out_ovr = 1;
	} else {
		wg_out_ovr = 0;
	}

	if (((wg_offset_set - 0.5f) > pos_vdc_voltage_mes)
			|| ((wg_offset_set - 0.5f) < neg_vdc_voltage_mes)) {
		wg_offset_ovr = 1;
	} else {
		wg_offset_ovr = 0;
	}
}

//function to scrolling values from encoder to int
uint32_t encoder_managment(uint32_t input, uint32_t min, uint32_t max,
		uint32_t step, uint8_t mode) {
	/*
	 * function scroll values from encoder on the screen depending on the specified parameters
	 * input - value to scroll
	 * min - min scroll value
	 * max - max scroll value
	 * step - step to scroll
	 * mode - if 1 is set the maximum value is not scrolled to the minimum value , if 0 after passing the maximum value returns the minimum value
	 */

	if (encoder_rot_prev > encoder_rot) {
		input = input + step;
		encoder_rot_prev = encoder_rot;
	} else if (encoder_rot_prev < encoder_rot) {
		input = input - step;
		encoder_rot_prev = encoder_rot;
	} else {

	}

	if (mode == 1) {
		input = constrain(input, min, max);
	} else {
		input = scroll_value(input, min, max);
	}

	return input;
}

//function to scrolling values from encoder to float
float encoder_managment_f(float input, float min, float max, float step,
		float mode) {
	/*
	 * function scroll values from encoder on the screen depending on the specified parameters
	 * input - value to scroll
	 * min - min scroll value
	 * max - max scroll value
	 * step - step to scroll
	 * mode - if 1 is set the maximum value is not scrolled to the minimum value , if 0 after passing the maximum value returns the minimum value
	 */
	if (encoder_rot_prev > encoder_rot) {
		input = input + step;
		encoder_rot_prev = encoder_rot;
	} else if (encoder_rot_prev < encoder_rot) {
		input = input - step;
		encoder_rot_prev = encoder_rot;
	} else {

	}

	if (mode == 1) {
		input = constrain_f(input, min, max);
	} else {
		input = scroll_value_f(input, min, max);
	}

	return input;
}


//function to read data from switch and encoder's button
void switch_managment(void) {
	/*
	 * function read hold time from encoder's button using timer 3
	 * function also read data from rocker switches, turn on wg and converter LEDs
	 */

	//encoder's button
	if ((HAL_GPIO_ReadPin(SW_ENCODER_GPIO_Port, SW_ENCODER_Pin) != 1)) {

		if ((timer_3_counter_encoder_sw - encoder_switch_timer
				>= endoer_hold_times[2])
				&& (timer_3_counter_encoder_sw - encoder_switch_timer
						<= endoer_hold_times[3])) //very long
				{
			if (encoder_switch_double_long_1 == 0) {
				encoder_switch_double_long_1++;
				encoder_switch_double_long_2++;
				encoder_switch_long_2--;
			}
		} else if ((timer_3_counter_encoder_sw - encoder_switch_timer
				>= endoer_hold_times[1])
				&& (timer_3_counter_encoder_sw - encoder_switch_timer
						< endoer_hold_times[2])) //long
				{
			if (encoder_switch_long_1 == 0) {
				encoder_switch_long_1++;
				encoder_switch_long_2++;
				encoder_switch_short_2--;
			}
		} else if ((timer_3_counter_encoder_sw - encoder_switch_timer
				>= endoer_hold_times[0])
				&& (timer_3_counter_encoder_sw - encoder_switch_timer
						< endoer_hold_times[1])) //short
				{
			if (encoder_switch_short_1 == 0) {
				encoder_switch_short_1++;
				encoder_switch_short_2++;
			}
		}
	} else {
		encoder_switch_timer = timer_3_counter_encoder_sw;
		encoder_switch_double_long_1 = 0;
		encoder_switch_long_1 = 0;
		encoder_switch_short_1 = 0;
		encoder_switch_double_long = encoder_switch_double_long_2;
		encoder_switch_long = encoder_switch_long_2;
		encoder_switch_short = encoder_switch_short_2;
	}

	//rocker switches status and leds activation

	//converter
	if (HAL_GPIO_ReadPin(GPIOA, SW_WORK_BB_Pin) == 1) {
		conv_sw_start = 1;
		HAL_GPIO_WritePin(GPIOB, LED_WORK_BB_Pin, SET);
	} else {
		conv_sw_start = 0;
		HAL_GPIO_WritePin(GPIOB, LED_WORK_BB_Pin, RESET);
		HAL_GPIO_WritePin(GPIOB, LED_STOP_BB_Pin, RESET);
	}

	//wage generator
	if (HAL_GPIO_ReadPin(GPIOB, SW_WORK_DAC_Pin) == 1) {
		wg_sw_start = 1;
		HAL_GPIO_WritePin(GPIOB, LED_WORK_DAC_Pin, SET);
	} else {
		wg_sw_start = 0;
		HAL_GPIO_WritePin(GPIOB, LED_WORK_DAC_Pin, RESET);
	}
}

//function to control wave generator
void wave_generator_managment(void) {
	//check switch state
	if (wg_sw_start == 1) {
		//set wave shape and frequency
		AD9833_SetWave(wg_wave_mode);
		AD9833_SetWaveData(dac_freq, 0);
	} else {
		AD9833_SetWaveData(0, 0);
		wg_raw_amplitude = 0.0f;
	}
}

//function to control converter work
void converter_managment(void) {

	/*
	 * function to control converter work
	 * converter is build in SEPIC topology
	 * constant values are selected experimentally
	 * code is a kind of PI algorithm
	 * code uses the current and last error
	 * the code first checks the current consumption, if it is lower than the set one,
	 * it checks the voltage difference and generates the PWM
	 * the code works better with the load connected to the output converter (with the lowest oscillations)
	 * converter without connected load has higher oscillation
	 * if the converter goes into an asynchronous state, the code can reset the PWM value
	 * PWM ramping depends on the current consumed, this prevents overly oscillating
	 */

	adc_read();	//fresh ADC data to converter

	//local variable and settings to converter
	conv_actual_voltage_diff = fabs(conv_voltage_set - conv_voltage_mes);//check voltage difference between set and output value
	float conv_voltage_diff_fac_1 = 10.0f; //threshold 1 - big voltage difference
	float conv_voltage_diff_fac_2 = 1.0f; //threshold 2 - small voltage difference
	float conv_diff_hyst_fac = 0.5f; //hysteresis value for switching conditions, to avoid infinite switching of states
	uint8_t conv_diff_state = 0; //variable to switch the correction power
	float conv_prev_voltage_diff_fac = 0; //factor for last value of voltage difference
	float conv_actual_voltage_diff_fac = 0; //factor for current value of voltage difference
	uint16_t conv_ovc_hyst_fac = 100;	// hysteresis to ovc protection
	uint8_t conv_ovc_state = 0;	//overcurrent protection state variable

	//set factors depending on current power consumption
	//if output is connected
	if (conv_current_mes > 0.1f) {
		//voltage difference is big
		if ((conv_actual_voltage_diff >= (conv_voltage_diff_fac_1 + conv_diff_hyst_fac))
				&& (conv_ovc_state == 0)) {
			conv_diff_state = 0;

			conv_prev_voltage_diff_fac = 3.0f;
			conv_actual_voltage_diff_fac = 1.7f;
		}
		//voltage difference is small
		else if ((conv_actual_voltage_diff >= (conv_voltage_diff_fac_2 + conv_diff_hyst_fac))
				&& (conv_actual_voltage_diff < conv_voltage_diff_fac_1)
				&& (conv_ovc_state == 0)) {
			conv_diff_state = 0;

			conv_prev_voltage_diff_fac = 2.5;
			conv_actual_voltage_diff_fac = 1.2;
		}
		//voltage difference is very small
		else {
			conv_diff_state = 1;
		}

		//calculate errors
		conv_actual_voltage_diff = conv_actual_voltage_diff * conv_actual_voltage_diff_fac;
		conv_prev_voltage_diff = conv_prev_voltage_diff * conv_prev_voltage_diff_fac;
	}
	//if output is unconnected
	else {
		//because unloaded converter is very "rapid"
		//lowest voltage difference to correct PWM values
		if (conv_actual_voltage_diff > 0.15f) {
			conv_diff_state = 1;
		}
		//don't correct PWM value
		else {
			conv_diff_state = 2;
		}
	}

	//let's work (if the rocker switch is on)
	if (conv_sw_start == 1) {

		//if the overcurrent protection does not work
		if (conv_current_mes < conv_current_set) {

			//turn off OVC LED
			HAL_GPIO_WritePin(GPIOB, LED_STOP_BB_Pin, RESET);

			//check voltage state and calculate PWM
			if (conv_voltage_set > conv_voltage_mes) {
				if (conv_diff_state == 0) {
					conv_pwm_control = conv_pwm_control
							+ (int16_t) (conv_actual_voltage_diff
									+ conv_prev_voltage_diff);
				} else if (conv_diff_state == 1) {
					conv_pwm_control++;
				} else {
				}
			} else if (conv_voltage_set < conv_voltage_mes) {
				if (conv_diff_state == 0) {
					conv_pwm_control = conv_pwm_control
							- (int16_t) (conv_actual_voltage_diff
									+ conv_prev_voltage_diff);
				} else if (conv_diff_state == 1) {
					conv_pwm_control--;
				} else {
				}
			}
			else {
			}
		}
		//if the overcurrent protection work
		else if (conv_current_mes > conv_current_set) {

			//turn on OVC LED
			HAL_GPIO_WritePin(GPIOB, LED_STOP_BB_Pin, SET);

			conv_ovc_state = 1;

			//reduce the PWM value
			//if difference is big
			if (fabs(conv_current_set - conv_current_mes)
					> ((float) conv_ovc_hyst_fac)) {
				conv_pwm_control = conv_pwm_control - (int16_t)((fabs(conv_current_set - conv_current_mes)* 0.2f));
			}
			//if the difference is small
			else {
				conv_pwm_control--;
			}
		} else {
		}
		//limitation of high value of PWM to protect the transistor in the converter
		conv_pwm_control = constrain(conv_pwm_control,
				((conv_pwm_arr_value * conv_pwm_min) / 100),
				((conv_pwm_arr_value * conv_pwm_max) / 100));
	}
	//if converter is off
	else {
		conv_pwm_control = 0;
		conv_ovc_state = 0;
	}

	//asynchronization protection - reset PWM value
	if ((conv_pwm_control >= ((conv_pwm_arr_value * conv_pwm_max) / 100))
			&& (conv_sw_start == 1)) {
		//set reset delay proportional to voltage difference - it is a core of this mechanism
		uint16_t max_reset_delay = (uint16_t) map_f(conv_actual_voltage_diff,
				0.0f, 20.0f, (float) 0, (float) 10);

		//reset time counter
		if (timer_3_counter_conv_sync >= max_reset_delay) {
			timer_3_counter_conv_sync = 0;
			conv_pwm_control = 0;
		}

	} else {
		timer_3_counter_conv_sync = 0;
	}

		/*
		 * at the first time of start the inverter, check that the load is connected
		 * by increasing the PWM (more than the calculations show)
		 * necessarily to check the current consumption of the initial part of this function
		 * then work with calculated PWM value
		 */
	if ((conv_sw_start == 1) && (conv_sw_start_prev == 0)) {
		//the initial PWM value is scaled and limited
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,
				constrain(((int16_t )conv_voltage_set * 5), 20,
						((conv_pwm_arr_value * conv_pwm_max) / 100)));
	} else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, conv_pwm_control);
	}

	//set new values to incoming calculations
	conv_prev_voltage_diff = conv_actual_voltage_diff;
	conv_sw_start_prev = conv_sw_start;
}

//wave generator screen
void oled_wave_generator(void) {

	//data to set cursor
	screen_cursor_config[0] = 1;	//active
	screen_cursor_config[1] = 0;	//first position
	screen_cursor_config[2] = 1;	//step

	//set title and shape of signal, add information about raw gain of signal (according with AD9833 datasheet)
	SSD1306_GotoXY(0, 0);
	SSD1306_DrawFilledRectangle(0, 0, 109, 7, 1);
	SSD1306_Puts("-GENERATOR- ", &Font_6x8, 0);
	if (wg_wave_mode == 0) {
		SSD1306_Puts("SINE", &Font_6x8, 0);
		wg_raw_amplitude = 0.55f;
	}
	else if (wg_wave_mode == 1) {
		SSD1306_Puts("PWM ", &Font_6x8, 0);
		wg_raw_amplitude = 3.1;
	}
	else if (wg_wave_mode == 2) {
		SSD1306_Puts("TRI ", &Font_6x8, 0);
		wg_raw_amplitude = 0.55f;
	}
	else {
		wg_wave_mode = 0;
		SSD1306_Puts("ERROR", &Font_6x8, 0);
		wg_raw_amplitude = 0.0f;
	}

	//set frequency
	SSD1306_GotoXY(0, 9);
	SSD1306_Puts("F [Hz]: ", &Font_6x8, 1);
	itoa(dac_freq, screen_text_buff, 10);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//show offset - set by potentiometer
	SSD1306_GotoXY(0, 18);
	SSD1306_Puts("V OS max [V]: ", &Font_6x8, 1);
	ftoa(screen_text_buff, wg_offset_set, 1);

	//add minus symbol (because between -1 and 0 is not displayed)
	if ((wg_offset_set < 0.0f) && (wg_offset_set > -1.0f)) {
		SSD1306_Puts("-", &Font_6x8, 1);
	}
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//show gain - set by potentiometer
	SSD1306_GotoXY(0, 27);
	SSD1306_Puts("GAIN [V]: ", &Font_6x8, 1);
	ftoa(screen_text_buff, wg_gain, 1);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//show average output voltage
	SSD1306_GotoXY(0, 36);
	SSD1306_Puts("OUT avr [V]: ", &Font_6x8, 1);
	ftoa(screen_text_buff, wg_cal_out, 1);

	//add minus symbol (because between -1 and 0 is not displayed)
	if ((wg_cal_out < 0.0f) && (wg_cal_out > -1.0f)) {
		SSD1306_Puts("-", &Font_6x8, 1);
	}
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//if offset or output is out of the voltage opamp circuit range, set alert
	SSD1306_GotoXY(0, 45);
	if (((wg_out_ovr == 1) || (wg_offset_ovr == 1))&&(error_screen_delay == 1)) {
		SSD1306_Puts("OV VCC !", &Font_6x8, 0);
	} else {
		SSD1306_Puts("        ", &Font_6x8, 1);
	}

	//show data
	SSD1306_UpdateScreen();
}

//converter screen
void oled_converter(void) {

	//data to set cursor
	screen_cursor_config[0] = 1;	//active
	screen_cursor_config[1] = 1;	//first position
	screen_cursor_config[2] = 2;	//step

	//show title
	SSD1306_GotoXY(0, 0);
	SSD1306_DrawFilledRectangle(0, 0, 109, 7, 1);
	SSD1306_Puts("-CONVERTER-", &Font_6x8, 0);

	//show and set voltage value on the output of converter
	SSD1306_GotoXY(0, 9);
	SSD1306_Puts("U SET [V]: ", &Font_6x8, 1);
	ftoa(screen_text_buff, conv_voltage_set, 1);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//show and set max current value on the output of converter
	SSD1306_GotoXY(0, 18);
	SSD1306_Puts("I SET [mA]: ", &Font_6x8, 1);
	itoa((uint16_t) conv_current_set, screen_text_buff, 10);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//voltage value on the converter output
	SSD1306_GotoXY(0, 27);
	SSD1306_Puts("U MES [V]: ", &Font_6x8, 1);
	ftoa(screen_text_buff, conv_voltage_mes, 1);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//current value on the converter output
	SSD1306_GotoXY(0, 36);
	SSD1306_Puts("I MES [mA]: ", &Font_6x8, 1);
	itoa((uint16_t) conv_current_mes, screen_text_buff, 10);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//pwm value to control the npn transistor of the sepic converter
	SSD1306_GotoXY(0, 45);
	SSD1306_Puts("PWM[-]: ", &Font_6x8, 1);
	ftoa(screen_text_buff, conv_pwm_control, 1);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("   ", &Font_6x8, 1);

	//show data
	SSD1306_UpdateScreen();
}

//voltage parameter screen
void oled_parameters(void) {

	//data to set cursor
	screen_cursor_config[0] = 0;	//idle
	screen_cursor_config[1] = 0;	//first pos
	screen_cursor_config[2] = 0;	//step

	//show title
	SSD1306_GotoXY(0, 0);
	SSD1306_DrawFilledRectangle(0, 0, 109, 7, 1);
	SSD1306_Puts("-POWER-", &Font_6x8, 0);

	//show SOC and battery voltage
	SSD1306_GotoXY(0, 9);
	SSD1306_Puts("BATT[%]|[V]: ", &Font_6x8, 1);
	itoa(vcc_batt_soc, screen_text_buff, 10);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("|", &Font_6x8, 1);
	ftoa(screen_text_buff, vcc_voltage_batt_mes, 1);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("  ", &Font_6x8, 1);

	//show positive voltage supply for opamp
	SSD1306_GotoXY(0, 18);
	SSD1306_Puts("CONV+[V]: ", &Font_6x8, 1);
	ftoa(screen_text_buff, pos_vdc_voltage_mes, 1);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("   ", &Font_6x8, 1);

	//show negative voltage supply for opamp
	SSD1306_GotoXY(0, 27);
	SSD1306_Puts("CONV-[V]: ", &Font_6x8, 1);
	ftoa(screen_text_buff, neg_vdc_voltage_mes, 1);
	SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
	SSD1306_Puts("   ", &Font_6x8, 1);

	//show data
	SSD1306_UpdateScreen();
}

//welcome screen
void oled_hello (void)
{
	//show start screen
	SSD1306_GotoXY(10, 10);
	SSD1306_Puts("INITIALIZATION", &Font_6x8, 1);
	HAL_Delay(2);
	SSD1306_UpdateScreen();

	//add random circles
	for(uint8_t n = 0; n<=10; n++)
	{
	SSD1306_DrawCircle(random_number(10,122), random_number(10,54), random_number(5,20), 1);
	SSD1306_UpdateScreen();
	HAL_Delay(2);
	}

}

/*
 * indicator screen function
 * this section add indicator (cursor) on the any screen if is needed
 * indicator shows what type of parameter is choose
 * section also adds a value multiplier (x1, x10, x100, x1000)
 * multiplier multiplies the parameter selected by the indicator
 * this function also allows the parameter to be set using the rotary encoder
 */
void oled_indicator(void) {
	//if screen has been changed or step number is out of range, reset the cursor position
	if ((encoder_switch_short > screen_cursor_config[2]) || (screen_new == 1)) {
		encoder_switch_short = screen_cursor_config[1];
		encoder_switch_short_2 = screen_cursor_config[1];
	}

	//set (new) cursor position
	if (screen_cursor_config[0] == 1) {

		//remove cursor from wrong positions
		for (uint16_t i = screen_cursor_config[1]; i <= screen_cursor_config[2]; i++)	//kasowanie kursora
				{
			SSD1306_GotoXY(110, screen_indicator_step_vertical * i);
			if ((screen_indicator_step_vertical * i) == (screen_indicator_step_vertical * encoder_switch_short)) {
				// but not from currently selected
			} else {
				SSD1306_Puts("  ", &Font_6x8, 1);
			}
		}

		//set cursor on correct position
		SSD1306_GotoXY(110, screen_indicator_step_vertical * encoder_switch_short);
		SSD1306_Puts("<-", &Font_6x8, 1);
	}

	//change selected parameters on the wave generator screem
	if (encoder_switch_long == 0) {
		switch (screen_indicator_step_vertical * encoder_switch_short) {
		case 0: {	//output wave type
			wg_wave_mode = encoder_managment(wg_wave_mode, 0, 2, 1, 1);
			break;
		}
		case screen_indicator_step_vertical: {	//output frequency
			dac_freq = encoder_managment(dac_freq, 0, wg_max_frequency,
					1 * screen_multiplier, 1);
			break;
		}
		default:	//reset
			encoder_switch_short = 0;
		}
	}

	//change selected parameters on the sepic converter screen
	else if (encoder_switch_long == 1) {
		switch (screen_indicator_step_vertical * encoder_switch_short) {
		case screen_indicator_step_vertical: {	//output voltage
			conv_voltage_set = encoder_managment_f(conv_voltage_set, 0.00f,
			U_bb_max, 0.1f * screen_multiplier, 1);
			break;
		}
		case (2*screen_indicator_step_vertical): {	//output current
			conv_current_set = encoder_managment_f(conv_current_set, 0.00f,
			I_bb_max, 1.0f * screen_multiplier, 1);
			break;
		}
		default:	//reset
			encoder_switch_short = 0;
		}

	} else {
		// no more screens
	}

	//add and set value multiplier
	switch (encoder_switch_double_long) {
	case 0: {
		screen_multiplier = screen_multiplier_value[0];
		break;
	}
	case 1: {
		screen_multiplier = screen_multiplier_value[1];
		break;
	}
	case 2: {
		screen_multiplier = screen_multiplier_value[2];
		break;
	}
	case 3: {
		screen_multiplier = screen_multiplier_value[3];
		break;
	}
	default:	//reset
		encoder_switch_double_long = 0;
		encoder_switch_double_long_2 = 0;
		screen_multiplier = 1;
	}

	//screens number where the value multiplier is needed
	if (encoder_switch_long < indicator_screens_number) {
		SSD1306_GotoXY(90, 48);
		itoa(screen_multiplier, screen_text_buff, 10);
		SSD1306_Puts("x", &Font_6x8, 1);
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
		SSD1306_Puts("   ", &Font_6x8, 1);
	}

	//show data on the screen
	SSD1306_UpdateScreen();
}

//screen with graphs
void oled_graph(void) {
	//data to set cursor
	screen_cursor_config[0] = 0;	//active
	screen_cursor_config[1] = 0;	//first position
	screen_cursor_config[2] = 0;	//step

	//draw new axis on the graph
	init_new_graph = 1;

	//select graph by rotary encoder
	graph_set = encoder_managment(graph_set, 0, 3, 1, 0);

	switch (graph_set) {

	case 0: {
		draw_graph((double)x_value_graph, conv_voltage_mes, 30, 50, 75, 30, 0, 100, 0,
				conv_voltage_set * 1.5, "U MES [V]", 0, " |50 ms ");
		break;
	}
	case 1: {
		draw_graph((double)x_value_graph, (uint16_t) conv_current_mes, 30, 50, 75, 30,
				0, 100, 0, conv_current_set * 1.5, "I MES [mA]", 1,
				" |50 ms ");
		break;
	}
	case 2: {
		draw_graph((double)x_value_graph, conv_pwm_control, 30, 50, 75, 30, 0, 100, 0,
				300, "PWM", 1, " |50 ms ");
		break;
	}
	default:	//reset
		graph_set = 0;
	}
}

//emergency screen - low battery
void oled_batt(void) {
	//conditions prevent screen jumping
	if ((vcc_batt_soc <  batt_low_soc) && (batt_low_state == 0)) {

		//turn off work of wg and converter
		wg_sw_start = 0;
		conv_sw_start = 0;
		wave_generator_managment();
		converter_managment();

		//show information about alerts
		SSD1306_Clear();
		SSD1306_GotoXY(0, 10);
		SSD1306_Puts("LOW BATTERY", &Font_11x18, 1);

		SSD1306_GotoXY(0, 30);
		SSD1306_Puts("BATT[%]|[V]: ", &Font_6x8, 1);
		itoa(vcc_batt_soc, screen_text_buff, 10);
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
		SSD1306_Puts("|", &Font_6x8, 1);
		ftoa(screen_text_buff, vcc_voltage_batt_mes, 1);
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
		SSD1306_Puts("  ", &Font_6x8, 1);

		SSD1306_UpdateScreen();
		HAL_Delay(5000);
		SSD1306_Clear();
		SSD1306_UpdateScreen();
		batt_low_state = 1;
	} else if (vcc_batt_soc > (batt_low_soc+10)) {
		batt_low_state = 0;
	} else {

	}
}

//emergency screen - supply to opamp
void oled_vcc_error(void) {
	//conditions prevent screen jumping
	if (((neg_vdc_voltage_mes > neg_vdc_voltage_low)
			|| (pos_vdc_voltage_mes < pos_vdc_voltage_low))
			&& (power_error_state == 0)) {

		//turn off work of wg and converter
		wg_sw_start = 0;
		conv_sw_start = 0;
		wave_generator_managment();
		converter_managment();
		SSD1306_Clear();
		SSD1306_GotoXY(0, 5);
		SSD1306_Puts("POWER ERROR", &Font_11x18, 1);

		//show information about alerts
		SSD1306_GotoXY(0, 25);
		SSD1306_Puts("CONV+[V]: ", &Font_6x8, 1);
		ftoa(screen_text_buff, pos_vdc_voltage_mes, 1);
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
		SSD1306_Puts("   ", &Font_6x8, 1);

		SSD1306_GotoXY(0, 35);
		SSD1306_Puts("CONV-[V]: ", &Font_6x8, 1);
		ftoa(screen_text_buff, neg_vdc_voltage_mes, 1);
		SSD1306_Puts(screen_text_buff, &Font_6x8, 1);
		SSD1306_Puts("   ", &Font_6x8, 1);

		SSD1306_UpdateScreen();
		HAL_Delay(5000);
		SSD1306_Clear();
		SSD1306_UpdateScreen();
		power_error_state = 1;

	} else if ((neg_vdc_voltage_mes < (neg_vdc_voltage_mes - 0.2f))
			&& (pos_vdc_voltage_mes > (pos_vdc_voltage_low + 0.2f))) {
		power_error_state = 0;
	} else {

	}
}

//function to control xl6009 - new solution to converter (in future)
//void bb_xl6009(void) {
//
//	float bb_voltage_diff = fabs(bb_voltage_set - bb_voltage_mes);
//
//	if (bb_sw_start == 1)	//przetwornica wlaczona
//			{
//		//on enable pin
//		//i out pwm pin 100 // off
//		if (bb_current_mes <= bb_current_set) {
//			HAL_GPIO_WritePin(GPIOB, LED_STOP_BB_Pin, RESET);
//
//			if(bb_voltage_set > bb_voltage_mes)
//			{
//				//bb_fb pin--;
//
//			}
//			else if (bb_voltage_set < bb_voltage_mes)
//			{
//				//bb_fb pin++;
//
//				}
//
//			else
//			{
//
//			}
//
//		}
//
//		else {
//			HAL_GPIO_WritePin(GPIOB, LED_STOP_BB_Pin, SET);
//			//bb_fb pin++;
//		}
//
//	}
//
//
//
//	else {
//		//off enable pin
//		//i out pwm 0 //off
//	}
//
////		bb_pwm_control = constrain(bb_pwm_control,
////				((bb_pwm_arr * bb_pwm_min) / 100),
////				((bb_pwm_arr * bb_pwm_max) / 100));
//	//maksymalne zakresy pinu fb
//
//	//zapisz pwm do pinu fb
//
//
//}

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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //INITs
  //turn LED on the start of Inits
  HAL_GPIO_WritePin(GPIOB, LED_WORK_BB_Pin, SET);
  HAL_GPIO_WritePin(GPIOB, LED_STOP_BB_Pin, SET);
  HAL_GPIO_WritePin(GPIOB, LED_WORK_DAC_Pin, SET);

  	//OLED init
  	SSD1306_Init();

  	//ADC calibration
	if(HAL_OK==HAL_ADCEx_Calibration_Start(&hadc1))
	{
		oled_hello();//after calibration show hello
	}

	//others modules
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	//encoder
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); 	//240 kHz PWM
	//moving average - slow and fast
	Moving_Average_Init_0(&filterStruct_slow[mes_size]);
	Moving_Average_Init_1(&filterStruct_fast[mes_size]);
	AD9833_Init(0, 0, 0);	//wave generator module

	//clean screen
	SSD1306_Clear();
	HAL_Delay(10);

	HAL_TIM_Base_Start_IT(&htim3);	//timer turn on

	//turns off LEDs when initialization is complete
	HAL_GPIO_WritePin(GPIOB, LED_WORK_BB_Pin, RESET);
	HAL_GPIO_WritePin(GPIOB, LED_STOP_BB_Pin, RESET);
	HAL_GPIO_WritePin(GPIOB, LED_WORK_DAC_Pin, RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		//read data
		encoder_rot =( __HAL_TIM_GET_COUNTER(&htim2)>> 1); //encoder counter
		adc_read();		 //adc value
		switch_managment();	//switches and encoder button

		//set work mode for outputs
		wave_generator_managment();	//zarzadzanie dac
		converter_managment();

		//screens management -
		switch ( encoder_switch_long )
		{
		case 0:
			oled_wave_generator();
			break;

		case 1:
			oled_converter();
			break;

		case 2:
			oled_graph();
			break;

		case 3:
			oled_parameters();
			break;

		default:
			//reset long encoder value
			encoder_switch_long = 0;
			encoder_switch_long_2 = 0;
			break;
		}

		//clean oled if new screen is set
		if(encoder_switch_long!=screen_old_num)
		{
			SSD1306_Clear();
			screen_old_num = encoder_switch_long;
			encoder_switch_short = 0;
			encoder_switch_short_2 =0;
			screen_new = 1;
		}
		else
		{
			screen_new = 0;
		}

		//cursor on the screen
		oled_indicator();

		//if delay after launching has passed, check supply conditions
		if(error_screen_delay == 1)
		{
		oled_batt(); //low battery
		oled_vcc_error();//opamp section supply errors (to low voltage levels)
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//timer interrupt - 10 ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {

		//main - reference time base
		timer_3_counter_main++;

		//graph
		timer_3_counter_graph++;

		//graph x value - time base
		if (timer_3_counter_graph > timer_3_counter_graph_max_fac ) {
			x_value_graph++;
			timer_3_counter_graph = 0;
		}

		//reset graph time base if new screen has been set
		if (encoder_switch_long != graph_screen_max) {
			x_value_graph = 0;
		}

		//reset graph progress if time base have max value
		if ((x_value_graph >= x_value_graph_max_fac) || (graph_set_prev != graph_set)) {
			x_value_graph = 0;
			graph_set_prev = graph_set;
			SSD1306_Clear();
			init_new_graph = 1;
		}

		//time base to converter, supply error functions and encoder button
		if (timer_3_counter_main > timer_3_counter_main_max_fac ) {
			timer_3_counter_encoder_sw++;
			timer_3_counter_conv_sync++;
			timer_3_counter_error_check_delay++;

			//time base to supply error functions
			if (timer_3_counter_error_check_delay > timer_3_counter_error_check_delay_fac) {

				//allow on the error screens after delayed and only 1 while device is running
				error_screen_delay = 1;
			}
			//reset the counter when it counts to a set value
			timer_3_counter_main = 0;

		} else {
		}
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
	while (1) {
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
