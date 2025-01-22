Function Generator With Voltage Converter
-----------------------------------------
-----------------------------------------

PARAMETERS:
	FUNCTION GENERATOR:
		-possible to make sine wave, triangle wave and square wave
		-DC offset (negative and positive)
		-wave amplification capability (max 5.5 V amplitude) 
	VOLTAGE CONVERTER:
		-in SEPIC topology
		-output voltage in the range 0-25 V
		-overcurrent protection
		-max output current up to 2A
	-power supply from lithium-ion battery or external source (via DC JACK port, recommended 12V DC but range is 7-20 V)
	-HMI via rotary encoder and OLED SSD1306 (additionally via LED and rocker switch)
	-banana plugs as external ports (to converter and function generator)
	-main uC: BluePill STM32F103C8T6
	-additional uC: Attiny13 (for op amp step up converter  - for future use)
	-charging via USB C (5V/1A), as well as via the DC JACK port (max. 1 A)
	-DC JACK port 5,5x2,1 mm

SHORT DESCRIPTION:
----------------------------------------

HISTORY OF SCHEMA VERSIONS:
026.MP.JW_v1
026.MP.JW_v2
026.MP.JW_v2.1
026.MP.JW_v2.2
026.MP.JW_v2.3
026.MP.JW_v2.3.1 - current version (but only generator, converter is in version 2.1)
026.MP.JW_v2.4 - new version of hardware (only converter section)

The electrical scheme was modified during testing (software and hardware) and measurements.
The main reason for the changes and development of the circuit is the low efficiency of the converter and noise in the gain circuit. In version 2.3.1 opamp circuit is roughly stable.

The “05-PIC” folder contains images with measurements and descriptions of the device's screens and components. 

The “04-CALCULATIONS” is a spreadsheet with offset voltage calculations and resistor dividers.

The device is controlled by the encoder and 2 rocker switches. The encoder has 3 holding modes:
-short (100-500 ms) - change of indicator position
-long (500-1000 ms) - changing the screen
-very long (1000-1500 ms) - changing the multiplier
If a parameter is set, rotating the encoder increases or decreases the selected value. But if 
chart screen is selected, rotating the knob changes the chart.
Rocker switches turn on the converter and waveform generator. LEDs are used for:
-Blue - wave generator ON/OFF
-Green - converter ON/OFF
-Red - active overcurrent protection in the converter
To turn on the whole device, there is a power switch on the back of the case. 

PROSPECTIVE DEVELOPMENTS:
-add new converter section with XL6009 and test it

SOURCES:
----------------------------------------
https://www.allaboutcircuits.com/projects/how-to-DIY-waveform-generator-analog-devices-ad9833-ATmega328p/

https://www.ti.com/zh-tw/design-resources/design-tools-simulation/analog-circuits/amplifier-circuits.html

https://www.instructables.com/Function-Generator-AD9833-Based/

https://www.youtube.com/watch?app=desktop&v=WBp-EF3PCwc

https://www.mdpi.com/1996-1073/15/21/7936

http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

https://www.stm32wrobotyce.pl/tag/filtr/