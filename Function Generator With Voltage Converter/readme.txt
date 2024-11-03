Function Generator With Voltage Converter

Parameters:
	FUNCTION GENERATOR:
		-possible to make sine wave, triangle wave and square wave
		-DC offset (negative and positive)
		-wave amplification capability (max 5.5 V amplitude) 
	VOLTAGE CONVERTER:
		-in SEPIC topology
		-output voltage in the range 0-25 V
		-overcurrent protection
		-max output current up to 2A
	-power supply from lithium-ion battery or external source (via DC JACK port)
	-HMI via rotary encoder and OLED SSD1306 (additionally via LED and rocker switch)
	-banana plugs as external ports (to converter and function generator)
	-main uC: BluePill STM32F103C8T6
	-additional uC: Attiny13 (for op amp step up converter  - for future use)


HISTORY OF SCHEMA VERSIONS:
026.MP.JW_v1
026.MP.JW_v2
026.MP.JW_v2.1
026.MP.JW_v2.2
026.MP.JW_v2.3
026.MP.JW_v2.3.1 - current version 
026.MP.JW_v2.4 - new version of hardware (only converter section)

The electrical scheme was modified during testing (software and hardware) and measurements.
The main reason for the changes and development of the circuit is the low efficiency of the converter and noise in the gain circuit.