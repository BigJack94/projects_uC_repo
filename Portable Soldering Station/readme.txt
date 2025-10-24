Portable Soldering Station – Concept and Technical Description
Main Assumptions
•	Two primary functions: regulated power supply and soldering station.
•	Uses a KADA soldering iron (5-pin connector, PTC sensor, 4 Ω heater).
•	Main power source: portable power tool battery.
•	Current sensing: via INA226 (later replaced by INA169).
•	Microcontroller: STM32G030F6P6 running with RTOS.
________________________________________
Diagram Versions
•	v1 – concept using a soldering iron with a K-type thermocouple.
•	v2 – converter module XL6009 with a shunt-based current sensor.
•	v3 – device as in v1, but using XL6009 or XL6019 (newer version), INA226 for current sensing, and a corrected resistor divider for converter control.
•	v3.1 – final hardware version with INA169 current sensor and SZ-08CV (LTC1625) converter module.
•	v4 – custom SEPIC converter version (not working properly).
________________________________________
Software Versions
•	v1 – based on diagram v3, without RTOS, insufficient power output.
•	v2 – for the v4 diagram, did not work properly.
•	v3.1 – final software version corresponding to diagram v3.1, using PID or table-based control (factors).
________________________________________
Development Stages
1. First Stage
Based on a previous project (e.g., Function Generator with Voltage Converter).
A ready-made boost/buck converter module (XL6009/XL6019 in SEPIC configuration) was used.
However, this module provided only about 35 W, which was insufficient for soldering applications.
Main parameters of this version:
•	Output voltage: 0–32 V
•	Maximum power: 35 W (limited by converter IC)
•	Input voltage: 7–28 V
•	Current sensing via INA226 (I²C)
•	Soldering iron: KADA, PTC sensor + 4 Ω heater
________________________________________
2. Second Stage
Attempted to design a custom SEPIC converter, but due to compatibility issues (TC4427 driver did not work properly with the MOSFETs), the concept was abandoned.
This corresponds to diagram v4 and software v2. 
________________________________________
3. Third Stage (Current / Final Stage)
Based on diagram v3.1, using the SZ-08CV (LTC1625) converter module.
According to the manufacturer, the module supports up to 80 W, though actual power depends on the supply source.
Hardware changes:
•	INA226 replaced by INA169 (faster and more accurate).
•	Converter module modified before integration:
		Adjust CC potentiometer for maximum output current (software controls current limit).
		Remove CV potentiometer.
		Remove the resistor and diode between pin 1 of the CV potentiometer and LTC1625.
		Connect the control signal to the KEY pin.
Software functions:
•	Operates in two modes:
		PID mode: closed-loop CV + CC feedback; less stable at low loads (±100 mV).
		Factor mode: uses a predefined PWM↔voltage correlation table (more stable, CC feedback only).
•	CC regulation lowers converter voltage until current drops below the limit.
•	Soldering station mode: PID temperature control, 30–500 °C range, max 50 W / 3 A / 20 V (fixed in firmware).
•	Automatic soldering iron disconnection detection (based on open-circuit PTC sensor).
Factor Table (09-FACTORS_ADC_PWM_SZ-08CV_LTC1625):
Contains the PWM–output voltage relationship and ADC correction factors (compensating for divider nonlinearity).
________________________________________
Final Device Capabilities
•	Adjustable output voltage: 0–32 V
•	Current range: up to 3 A
•	Maximum power: ~80 W (depending on input source)
•	Input selection: internal battery or external DC source (2.5×5.5 mm jack or banana connectors)
•	Banana connectors can act as power source or load terminals.
•	Current sensing: INA169
•	Converter module: LTC1625 (SZ-08CV), hardware modified as described.
•	LED indicators:
		In converter mode – indicates PWM power level.
		In soldering mode – indicates power increase/decrease.
•	Display: OLED SH1306 128×64 (I²C).
•	Encoder control:
		Rotate → adjust value.
		Short press → move between screen rows.
		Long press → change mode/screen.



