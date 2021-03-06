EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:linear
LIBS:regul
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:LCSDR
LIBS:4ms-ic
LIBS:Display
LIBS:Connector
LIBS:Relay
LIBS:RX998
LIBS:LCSDR-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 7
Title "Hermes-Lite"
Date "2016-02-18"
Rev "2.0-pre1"
Comp "SofterHardware"
Comment1 "Vasyl Kuzmenko"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 3600 750  2600 1700
U 5DC3176E
F0 "syntez" 60
F1 "syntez.sch" 60
$EndSheet
$Sheet
S 3600 2700 2600 1700
U 5DC357A0
F0 "INPUT_FREQUENCY" 60
F1 "INPUT_FREQUENCY.sch" 60
$EndSheet
$Sheet
S 750  750  2600 1700
U 5DD2EDB8
F0 "MCU" 60
F1 "MCU.sch" 60
$EndSheet
$Sheet
S 750  4650 2600 1700
U 5DD3CF89
F0 "Display Keyboard Encoder" 60
F1 "display.sch" 60
$EndSheet
$Sheet
S 750  2700 2600 1700
U 5DD97E41
F0 "PowerSupplay" 60
F1 "PowerSupplay.sch" 60
$EndSheet
Text GLabel 9750 1650 0    60   Output ~ 0
F_Q
Text GLabel 9750 1450 0    60   Input ~ 0
USART_TX
$Comp
L Conn_01x01 J8
U 1 1 5DE25479
P 9450 5950
F 0 "J8" H 9450 6050 50  0000 C CNN
F 1 "Conn_01x01" H 9450 5850 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3_DIN965_Pad" H 9450 5950 50  0001 C CNN
F 3 "" H 9450 5950 50  0001 C CNN
	1    9450 5950
	0    -1   -1   0   
$EndComp
$Comp
L Conn_01x01 J9
U 1 1 5DE254BB
P 9800 5950
F 0 "J9" H 9800 6050 50  0000 C CNN
F 1 "Conn_01x01" H 9800 5850 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3_DIN965_Pad" H 9800 5950 50  0001 C CNN
F 3 "" H 9800 5950 50  0001 C CNN
	1    9800 5950
	0    -1   -1   0   
$EndComp
$Comp
L Conn_01x01 J10
U 1 1 5DE254E5
P 10150 5950
F 0 "J10" H 10150 6050 50  0000 C CNN
F 1 "Conn_01x01" H 10150 5850 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3_DIN965_Pad" H 10150 5950 50  0001 C CNN
F 3 "" H 10150 5950 50  0001 C CNN
	1    10150 5950
	0    -1   -1   0   
$EndComp
$Comp
L Conn_01x01 J11
U 1 1 5DE2550E
P 10500 5950
F 0 "J11" H 10500 6050 50  0000 C CNN
F 1 "Conn_01x01" H 10500 5850 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3_DIN965_Pad" H 10500 5950 50  0001 C CNN
F 3 "" H 10500 5950 50  0001 C CNN
	1    10500 5950
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR3
U 1 1 5DE25560
P 9450 6150
F 0 "#PWR3" H 9450 5900 50  0001 C CNN
F 1 "GND" H 9450 6000 50  0000 C CNN
F 2 "" H 9450 6150 50  0001 C CNN
F 3 "" H 9450 6150 50  0001 C CNN
	1    9450 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR4
U 1 1 5DE25589
P 9800 6150
F 0 "#PWR4" H 9800 5900 50  0001 C CNN
F 1 "GND" H 9800 6000 50  0000 C CNN
F 2 "" H 9800 6150 50  0001 C CNN
F 3 "" H 9800 6150 50  0001 C CNN
	1    9800 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR5
U 1 1 5DE255A6
P 10150 6150
F 0 "#PWR5" H 10150 5900 50  0001 C CNN
F 1 "GND" H 10150 6000 50  0000 C CNN
F 2 "" H 10150 6150 50  0001 C CNN
F 3 "" H 10150 6150 50  0001 C CNN
	1    10150 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR8
U 1 1 5DE255C3
P 10500 6150
F 0 "#PWR8" H 10500 5900 50  0001 C CNN
F 1 "GND" H 10500 6000 50  0000 C CNN
F 2 "" H 10500 6150 50  0001 C CNN
F 3 "" H 10500 6150 50  0001 C CNN
	1    10500 6150
	1    0    0    -1  
$EndComp
$Sheet
S 3600 4650 2600 1700
U 5DE7E102
F0 "G3UUR" 60
F1 "G3UUR.sch" 60
$EndSheet
Text GLabel 7100 1350 0    60   Output ~ 0
I2C_SDA
Text GLabel 7100 1150 0    60   Output ~ 0
I2C_SCL
$Comp
L Conn_01x04 J4
U 1 1 5DF4B2F3
P 7850 1200
F 0 "J4" H 7850 1400 50  0000 C CNN
F 1 "Conn_01x04" H 7850 900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 7850 1200 50  0001 C CNN
F 3 "" H 7850 1200 50  0001 C CNN
	1    7850 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 1300 7250 1300
Wire Wire Line
	7250 1300 7250 1350
Wire Wire Line
	7250 1350 7100 1350
Wire Wire Line
	7100 1150 7250 1150
Wire Wire Line
	7250 1150 7250 1200
Wire Wire Line
	7250 1200 7650 1200
$Comp
L GND #PWR1
U 1 1 5DF86413
P 7550 1450
F 0 "#PWR1" H 7550 1200 50  0001 C CNN
F 1 "GND" H 7550 1300 50  0000 C CNN
F 2 "" H 7550 1450 50  0001 C CNN
F 3 "" H 7550 1450 50  0001 C CNN
	1    7550 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 1450 7550 1400
Wire Wire Line
	7550 1400 7650 1400
$Comp
L Conn_01x05 J6
U 1 1 5DF86443
P 10650 1350
F 0 "J6" H 10650 1650 50  0000 C CNN
F 1 "NEO-7M" H 10650 1050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05_Pitch2.54mm" H 10650 1350 50  0001 C CNN
F 3 "" H 10650 1350 50  0001 C CNN
	1    10650 1350
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR6
U 1 1 5DF86484
P 10300 1000
F 0 "#PWR6" H 10300 850 50  0001 C CNN
F 1 "+3V3" H 10300 1140 50  0000 C CNN
F 2 "" H 10300 1000 50  0001 C CNN
F 3 "" H 10300 1000 50  0001 C CNN
	1    10300 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 1450 10450 1450
Text GLabel 9750 1300 0    60   Output ~ 0
USART_RX
Wire Wire Line
	9750 1300 10000 1300
Wire Wire Line
	10000 1300 10000 1350
Wire Wire Line
	10000 1350 10450 1350
$Comp
L GND #PWR7
U 1 1 5DF86FED
P 10450 1250
F 0 "#PWR7" H 10450 1000 50  0001 C CNN
F 1 "GND" H 10450 1100 50  0001 C CNN
F 2 "" H 10450 1250 50  0001 C CNN
F 3 "" H 10450 1250 50  0001 C CNN
	1    10450 1250
	0    1    1    0   
$EndComp
Wire Wire Line
	10300 1000 10300 1150
Wire Wire Line
	10300 1150 10450 1150
Wire Wire Line
	10450 1550 10000 1550
Wire Wire Line
	10000 1550 10000 1650
Wire Wire Line
	10000 1650 9750 1650
$Comp
L +3V3 #PWR2
U 1 1 5DF8AD5E
P 7600 950
F 0 "#PWR2" H 7600 800 50  0001 C CNN
F 1 "+3V3" H 7600 1090 50  0000 C CNN
F 2 "" H 7600 950 50  0001 C CNN
F 3 "" H 7600 950 50  0001 C CNN
	1    7600 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 950  7600 1100
Wire Wire Line
	7600 1100 7650 1100
$EndSCHEMATC
