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
Sheet 7 7
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Audio-Jack-3_2Switches J4
U 1 1 5DE7E36D
P 1850 1850
F 0 "J4" H 1800 2150 50  0000 C CNN
F 1 "LINEIN" H 1750 1650 50  0000 L CNN
F 2 "UI:audio_jack_3_5mm_PJ307" H 2100 1950 50  0001 C CNN
F 3 "" H 2100 1950 50  0001 C CNN
	1    1850 1850
	1    0    0    -1  
$EndComp
Text GLabel 6950 2400 2    60   Output ~ 0
LINE_IN_LEFT
$Comp
L CP_Small C53
U 1 1 5DE7E375
P 6750 2400
F 0 "C53" V 6500 2350 50  0000 L CNN
F 1 "1uF" V 6600 2350 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 6750 2400 50  0001 C CNN
F 3 "" H 6750 2400 50  0001 C CNN
	1    6750 2400
	0    -1   -1   0   
$EndComp
$Comp
L CP_Small C52
U 1 1 5DE7E37C
P 6750 1600
F 0 "C52" V 6500 1550 50  0000 L CNN
F 1 "1uF" V 6600 1550 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 6750 1600 50  0001 C CNN
F 3 "" H 6750 1600 50  0001 C CNN
	1    6750 1600
	0    -1   -1   0   
$EndComp
Text GLabel 6950 1600 2    60   Output ~ 0
LINE_IN_RIGHT
Text GLabel 3350 1950 0    60   Input ~ 0
I_IN
Text GLabel 3350 2250 0    60   Input ~ 0
Q_IN
$Comp
L GND #PWR0102
U 1 1 5DE7E386
P 1650 2050
F 0 "#PWR0102" H 1650 1800 50  0001 C CNN
F 1 "GND" H 1650 1900 50  0001 C CNN
F 2 "" H 1650 2050 50  0001 C CNN
F 3 "" H 1650 2050 50  0001 C CNN
	1    1650 2050
	1    0    0    -1  
$EndComp
$Comp
L C_Small C48
U 1 1 5DE7E38C
P 5250 3300
F 0 "C48" V 5500 3300 50  0000 L CNN
F 1 "22..47nF" V 5400 3100 50  0000 L CNN
F 2 "UI:C_1206_0603" H 5250 3300 50  0001 C CNN
F 3 "" H 5250 3300 50  0001 C CNN
	1    5250 3300
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR0103
U 1 1 5DE7E3AF
P 5050 3850
F 0 "#PWR0103" H 5050 3600 50  0001 C CNN
F 1 "GND" H 5050 3700 50  0001 C CNN
F 2 "" H 5050 3850 50  0001 C CNN
F 3 "" H 5050 3850 50  0001 C CNN
	1    5050 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0104
U 1 1 5DE7E3B9
P 5350 3300
F 0 "#PWR0104" H 5350 3050 50  0001 C CNN
F 1 "GND" H 5350 3150 50  0001 C CNN
F 2 "" H 5350 3300 50  0001 C CNN
F 3 "" H 5350 3300 50  0001 C CNN
	1    5350 3300
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R18
U 1 1 5DE7E3CC
P 5900 3650
F 0 "R18" V 6100 3650 50  0000 L CNN
F 1 "2.2k" V 6000 3550 50  0000 L CNN
F 2 "UI:R_1206_0603" H 5900 3650 50  0001 C CNN
F 3 "" H 5900 3650 50  0001 C CNN
	1    5900 3650
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C51
U 1 1 5DE7E3D4
P 5500 3800
F 0 "C51" H 5650 3850 50  0000 L CNN
F 1 "15nF" H 5650 3750 50  0000 L CNN
F 2 "UI:C_1206_0603" H 5500 3800 50  0001 C CNN
F 3 "" H 5500 3800 50  0001 C CNN
	1    5500 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0105
U 1 1 5DE7E3DB
P 5500 3900
F 0 "#PWR0105" H 5500 3650 50  0001 C CNN
F 1 "GND" H 5500 3750 50  0001 C CNN
F 2 "" H 5500 3900 50  0001 C CNN
F 3 "" H 5500 3900 50  0001 C CNN
	1    5500 3900
	1    0    0    -1  
$EndComp
Text GLabel 6100 3650 2    60   Input ~ 0
TX2
$Comp
L 4053 U8
U 1 1 5DE7E526
P 4700 2150
F 0 "U8" H 4800 2150 50  0000 C CNN
F 1 "4053" H 4800 1950 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 4700 2150 60  0001 C CNN
F 3 "" H 4700 2150 60  0001 C CNN
	1    4700 2150
	1    0    0    -1  
$EndComp
$Comp
L R_Small R6
U 1 1 5DE7E858
P 5650 1350
F 0 "R6" H 5700 1400 50  0000 L CNN
F 1 "10k" H 5700 1300 50  0000 L CNN
F 2 "UI:R_1206_0603" H 5650 1350 50  0001 C CNN
F 3 "" H 5650 1350 50  0001 C CNN
	1    5650 1350
	1    0    0    -1  
$EndComp
$Comp
L R_Small R22
U 1 1 5DE7E9B1
P 6100 1350
F 0 "R22" H 6150 1400 50  0000 L CNN
F 1 "10k" H 6150 1300 50  0000 L CNN
F 2 "UI:R_1206_0603" H 6100 1350 50  0001 C CNN
F 3 "" H 6100 1350 50  0001 C CNN
	1    6100 1350
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C15
U 1 1 5DE7ED0D
P 3600 1950
F 0 "C15" V 3350 1900 50  0000 L CNN
F 1 "1uF" V 3450 1900 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 3600 1950 50  0001 C CNN
F 3 "" H 3600 1950 50  0001 C CNN
	1    3600 1950
	0    -1   -1   0   
$EndComp
$Comp
L CP_Small C16
U 1 1 5DE7EDD4
P 3600 2250
F 0 "C16" V 3700 2200 50  0000 L CNN
F 1 "1uF" V 3800 2200 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 3600 2250 50  0001 C CNN
F 3 "" H 3600 2250 50  0001 C CNN
	1    3600 2250
	0    -1   -1   0   
$EndComp
$Comp
L CP_Small C5
U 1 1 5DE7F0C8
P 2550 1450
F 0 "C5" V 2300 1400 50  0000 L CNN
F 1 "1uF" V 2400 1400 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 2550 1450 50  0001 C CNN
F 3 "" H 2550 1450 50  0001 C CNN
	1    2550 1450
	0    1    1    0   
$EndComp
$Comp
L CP_Small C6
U 1 1 5DE7F228
P 2550 2150
F 0 "C6" V 2650 2100 50  0000 L CNN
F 1 "1uF" V 2750 2100 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 2550 2150 50  0001 C CNN
F 3 "" H 2550 2150 50  0001 C CNN
	1    2550 2150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR0106
U 1 1 5DE7F372
P 4700 2850
F 0 "#PWR0106" H 4700 2600 50  0001 C CNN
F 1 "GND" H 4700 2700 50  0001 C CNN
F 2 "" H 4700 2850 50  0001 C CNN
F 3 "" H 4700 2850 50  0001 C CNN
	1    4700 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0107
U 1 1 5DE7F3EE
P 5400 2650
F 0 "#PWR0107" H 5400 2400 50  0001 C CNN
F 1 "GND" H 5400 2500 50  0001 C CNN
F 2 "" H 5400 2650 50  0001 C CNN
F 3 "" H 5400 2650 50  0001 C CNN
	1    5400 2650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR0108
U 1 1 5DE7F423
P 4000 2450
F 0 "#PWR0108" H 4000 2200 50  0001 C CNN
F 1 "GND" H 4000 2300 50  0001 C CNN
F 2 "" H 4000 2450 50  0001 C CNN
F 3 "" H 4000 2450 50  0001 C CNN
	1    4000 2450
	0    1    1    0   
$EndComp
$Comp
L +9V #PWR0109
U 1 1 5DE7F5C0
P 4700 850
F 0 "#PWR0109" H 4700 700 50  0001 C CNN
F 1 "+9V" H 4700 990 50  0000 C CNN
F 2 "" H 4700 850 50  0001 C CNN
F 3 "" H 4700 850 50  0001 C CNN
	1    4700 850 
	1    0    0    -1  
$EndComp
$Comp
L R_Small R5
U 1 1 5DE7FA40
P 3450 3300
F 0 "R5" V 3650 3300 50  0000 L CNN
F 1 "10k" V 3550 3250 50  0000 L CNN
F 2 "UI:R_1206_0603" H 3450 3300 50  0001 C CNN
F 3 "" H 3450 3300 50  0001 C CNN
	1    3450 3300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6850 2400 6950 2400
Wire Wire Line
	6650 1600 6400 1600
Wire Wire Line
	6950 1600 6850 1600
Wire Wire Line
	2150 1750 2050 1750
Wire Wire Line
	2150 1450 2150 1750
Connection ~ 2150 1650
Wire Wire Line
	2050 1850 2150 1850
Wire Wire Line
	2150 1950 2050 1950
Wire Wire Line
	3350 2250 3500 2250
Wire Wire Line
	2150 1850 2150 2150
Connection ~ 2150 1850
Wire Wire Line
	3350 1950 3500 1950
Wire Wire Line
	2150 1650 2050 1650
Wire Wire Line
	3550 3300 5150 3300
Wire Wire Line
	5050 3300 5050 3450
Wire Wire Line
	5350 3650 5800 3650
Wire Wire Line
	5500 3700 5500 3650
Connection ~ 5500 3650
Wire Wire Line
	6000 3650 6100 3650
Wire Wire Line
	5650 2400 6650 2400
Wire Wire Line
	5400 1850 6400 1850
Wire Wire Line
	6400 1850 6400 1600
Wire Wire Line
	4000 2550 3850 2550
Wire Wire Line
	3850 2550 3850 6450
Wire Wire Line
	3850 2750 4000 2750
Wire Wire Line
	4000 2650 3850 2650
Connection ~ 3850 2650
Connection ~ 3850 2750
Wire Wire Line
	4000 1950 3700 1950
Wire Wire Line
	2900 1850 4000 1850
Wire Wire Line
	2450 1450 2150 1450
Connection ~ 2150 1950
Wire Wire Line
	2150 2150 2450 2150
Wire Wire Line
	4700 2850 4700 2800
Connection ~ 3850 3300
Wire Wire Line
	4700 850  4700 1500
$Comp
L +9V #PWR0110
U 1 1 5DE7FF30
P 3350 3300
F 0 "#PWR0110" H 3350 3150 50  0001 C CNN
F 1 "+9V" V 3350 3500 50  0000 C CNN
F 2 "" H 3350 3300 50  0001 C CNN
F 3 "" H 3350 3300 50  0001 C CNN
	1    3350 3300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6100 1450 6100 1850
Connection ~ 6100 1850
Wire Wire Line
	5650 900  5650 1250
Wire Wire Line
	6100 900  6100 1250
$Comp
L CP_Small C57
U 1 1 5DE8064A
P 7400 3700
F 0 "C57" H 7500 3750 50  0000 L CNN
F 1 "10uF" H 7500 3650 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 7400 3700 50  0001 C CNN
F 3 "" H 7400 3700 50  0001 C CNN
	1    7400 3700
	1    0    0    -1  
$EndComp
$Comp
L C_Small C31
U 1 1 5DE80937
P 4200 1150
F 0 "C31" H 4350 1200 50  0000 L CNN
F 1 "100nF" H 4350 1100 50  0000 L CNN
F 2 "UI:C_1206_0603" H 4200 1150 50  0001 C CNN
F 3 "" H 4200 1150 50  0001 C CNN
	1    4200 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0111
U 1 1 5DE80A43
P 4200 1250
F 0 "#PWR0111" H 4200 1000 50  0001 C CNN
F 1 "GND" H 4200 1100 50  0001 C CNN
F 2 "" H 4200 1250 50  0001 C CNN
F 3 "" H 4200 1250 50  0001 C CNN
	1    4200 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 1050 4200 900 
Wire Wire Line
	4200 900  4700 900 
Connection ~ 4700 900 
$Comp
L Q_NMOS_GSD Q1
U 1 1 5DE810BE
P 5150 3650
F 0 "Q1" H 5350 3700 50  0000 L CNN
F 1 "2N7002" H 5350 3600 50  0000 L CNN
F 2 "" H 5350 3750 50  0001 C CNN
F 3 "" H 5150 3650 50  0001 C CNN
	1    5150 3650
	-1   0    0    -1  
$EndComp
Text GLabel 6950 5150 2    60   Input ~ 0
LINE_OUT_LEFT
Text GLabel 6950 5950 2    60   Input ~ 0
LINE_OUT_RIGHT
Text GLabel 2850 5400 0    60   Output ~ 0
I_OUT
Text GLabel 2850 5100 0    60   Output ~ 0
Q_OUT
$Comp
L CP_Small C55
U 1 1 5DE82782
P 6750 5950
F 0 "C55" V 6500 5900 50  0000 L CNN
F 1 "1uF" V 6600 5900 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 6750 5950 50  0001 C CNN
F 3 "" H 6750 5950 50  0001 C CNN
	1    6750 5950
	0    -1   -1   0   
$EndComp
$Comp
L CP_Small C54
U 1 1 5DE82788
P 6750 5150
F 0 "C54" V 6500 5100 50  0000 L CNN
F 1 "1uF" V 6600 5100 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 6750 5150 50  0001 C CNN
F 3 "" H 6750 5150 50  0001 C CNN
	1    6750 5150
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C21
U 1 1 5DE82797
P 3850 6550
F 0 "C21" H 4000 6600 50  0000 L CNN
F 1 "22..47nF" H 4000 6500 50  0000 L CNN
F 2 "UI:C_1206_0603" H 3850 6550 50  0001 C CNN
F 3 "" H 3850 6550 50  0001 C CNN
	1    3850 6550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0112
U 1 1 5DE827A3
P 3850 6650
F 0 "#PWR0112" H 3850 6400 50  0001 C CNN
F 1 "GND" H 3850 6500 50  0001 C CNN
F 2 "" H 3850 6650 50  0001 C CNN
F 3 "" H 3850 6650 50  0001 C CNN
	1    3850 6650
	1    0    0    -1  
$EndComp
$Comp
L 4053 U9
U 1 1 5DE827BC
P 4700 5700
F 0 "U9" H 4800 5700 50  0000 C CNN
F 1 "4053" H 4800 5500 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 4700 5700 60  0001 C CNN
F 3 "" H 4700 5700 60  0001 C CNN
	1    4700 5700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R17
U 1 1 5DE827C2
P 5650 4900
F 0 "R17" H 5700 4950 50  0000 L CNN
F 1 "10k" H 5700 4850 50  0000 L CNN
F 2 "UI:R_1206_0603" H 5650 4900 50  0001 C CNN
F 3 "" H 5650 4900 50  0001 C CNN
	1    5650 4900
	1    0    0    -1  
$EndComp
$Comp
L R_Small R23
U 1 1 5DE827C8
P 6100 4900
F 0 "R23" H 6150 4950 50  0000 L CNN
F 1 "10k" H 6150 4850 50  0000 L CNN
F 2 "UI:R_1206_0603" H 6100 4900 50  0001 C CNN
F 3 "" H 6100 4900 50  0001 C CNN
	1    6100 4900
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C8
U 1 1 5DE827DA
P 3150 5400
F 0 "C8" V 2900 5350 50  0000 L CNN
F 1 "1uF" V 3000 5350 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 3150 5400 50  0001 C CNN
F 3 "" H 3150 5400 50  0001 C CNN
	1    3150 5400
	0    -1   -1   0   
$EndComp
$Comp
L CP_Small C14
U 1 1 5DE827E0
P 3150 5100
F 0 "C14" V 3250 5050 50  0000 L CNN
F 1 "1uF" V 3350 5050 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D4.0mm_P2.00mm" H 3150 5100 50  0001 C CNN
F 3 "" H 3150 5100 50  0001 C CNN
	1    3150 5100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR0113
U 1 1 5DE827E6
P 4700 6400
F 0 "#PWR0113" H 4700 6150 50  0001 C CNN
F 1 "GND" H 4700 6250 50  0001 C CNN
F 2 "" H 4700 6400 50  0001 C CNN
F 3 "" H 4700 6400 50  0001 C CNN
	1    4700 6400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0114
U 1 1 5DE827EC
P 5400 6200
F 0 "#PWR0114" H 5400 5950 50  0001 C CNN
F 1 "GND" H 5400 6050 50  0001 C CNN
F 2 "" H 5400 6200 50  0001 C CNN
F 3 "" H 5400 6200 50  0001 C CNN
	1    5400 6200
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR0115
U 1 1 5DE827F2
P 4000 6000
F 0 "#PWR0115" H 4000 5750 50  0001 C CNN
F 1 "GND" H 4000 5850 50  0001 C CNN
F 2 "" H 4000 6000 50  0001 C CNN
F 3 "" H 4000 6000 50  0001 C CNN
	1    4000 6000
	0    1    1    0   
$EndComp
$Comp
L +9V #PWR0116
U 1 1 5DE827F8
P 4700 4400
F 0 "#PWR0116" H 4700 4250 50  0001 C CNN
F 1 "+9V" H 4700 4540 50  0000 C CNN
F 2 "" H 4700 4400 50  0001 C CNN
F 3 "" H 4700 4400 50  0001 C CNN
	1    4700 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 5950 6950 5950
Wire Wire Line
	6650 5150 6400 5150
Wire Wire Line
	6950 5150 6850 5150
Wire Wire Line
	5650 5950 6650 5950
Wire Wire Line
	5400 5400 6400 5400
Wire Wire Line
	6400 5400 6400 5150
Wire Wire Line
	3850 6100 4000 6100
Wire Wire Line
	3850 6300 4000 6300
Wire Wire Line
	4000 6200 3850 6200
Connection ~ 3850 6200
Connection ~ 3850 6300
Wire Wire Line
	4700 6400 4700 6350
Wire Wire Line
	4700 4400 4700 5050
Wire Wire Line
	6100 5000 6100 5400
Connection ~ 6100 5400
Wire Wire Line
	5650 4450 5650 4800
Wire Wire Line
	6100 4450 6100 4800
$Comp
L C_Small C47
U 1 1 5DE82849
P 4200 4700
F 0 "C47" H 4350 4750 50  0000 L CNN
F 1 "100nF" H 4350 4650 50  0000 L CNN
F 2 "UI:C_1206_0603" H 4200 4700 50  0001 C CNN
F 3 "" H 4200 4700 50  0001 C CNN
	1    4200 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0117
U 1 1 5DE8284F
P 4200 4800
F 0 "#PWR0117" H 4200 4550 50  0001 C CNN
F 1 "GND" H 4200 4650 50  0001 C CNN
F 2 "" H 4200 4800 50  0001 C CNN
F 3 "" H 4200 4800 50  0001 C CNN
	1    4200 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4600 4200 4450
Wire Wire Line
	4200 4450 4700 4450
Connection ~ 4700 4450
Wire Wire Line
	2850 5100 3050 5100
Wire Wire Line
	2850 5400 3050 5400
Connection ~ 6100 4450
Connection ~ 6100 900 
Wire Wire Line
	7900 900  7900 4450
Wire Wire Line
	5650 4450 8250 4450
Connection ~ 5050 3300
Connection ~ 3850 6100
$Comp
L C_Small C56
U 1 1 5DE853B7
P 7400 3050
F 0 "C56" H 7550 3100 50  0000 L CNN
F 1 "100nF" H 7550 3000 50  0000 L CNN
F 2 "UI:C_1206_0603" H 7400 3050 50  0001 C CNN
F 3 "" H 7400 3050 50  0001 C CNN
	1    7400 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 2950 7400 2800
Wire Wire Line
	7400 2800 7900 2800
Connection ~ 7900 2800
Wire Wire Line
	7400 3600 7400 3500
Wire Wire Line
	7400 3500 7900 3500
Connection ~ 7900 3500
$Comp
L GND #PWR0118
U 1 1 5DE857EC
P 7400 3150
F 0 "#PWR0118" H 7400 2900 50  0001 C CNN
F 1 "GND" H 7400 3000 50  0001 C CNN
F 2 "" H 7400 3150 50  0001 C CNN
F 3 "" H 7400 3150 50  0001 C CNN
	1    7400 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0119
U 1 1 5DE8585A
P 7400 3800
F 0 "#PWR0119" H 7400 3550 50  0001 C CNN
F 1 "GND" H 7400 3650 50  0001 C CNN
F 2 "" H 7400 3800 50  0001 C CNN
F 3 "" H 7400 3800 50  0001 C CNN
	1    7400 3800
	1    0    0    -1  
$EndComp
$Comp
L R_Small R26
U 1 1 5DE85B0C
P 8250 4250
F 0 "R26" H 8300 4300 50  0000 L CNN
F 1 "10k" H 8300 4200 50  0000 L CNN
F 2 "UI:R_1206_0603" H 8250 4250 50  0001 C CNN
F 3 "" H 8250 4250 50  0001 C CNN
	1    8250 4250
	1    0    0    -1  
$EndComp
$Comp
L R_Small R27
U 1 1 5DE85B99
P 8250 4700
F 0 "R27" H 8300 4750 50  0000 L CNN
F 1 "10k" H 8300 4650 50  0000 L CNN
F 2 "UI:R_1206_0603" H 8250 4700 50  0001 C CNN
F 3 "" H 8250 4700 50  0001 C CNN
	1    8250 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0120
U 1 1 5DE85C8E
P 8250 4800
F 0 "#PWR0120" H 8250 4550 50  0001 C CNN
F 1 "GND" H 8250 4650 50  0001 C CNN
F 2 "" H 8250 4800 50  0001 C CNN
F 3 "" H 8250 4800 50  0001 C CNN
	1    8250 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 4350 8250 4600
Connection ~ 8250 4450
Connection ~ 7900 4450
$Comp
L +9V #PWR0121
U 1 1 5DE85EBD
P 8250 4150
F 0 "#PWR0121" H 8250 4000 50  0001 C CNN
F 1 "+9V" H 8250 4290 50  0000 C CNN
F 2 "" H 8250 4150 50  0001 C CNN
F 3 "" H 8250 4150 50  0001 C CNN
	1    8250 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1550 5650 1550
Wire Wire Line
	5650 1450 5650 2400
Connection ~ 5650 1550
Wire Wire Line
	2650 2150 2900 2150
Wire Wire Line
	3700 2250 3900 2250
Wire Wire Line
	3900 2250 3900 1650
Wire Wire Line
	3900 1650 4000 1650
Wire Wire Line
	2900 2150 2900 1850
Wire Wire Line
	4000 1550 2900 1550
Wire Wire Line
	2900 1550 2900 1450
Wire Wire Line
	2900 1450 2650 1450
Wire Wire Line
	5650 900  7900 900 
Wire Wire Line
	5650 5000 5650 5950
Connection ~ 5650 5100
Wire Wire Line
	5650 5100 5400 5100
Wire Wire Line
	4000 5100 3250 5100
Wire Wire Line
	4000 5400 3250 5400
$EndSCHEMATC
