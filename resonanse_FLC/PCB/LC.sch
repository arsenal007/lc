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
LIBS:MCU_Microchip_PIC16
LIBS:ftdi
LIBS:Connector
LIBS:bf998
LIBS:Relay
LIBS:LC-cache
EELAYER 25 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
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
L PIC16F628A-IP U?
U 1 1 5DC4FB47
P 7900 2450
F 0 "U?" H 6950 3150 50  0000 L CNN
F 1 "PIC16F628A-IP" H 6950 3050 50  0000 L CNN
F 2 "" H 7900 2450 50  0001 C CIN
F 3 "" H 7900 2450 50  0001 C CNN
	1    7900 2450
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y?
U 1 1 5DC4FBAD
P 9900 3200
F 0 "Y?" H 9900 3350 50  0000 C CNN
F 1 "20Mhz" H 9900 3050 50  0000 C CNN
F 2 "" H 9900 3200 50  0001 C CNN
F 3 "" H 9900 3200 50  0001 C CNN
	1    9900 3200
	-1   0    0    -1  
$EndComp
$Comp
L FT232RL U?
U 1 1 5DC4FC74
P 4300 2850
F 0 "U?" H 3650 3750 50  0000 L CNN
F 1 "FT232RL" H 4700 3750 50  0000 L CNN
F 2 "Housings_SSOP:SSOP-28_5.3x10.2mm_Pitch0.65mm" H 4300 2850 50  0001 C CNN
F 3 "" H 4300 2850 50  0001 C CNN
	1    4300 2850
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5DC4FD80
P 9650 3500
F 0 "C?" H 9750 3550 50  0000 L CNN
F 1 "22pF" H 9750 3450 50  0000 L CNN
F 2 "" H 9650 3500 50  0001 C CNN
F 3 "" H 9650 3500 50  0001 C CNN
	1    9650 3500
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5DC4FDE3
P 10150 3500
F 0 "C?" H 10250 3550 50  0000 L CNN
F 1 "22pF" H 10250 3450 50  0000 L CNN
F 2 "" H 10150 3500 50  0001 C CNN
F 3 "" H 10150 3500 50  0001 C CNN
	1    10150 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC4FE29
P 9650 3600
F 0 "#PWR?" H 9650 3350 50  0001 C CNN
F 1 "GND" H 9650 3450 50  0001 C CNN
F 2 "" H 9650 3600 50  0001 C CNN
F 3 "" H 9650 3600 50  0001 C CNN
	1    9650 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC4FE6D
P 10150 3600
F 0 "#PWR?" H 10150 3350 50  0001 C CNN
F 1 "GND" H 10150 3450 50  0001 C CNN
F 2 "" H 10150 3600 50  0001 C CNN
F 3 "" H 10150 3600 50  0001 C CNN
	1    10150 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC4FFA3
P 7900 3150
F 0 "#PWR?" H 7900 2900 50  0001 C CNN
F 1 "GND" H 7900 3000 50  0001 C CNN
F 2 "" H 7900 3150 50  0001 C CNN
F 3 "" H 7900 3150 50  0001 C CNN
	1    7900 3150
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J?
U 1 1 5DC4FFCE
P 6900 1150
F 0 "J?" H 6900 1450 50  0000 C CNN
F 1 "ICSP" H 6900 750 50  0000 C CNN
F 2 "" H 6900 1150 50  0001 C CNN
F 3 "" H 6900 1150 50  0001 C CNN
	1    6900 1150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5DC50423
P 4400 550
F 0 "#PWR?" H 4400 400 50  0001 C CNN
F 1 "+5V" H 4400 690 50  0000 C CNN
F 2 "" H 4400 550 50  0001 C CNN
F 3 "" H 4400 550 50  0001 C CNN
	1    4400 550 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC50634
P 6700 1150
F 0 "#PWR?" H 6700 900 50  0001 C CNN
F 1 "GND" H 6700 1000 50  0001 C CNN
F 2 "" H 6700 1150 50  0001 C CNN
F 3 "" H 6700 1150 50  0001 C CNN
	1    6700 1150
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR?
U 1 1 5DC50659
P 7900 1650
F 0 "#PWR?" H 7900 1500 50  0001 C CNN
F 1 "+5V" H 7900 1790 50  0000 C CNN
F 2 "" H 7900 1650 50  0001 C CNN
F 3 "" H 7900 1650 50  0001 C CNN
	1    7900 1650
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5DC506FA
P 6700 1050
F 0 "#PWR?" H 6700 900 50  0001 C CNN
F 1 "+5V" V 6700 1250 50  0000 C CNN
F 2 "" H 6700 1050 50  0001 C CNN
F 3 "" H 6700 1050 50  0001 C CNN
	1    6700 1050
	0    -1   -1   0   
$EndComp
Text Label 6150 1350 0    60   ~ 0
ICSP_CLK
Text Label 6050 1250 0    60   ~ 0
ICSP_DATA
Text Label 6700 750  0    60   ~ 0
ICSP_MCLR
$Comp
L R_Small R?
U 1 1 5DC5091C
P 8300 900
F 0 "R?" H 8330 920 50  0000 L CNN
F 1 "27k" H 8330 860 50  0000 L CNN
F 2 "" H 8300 900 50  0001 C CNN
F 3 "" H 8300 900 50  0001 C CNN
	1    8300 900 
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5DC50A30
P 8300 1000
F 0 "#PWR?" H 8300 850 50  0001 C CNN
F 1 "+5V" H 8300 1140 50  0000 C CNN
F 2 "" H 8300 1000 50  0001 C CNN
F 3 "" H 8300 1000 50  0001 C CNN
	1    8300 1000
	-1   0    0    1   
$EndComp
$Comp
L R_Small R?
U 1 1 5DC50BCE
P 5750 2150
F 0 "R?" H 5780 2170 50  0000 L CNN
F 1 "R_Small" H 5780 2110 50  0000 L CNN
F 2 "" H 5750 2150 50  0001 C CNN
F 3 "" H 5750 2150 50  0001 C CNN
	1    5750 2150
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R?
U 1 1 5DC50D00
P 5750 2250
F 0 "R?" H 5780 2270 50  0000 L CNN
F 1 "R_Small" H 5780 2210 50  0000 L CNN
F 2 "" H 5750 2250 50  0001 C CNN
F 3 "" H 5750 2250 50  0001 C CNN
	1    5750 2250
	0    -1   -1   0   
$EndComp
$Comp
L USB_B_Mini J?
U 1 1 5DC50E33
P 2450 2450
F 0 "J?" H 2250 2900 50  0000 L CNN
F 1 "USB_B_Mini" H 2250 2800 50  0000 L CNN
F 2 "" H 2600 2400 50  0001 C CNN
F 3 "" H 2600 2400 50  0001 C CNN
	1    2450 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC51185
P 2400 2950
F 0 "#PWR?" H 2400 2700 50  0001 C CNN
F 1 "GND" H 2400 2800 50  0001 C CNN
F 2 "" H 2400 2950 50  0001 C CNN
F 3 "" H 2400 2950 50  0001 C CNN
	1    2400 2950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC5129B
P 4100 4000
F 0 "#PWR?" H 4100 3750 50  0001 C CNN
F 1 "GND" H 4100 3850 50  0001 C CNN
F 2 "" H 4100 4000 50  0001 C CNN
F 3 "" H 4100 4000 50  0001 C CNN
	1    4100 4000
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5DC51387
P 3350 2150
F 0 "C?" V 3550 2050 50  0000 L CNN
F 1 "100nF" V 3450 2050 50  0000 L CNN
F 2 "" H 3350 2150 50  0001 C CNN
F 3 "" H 3350 2150 50  0001 C CNN
	1    3350 2150
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC517FC
P 3250 2150
F 0 "#PWR?" H 3250 1900 50  0001 C CNN
F 1 "GND" H 3250 2000 50  0001 C CNN
F 2 "" H 3250 2150 50  0001 C CNN
F 3 "" H 3250 2150 50  0001 C CNN
	1    3250 2150
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR?
U 1 1 5DC518E3
P 3200 2850
F 0 "#PWR?" H 3200 2700 50  0001 C CNN
F 1 "+5V" V 3200 3050 50  0000 C CNN
F 2 "" H 3200 2850 50  0001 C CNN
F 3 "" H 3200 2850 50  0001 C CNN
	1    3200 2850
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C?
U 1 1 5DC51A16
P 3300 3050
F 0 "C?" H 3400 3100 50  0000 L CNN
F 1 "100nF" H 3400 3000 50  0000 L CNN
F 2 "" H 3300 3050 50  0001 C CNN
F 3 "" H 3300 3050 50  0001 C CNN
	1    3300 3050
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC51B40
P 3300 3150
F 0 "#PWR?" H 3300 2900 50  0001 C CNN
F 1 "GND" H 3300 3000 50  0001 C CNN
F 2 "" H 3300 3150 50  0001 C CNN
F 3 "" H 3300 3150 50  0001 C CNN
	1    3300 3150
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BEC Q?
U 1 1 5DC520BD
P 8550 5250
F 0 "Q?" H 8750 5300 50  0000 L CNN
F 1 "SS9018" H 8750 5200 50  0000 L CNN
F 2 "" H 8750 5350 50  0001 C CNN
F 3 "" H 8550 5250 50  0001 C CNN
	1    8550 5250
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 5DC5217B
P 8400 4950
F 0 "R?" V 8600 4950 50  0000 L CNN
F 1 "47k" V 8500 4900 50  0000 L CNN
F 2 "" H 8400 4950 50  0001 C CNN
F 3 "" H 8400 4950 50  0001 C CNN
	1    8400 4950
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R?
U 1 1 5DC524B8
P 8650 4500
F 0 "R?" H 8700 4550 50  0000 L CNN
F 1 "470" H 8700 4450 50  0000 L CNN
F 2 "" H 8650 4500 50  0001 C CNN
F 3 "" H 8650 4500 50  0001 C CNN
	1    8650 4500
	1    0    0    -1  
$EndComp
$Comp
L L_Small L?
U 1 1 5DC525BD
P 9250 4200
F 0 "L?" H 9280 4240 50  0000 L CNN
F 1 "L_Small" H 9280 4160 50  0000 L CNN
F 2 "" H 9250 4200 50  0001 C CNN
F 3 "" H 9250 4200 50  0001 C CNN
	1    9250 4200
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR?
U 1 1 5DC5279B
P 9800 4200
F 0 "#PWR?" H 9800 4050 50  0001 C CNN
F 1 "+5V" V 9800 4400 50  0000 C CNN
F 2 "" H 9800 4200 50  0001 C CNN
F 3 "" H 9800 4200 50  0001 C CNN
	1    9800 4200
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC52911
P 8650 5450
F 0 "#PWR?" H 8650 5200 50  0001 C CNN
F 1 "GND" H 8650 5300 50  0001 C CNN
F 2 "" H 8650 5450 50  0001 C CNN
F 3 "" H 8650 5450 50  0001 C CNN
	1    8650 5450
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5DC5296C
P 6950 5250
F 0 "C?" V 7150 5150 50  0000 L CNN
F 1 "100nF" V 7050 5150 50  0000 L CNN
F 2 "" H 6950 5250 50  0001 C CNN
F 3 "" H 6950 5250 50  0001 C CNN
	1    6950 5250
	0    1    -1   0   
$EndComp
$Comp
L BF998 Q?
U 1 1 5DC53924
P 6650 5650
F 0 "Q?" H 6850 5700 50  0000 L CNN
F 1 "BF998" H 6850 5600 50  0000 L CNN
F 2 "" H 6850 5750 50  0001 C CNN
F 3 "" H 6650 5650 50  0001 C CNN
	1    6650 5650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC539ED
P 6750 5850
F 0 "#PWR?" H 6750 5600 50  0001 C CNN
F 1 "GND" H 6750 5700 50  0001 C CNN
F 2 "" H 6750 5850 50  0001 C CNN
F 3 "" H 6750 5850 50  0001 C CNN
	1    6750 5850
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 5DC53B8F
P 6750 4600
F 0 "R?" H 6800 4650 50  0000 L CNN
F 1 "330" H 6800 4550 50  0000 L CNN
F 2 "" H 6750 4600 50  0001 C CNN
F 3 "" H 6750 4600 50  0001 C CNN
	1    6750 4600
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 5DC53DFD
P 6200 5300
F 0 "R?" H 6000 5350 50  0000 L CNN
F 1 "470" H 6000 5250 50  0000 L CNN
F 2 "" H 6200 5300 50  0001 C CNN
F 3 "" H 6200 5300 50  0001 C CNN
	1    6200 5300
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5DC54036
P 6200 5900
F 0 "C?" H 6300 5950 50  0000 L CNN
F 1 "100nF" H 6300 5850 50  0000 L CNN
F 2 "" H 6200 5900 50  0001 C CNN
F 3 "" H 6200 5900 50  0001 C CNN
	1    6200 5900
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC541B7
P 6200 6000
F 0 "#PWR?" H 6200 5750 50  0001 C CNN
F 1 "GND" H 6200 5850 50  0001 C CNN
F 2 "" H 6200 6000 50  0001 C CNN
F 3 "" H 6200 6000 50  0001 C CNN
	1    6200 6000
	1    0    0    -1  
$EndComp
$Comp
L D_x2_Serial_AKC D?
U 1 1 5DC543F5
P 5450 6650
F 0 "D?" H 5550 6800 50  0000 C CNN
F 1 "BAV99" H 5500 6900 50  0000 C CNN
F 2 "" H 5450 6650 50  0001 C CNN
F 3 "" H 5450 6650 50  0001 C CNN
	1    5450 6650
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC544E4
P 5150 6650
F 0 "#PWR?" H 5150 6400 50  0001 C CNN
F 1 "GND" H 5150 6500 50  0001 C CNN
F 2 "" H 5150 6650 50  0001 C CNN
F 3 "" H 5150 6650 50  0001 C CNN
	1    5150 6650
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC54930
P 5750 6650
F 0 "#PWR?" H 5750 6400 50  0001 C CNN
F 1 "GND" H 5750 6500 50  0001 C CNN
F 2 "" H 5750 6650 50  0001 C CNN
F 3 "" H 5750 6650 50  0001 C CNN
	1    5750 6650
	0    -1   -1   0   
$EndComp
$Comp
L Q_PNP_BEC Q?
U 1 1 5DC54B7E
P 4100 5250
F 0 "Q?" H 4300 5100 50  0000 L CNN
F 1 "BFT92" H 4300 5200 50  0000 L CNN
F 2 "" H 4300 5350 50  0001 C CNN
F 3 "" H 4100 5250 50  0001 C CNN
	1    4100 5250
	1    0    0    1   
$EndComp
$Comp
L Q_PNP_BEC Q?
U 1 1 5DC54C90
P 4500 5550
F 0 "Q?" H 4700 5400 50  0000 L CNN
F 1 "BFT92" H 4700 5500 50  0000 L CNN
F 2 "" H 4700 5650 50  0001 C CNN
F 3 "" H 4500 5550 50  0001 C CNN
	1    4500 5550
	1    0    0    1   
$EndComp
$Comp
L R_Small R?
U 1 1 5DC54E03
P 4200 5750
F 0 "R?" H 4250 5800 50  0000 L CNN
F 1 "470" H 4250 5700 50  0000 L CNN
F 2 "" H 4200 5750 50  0001 C CNN
F 3 "" H 4200 5750 50  0001 C CNN
	1    4200 5750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC55166
P 4200 5850
F 0 "#PWR?" H 4200 5600 50  0001 C CNN
F 1 "GND" H 4200 5700 50  0001 C CNN
F 2 "" H 4200 5850 50  0001 C CNN
F 3 "" H 4200 5850 50  0001 C CNN
	1    4200 5850
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5DC55220
P 6200 4800
F 0 "C?" V 6400 4700 50  0000 L CNN
F 1 "100nF" V 6300 4700 50  0000 L CNN
F 2 "" H 6200 4800 50  0001 C CNN
F 3 "" H 6200 4800 50  0001 C CNN
	1    6200 4800
	0    1    -1   0   
$EndComp
$Comp
L D_x2_Serial_AKC D?
U 1 1 5DC55B01
P 5600 4550
F 0 "D?" H 5700 4700 50  0000 C CNN
F 1 "BAV99" H 5650 4800 50  0000 C CNN
F 2 "" H 5600 4550 50  0001 C CNN
F 3 "" H 5600 4550 50  0001 C CNN
	1    5600 4550
	0    -1   1    0   
$EndComp
$Comp
L Q_PNP_BEC Q?
U 1 1 5DC55C2E
P 4900 4600
F 0 "Q?" H 5200 4450 50  0000 L CNN
F 1 "BFT92" H 5100 4550 50  0000 L CNN
F 2 "" H 5100 4700 50  0001 C CNN
F 3 "" H 4900 4600 50  0001 C CNN
	1    4900 4600
	-1   0    0    1   
$EndComp
$Comp
L R_Small R?
U 1 1 5DC5609D
P 4500 4850
F 0 "R?" V 4300 4850 50  0000 L CNN
F 1 "1k" V 4400 4850 50  0000 L CNN
F 2 "" H 4500 4850 50  0001 C CNN
F 3 "" H 4500 4850 50  0001 C CNN
	1    4500 4850
	0    -1   1    0   
$EndComp
$Comp
L CP_Small C?
U 1 1 5DC56BA0
P 5200 4400
F 0 "C?" H 5210 4470 50  0000 L CNN
F 1 "CP_Small" H 5210 4320 50  0000 L CNN
F 2 "" H 5200 4400 50  0001 C CNN
F 3 "" H 5200 4400 50  0001 C CNN
	1    5200 4400
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 5DC57379
P 5200 5050
F 0 "R?" H 5250 5100 50  0000 L CNN
F 1 "470" H 5250 5000 50  0000 L CNN
F 2 "" H 5200 5050 50  0001 C CNN
F 3 "" H 5200 5050 50  0001 C CNN
	1    5200 5050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC57475
P 5200 5150
F 0 "#PWR?" H 5200 4900 50  0001 C CNN
F 1 "GND" H 5200 5000 50  0001 C CNN
F 2 "" H 5200 5150 50  0001 C CNN
F 3 "" H 5200 5150 50  0001 C CNN
	1    5200 5150
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 5DC575AB
P 5100 6200
F 0 "R?" V 5200 6200 50  0000 L CNN
F 1 "470" V 5300 6150 50  0000 L CNN
F 2 "" H 5100 6200 50  0001 C CNN
F 3 "" H 5100 6200 50  0001 C CNN
	1    5100 6200
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 5DC57655
P 4800 6200
F 0 "C?" V 5000 6100 50  0000 L CNN
F 1 "100nF" V 4900 6100 50  0000 L CNN
F 2 "" H 4800 6200 50  0001 C CNN
F 3 "" H 4800 6200 50  0001 C CNN
	1    4800 6200
	0    1    -1   0   
$EndComp
$Comp
L C_Small C?
U 1 1 5DC57758
P 4900 5850
F 0 "C?" V 5100 5750 50  0000 L CNN
F 1 "100nF" V 5000 5750 50  0000 L CNN
F 2 "" H 4900 5850 50  0001 C CNN
F 3 "" H 4900 5850 50  0001 C CNN
	1    4900 5850
	0    1    -1   0   
$EndComp
Wire Wire Line
	9000 2650 10150 2650
Wire Wire Line
	10150 2650 10150 3400
Wire Wire Line
	10150 3200 10050 3200
Wire Wire Line
	9750 3200 9650 3200
Wire Wire Line
	9650 2750 9650 3400
Wire Wire Line
	9650 2750 9000 2750
Connection ~ 10150 3200
Connection ~ 9650 3200
Wire Wire Line
	9000 2450 10900 2450
Wire Wire Line
	9000 2550 9400 2550
Wire Wire Line
	9400 2550 9400 750 
Wire Wire Line
	9400 750  6450 750 
Wire Wire Line
	6450 750  6450 950 
Wire Wire Line
	6450 950  6700 950 
Wire Wire Line
	4400 550  4400 1850
Wire Wire Line
	2850 1750 4400 1750
Wire Wire Line
	4200 1750 4200 1850
Connection ~ 4400 1750
Wire Wire Line
	6700 1250 6000 1250
Wire Wire Line
	6000 1250 6000 2750
Wire Wire Line
	6000 2750 6800 2750
Wire Wire Line
	6800 2650 6100 2650
Wire Wire Line
	6100 2650 6100 1350
Wire Wire Line
	6100 1350 6700 1350
Wire Wire Line
	7900 1650 7900 1750
Wire Wire Line
	8300 800  8300 750 
Connection ~ 8300 750 
Wire Wire Line
	5850 2150 6800 2150
Wire Wire Line
	5650 2150 5100 2150
Wire Wire Line
	5850 2250 6800 2250
Wire Wire Line
	5650 2250 5100 2250
Wire Wire Line
	2750 2450 3500 2450
Wire Wire Line
	2750 2550 3500 2550
Wire Wire Line
	3200 2850 3500 2850
Connection ~ 4200 1750
Wire Wire Line
	4100 3850 4100 4000
Wire Wire Line
	3400 3950 4500 3950
Wire Wire Line
	4300 3950 4300 3850
Wire Wire Line
	4400 3950 4400 3850
Connection ~ 4300 3950
Wire Wire Line
	4500 3950 4500 3850
Connection ~ 4400 3950
Wire Wire Line
	3500 3550 3400 3550
Wire Wire Line
	3400 3550 3400 3950
Connection ~ 4100 3950
Wire Wire Line
	3450 2150 3500 2150
Wire Wire Line
	2750 2250 2850 2250
Wire Wire Line
	2850 2250 2850 1750
Wire Wire Line
	3300 2950 3300 2850
Connection ~ 3300 2850
Wire Wire Line
	2350 2850 2350 2900
Wire Wire Line
	2350 2900 2450 2900
Wire Wire Line
	2450 2900 2450 2850
Wire Wire Line
	2400 2900 2400 2950
Connection ~ 2400 2900
Wire Wire Line
	7150 4950 8300 4950
Wire Wire Line
	7150 4950 7150 6700
Wire Wire Line
	7050 5250 8350 5250
Wire Wire Line
	9800 4200 9350 4200
Connection ~ 7150 5250
Wire Wire Line
	6750 4700 6750 5450
Wire Wire Line
	6750 5250 6850 5250
Wire Wire Line
	6750 4200 6750 4500
Connection ~ 6750 5250
Wire Wire Line
	6200 5600 6450 5600
Wire Wire Line
	6200 5400 6200 5800
Wire Wire Line
	5800 5700 6450 5700
Wire Wire Line
	4200 5450 4200 5650
Wire Wire Line
	4300 5550 4200 5550
Connection ~ 4200 5550
Wire Wire Line
	4600 6200 4600 5750
Wire Wire Line
	3800 5250 3900 5250
Wire Wire Line
	6200 5050 6750 5050
Connection ~ 6750 5050
Connection ~ 6200 5600
Wire Wire Line
	6200 5200 6200 5050
Wire Wire Line
	6300 4800 6750 4800
Connection ~ 6750 4800
Wire Wire Line
	5100 4600 5200 4600
Wire Wire Line
	4200 4850 4200 5050
Wire Wire Line
	4200 4950 4600 4950
Wire Wire Line
	4600 4950 4600 5350
Wire Wire Line
	4600 4850 4800 4850
Wire Wire Line
	4200 4850 4400 4850
Connection ~ 4200 4950
Wire Wire Line
	4800 4850 4800 4800
Connection ~ 5200 4600
Wire Wire Line
	4800 4400 4800 4200
Wire Wire Line
	4800 4200 9150 4200
Wire Wire Line
	5200 4200 5200 4300
Wire Wire Line
	5600 4200 5600 4250
Connection ~ 5200 4200
Wire Wire Line
	5800 4550 6050 4550
Wire Wire Line
	6050 4550 6050 4800
Wire Wire Line
	6050 4800 6100 4800
Connection ~ 5600 4200
Connection ~ 6750 4200
Wire Wire Line
	5200 4500 5200 4950
Wire Wire Line
	5200 4900 5600 4900
Wire Wire Line
	5600 4900 5600 4850
Connection ~ 5200 4900
Wire Wire Line
	900  4800 900  4750
Wire Wire Line
	4900 6200 5000 6200
Wire Wire Line
	3800 6200 4700 6200
Wire Wire Line
	5800 6200 5800 5700
Wire Wire Line
	5200 6200 5800 6200
Wire Wire Line
	4600 5850 4800 5850
Connection ~ 4600 5850
Wire Wire Line
	5000 5850 5800 5850
Connection ~ 5800 5850
Wire Wire Line
	3800 5250 3800 6300
Connection ~ 4600 6200
$Comp
L C_Small C?
U 1 1 5DC6682D
P 8150 5700
F 0 "C?" V 8350 5600 50  0000 L CNN
F 1 "100nF" V 8250 5600 50  0000 L CNN
F 2 "" H 8150 5700 50  0001 C CNN
F 3 "" H 8150 5700 50  0001 C CNN
	1    8150 5700
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5DC67CB6
P 7900 6450
F 0 "#PWR?" H 7900 6200 50  0001 C CNN
F 1 "GND" H 7900 6300 50  0001 C CNN
F 2 "" H 7900 6450 50  0001 C CNN
F 3 "" H 7900 6450 50  0001 C CNN
	1    7900 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 6450 5450 6200
Connection ~ 5450 6200
$Comp
L C_Small C?
U 1 1 5DC6A408
P 4250 7350
F 0 "C?" H 4350 7400 50  0000 L CNN
F 1 "100nF" H 4350 7300 50  0000 L CNN
F 2 "" H 4250 7350 50  0001 C CNN
F 3 "" H 4250 7350 50  0001 C CNN
	1    4250 7350
	-1   0    0    -1  
$EndComp
$Comp
L L_Small L?
U 1 1 5DC6A51A
P 3750 7350
F 0 "L?" H 3780 7390 50  0000 L CNN
F 1 "L_Small" H 3780 7310 50  0000 L CNN
F 2 "" H 3750 7350 50  0001 C CNN
F 3 "" H 3750 7350 50  0001 C CNN
	1    3750 7350
	-1   0    0    1   
$EndComp
Connection ~ 3800 6200
Connection ~ 4300 6200
Wire Wire Line
	10900 2450 10900 4950
Wire Wire Line
	8650 5050 8650 4600
Wire Wire Line
	8500 4950 8650 4950
Connection ~ 8650 4950
Wire Wire Line
	8650 4400 8650 4200
Connection ~ 8650 4200
$EndSCHEMATC