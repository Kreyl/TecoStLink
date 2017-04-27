EESchema Schematic File Version 2
LIBS:Connectors_kl
LIBS:ESDProtection
LIBS:pcb_details
LIBS:power
LIBS:Power_kl
LIBS:st_kl
LIBS:Switches
LIBS:Tittar_kl
LIBS:Transistors_kl
LIBS:Teco-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
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
L STM32F10X-48 DD1
U 1 1 5901E00F
P 3700 2600
F 0 "DD1" H 3700 4087 60  0000 C CNN
F 1 "STM32F103CB" H 3700 3981 60  0000 C CNN
F 2 "LQFP_TQFP:LQFP48" H 3700 2600 60  0001 C CNN
F 3 "" H 3700 2600 60  0000 C CNN
F 4 "STM32F103CBT6" H 3700 2600 60  0001 C CNN "PN"
F 5 "48" H 3700 2600 60  0001 C CNN "SolderPoints"
F 6 "209" H 3700 2600 60  0001 C CNN "Price"
	1    3700 2600
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5901E0B8
P 1850 5600
F 0 "C3" V 1800 5650 50  0000 L CNN
F 1 "0.1u" V 1800 5400 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 1950 5450 28  0001 C BNN
F 3 "" H 1750 5500 60  0001 C CNN
F 4 "0.5" H 1850 5600 60  0001 C CNN "Price"
F 5 "2" H 1950 5700 60  0001 C CNN "SolderPoints"
	1    1850 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5901E15A
P 1850 5900
F 0 "#PWR01" H 1940 5880 30  0001 C CNN
F 1 "GND" H 1850 5820 30  0001 C CNN
F 2 "" H 1850 5900 60  0000 C CNN
F 3 "" H 1850 5900 60  0000 C CNN
	1    1850 5900
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR02
U 1 1 5901E1BD
P 1400 5300
F 0 "#PWR02" H 1400 5260 30  0001 C CNN
F 1 "+3.3V" H 1439 5368 30  0000 C CNN
F 2 "" H 1400 5300 60  0000 C CNN
F 3 "" H 1400 5300 60  0000 C CNN
	1    1400 5300
	-1   0    0    -1  
$EndComp
Text Notes 1800 5150 0    60   ~ 0
1
$Comp
L +3.3V #PWR03
U 1 1 5901E257
P 2250 3250
F 0 "#PWR03" H 2250 3210 30  0001 C CNN
F 1 "+3.3V" H 2289 3318 30  0000 C CNN
F 2 "" H 2250 3250 60  0000 C CNN
F 3 "" H 2250 3250 60  0000 C CNN
	1    2250 3250
	-1   0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5901E3CC
P 2100 5600
F 0 "C4" V 2050 5650 50  0000 L CNN
F 1 "0.1u" V 2050 5400 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 2200 5450 28  0001 C BNN
F 3 "" H 2000 5500 60  0001 C CNN
F 4 "0.5" H 2100 5600 60  0001 C CNN "Price"
F 5 "2" H 2200 5700 60  0001 C CNN "SolderPoints"
	1    2100 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 5901E3D2
P 2100 5900
F 0 "#PWR04" H 2190 5880 30  0001 C CNN
F 1 "GND" H 2100 5820 30  0001 C CNN
F 2 "" H 2100 5900 60  0000 C CNN
F 3 "" H 2100 5900 60  0000 C CNN
	1    2100 5900
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 5901E44D
P 2350 5600
F 0 "C5" V 2300 5650 50  0000 L CNN
F 1 "0.1u" V 2300 5400 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 2450 5450 28  0001 C BNN
F 3 "" H 2250 5500 60  0001 C CNN
F 4 "0.5" H 2350 5600 60  0001 C CNN "Price"
F 5 "2" H 2450 5700 60  0001 C CNN "SolderPoints"
	1    2350 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5901E453
P 2350 5900
F 0 "#PWR05" H 2440 5880 30  0001 C CNN
F 1 "GND" H 2350 5820 30  0001 C CNN
F 2 "" H 2350 5900 60  0000 C CNN
F 3 "" H 2350 5900 60  0000 C CNN
	1    2350 5900
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 5901E45C
P 2600 5600
F 0 "C6" V 2550 5650 50  0000 L CNN
F 1 "0.1u" V 2550 5400 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 2700 5450 28  0001 C BNN
F 3 "" H 2500 5500 60  0001 C CNN
F 4 "0.5" H 2600 5600 60  0001 C CNN "Price"
F 5 "2" H 2700 5700 60  0001 C CNN "SolderPoints"
	1    2600 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5901E462
P 2600 5900
F 0 "#PWR06" H 2690 5880 30  0001 C CNN
F 1 "GND" H 2600 5820 30  0001 C CNN
F 2 "" H 2600 5900 60  0000 C CNN
F 3 "" H 2600 5900 60  0000 C CNN
	1    2600 5900
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5901E4C7
P 2850 5600
F 0 "C7" V 2800 5650 50  0000 L CNN
F 1 "0.1u" V 2800 5400 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 2950 5450 28  0001 C BNN
F 3 "" H 2750 5500 60  0001 C CNN
F 4 "0.5" H 2850 5600 60  0001 C CNN "Price"
F 5 "2" H 2950 5700 60  0001 C CNN "SolderPoints"
	1    2850 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5901E4CD
P 2850 5900
F 0 "#PWR07" H 2940 5880 30  0001 C CNN
F 1 "GND" H 2850 5820 30  0001 C CNN
F 2 "" H 2850 5900 60  0000 C CNN
F 3 "" H 2850 5900 60  0000 C CNN
	1    2850 5900
	1    0    0    -1  
$EndComp
Text Notes 2050 5150 0    60   ~ 0
9
Text Notes 2300 5150 0    60   ~ 0
24
Text Notes 2550 5150 0    60   ~ 0
36
Text Notes 2800 5150 0    60   ~ 0
48
$Comp
L C C1
U 1 1 5901E675
P 1550 5600
F 0 "C1" V 1500 5650 50  0000 L CNN
F 1 "10u" V 1500 5400 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 1650 5450 28  0001 C BNN
F 3 "" H 1450 5500 60  0001 C CNN
F 4 "0.5" H 1550 5600 60  0001 C CNN "Price"
F 5 "2" H 1650 5700 60  0001 C CNN "SolderPoints"
	1    1550 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5901E67B
P 1550 5900
F 0 "#PWR08" H 1640 5880 30  0001 C CNN
F 1 "GND" H 1550 5820 30  0001 C CNN
F 2 "" H 1550 5900 60  0000 C CNN
F 3 "" H 1550 5900 60  0000 C CNN
	1    1550 5900
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 5901E838
P 5400 3650
F 0 "R9" H 5469 3696 50  0000 L CNN
F 1 "10k" H 5469 3605 50  0000 L CNN
F 2 "Resistors:RES_0603" V 5480 3750 28  0001 C CNN
F 3 "" V 5480 3500 60  0000 C CNN
F 4 "0.5" V 5580 3600 60  0001 C CNN "Price"
F 5 "2" V 5680 3700 60  0001 C CNN "SolderPoints"
	1    5400 3650
	1    0    0    -1  
$EndComp
NoConn ~ 4950 3500
$Comp
L GND #PWR09
U 1 1 5901E917
P 5400 4000
F 0 "#PWR09" H 5490 3980 30  0001 C CNN
F 1 "GND" H 5400 3920 30  0001 C CNN
F 2 "" H 5400 4000 60  0000 C CNN
F 3 "" H 5400 4000 60  0000 C CNN
	1    5400 4000
	1    0    0    -1  
$EndComp
Text Notes 5000 4200 0    60   ~ 0
Board ident:\nPC13=0
$Comp
L GND #PWR011
U 1 1 5901EAD2
P 3200 4250
F 0 "#PWR011" H 3290 4230 30  0001 C CNN
F 1 "GND" H 3200 4170 30  0001 C CNN
F 2 "" H 3200 4250 60  0000 C CNN
F 3 "" H 3200 4250 60  0000 C CNN
	1    3200 4250
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 5901EC07
P 5500 3150
F 0 "R10" V 5600 3200 50  0000 C CNN
F 1 "1k" V 5500 3150 50  0000 C CNN
F 2 "Resistors:RES_0603" V 5580 3250 28  0001 C CNN
F 3 "" V 5580 3000 60  0000 C CNN
F 4 "0.5" V 5680 3100 60  0001 C CNN "Price"
F 5 "2" V 5780 3200 60  0001 C CNN "SolderPoints"
	1    5500 3150
	0    1    1    0   
$EndComp
$Comp
L CRYSTAL_H XTAL1
U 1 1 5901ED75
P 6100 3400
F 0 "XTAL1" H 6100 3662 60  0000 C CNN
F 1 "8MHz" H 6100 3556 60  0000 C CNN
F 2 "Quartz:0503x4-4" H 6125 3525 39  0001 C CNN
F 3 "" H 6100 3675 60  0001 C CNN
F 4 "15" H 6200 3775 60  0001 C CNN "Price"
F 5 "4" H 6300 3875 60  0001 C CNN "SolderPoints"
	1    6100 3400
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 5901F289
P 5850 3700
F 0 "C10" V 5800 3750 50  0000 L CNN
F 1 "18pF" V 5800 3450 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 5950 3550 28  0001 C BNN
F 3 "" H 5750 3600 60  0001 C CNN
F 4 "0.5" H 5850 3700 60  0001 C CNN "Price"
F 5 "2" H 5950 3800 60  0001 C CNN "SolderPoints"
	1    5850 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 5901F28F
P 5850 4000
F 0 "#PWR012" H 5940 3980 30  0001 C CNN
F 1 "GND" H 5850 3920 30  0001 C CNN
F 2 "" H 5850 4000 60  0000 C CNN
F 3 "" H 5850 4000 60  0000 C CNN
	1    5850 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5901F3AF
P 6100 4000
F 0 "#PWR013" H 6190 3980 30  0001 C CNN
F 1 "GND" H 6100 3920 30  0001 C CNN
F 2 "" H 6100 4000 60  0000 C CNN
F 3 "" H 6100 4000 60  0000 C CNN
	1    6100 4000
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 5901F459
P 6350 3700
F 0 "C11" V 6300 3750 50  0000 L CNN
F 1 "18pF" V 6300 3450 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 6450 3550 28  0001 C BNN
F 3 "" H 6250 3600 60  0001 C CNN
F 4 "0.5" H 6350 3700 60  0001 C CNN "Price"
F 5 "2" H 6450 3800 60  0001 C CNN "SolderPoints"
	1    6350 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 5901F45F
P 6350 4000
F 0 "#PWR014" H 6440 3980 30  0001 C CNN
F 1 "GND" H 6350 3920 30  0001 C CNN
F 2 "" H 6350 4000 60  0000 C CNN
F 3 "" H 6350 4000 60  0000 C CNN
	1    6350 4000
	1    0    0    -1  
$EndComp
$Comp
L R R22
U 1 1 5902036E
P 9750 900
F 0 "R22" H 9681 854 50  0000 R CNN
F 1 "4k7" H 9681 945 50  0000 R CNN
F 2 "Resistors:RES_0603" V 9830 1000 28  0001 C CNN
F 3 "" V 9830 750 60  0000 C CNN
F 4 "0.5" V 9930 850 60  0001 C CNN "Price"
F 5 "2" V 10030 950 60  0001 C CNN "SolderPoints"
	1    9750 900 
	1    0    0    1   
$EndComp
$Comp
L GND #PWR015
U 1 1 59020374
P 9750 550
F 0 "#PWR015" H 9840 530 30  0001 C CNN
F 1 "GND" H 9750 470 30  0001 C CNN
F 2 "" H 9750 550 60  0000 C CNN
F 3 "" H 9750 550 60  0000 C CNN
	1    9750 550 
	1    0    0    1   
$EndComp
Text Label 5200 1400 0    60   ~ 0
AIN1
NoConn ~ 4950 1500
Text Label 5150 1600 0    60   ~ 0
TXD
Text Label 5150 1700 0    60   ~ 0
RXD
$Comp
L CONN_6 XL3
U 1 1 59020A59
P 10450 1600
F 0 "XL3" H 10578 1657 50  0000 L CNN
F 1 "CONN_6" H 10578 1566 50  0000 L CNN
F 2 "Connectors:PBS-6R" V 10300 1500 60  0001 C CNN
F 3 "" V 10400 1600 60  0001 C CNN
F 4 "0" V 10600 1800 60  0001 C CNN "SolderPoints"
F 5 "6" V 10700 1900 60  0001 C CNN "SolderPointsDIP"
	1    10450 1600
	1    0    0    -1  
$EndComp
Text Label 9950 1850 0    60   ~ 0
TXD
Text Label 9950 1750 0    60   ~ 0
RXD
$Comp
L GND #PWR016
U 1 1 59020D2A
P 10150 1550
F 0 "#PWR016" H 10240 1530 30  0001 C CNN
F 1 "GND" H 10150 1470 30  0001 C CNN
F 2 "" H 10150 1550 60  0000 C CNN
F 3 "" H 10150 1550 60  0000 C CNN
	1    10150 1550
	0    1    1    0   
$EndComp
Text Label 8850 1250 0    60   ~ 0
AIN1
$Comp
L R R17
U 1 1 590213E9
P 9400 1250
F 0 "R17" V 9450 1500 50  0000 C CNN
F 1 "4k7" V 9400 1250 50  0000 C CNN
F 2 "Resistors:RES_0603" V 9480 1350 28  0001 C CNN
F 3 "" V 9480 1100 60  0000 C CNN
F 4 "0.5" V 9580 1200 60  0001 C CNN "Price"
F 5 "2" V 9680 1300 60  0001 C CNN "SolderPoints"
	1    9400 1250
	0    1    -1   0   
$EndComp
Text Label 9800 1250 0    60   ~ 0
EXTPWR
$Comp
L R R11
U 1 1 590217F6
P 7450 5450
F 0 "R11" H 7550 5500 50  0000 C CNN
F 1 "1k" V 7450 5450 50  0000 C CNN
F 2 "Resistors:RES_0603" V 7530 5550 28  0001 C CNN
F 3 "" V 7530 5300 60  0000 C CNN
F 4 "0.5" V 7630 5400 60  0001 C CNN "Price"
F 5 "2" V 7730 5500 60  0001 C CNN "SolderPoints"
	1    7450 5450
	-1   0    0    1   
$EndComp
Text Label 7850 5150 2    60   ~ 0
EXTPWR
$Comp
L LED D1
U 1 1 59021A29
P 7450 6000
F 0 "D1" V 7359 6112 50  0000 L CNN
F 1 "LED" V 7450 6112 50  0000 L CNN
F 2 "LEDs:LED_0603" H 7450 6040 60  0001 C CNN
F 3 "" H 7550 6140 60  0001 C CNN
F 4 "4" H 7650 6240 60  0001 C CNN "Price"
F 5 "2" H 7750 6340 60  0001 C CNN "SolderPoints"
F 6 "Green" V 7541 6112 50  0000 L CIN "Color"
	1    7450 6000
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR017
U 1 1 59021F2D
P 7450 6300
F 0 "#PWR017" H 7540 6280 30  0001 C CNN
F 1 "GND" H 7450 6220 30  0001 C CNN
F 2 "" H 7450 6300 60  0000 C CNN
F 3 "" H 7450 6300 60  0000 C CNN
	1    7450 6300
	-1   0    0    -1  
$EndComp
Text Notes 7700 6100 0    60   ~ 0
Connected\ndevice is\npowered
NoConn ~ 4950 1800
Text Label 5100 1900 0    60   ~ 0
T_JTCK
Text Label 2050 1400 0    60   ~ 0
T_NRST
NoConn ~ 2450 1500
$Comp
L R R18
U 1 1 59022AAD
P 9400 1450
F 0 "R18" V 9450 1700 50  0000 C CNN
F 1 "22R" V 9400 1450 50  0000 C CNN
F 2 "Resistors:RES_0603" V 9480 1550 28  0001 C CNN
F 3 "" V 9480 1300 60  0000 C CNN
F 4 "0.5" V 9580 1400 60  0001 C CNN "Price"
F 5 "2" V 9680 1500 60  0001 C CNN "SolderPoints"
	1    9400 1450
	0    1    -1   0   
$EndComp
Text Label 8800 1450 0    60   ~ 0
T_JTCK
NoConn ~ 4950 2000
Text Notes 5100 2000 0    60   ~ 0
T_JTDO
Text Notes 5100 2100 0    60   ~ 0
T_JTDI
NoConn ~ 4950 2100
Text Label 2050 2700 0    60   ~ 0
T_JTCK
$Comp
L R R20
U 1 1 590241D2
P 9450 2200
F 0 "R20" V 9500 2450 50  0000 C CNN
F 1 "22R" V 9450 2200 50  0000 C CNN
F 2 "Resistors:RES_0603" V 9530 2300 28  0001 C CNN
F 3 "" V 9530 2050 60  0000 C CNN
F 4 "0.5" V 9630 2150 60  0001 C CNN "Price"
F 5 "2" V 9730 2250 60  0001 C CNN "SolderPoints"
	1    9450 2200
	0    1    -1   0   
$EndComp
Text Label 8800 2200 0    60   ~ 0
T_NRST
Text Notes 2050 1500 0    60   ~ 0
T_JRST
$Comp
L GND #PWR018
U 1 1 5902498F
P 2300 1600
F 0 "#PWR018" H 2390 1580 30  0001 C CNN
F 1 "GND" H 2300 1520 30  0001 C CNN
F 2 "" H 2300 1600 60  0000 C CNN
F 3 "" H 2300 1600 60  0000 C CNN
	1    2300 1600
	0    1    1    0   
$EndComp
NoConn ~ 2450 2400
NoConn ~ 2450 2500
Text Label 1850 2600 0    60   ~ 0
T_SWDIO_IN
NoConn ~ 2450 2900
Text Notes 2050 2900 0    60   ~ 0
PWR_EN
Text Label 2050 2800 0    60   ~ 0
T_JTMS
Text Label 8800 1650 0    60   ~ 0
T_JTMS
$Comp
L R R19
U 1 1 59025889
P 9400 1650
F 0 "R19" V 9450 1900 50  0000 C CNN
F 1 "22R" V 9400 1650 50  0000 C CNN
F 2 "Resistors:RES_0603" V 9480 1750 28  0001 C CNN
F 3 "" V 9480 1500 60  0000 C CNN
F 4 "0.5" V 9580 1600 60  0001 C CNN "Price"
F 5 "2" V 9680 1700 60  0001 C CNN "SolderPoints"
	1    9400 1650
	0    1    -1   0   
$EndComp
$Comp
L R R1
U 1 1 59025DEB
P 1500 2600
F 0 "R1" V 1600 2600 50  0000 C CNN
F 1 "100R" V 1500 2600 50  0000 C CNN
F 2 "Resistors:RES_0603" V 1580 2700 28  0001 C CNN
F 3 "" V 1580 2450 60  0000 C CNN
F 4 "0.5" V 1680 2550 60  0001 C CNN "Price"
F 5 "2" V 1780 2650 60  0001 C CNN "SolderPoints"
	1    1500 2600
	0    1    -1   0   
$EndComp
Text Label 850  2600 0    60   ~ 0
T_JTMS
Text Notes 5100 2200 0    60   ~ 0
MCO
NoConn ~ 4950 2200
Text Label 5100 2300 0    60   ~ 0
LED_STLINK
Text Label 8250 5450 0    60   ~ 0
LED_STLINK
$Comp
L R R15
U 1 1 5902725A
P 9150 5450
F 0 "R15" V 9250 5450 50  0000 C CNN
F 1 "1k" V 9150 5450 50  0000 C CNN
F 2 "Resistors:RES_0603" V 9230 5550 28  0001 C CNN
F 3 "" V 9230 5300 60  0000 C CNN
F 4 "0.5" V 9330 5400 60  0001 C CNN "Price"
F 5 "2" V 9430 5500 60  0001 C CNN "SolderPoints"
	1    9150 5450
	0    1    -1   0   
$EndComp
$Comp
L R R16
U 1 1 59027330
P 9150 6050
F 0 "R16" V 9250 6050 50  0000 C CNN
F 1 "1k" V 9150 6050 50  0000 C CNN
F 2 "Resistors:RES_0603" V 9230 6150 28  0001 C CNN
F 3 "" V 9230 5900 60  0000 C CNN
F 4 "0.5" V 9330 6000 60  0001 C CNN "Price"
F 5 "2" V 9430 6100 60  0001 C CNN "SolderPoints"
	1    9150 6050
	0    1    -1   0   
$EndComp
$Comp
L LED D2
U 1 1 5902737C
P 9700 5450
F 0 "D2" H 9700 5789 50  0000 C CNN
F 1 "LED" H 9700 5698 50  0000 C CNN
F 2 "LEDs:LED_0603" H 9700 5490 60  0001 C CNN
F 3 "" H 9800 5590 60  0001 C CNN
F 4 "4" H 9900 5690 60  0001 C CNN "Price"
F 5 "2" H 10000 5790 60  0001 C CNN "SolderPoints"
F 6 "Green" H 9700 5607 50  0000 C CIN "Color"
	1    9700 5450
	1    0    0    -1  
$EndComp
$Comp
L LED D3
U 1 1 59027419
P 9700 6050
F 0 "D3" H 9700 6389 50  0000 C CNN
F 1 "LED" H 9700 6298 50  0000 C CNN
F 2 "LEDs:LED_0603" H 9700 6090 60  0001 C CNN
F 3 "" H 9800 6190 60  0001 C CNN
F 4 "4" H 9900 6290 60  0001 C CNN "Price"
F 5 "2" H 10000 6390 60  0001 C CNN "SolderPoints"
F 6 "Blue" H 9700 6207 50  0000 C CIN "Color"
	1    9700 6050
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 59027B26
P 10000 5450
F 0 "#PWR019" H 10090 5430 30  0001 C CNN
F 1 "GND" H 10000 5370 30  0001 C CNN
F 2 "" H 10000 5450 60  0000 C CNN
F 3 "" H 10000 5450 60  0000 C CNN
	1    10000 5450
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR020
U 1 1 59027C46
P 10000 6050
F 0 "#PWR020" H 10000 6010 30  0001 C CNN
F 1 "+3.3V" H 10038 6118 30  0000 C CNN
F 2 "" H 10000 6050 60  0000 C CNN
F 3 "" H 10000 6050 60  0000 C CNN
	1    10000 6050
	1    0    0    -1  
$EndComp
$Comp
L R R21
U 1 1 59028020
P 9450 2400
F 0 "R21" V 9500 2650 50  0000 C CNN
F 1 "22R" V 9450 2400 50  0000 C CNN
F 2 "Resistors:RES_0603" V 9530 2500 28  0001 C CNN
F 3 "" V 9530 2250 60  0000 C CNN
F 4 "0.5" V 9630 2350 60  0001 C CNN "Price"
F 5 "2" V 9730 2450 60  0001 C CNN "SolderPoints"
	1    9450 2400
	0    1    -1   0   
$EndComp
Text Label 5100 2400 0    60   ~ 0
T_SWO
Text Label 8800 2400 0    60   ~ 0
T_SWO
$Comp
L USB_MINI_B XL1
U 1 1 590288E7
P 1250 6900
F 0 "XL1" H 1151 7387 60  0000 C CNN
F 1 "USB_MINI_B" H 1151 7281 60  0000 C CNN
F 2 "Connectors:USBmicro_MOLEX_WM17142" H 1250 6900 60  0001 C CNN
F 3 "" H 1250 6900 60  0000 C CNN
	1    1250 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 5900 1850 5800
Wire Wire Line
	1400 5300 2850 5300
Wire Wire Line
	1850 5300 1850 5400
Wire Wire Line
	2250 3250 2450 3250
Wire Wire Line
	2350 3250 2350 3750
Wire Wire Line
	2350 3350 2450 3350
Connection ~ 2350 3250
Wire Wire Line
	2350 3450 2450 3450
Connection ~ 2350 3350
Wire Wire Line
	2350 3600 2450 3600
Connection ~ 2350 3450
Wire Wire Line
	2350 3750 2450 3750
Connection ~ 2350 3600
Wire Wire Line
	2100 5900 2100 5800
Wire Wire Line
	2350 5900 2350 5800
Wire Wire Line
	2600 5900 2600 5800
Wire Wire Line
	2850 5900 2850 5800
Wire Wire Line
	2100 5300 2100 5400
Connection ~ 1850 5300
Wire Wire Line
	2350 5300 2350 5400
Connection ~ 2100 5300
Wire Wire Line
	2600 5300 2600 5400
Connection ~ 2350 5300
Wire Wire Line
	2850 5300 2850 5400
Connection ~ 2600 5300
Wire Wire Line
	1550 5900 1550 5800
Wire Wire Line
	1550 5300 1550 5400
Connection ~ 1550 5300
Wire Wire Line
	4950 3300 5400 3300
Wire Wire Line
	5400 3300 5400 3400
Wire Wire Line
	5400 4000 5400 3900
Wire Wire Line
	3200 4050 3200 4250
Wire Wire Line
	3300 4050 3300 4150
Wire Wire Line
	3200 4150 3650 4150
Connection ~ 3200 4150
Wire Wire Line
	3400 4150 3400 4050
Connection ~ 3300 4150
Wire Wire Line
	3500 4150 3500 4050
Connection ~ 3400 4150
Wire Wire Line
	3650 4150 3650 4050
Connection ~ 3500 4150
Wire Wire Line
	5850 4000 5850 3900
Wire Wire Line
	6100 3550 6100 4000
Wire Wire Line
	6350 4000 6350 3900
Wire Wire Line
	4950 3150 5250 3150
Wire Wire Line
	5750 3150 5850 3150
Wire Wire Line
	5850 3150 5850 3500
Wire Wire Line
	5950 3400 5850 3400
Connection ~ 5850 3400
Wire Wire Line
	6350 3050 6350 3500
Wire Wire Line
	6350 3400 6250 3400
Wire Wire Line
	4950 3050 6350 3050
Connection ~ 6350 3400
Wire Wire Line
	9750 550  9750 650 
Wire Wire Line
	4950 1400 5200 1400
Wire Wire Line
	5150 1700 4950 1700
Wire Wire Line
	4950 1600 5150 1600
Wire Wire Line
	9950 1850 10250 1850
Wire Wire Line
	10250 1750 9950 1750
Wire Wire Line
	10150 1550 10250 1550
Wire Wire Line
	8850 1250 9150 1250
Wire Wire Line
	7450 6300 7450 6200
Wire Wire Line
	7450 5800 7450 5700
Wire Wire Line
	7450 5200 7450 5150
Wire Wire Line
	7450 5150 7850 5150
Wire Wire Line
	5100 1900 4950 1900
Wire Wire Line
	2050 1400 2450 1400
Wire Wire Line
	9650 1250 10200 1250
Wire Wire Line
	9650 1450 10250 1450
Wire Wire Line
	8800 1450 9150 1450
Wire Wire Line
	2050 2700 2450 2700
Wire Wire Line
	8800 2200 9200 2200
Wire Wire Line
	2300 1600 2450 1600
Wire Wire Line
	2450 2600 1750 2600
Wire Wire Line
	2050 2800 2450 2800
Wire Wire Line
	9650 1650 10250 1650
Wire Wire Line
	9150 1650 8800 1650
Wire Wire Line
	850  2600 1250 2600
Wire Wire Line
	5100 2300 4950 2300
Wire Wire Line
	8250 5450 8900 5450
Wire Wire Line
	8800 5450 8800 6050
Wire Wire Line
	8800 6050 8900 6050
Connection ~ 8800 5450
Wire Wire Line
	9400 5450 9500 5450
Wire Wire Line
	9400 6050 9500 6050
Wire Wire Line
	10000 5450 9900 5450
Wire Wire Line
	10000 6050 9900 6050
Wire Wire Line
	5100 2400 4950 2400
Wire Wire Line
	8800 2400 9200 2400
Wire Wire Line
	900  7400 900  7450
Wire Wire Line
	900  7450 1000 7450
Wire Wire Line
	1000 7450 1000 7400
Wire Wire Line
	1100 7400 1100 7450
Wire Wire Line
	1100 7450 1200 7450
Wire Wire Line
	1200 7450 1200 7400
$Comp
L GND #PWR021
U 1 1 59028E7A
P 2700 7200
F 0 "#PWR021" H 2790 7180 30  0001 C CNN
F 1 "GND" H 2700 7120 30  0001 C CNN
F 2 "" H 2700 7200 60  0000 C CNN
F 3 "" H 2700 7200 60  0000 C CNN
	1    2700 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 7000 1450 7000
Wire Wire Line
	1450 7100 2200 7100
Connection ~ 1550 7100
Text Label 1650 6900 0    60   ~ 0
USB_D+
Text Label 1650 6800 0    60   ~ 0
USB_D-
Wire Wire Line
	1650 6800 1450 6800
Wire Wire Line
	1450 6900 1650 6900
Text Label 5200 2600 0    60   ~ 0
USB_D+
Text Label 5200 2500 0    60   ~ 0
USB_D-
Wire Wire Line
	5200 2500 4950 2500
Wire Wire Line
	4950 2600 5200 2600
Text Label 5100 2700 0    60   ~ 0
STM_JTMS
Wire Wire Line
	5100 2700 4950 2700
$Comp
L CONN_4 XL2
U 1 1 59029BAC
P 8000 1500
F 0 "XL2" H 8057 1842 40  0000 C CNN
F 1 "CONN_4" H 8057 1766 40  0000 C CNN
F 2 "Connectors:PLS-4" V 7850 1400 60  0001 C CNN
F 3 "" V 7950 1500 60  0001 C CNN
F 4 "0" V 8350 1900 60  0001 C CNN "SolderPoints"
F 5 "4" V 8450 2000 60  0001 C CNN "SolderPointsDIP"
	1    8000 1500
	-1   0    0    -1  
$EndComp
$Comp
L +3.3V #PWR022
U 1 1 5902A6EC
P 7650 1350
F 0 "#PWR022" H 7650 1310 30  0001 C CNN
F 1 "+3.3V" H 7689 1418 30  0000 C CNN
F 2 "" H 7650 1350 60  0000 C CNN
F 3 "" H 7650 1350 60  0000 C CNN
	1    7650 1350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7650 1350 7800 1350
$Comp
L GND #PWR023
U 1 1 5902A95C
P 7700 1550
F 0 "#PWR023" H 7790 1530 30  0001 C CNN
F 1 "GND" H 7700 1470 30  0001 C CNN
F 2 "" H 7700 1550 60  0000 C CNN
F 3 "" H 7700 1550 60  0000 C CNN
	1    7700 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 1550 7800 1550
Text Label 7150 1650 0    60   ~ 0
STM_JTMS
Wire Wire Line
	7150 1650 7800 1650
Text Label 5100 2800 0    60   ~ 0
STM_JTCK
Wire Wire Line
	5100 2800 4950 2800
Text Label 7150 1450 0    60   ~ 0
STM_JTCK
Wire Wire Line
	7150 1450 7800 1450
Text Notes 7450 1050 0    60   ~ 0
Inner\nprogramming\nconnector
Text Label 5200 2900 0    60   ~ 0
USB_RENUM
Wire Wire Line
	5200 2900 4950 2900
Text Label 5600 6950 0    60   ~ 0
USB_RENUM
Wire Wire Line
	1450 6700 2200 6700
$Comp
L R R4
U 1 1 5902C744
P 4150 7250
F 0 "R4" V 4250 7250 50  0000 C CNN
F 1 "1k5" V 4150 7250 50  0000 C CNN
F 2 "Resistors:RES_0603" V 4230 7350 28  0001 C CNN
F 3 "" V 4230 7100 60  0000 C CNN
F 4 "0.5" V 4330 7200 60  0001 C CNN "Price"
F 5 "2" V 4430 7300 60  0001 C CNN "SolderPoints"
	1    4150 7250
	0    1    -1   0   
$EndComp
Text Label 3500 7250 0    60   ~ 0
USB_D+
Wire Wire Line
	3500 7250 3900 7250
$Comp
L NPN Q1
U 1 1 5902CC3A
P 4550 6950
F 0 "Q1" H 4688 7003 60  0000 L CNN
F 1 "BC848" H 4688 6897 60  0000 L CNN
F 2 "SOT:SOT23-3" H 4650 6900 60  0001 C CNN
F 3 "" H 4750 7000 60  0001 C CNN
F 4 "3" H 4850 7100 60  0001 C CNN "SolderPoints"
	1    4550 6950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4500 7150 4500 7250
Wire Wire Line
	4500 7250 4400 7250
$Comp
L +3.3V #PWR024
U 1 1 5902CE63
P 4500 6650
F 0 "#PWR024" H 4500 6610 30  0001 C CNN
F 1 "+3.3V" V 4539 6687 30  0000 L CNN
F 2 "" H 4500 6650 60  0000 C CNN
F 3 "" H 4500 6650 60  0000 C CNN
	1    4500 6650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4500 6650 4500 6750
$Comp
L R R7
U 1 1 5902D049
P 5250 6950
F 0 "R7" V 5350 6950 50  0000 C CNN
F 1 "100R" V 5250 6950 50  0000 C CNN
F 2 "Resistors:RES_0603" V 5330 7050 28  0001 C CNN
F 3 "" V 5330 6800 60  0000 C CNN
F 4 "0.5" V 5430 6900 60  0001 C CNN "Price"
F 5 "2" V 5530 7000 60  0001 C CNN "SolderPoints"
	1    5250 6950
	0    1    -1   0   
$EndComp
Wire Wire Line
	5600 6950 5500 6950
$Comp
L R R6
U 1 1 5902D9DF
P 4850 7250
F 0 "R6" H 4950 7300 50  0000 C CNN
F 1 "36k" V 4850 7250 50  0000 C CNN
F 2 "Resistors:RES_0603" V 4930 7350 28  0001 C CNN
F 3 "" V 4930 7100 60  0000 C CNN
F 4 "0.5" V 5030 7200 60  0001 C CNN "Price"
F 5 "2" V 5130 7300 60  0001 C CNN "SolderPoints"
	1    4850 7250
	-1   0    0    1   
$EndComp
$Comp
L R R5
U 1 1 5902DAA3
P 4850 6600
F 0 "R5" H 4950 6650 50  0000 C CNN
F 1 "10k" V 4850 6600 50  0000 C CNN
F 2 "Resistors:RES_0603" V 4930 6700 28  0001 C CNN
F 3 "" V 4930 6450 60  0000 C CNN
F 4 "0.5" V 5030 6550 60  0001 C CNN "Price"
F 5 "2" V 5130 6650 60  0001 C CNN "SolderPoints"
	1    4850 6600
	-1   0    0    1   
$EndComp
Wire Wire Line
	4850 6850 4850 7000
Wire Wire Line
	4750 6950 5000 6950
Connection ~ 4850 6950
$Comp
L GND #PWR025
U 1 1 5902DE60
P 4850 7600
F 0 "#PWR025" H 4940 7580 30  0001 C CNN
F 1 "GND" H 4850 7520 30  0001 C CNN
F 2 "" H 4850 7600 60  0000 C CNN
F 3 "" H 4850 7600 60  0000 C CNN
	1    4850 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 7600 4850 7500
Wire Notes Line
	500  6000 6950 6000
Wire Notes Line
	6950 500  6950 6550
$Comp
L R R2
U 1 1 5902ECD8
P 1650 1700
F 0 "R2" V 1750 1750 50  0000 C CNN
F 1 "10k" V 1650 1700 50  0000 C CNN
F 2 "Resistors:RES_0603" V 1730 1800 28  0001 C CNN
F 3 "" V 1730 1550 60  0000 C CNN
F 4 "0.5" V 1830 1650 60  0001 C CNN "Price"
F 5 "2" V 1930 1750 60  0001 C CNN "SolderPoints"
	1    1650 1700
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR026
U 1 1 5902ED5A
P 1250 1700
F 0 "#PWR026" H 1340 1680 30  0001 C CNN
F 1 "GND" H 1250 1620 30  0001 C CNN
F 2 "" H 1250 1700 60  0000 C CNN
F 3 "" H 1250 1700 60  0000 C CNN
	1    1250 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	1250 1700 1400 1700
Wire Wire Line
	1900 1700 2450 1700
Text Notes 1850 1700 0    60   ~ 0
PWR_EXT
NoConn ~ 2450 1800
NoConn ~ 2450 1900
NoConn ~ 2450 2000
NoConn ~ 2450 2100
NoConn ~ 2450 2200
NoConn ~ 2450 2300
Connection ~ 9750 1250
Wire Wire Line
	9750 1250 9750 1150
Wire Wire Line
	10200 1250 10200 1350
Wire Wire Line
	10200 1350 10250 1350
$Comp
L LD1117 DA1
U 1 1 5903155C
P 4250 5300
F 0 "DA1" H 4300 5637 60  0000 C CNN
F 1 "LD1117" H 4300 5531 60  0000 C CNN
F 2 "SOT:SOT223" H 4250 5300 60  0001 C CNN
F 3 "" H 4250 5300 60  0000 C CNN
	1    4250 5300
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 59031CF2
P 3800 5550
F 0 "C8" V 3750 5600 50  0000 L CNN
F 1 "1u" V 3750 5350 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 3900 5400 28  0001 C BNN
F 3 "" H 3700 5450 60  0001 C CNN
F 4 "0.5" H 3800 5550 60  0001 C CNN "Price"
F 5 "2" H 3900 5650 60  0001 C CNN "SolderPoints"
	1    3800 5550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 59031CF8
P 3800 5850
F 0 "#PWR027" H 3890 5830 30  0001 C CNN
F 1 "GND" H 3800 5770 30  0001 C CNN
F 2 "" H 3800 5850 60  0000 C CNN
F 3 "" H 3800 5850 60  0000 C CNN
	1    3800 5850
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 59031D00
P 4950 5550
F 0 "C9" V 4900 5600 50  0000 L CNN
F 1 "1u" V 4900 5350 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 5050 5400 28  0001 C BNN
F 3 "" H 4850 5450 60  0001 C CNN
F 4 "0.5" H 4950 5550 60  0001 C CNN "Price"
F 5 "2" H 5050 5650 60  0001 C CNN "SolderPoints"
	1    4950 5550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR028
U 1 1 59031D06
P 4950 5850
F 0 "#PWR028" H 5040 5830 30  0001 C CNN
F 1 "GND" H 4950 5770 30  0001 C CNN
F 2 "" H 4950 5850 60  0000 C CNN
F 3 "" H 4950 5850 60  0000 C CNN
	1    4950 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 5850 3800 5750
Wire Wire Line
	4950 5850 4950 5750
$Comp
L +5V #PWR029
U 1 1 590322EA
P 2700 6700
F 0 "#PWR029" H 2700 6660 30  0001 C CNN
F 1 "+5V" H 2738 6768 30  0000 C CNN
F 2 "" H 2700 6700 60  0000 C CNN
F 3 "" H 2700 6700 60  0000 C CNN
	1    2700 6700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR030
U 1 1 590323CE
P 4850 6250
F 0 "#PWR030" H 4850 6210 30  0001 C CNN
F 1 "+5V" V 4889 6288 30  0000 L CNN
F 2 "" H 4850 6250 60  0000 C CNN
F 3 "" H 4850 6250 60  0000 C CNN
	1    4850 6250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4850 6250 4850 6350
$Comp
L +5V #PWR031
U 1 1 59032DF7
P 3650 5250
F 0 "#PWR031" H 3650 5210 30  0001 C CNN
F 1 "+5V" H 3689 5318 30  0000 C CNN
F 2 "" H 3650 5250 60  0000 C CNN
F 3 "" H 3650 5250 60  0000 C CNN
	1    3650 5250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3650 5250 3950 5250
Wire Wire Line
	3800 5350 3800 5250
Connection ~ 3800 5250
Wire Wire Line
	4650 5400 4750 5400
Wire Wire Line
	4750 5400 4750 5250
Wire Wire Line
	4650 5250 5050 5250
Wire Wire Line
	4950 5250 4950 5350
Connection ~ 4750 5250
$Comp
L GND #PWR032
U 1 1 59033228
P 4250 5850
F 0 "#PWR032" H 4340 5830 30  0001 C CNN
F 1 "GND" H 4250 5770 30  0001 C CNN
F 2 "" H 4250 5850 60  0000 C CNN
F 3 "" H 4250 5850 60  0000 C CNN
	1    4250 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 5650 4250 5850
$Comp
L +3.3V #PWR033
U 1 1 59033A01
P 5050 5250
F 0 "#PWR033" H 5050 5210 30  0001 C CNN
F 1 "+3.3V" H 5088 5318 30  0000 C CNN
F 2 "" H 5050 5250 60  0000 C CNN
F 3 "" H 5050 5250 60  0000 C CNN
	1    5050 5250
	1    0    0    -1  
$EndComp
Connection ~ 4950 5250
Wire Notes Line
	500  4800 11200 4800
$Comp
L Logo Logo1
U 1 1 59034F61
P 10750 5850
F 0 "Logo1" H 10700 6500 60  0000 L CNN
F 1 "Logo" H 10700 6400 60  0000 L CNN
F 2 "Pictures:Ostranna_12d7_10d1" H 10750 5850 60  0001 C CNN
F 3 "" H 10750 5850 60  0001 C CNN
	1    10750 5850
	1    0    0    -1  
$EndComp
Wire Notes Line
	10350 4800 10350 6500
Text Label 10750 3200 2    60   ~ 0
EXTPWR
Wire Notes Line
	6950 2850 11200 2850
Wire Notes Line
	8400 500  8400 2850
$Comp
L CONN_2 XL4
U 1 1 5901F868
P 10450 2300
F 0 "XL4" H 10577 2328 40  0000 L CNN
F 1 "CONN_2" H 10577 2252 40  0000 L CNN
F 2 "Connectors:PBS-2R" H 10450 2300 60  0001 C CNN
F 3 "" H 10450 2300 60  0000 C CNN
	1    10450 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 2200 9700 2200
Wire Wire Line
	9700 2400 10250 2400
$Comp
L HOLE_METALLED HOLE5
U 1 1 5902043C
P 7700 2450
F 0 "HOLE5" H 7600 2700 60  0000 C CNN
F 1 "HOLE_METALLED" H 7450 2600 60  0000 C CNN
F 2 "PCB:Hole3_5_out6mm" H 7100 2300 60  0001 C CNN
F 3 "" H 7200 2400 60  0001 C CNN
F 4 "0" H 7300 2500 60  0001 C CNN "Price"
F 5 "0" H 7400 2600 60  0001 C CNN "SolderPoints"
	1    7700 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR034
U 1 1 5902104A
P 8000 2550
F 0 "#PWR034" H 8090 2530 30  0001 C CNN
F 1 "GND" H 8000 2470 30  0001 C CNN
F 2 "" H 8000 2550 60  0000 C CNN
F 3 "" H 8000 2550 60  0000 C CNN
	1    8000 2550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8000 2550 8000 2450
Text Notes 7200 2100 0    60   ~ 0
Aux GND connection
$Comp
L P_3PIN Q3
U 1 1 59024EC3
P 8750 3250
F 0 "Q3" H 8750 3525 50  0000 C CNN
F 1 "IRLML9303" H 8750 3434 50  0000 C CNN
F 2 "SOT:SOT23-3" V 8800 3350 60  0001 C CNN
F 3 "" H 8950 3300 60  0001 C CNN
F 4 "3" H 9050 3400 60  0001 C CNN "SolderPoints"
	1    8750 3250
	1    0    0    -1  
$EndComp
$Comp
L D_Shottky D4
U 1 1 590261ED
P 10150 3200
F 0 "D4" H 10150 3392 40  0000 C CNN
F 1 "PMEG2020" H 10150 3316 40  0000 C CNN
F 2 "Diodes:SOD323" H 10050 3200 60  0001 C CNN
F 3 "" H 10150 3300 60  0001 C CNN
F 4 "5" H 10250 3400 60  0001 C CNN "Price"
F 5 "2" H 10350 3500 60  0001 C CNN "SolderPoints"
	1    10150 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 3200 10750 3200
$Comp
L NPN Q2
U 1 1 59026BCA
P 8050 4150
F 0 "Q2" H 8188 4203 60  0000 L CNN
F 1 "BC848" H 8188 4097 60  0000 L CNN
F 2 "SOT:SOT23-3" H 8150 4100 60  0001 C CNN
F 3 "" H 8250 4200 60  0001 C CNN
F 4 "3" H 8350 4300 60  0001 C CNN "SolderPoints"
	1    8050 4150
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR035
U 1 1 59027180
P 8000 4450
F 0 "#PWR035" H 8090 4430 30  0001 C CNN
F 1 "GND" H 8000 4370 30  0001 C CNN
F 2 "" H 8000 4450 60  0000 C CNN
F 3 "" H 8000 4450 60  0000 C CNN
	1    8000 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 4450 8000 4350
$Comp
L R R12
U 1 1 59027440
P 8000 3500
F 0 "R12" H 8069 3546 50  0000 L CNN
F 1 "10k" H 8069 3455 50  0000 L CNN
F 2 "Resistors:RES_0603" V 8080 3600 28  0001 C CNN
F 3 "" V 8080 3350 60  0000 C CNN
F 4 "0.5" V 8180 3450 60  0001 C CNN "Price"
F 5 "2" V 8280 3550 60  0001 C CNN "SolderPoints"
	1    8000 3500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR036
U 1 1 59027B66
P 7850 3200
F 0 "#PWR036" H 7850 3160 30  0001 C CNN
F 1 "+3.3V" H 7889 3268 30  0000 C CNN
F 2 "" H 7850 3200 60  0000 C CNN
F 3 "" H 7850 3200 60  0000 C CNN
	1    7850 3200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8000 3250 8000 3200
Connection ~ 8000 3200
$Comp
L R R14
U 1 1 590285BC
P 8600 4150
F 0 "R14" V 8500 4100 50  0000 L CNN
F 1 "100k" V 8600 4050 50  0000 L CNN
F 2 "Resistors:RES_0603" V 8680 4250 28  0001 C CNN
F 3 "" V 8680 4000 60  0000 C CNN
F 4 "0.5" V 8780 4100 60  0001 C CNN "Price"
F 5 "2" V 8880 4200 60  0001 C CNN "SolderPoints"
	1    8600 4150
	0    1    1    0   
$EndComp
$Comp
L C C12
U 1 1 59029D66
P 9850 4150
F 0 "C12" V 9800 4200 50  0000 L CNN
F 1 "0.1u" V 9800 3950 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 9950 4000 28  0001 C BNN
F 3 "" H 9750 4050 60  0001 C CNN
F 4 "0.5" H 9850 4150 60  0001 C CNN "Price"
F 5 "2" H 9950 4250 60  0001 C CNN "SolderPoints"
	1    9850 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR037
U 1 1 59029D6C
P 9850 4450
F 0 "#PWR037" H 9940 4430 30  0001 C CNN
F 1 "GND" H 9850 4370 30  0001 C CNN
F 2 "" H 9850 4450 60  0000 C CNN
F 3 "" H 9850 4450 60  0000 C CNN
	1    9850 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 4450 9850 4350
$Comp
L R R23
U 1 1 59029EF8
P 9850 3500
F 0 "R23" H 9919 3546 50  0000 L CNN
F 1 "1M" H 9919 3455 50  0000 L CNN
F 2 "Resistors:RES_0603" V 9930 3600 28  0001 C CNN
F 3 "" V 9930 3350 60  0000 C CNN
F 4 "0.5" V 10030 3450 60  0001 C CNN "Price"
F 5 "2" V 10130 3550 60  0001 C CNN "SolderPoints"
	1    9850 3500
	1    0    0    -1  
$EndComp
Connection ~ 9850 3200
$Comp
L BUTTON SW1
U 1 1 5902A268
P 9450 3850
F 0 "SW1" H 9450 4105 50  0000 C CNN
F 1 "BUTTON" H 9450 4014 50  0000 C CNN
F 2 "BtnsSwitches:BTN_4x4_SMD" H 9150 3900 60  0001 C CNN
F 3 "" H 9250 4000 60  0001 C CNN
F 4 "1" H 9350 4100 60  0001 C CNN "Price"
F 5 "4" H 9450 4200 60  0001 C CNN "SolderPoints"
	1    9450 3850
	1    0    0    -1  
$EndComp
Connection ~ 9850 3850
$Comp
L R R13
U 1 1 5902A866
P 8350 3850
F 0 "R13" V 8250 3800 50  0000 L CNN
F 1 "10k" V 8350 3750 50  0000 L CNN
F 2 "Resistors:RES_0603" V 8430 3950 28  0001 C CNN
F 3 "" V 8430 3700 60  0000 C CNN
F 4 "0.5" V 8530 3800 60  0001 C CNN "Price"
F 5 "2" V 8630 3900 60  0001 C CNN "SolderPoints"
	1    8350 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 3850 8100 3850
Connection ~ 8000 3850
Connection ~ 8700 3850
Wire Wire Line
	9850 3200 9850 3250
Wire Wire Line
	8000 3750 8000 3950
Wire Wire Line
	8700 3450 8700 3850
Wire Wire Line
	9850 3750 9850 3950
Wire Wire Line
	8600 3850 9150 3850
Wire Wire Line
	7850 3200 8550 3200
Wire Wire Line
	8250 4150 8350 4150
Wire Wire Line
	8850 4150 9050 4150
Wire Wire Line
	9050 4150 9050 3200
Wire Wire Line
	8950 3200 9950 3200
Wire Wire Line
	9750 3850 9850 3850
Connection ~ 9050 3200
$Comp
L L L1
U 1 1 590314AA
P 2400 6700
F 0 "L1" V 2245 6700 40  0000 C CNN
F 1 "BLM15AG102" V 2321 6700 40  0000 C CNN
F 2 "Inductors:IND_0402" V 2460 6710 40  0001 C CNN
F 3 "" V 2300 6560 60  0001 C CNN
F 4 "4" V 2400 6660 60  0001 C CNN "Price"
F 5 "2" V 2500 6760 60  0001 C CNN "SolderPoints"
	1    2400 6700
	0    1    1    0   
$EndComp
Wire Wire Line
	2700 6700 2600 6700
$Comp
L L L2
U 1 1 5903285E
P 2400 7100
F 0 "L2" V 2245 7100 40  0000 C CNN
F 1 "BLM15AG102" V 2321 7100 40  0000 C CNN
F 2 "Inductors:IND_0402" V 2460 7110 40  0001 C CNN
F 3 "" V 2300 6960 60  0001 C CNN
F 4 "4" V 2400 7060 60  0001 C CNN "Price"
F 5 "2" V 2500 7160 60  0001 C CNN "SolderPoints"
	1    2400 7100
	0    1    1    0   
$EndComp
Wire Wire Line
	1550 7100 1550 7000
Wire Wire Line
	2700 7200 2700 7100
Wire Wire Line
	2700 7100 2600 7100
Text Label 1650 6700 0    60   ~ 0
5VIN
Text Label 1650 7100 0    60   ~ 0
GNDIN
Text Label 1350 7600 0    60   ~ 0
5VIN
Text Label 2400 7600 2    60   ~ 0
GNDIN
$Comp
L C C13
U 1 1 59034D7D
P 1850 7600
F 0 "C13" V 1800 7650 50  0000 L CNN
F 1 "10n" V 1800 7400 50  0000 L CNN
F 2 "Capacitors:CAP_0603" V 1950 7450 28  0001 C BNN
F 3 "" H 1750 7500 60  0001 C CNN
F 4 "0.5" H 1850 7600 60  0001 C CNN "Price"
F 5 "2" H 1950 7700 60  0001 C CNN "SolderPoints"
	1    1850 7600
	0    1    1    0   
$EndComp
Wire Wire Line
	1350 7600 1650 7600
Wire Wire Line
	2050 7600 2400 7600
NoConn ~ 2450 3100
NoConn ~ 4950 3400
$EndSCHEMATC
