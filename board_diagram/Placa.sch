EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Floresta"
Date "2021-10-04"
Rev "2"
Comp "Estudio Guto Requena"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ADS1115:ADS1115 IC1
U 1 1 614B5421
P 3900 2950
F 0 "IC1" H 4450 3215 50  0000 C CNN
F 1 "ADS1115" H 4450 3124 50  0000 C CNN
F 2 "ADS1115:ADS1115" H 4850 3050 50  0001 L CNN
F 3 "https://cdn-learn.adafruit.com/downloads/pdf/adafruit-4-channel-adc-breakouts.pdf" H 4850 2950 50  0001 L CNN
F 4 "ADAFRUIT INDUSTRIES - ADS1115 - 16 BID ADC- 4 CHANNEL PROGRAMMABLE" H 4850 2850 50  0001 L CNN "Description"
F 5 "3.25" H 4850 2750 50  0001 L CNN "Height"
F 6 "Adafruit" H 4850 2650 50  0001 L CNN "Manufacturer_Name"
F 7 "ADS1115" H 4850 2550 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 4850 2450 50  0001 L CNN "Mouser Part Number"
F 9 "" H 4850 2350 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 4850 2250 50  0001 L CNN "Arrow Part Number"
F 11 "" H 4850 2150 50  0001 L CNN "Arrow Price/Stock"
	1    3900 2950
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0101
U 1 1 614BC9AA
P 1500 2050
F 0 "#PWR0101" H 1500 1900 50  0001 C CNN
F 1 "VCC" H 1517 2223 50  0000 C CNN
F 2 "" H 1500 2050 50  0001 C CNN
F 3 "" H 1500 2050 50  0001 C CNN
	1    1500 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 614BE129
P 3700 2700
F 0 "#PWR0102" H 3700 2450 50  0001 C CNN
F 1 "GND" H 3705 2527 50  0000 C CNN
F 2 "" H 3700 2700 50  0001 C CNN
F 3 "" H 3700 2700 50  0001 C CNN
	1    3700 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 2700 3700 3050
Wire Wire Line
	3700 3050 3900 3050
Wire Wire Line
	1500 2050 1500 2750
$Comp
L ESP32_DevKit_V1_DOIT:ESP32_DevKit_V1_DOIT U1
U 1 1 614B4FFB
P 1500 4150
F 0 "U1" H 1500 5731 50  0000 C CNN
F 1 "ESP32_DevKit_V1_DOIT" H 1500 5640 50  0000 C CNN
F 2 "ESP32_DevKit_V1_DOIT:esp32_devkit_v1_doit" H 1050 5500 50  0001 C CNN
F 3 "https://aliexpress.com/item/32864722159.html" H 1050 5500 50  0001 C CNN
	1    1500 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3150 2900 3150
Wire Wire Line
	3900 3250 3000 3250
$Comp
L ADS1115:ADS1115 IC2
U 1 1 614C61D3
P 3900 4350
F 0 "IC2" H 4450 4615 50  0000 C CNN
F 1 "ADS1115" H 4450 4524 50  0000 C CNN
F 2 "ADS1115:ADS1115" H 4850 4450 50  0001 L CNN
F 3 "https://cdn-learn.adafruit.com/downloads/pdf/adafruit-4-channel-adc-breakouts.pdf" H 4850 4350 50  0001 L CNN
F 4 "ADAFRUIT INDUSTRIES - ADS1115 - 16 BID ADC- 4 CHANNEL PROGRAMMABLE" H 4850 4250 50  0001 L CNN "Description"
F 5 "3.25" H 4850 4150 50  0001 L CNN "Height"
F 6 "Adafruit" H 4850 4050 50  0001 L CNN "Manufacturer_Name"
F 7 "ADS1115" H 4850 3950 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 4850 3850 50  0001 L CNN "Mouser Part Number"
F 9 "" H 4850 3750 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 4850 3650 50  0001 L CNN "Arrow Part Number"
F 11 "" H 4850 3550 50  0001 L CNN "Arrow Price/Stock"
	1    3900 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 614C61E0
P 3700 4100
F 0 "#PWR0103" H 3700 3850 50  0001 C CNN
F 1 "GND" H 3705 3927 50  0000 C CNN
F 2 "" H 3700 4100 50  0001 C CNN
F 3 "" H 3700 4100 50  0001 C CNN
	1    3700 4100
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 4100 3700 4450
Wire Wire Line
	3700 4450 3900 4450
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 614CAEF6
P 1300 1500
F 0 "J1" V 1172 1580 50  0000 L CNN
F 1 "POWER_IN" V 1400 1300 50  0000 L CNN
F 2 "Connector_Phoenix_MC_HighVoltage:PhoenixContact_MCV_1,5_2-G-5.08_1x02_P5.08mm_Vertical" H 1300 1500 50  0001 C CNN
F 3 "~" H 1300 1500 50  0001 C CNN
	1    1300 1500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 614CBFE9
P 1200 1200
F 0 "#PWR0105" H 1200 950 50  0001 C CNN
F 1 "GND" H 1205 1027 50  0000 C CNN
F 2 "" H 1200 1200 50  0001 C CNN
F 3 "" H 1200 1200 50  0001 C CNN
	1    1200 1200
	-1   0    0    1   
$EndComp
Wire Wire Line
	1200 1200 1200 1300
Wire Wire Line
	1300 1200 1300 1300
$Comp
L power:+5V #PWR0106
U 1 1 614CE2DD
P 1300 1200
F 0 "#PWR0106" H 1300 1050 50  0001 C CNN
F 1 "+5V" H 1315 1373 50  0000 C CNN
F 2 "" H 1300 1200 50  0001 C CNN
F 3 "" H 1300 1200 50  0001 C CNN
	1    1300 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0107
U 1 1 614CF3E0
P 1400 2200
F 0 "#PWR0107" H 1400 2050 50  0001 C CNN
F 1 "+5V" H 1415 2373 50  0000 C CNN
F 2 "" H 1400 2200 50  0001 C CNN
F 3 "" H 1400 2200 50  0001 C CNN
	1    1400 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 2200 1400 2750
Wire Wire Line
	2900 3150 2900 4550
Wire Wire Line
	2100 4950 2900 4950
Wire Wire Line
	3000 3250 3000 4650
Wire Wire Line
	2100 5050 3000 5050
Wire Wire Line
	3900 4550 2900 4550
Connection ~ 2900 4550
Wire Wire Line
	2900 4550 2900 4950
Wire Wire Line
	3900 4650 3000 4650
Connection ~ 3000 4650
Wire Wire Line
	3000 4650 3000 5050
Text Label 2900 3150 0    50   ~ 0
SCL
Text Label 3000 3250 0    50   ~ 0
SDA
$Comp
L power:GND #PWR0108
U 1 1 614DD32D
P 3750 3350
F 0 "#PWR0108" H 3750 3100 50  0001 C CNN
F 1 "GND" V 3755 3222 50  0000 R CNN
F 2 "" H 3750 3350 50  0001 C CNN
F 3 "" H 3750 3350 50  0001 C CNN
	1    3750 3350
	0    1    1    0   
$EndComp
Wire Wire Line
	3900 3350 3750 3350
$Comp
L Connector:Conn_01x04_Female J3
U 1 1 614DE31D
P 4250 1950
F 0 "J3" H 4278 1926 50  0000 L CNN
F 1 "BMP_180" H 4150 2200 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4250 1950 50  0001 C CNN
F 3 "~" H 4250 1950 50  0001 C CNN
	1    4250 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0109
U 1 1 614DF4FF
P 4050 1600
F 0 "#PWR0109" H 4050 1450 50  0001 C CNN
F 1 "+5V" H 4065 1773 50  0000 C CNN
F 2 "" H 4050 1600 50  0001 C CNN
F 3 "" H 4050 1600 50  0001 C CNN
	1    4050 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 1600 4050 1850
$Comp
L power:+5V #PWR0110
U 1 1 614E1D73
P 3900 2650
F 0 "#PWR0110" H 3900 2500 50  0001 C CNN
F 1 "+5V" H 3915 2823 50  0000 C CNN
F 2 "" H 3900 2650 50  0001 C CNN
F 3 "" H 3900 2650 50  0001 C CNN
	1    3900 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 2650 3900 2950
$Comp
L power:+5V #PWR0111
U 1 1 614E2775
P 3900 4200
F 0 "#PWR0111" H 3900 4050 50  0001 C CNN
F 1 "+5V" H 3915 4373 50  0000 C CNN
F 2 "" H 3900 4200 50  0001 C CNN
F 3 "" H 3900 4200 50  0001 C CNN
	1    3900 4200
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0112
U 1 1 614E3165
P 3650 4750
F 0 "#PWR0112" H 3650 4600 50  0001 C CNN
F 1 "+5V" V 3665 4878 50  0000 L CNN
F 2 "" H 3650 4750 50  0001 C CNN
F 3 "" H 3650 4750 50  0001 C CNN
	1    3650 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3650 4750 3900 4750
$Comp
L power:GND #PWR0113
U 1 1 614E5564
P 3900 1600
F 0 "#PWR0113" H 3900 1350 50  0001 C CNN
F 1 "GND" H 3905 1427 50  0000 C CNN
F 2 "" H 3900 1600 50  0001 C CNN
F 3 "" H 3900 1600 50  0001 C CNN
	1    3900 1600
	-1   0    0    1   
$EndComp
Wire Wire Line
	3900 1600 3900 1950
Wire Wire Line
	3900 1950 4050 1950
Wire Wire Line
	4050 2050 2900 2050
Wire Wire Line
	2900 2050 2900 3150
Connection ~ 2900 3150
Wire Wire Line
	4050 2150 3000 2150
Wire Wire Line
	3000 2150 3000 3250
Connection ~ 3000 3250
$Comp
L Connector:Conn_01x04_Female J2
U 1 1 614E8108
P 4200 1050
F 0 "J2" H 4228 1026 50  0000 L CNN
F 1 "DHT11" H 4100 1300 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4200 1050 50  0001 C CNN
F 3 "~" H 4200 1050 50  0001 C CNN
	1    4200 1050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0114
U 1 1 614EA052
P 4000 750
F 0 "#PWR0114" H 4000 600 50  0001 C CNN
F 1 "+5V" H 4015 923 50  0000 C CNN
F 2 "" H 4000 750 50  0001 C CNN
F 3 "" H 4000 750 50  0001 C CNN
	1    4000 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 750  4000 950 
Wire Wire Line
	2100 4550 2450 4550
Wire Wire Line
	2450 4550 2450 1050
Wire Wire Line
	2450 1050 4000 1050
Text Label 2450 1050 0    50   ~ 0
DHT11
$Comp
L power:GND #PWR0115
U 1 1 614EF3A6
P 3700 1250
F 0 "#PWR0115" H 3700 1000 50  0001 C CNN
F 1 "GND" V 3705 1122 50  0000 R CNN
F 2 "" H 3700 1250 50  0001 C CNN
F 3 "" H 3700 1250 50  0001 C CNN
	1    3700 1250
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 1250 3700 1250
Wire Wire Line
	3900 4200 3900 4350
$Comp
L Connector:Conn_01x04_Female J5
U 1 1 614F41EF
P 6250 1050
F 0 "J5" H 6278 1026 50  0000 L CNN
F 1 "MQ7" H 6200 1250 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6250 1050 50  0001 C CNN
F 3 "~" H 6250 1050 50  0001 C CNN
	1    6250 1050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0116
U 1 1 614FAB9A
P 5900 750
F 0 "#PWR0116" H 5900 600 50  0001 C CNN
F 1 "+5V" H 5915 923 50  0000 C CNN
F 2 "" H 5900 750 50  0001 C CNN
F 3 "" H 5900 750 50  0001 C CNN
	1    5900 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 750  5900 950 
Wire Wire Line
	5900 950  6050 950 
$Comp
L power:GND #PWR0117
U 1 1 614FD800
P 5750 950
F 0 "#PWR0117" H 5750 700 50  0001 C CNN
F 1 "GND" H 5755 777 50  0000 C CNN
F 2 "" H 5750 950 50  0001 C CNN
F 3 "" H 5750 950 50  0001 C CNN
	1    5750 950 
	-1   0    0    1   
$EndComp
Wire Wire Line
	6050 1050 5750 1050
Wire Wire Line
	5750 1050 5750 950 
Wire Wire Line
	5500 1250 6050 1250
Text Label 5500 1250 0    50   ~ 0
MQ7_sig
Wire Wire Line
	3900 3850 3450 3850
Text Label 3450 3850 0    50   ~ 0
MQ7_sig
Wire Wire Line
	3900 3750 3350 3750
Text Label 3350 3750 0    50   ~ 0
LDR_sig
$Comp
L Sensor_Optical:LDR03 R1
U 1 1 6150CFC0
P 5950 1700
F 0 "R1" V 5625 1700 50  0000 C CNN
F 1 "LDR03" V 5716 1700 50  0000 C CNN
F 2 "OptoDevice:R_LDR_10x8.5mm_P7.6mm_Vertical" V 6125 1700 50  0001 C CNN
F 3 "http://www.elektronica-componenten.nl/WebRoot/StoreNL/Shops/61422969/54F1/BA0C/C664/31B9/2173/C0A8/2AB9/2AEF/LDR03IMP.pdf" H 5950 1650 50  0001 C CNN
	1    5950 1700
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 6150E8D6
P 6350 1700
F 0 "R3" V 6143 1700 50  0000 C CNN
F 1 "10k" V 6234 1700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6280 1700 50  0001 C CNN
F 3 "~" H 6350 1700 50  0001 C CNN
	1    6350 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	6100 1700 6200 1700
$Comp
L power:GND #PWR0118
U 1 1 6150FA5B
P 5650 1700
F 0 "#PWR0118" H 5650 1450 50  0001 C CNN
F 1 "GND" V 5655 1572 50  0000 R CNN
F 2 "" H 5650 1700 50  0001 C CNN
F 3 "" H 5650 1700 50  0001 C CNN
	1    5650 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	5800 1700 5650 1700
$Comp
L power:+5V #PWR0119
U 1 1 61510C38
P 6650 1700
F 0 "#PWR0119" H 6650 1550 50  0001 C CNN
F 1 "+5V" V 6665 1828 50  0000 L CNN
F 2 "" H 6650 1700 50  0001 C CNN
F 3 "" H 6650 1700 50  0001 C CNN
	1    6650 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	6650 1700 6500 1700
Wire Wire Line
	6100 1700 6100 1950
Connection ~ 6100 1700
Text Label 6100 1950 0    50   ~ 0
LDR_sig
$Comp
L Connector:Conn_01x03_Female J4
U 1 1 61516921
P 6200 2500
F 0 "J4" H 6228 2526 50  0000 L CNN
F 1 "UV" H 6200 2700 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6200 2500 50  0001 C CNN
F 3 "~" H 6200 2500 50  0001 C CNN
	1    6200 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2500 5650 2500
$Comp
L power:GND #PWR0120
U 1 1 61518510
P 6000 2300
F 0 "#PWR0120" H 6000 2050 50  0001 C CNN
F 1 "GND" H 6005 2127 50  0000 C CNN
F 2 "" H 6000 2300 50  0001 C CNN
F 3 "" H 6000 2300 50  0001 C CNN
	1    6000 2300
	-1   0    0    1   
$EndComp
Wire Wire Line
	6000 2300 6000 2400
Text Label 5650 2500 0    50   ~ 0
UV_sig
$Comp
L Connector:Conn_01x04_Female J6
U 1 1 615250E4
P 6400 3800
F 0 "J6" H 6428 3776 50  0000 L CNN
F 1 "Rain" H 6350 4000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6400 3800 50  0001 C CNN
F 3 "~" H 6400 3800 50  0001 C CNN
	1    6400 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0122
U 1 1 615250EA
P 6050 3500
F 0 "#PWR0122" H 6050 3350 50  0001 C CNN
F 1 "+5V" H 6065 3673 50  0000 C CNN
F 2 "" H 6050 3500 50  0001 C CNN
F 3 "" H 6050 3500 50  0001 C CNN
	1    6050 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 3500 6050 3700
Wire Wire Line
	6050 3700 6200 3700
$Comp
L power:GND #PWR0123
U 1 1 615250F2
P 5900 3700
F 0 "#PWR0123" H 5900 3450 50  0001 C CNN
F 1 "GND" H 5905 3527 50  0000 C CNN
F 2 "" H 5900 3700 50  0001 C CNN
F 3 "" H 5900 3700 50  0001 C CNN
	1    5900 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	6200 3800 5900 3800
Wire Wire Line
	5900 3800 5900 3700
Wire Wire Line
	5650 4000 6200 4000
Text Label 5650 4000 0    50   ~ 0
RAIN_sig
Wire Wire Line
	3900 3650 3300 3650
Text Label 3300 3650 0    50   ~ 0
UV_sig
Wire Wire Line
	3900 5250 3600 5250
Text Label 3600 5250 0    50   ~ 0
RAIN_sig
$Comp
L Connector:Conn_01x04_Female J8
U 1 1 6153CCCA
P 6450 4800
F 0 "J8" H 6478 4776 50  0000 L CNN
F 1 "Moisture" H 6400 5000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6450 4800 50  0001 C CNN
F 3 "~" H 6450 4800 50  0001 C CNN
	1    6450 4800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0124
U 1 1 6153CCD0
P 6100 4500
F 0 "#PWR0124" H 6100 4350 50  0001 C CNN
F 1 "+5V" H 6115 4673 50  0000 C CNN
F 2 "" H 6100 4500 50  0001 C CNN
F 3 "" H 6100 4500 50  0001 C CNN
	1    6100 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4500 6100 4700
Wire Wire Line
	6100 4700 6250 4700
$Comp
L power:GND #PWR0125
U 1 1 6153CCD8
P 5950 4700
F 0 "#PWR0125" H 5950 4450 50  0001 C CNN
F 1 "GND" H 5955 4527 50  0000 C CNN
F 2 "" H 5950 4700 50  0001 C CNN
F 3 "" H 5950 4700 50  0001 C CNN
	1    5950 4700
	-1   0    0    1   
$EndComp
Wire Wire Line
	6250 4800 5950 4800
Wire Wire Line
	5950 4800 5950 4700
Wire Wire Line
	5700 5000 6250 5000
Text Label 5700 5000 0    50   ~ 0
MOIST_sig
Wire Wire Line
	3900 5150 3450 5150
Text Label 3450 5150 0    50   ~ 0
MOIST_sig
$Comp
L Connector:Screw_Terminal_01x02 J7
U 1 1 6153FD53
P 6400 5550
F 0 "J7" H 6480 5542 50  0000 L CNN
F 1 "Wind" H 6480 5451 50  0000 L CNN
F 2 "Connector_Phoenix_MC_HighVoltage:PhoenixContact_MCV_1,5_2-G-5.08_1x02_P5.08mm_Vertical" H 6400 5550 50  0001 C CNN
F 3 "~" H 6400 5550 50  0001 C CNN
	1    6400 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6154144F
P 6200 6050
F 0 "R2" H 6270 6096 50  0000 L CNN
F 1 "10k" H 6270 6005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6130 6050 50  0001 C CNN
F 3 "~" H 6200 6050 50  0001 C CNN
	1    6200 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 5650 6200 5900
Text Label 5900 5650 0    50   ~ 0
WIND_sig
Wire Wire Line
	6200 5650 5900 5650
Connection ~ 6200 5650
$Comp
L power:GND #PWR0126
U 1 1 6154AE7B
P 6200 6300
F 0 "#PWR0126" H 6200 6050 50  0001 C CNN
F 1 "GND" H 6205 6127 50  0000 C CNN
F 2 "" H 6200 6300 50  0001 C CNN
F 3 "" H 6200 6300 50  0001 C CNN
	1    6200 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 6200 6200 6300
$Comp
L power:VCC #PWR0127
U 1 1 6154C933
P 5650 5550
F 0 "#PWR0127" H 5650 5400 50  0001 C CNN
F 1 "VCC" H 5667 5723 50  0000 C CNN
F 2 "" H 5650 5550 50  0001 C CNN
F 3 "" H 5650 5550 50  0001 C CNN
	1    5650 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 5550 6200 5550
$Comp
L Connector:Conn_01x06_Female J9
U 1 1 6155179C
P 8550 3250
F 0 "J9" H 8578 3226 50  0000 L CNN
F 1 "Conn_01x06_Female" H 8578 3135 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 8550 3250 50  0001 C CNN
F 3 "~" H 8550 3250 50  0001 C CNN
	1    8550 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 3350 7900 3350
Wire Wire Line
	7900 3350 7900 3150
Wire Wire Line
	7900 3150 8350 3150
$Comp
L power:GND #PWR0128
U 1 1 615556F9
P 7900 2700
F 0 "#PWR0128" H 7900 2450 50  0001 C CNN
F 1 "GND" H 7905 2527 50  0000 C CNN
F 2 "" H 7900 2700 50  0001 C CNN
F 3 "" H 7900 2700 50  0001 C CNN
	1    7900 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	7900 2700 7900 3150
Connection ~ 7900 3150
$Comp
L Device:R R4
U 1 1 61557952
P 8350 2700
F 0 "R4" H 8420 2746 50  0000 L CNN
F 1 "R" H 8420 2655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8280 2700 50  0001 C CNN
F 3 "~" H 8350 2700 50  0001 C CNN
	1    8350 2700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0129
U 1 1 6155D162
P 8350 2300
F 0 "#PWR0129" H 8350 2150 50  0001 C CNN
F 1 "+5V" H 8365 2473 50  0000 C CNN
F 2 "" H 8350 2300 50  0001 C CNN
F 3 "" H 8350 2300 50  0001 C CNN
	1    8350 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 2300 8350 2550
Wire Wire Line
	8350 2850 8350 3050
$Comp
L Device:CP C1
U 1 1 61562E77
P 8800 2850
F 0 "C1" V 9055 2850 50  0000 C CNN
F 1 "CP" V 8964 2850 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 8838 2700 50  0001 C CNN
F 3 "~" H 8800 2850 50  0001 C CNN
	1    8800 2850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8650 2850 8350 2850
Connection ~ 8350 2850
$Comp
L power:GND #PWR0130
U 1 1 61566504
P 9150 2850
F 0 "#PWR0130" H 9150 2600 50  0001 C CNN
F 1 "GND" H 9155 2677 50  0000 C CNN
F 2 "" H 9150 2850 50  0001 C CNN
F 3 "" H 9150 2850 50  0001 C CNN
	1    9150 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2850 9150 2850
Wire Wire Line
	8350 3250 7250 3250
Text Label 7250 3250 0    50   ~ 0
LED_ctr
Wire Wire Line
	2100 3850 2300 3850
Text Label 2300 3850 0    50   ~ 0
LED_ctr
$Comp
L power:+5V #PWR0131
U 1 1 615717FC
P 8000 3550
F 0 "#PWR0131" H 8000 3400 50  0001 C CNN
F 1 "+5V" V 8015 3678 50  0000 L CNN
F 2 "" H 8000 3550 50  0001 C CNN
F 3 "" H 8000 3550 50  0001 C CNN
	1    8000 3550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8000 3550 8350 3550
Wire Wire Line
	8350 3450 7250 3450
Text Label 7250 3450 0    50   ~ 0
DUST_sig
Wire Wire Line
	3900 5050 3350 5050
Text Label 3350 5050 0    50   ~ 0
DUST_sig
Wire Wire Line
	2100 3350 2250 3350
Text Label 2250 3350 0    50   ~ 0
WIND_sig
$Comp
L power:+5V #PWR0121
U 1 1 615B5B93
P 6000 2850
F 0 "#PWR0121" H 6000 2700 50  0001 C CNN
F 1 "+5V" H 6015 3023 50  0000 C CNN
F 2 "" H 6000 2850 50  0001 C CNN
F 3 "" H 6000 2850 50  0001 C CNN
	1    6000 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	6000 2600 6000 2850
$Comp
L power:GND #PWR0104
U 1 1 615425BF
P 1400 5950
F 0 "#PWR0104" H 1400 5700 50  0001 C CNN
F 1 "GND" H 1405 5777 50  0000 C CNN
F 2 "" H 1400 5950 50  0001 C CNN
F 3 "" H 1400 5950 50  0001 C CNN
	1    1400 5950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 61542A11
P 1500 5950
F 0 "#PWR0132" H 1500 5700 50  0001 C CNN
F 1 "GND" H 1505 5777 50  0000 C CNN
F 2 "" H 1500 5950 50  0001 C CNN
F 3 "" H 1500 5950 50  0001 C CNN
	1    1500 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 5550 1500 5950
Wire Wire Line
	1400 5550 1400 5950
$EndSCHEMATC