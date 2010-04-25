#ifndef PINS_H
#define PINS_H


/****************************************************************************************
* Sanguino/RepRap Motherboard with direct-drive extruders
***
*ATMega644P
*
*                        +---\/---+
*            (D 0) PB0  1|        |40  PA0 (AI 0 / D31)
*            (D 1) PB1  2|        |39  PA1 (AI 1 / D30)
*       INT2 (D 2) PB2  3|        |38  PA2 (AI 2 / D29)
*        PWM (D 3) PB3  4|        |37  PA3 (AI 3 / D28)
*        PWM (D 4) PB4  5|        |36  PA4 (AI 4 / D27)
*       MOSI (D 5) PB5  6|        |35  PA5 (AI 5 / D26)
*       MISO (D 6) PB6  7|        |34  PA6 (AI 6 / D25)
*        SCK (D 7) PB7  8|        |33  PA7 (AI 7 / D24)
*                  RST  9|        |32  AREF
*                  VCC 10|        |31  GND 
*                  GND 11|        |30  AVCC
*                XTAL2 12|        |29  PC7 (D 23)
*                XTAL1 13|        |28  PC6 (D 22)
*       RX0 (D 8)  PD0 14|        |27  PC5 (D 21) TDI
*       TX0 (D 9)  PD1 15|        |26  PC4 (D 20) TDO
*  INT0 RX1 (D 10) PD2 16|        |25  PC3 (D 19) TMS
*  INT1 TX1 (D 11) PD3 17|        |24  PC2 (D 18) TCK
*       PWM (D 12) PD4 18|        |23  PC1 (D 17) SDA
*       PWM (D 13) PD5 19|        |22  PC0 (D 16) SCL
*       PWM (D 14) PD6 20|        |21  PD7 (D 15) PWM
*                        +--------+
*
****************************************************************************************/
#if MOTHERBOARD == 1

/*
#ifndef __AVR_ATmega644P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif
*/
#define LEDPIN        0  // Previously called DEBUG_PIN

#define X_USE_DIRSTEP 1 // change this to 1 if you already have a motor driver and only want the Arduino sending dir and step commands
#define X_STEP_PIN   (byte)15
#define X_DIR_PIN    (byte)18
#define X_ENABLE_PIN (byte)19
#define X_MIN_PIN    (byte)20
#define X_MAX_PIN    (byte)21

#define Y_USE_DIRSTEP 1
#define Y_STEP_PIN   (byte)23
#define Y_DIR_PIN    (byte)22
#define Y_ENABLE_PIN (byte)19
#define Y_MIN_PIN    (byte)25
#define Y_MAX_PIN    (byte)26

#define Z_USE_DIRSTEP 1
#define Z_STEP_PINN   (byte)29
#define Z_DIR_PINN    (byte)30
#define Z_ENABLE_PIN (byte)31
#define Z_MIN_PIN    (byte)2
#define Z_MAX_PIN    (byte)1

#define E_USE_DIRSTEP 1
#define E_STEP_PIN   (byte)12
#define E_DIR_PIN    (byte)16
#define E_ENABLE_PIN (byte)3

//#define FANPIN
//#define PS_ON_PIN
//#define LIGHTPIN
//#define SPINDLEPIN

#define HEATER_0_PIN  (byte)14
#define TEMP_0_PIN  (byte)4    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT (EQUIVALENT) NUMBERING!!!!!!!!!

/*  Unused (1) (2) (3) 4 5 6 7 8 9 10 11 12 13 (14) (15) (16) 17 (18) (19) (20) (21) (22) (23) 24 (25) (26) (27)AI 28 (29) (30) (31)  */


/****************************************************************************************
* RepRap Motherboard  ****---NOOOOOO RS485/EXTRUDER CONTROLLER!!!!!!!!!!!!!!!!!---*******
*
****************************************************************************************/
#elif MOTHERBOARD == 2
/*
#ifndef __AVR_ATmega644P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif
*/
#define X_USE_DIRSTEP 1 // change this to 1 if you already have a motor driver and only want the Arduino sending dir and step commands
#define X_STEP_PIN      15
#define X_DIR_PIN       18
#define X_ENABLE_PIN    19
#define X_MIN_PIN       20
#define X_MAX_PIN       21

#define Y_USE_DIRSTEP 1
#define Y_STEP_PIN      23
#define Y_DIR_PIN       22
#define Y_ENABLE_PIN    24
#define Y_MIN_PIN       25
#define Y_MAX_PIN       26

#define Z_USE_DIRSTEP 1
#define Z_STEP_PINN     27
#define Z_DIR_PINN      28
#define Z_ENABLE_PIN    29
#define Z_MIN_PIN       30
#define Z_MAX_PIN       31

#define E_USE_DIRSTEP 1
#define E_STEP_PIN      17
#define E_DIR_PIN       16

#define LEDPIN           0

#define SD_CARD_WRITE    2
#define SD_CARD_DETECT   3
#define SD_CARD_SELECT   4

//our RS485 pins
#define TX_ENABLE_PIN	12
#define RX_ENABLE_PIN	13

//pin for controlling the PSU.
#define PS_ON_PIN       14

#elif MOTHERBOARD == 3

#define LEDPIN        0  // Previously called DEBUG_PIN

#define X_USE_DIRSTEP 1 // change this to 1 if you already have a motor driver and only want the Arduino sending dir and step commands
#define X_STEP_PIN   (byte)22
#define X_DIR_PIN    (byte)23
#define X_ENABLE_PIN (byte)24
#define X_MIN_PIN    (byte)2
#define X_MAX_PIN    (byte)3

#define Y_USE_DIRSTEP 1
#define Y_STEP_PIN   (byte)25
#define Y_DIR_PIN    (byte)26
#define Y_ENABLE_PIN (byte)27
#define Y_MIN_PIN    (byte)18
#define Y_MAX_PIN    (byte)19

#define Z_USE_DIRSTEP 1
#define Z_STEP_PINN   (byte)28
#define Z_DIR_PINN    (byte)29
#define Z_ENABLE_PIN (byte)30
#define Z_MIN_PIN    (byte)20
#define Z_MAX_PIN    (byte)21


#define E_USE_DC 1
#define E_USE_DIRSTEP 0
#define E_STEP_PIN   (byte)4 // also used for pwm pin on dc motor
#define E_DIR_PIN    (byte)31
#define E_ENABLE_PIN (int)-1

//#define FANPIN
//#define PS_ON_PIN
//#define LIGHTPIN
//#define SPINDLEPIN

#define HEATER_0_PIN  (byte)6
#define TEMP_0_PIN  (byte)0  

#define LOW_HEAT_0 (byte)110
#define HIGH_HEAT_0 (byte)255

/************************
*  HYDRA PINS
************************/
#elif MOTHERBOARD == 64

// x motor declarations
//const boolean INVERT_X_DIR = false;
#define X_USE_DIRSTEP 1 // change this to 1 if you already have a motor driver and only want the Arduino sending dir and step commands
#if X_USE_DIRSTEP
  const int X_DIR_PIN = 30;
  const int X_STEP_PIN = 31;
#else
  const int X_COIL1A = 2;
  const int X_COIL1B = 3;
  const int X_COIL2A = 4;
  const int X_COIL2B = 5;
#endif
#define X_MIN_PIN 48
#define X_MAX_PIN 49

// y motor declarations
//const boolean INVERT_Y_DIR = false;
#define Y_USE_DIRSTEP 1 // change this to 1 if you already have a motor driver and only want the Arduino sending dir and step commands
#if Y_USE_DIRSTEP
  const int Y_DIR_PIN = 32;
  const int Y_STEP_PIN = 33;
#else
  const int Y_COIL1A = 6;
  const int Y_COIL1B = 7;
  const int Y_COIL2A = 8;
  const int Y_COIL2B = 9;
#endif
#define Y_MIN_PIN 46
#define Y_MAX_PIN 47

// z motor declarations
//const boolean INVERT_Z_DIR = false;
#define Z_USE_DIRSTEP 1 // change this to 1 if you already have a motor driver and only want the Arduino sending dir and step commands
#if Z_USE_DIRSTEP
#define Z_DIR_PINN 34
#define Z_STEP_PINN 35
//  const int Z_DIR_PIN[4] = {34, 34, 34, 34};
//  const int Z_STEP_PIN[4] = {35, 35, 35, 35};
#else
  const int Z_COIL1A = 38;
  const int Z_COIL1B = 40;
  const int Z_COIL2A = 42;
  const int Z_COIL2B = 44;
#endif
#define Z_MIN_PIN 44
#define Z_MAX_PIN 45

// extruder motor declarations
//const boolean INVERT_E_DIR = false;
#define E_USE_DIRSTEP 1 // change this to 1 if you already have a motor driver and only want the Arduino sending dir and step commands
#if E_USE_DIRSTEP
  const int E_DIR_PIN = 36;
  const int E_STEP_PIN = 37;
#else
  const int E_COIL1A = 22;
  const int E_COIL1B = 23;
  const int E_COIL2A = 24;
  const int E_COIL2B = 25;
#endif

// accessory declarations
const int LEDPIN = 13;
const int FANPIN = 51;
const int SPINDLEPIN = 52;
const int LIGHTPIN = 50;

#define HEATER_0_PIN  (byte)12
#define TEMP_0_PIN  (byte)2    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT (EQUIVALENT) NUMBERING!!!!!!!!!


//#else

//#error Unknown MOTHERBOARD value in parameters.h

#endif

#endif
