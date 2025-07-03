// Main.h
#pragma once
//#include <ADS1256.h>

//#define PRINT_TASK_FREE_STACKSIZE_IN_WORDS

/********************************************************************/
/*                      Other defines       */
/********************************************************************/

// 2.0kHz
#define ADC_SAMPLE_RATE ADS1256_DRATE_2000SPS

/********************************************************************/
/*                      Loadcell defines                            */
/********************************************************************/
#define LOADCELL_WEIGHT_RATING_KG 200.0f
#define LOADCELL_EXCITATION_V 5.0f
#define LOADCELL_SENSITIVITY_MV_V 2.0f

// ADC defines
#define PIN_SCK 6    // 16 -->SCLK
#define PIN_MOSI 7   // 17 --> DIN
#define PIN_MISO 15  // 18 --> DOUT
#define PIN_DRDY 16  // 19 --> DRDY
#define PIN_CS 17    // 21 --> CS
#define PIN_RST 18   // 18 --> PDOWN

/********************************************************************/
/*                       Other input pins                           */
/********************************************************************/

#define CONTROLLER_BUTTON_COUNT 2
#define SHIFTER1_PIN 4
#define SHIFTER2_PIN 5
#define ACCEL_PIN_1 9
#define ACCEL_PIN_2 10
#define CLUTCH_PIN_1 11
#define CLUTCH_PIN_2 12

// HX711 pins (for handbrake)
#define HANDBRAKE_DOUT 1
#define HANDBRAKE_SCK 2