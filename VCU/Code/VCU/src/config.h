#pragma once
#include <Arduino.h>

// Pin Definitions
#define SCK 4
#define MOSI 6 
#define MISO 5
#define SPI_CS_PIN 36
#define CAN_INT_PIN 45
#define MAX_DATA_SIZE 8

// Relay Pins
#define PUMP 38
#define PW1 39
#define CONTACTOR 11
#define NLGKL15 12
#define DMCKL15 13
#define BSCKL15 14
#define BCKLIGHT 17
#define LWP5 18
#define LWP6 21
#define LWP7 16

// Input Pins
#define NLG_HW_Wakeup 7
#define IGNITION 8
#define UNLCKCON 9
#define BUTTON_PIN_BITMASK 0x380

// ADC Channels
#define GASPEDAL1 0
#define GASPEDAL2 1
#define REVERSE 0
#define MinValPot 15568
#define MaxValPot 11200

// Battery Parameters
#define MIN_U_BAT 360
#define NOM_U_BAT 382
#define MAX_U_BAT 436
#define MAX_DMC_CURRENT 300
#define MAX_NLG_CURRENT 72
#define PRECHARGE_CURRENT 20

// Vehicle Constants
#define NORMAL_RATIO 1.2
#define REDUCED_RATIO 2.1
#define DIFF_RATIO 3.9
#define WHEEL_CIRC 2.08

// CAN Message IDs
#define BSC_COMM 0x260
#define BSC_LIM 0x261
#define DMCCTRL 0x210
#define DMCLIM 0x211
#define NLG_DEM_LIM 0x711
#define NLG_ACT_ERR 0x799
#define NLG_ACT_LIM 0x728
#define NLG_ACT_PLUG 0x739