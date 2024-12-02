
// Author: Christian Obrecht
// Date: 07.11.2023
// Description: Communication with BSC and DMC over CAN and Controll of ADAC


//Libraries
#include <Arduino.h>              //For Arduino syntax
#include <cstdint>
#include <esp_task_wdt.h>         //Multicore tasking
#include <Wire.h>                 //I2C bus driver
#include <SPI.h>                  //SPI bus driver
#include "mcp2515_can.h"          //Can Bus driver
#include <esp_adc_cal.h>          //ADC calib
#include <esp32-hal-adc.h>        //ADC driver
#include "ADS1X15.h"
#include "AD5593R.h"

//Create Tasks for each Core
void CAN_COM (void * pvParameters);
void BACKBONE (void * pvParameters);

SPIClass *customSPI = NULL;

//Task Handles
TaskHandle_t Task1;
TaskHandle_t Task2;


//Function Prototypes
void relayControll();

//CAN functions
void sendNLG();               //Send NLG CAN Messages
void sendBSC();               //Send BSC CAN Messages
void sendDMC();               //Send DMC CAN Messages
void reciveBSC();             //Recive BSC CAN Messages
void reciveDMC();             //Recive DMC CAN Messages
void reciveNLG();             //Recive NLG CAN Messages
void reciveBMS();             //Recive BMS CAN Messages
void reciveINFO();            //Data from DAU(LUCA)
void armColingSys(bool arm);  //Arm Cooling System
void armBattery(bool arm);    //Arm HV-Battery
void updateGearState();       //What do you think????
void chargeManage();          //Manage Charging Process

uint8_t print_GPIO_wake_up();


int16_t calculateTorque5S(); //Calculate Torque from Pedal Position


//Pinout
//SPI
#define SCK 4
#define MOSI 6
#define MISO 5
//CAN
const int SPI_CS_PIN = 36;
const int CAN_INT_PIN = 45; //AKKA BLANK

#define MAX_DATA_SIZE 8

//Relay pinout VCU1.0
/*
#define RELAIS1 11  //HV Battery
#define RELAIS2 12  //Cooling Pump
#define RELAIS3 13  //HV Battery
#define RELAIS4 14  //Charger KL15
#define RELAIS5 17  //Reverse Signal
#define RELAIS6 18  //DMC KL15
#define RELAIS7 21  //BSC KL15
#define RELAIS8 33  //Not Used
*/

#define PUMP 38  //Cooling Pump PW0
#define PW1 39
#define CONTACTOR 11  //HV Battery LPW0
#define NLGKL15 12 //NLG KL15 LPW1
#define DMCKL15 13  //DMC KL15 LWP2
#define BSCKL15 14  //BSC KL15 LWP3
#define BCKLIGHT 17  //Reverse Signal LWP4
#define LWP5 18
#define LWP6 21
#define LWP7 16



//set ADC set adress
ADS1115 ADS(0x48);

//Defining CAN Indexes

//BSC Indexes
#define BSC_COMM 0x260
#define BSC_LIM 0x261

//DMC Indexes
#define DMCCTRL 0x210
#define DMCLIM 0x211
#define DMCCTRL2 0x212

//NLG Indexes
#define NLG_DEM_LIM 0x711
#define NLG_ACT_ERR 0x799
#define NLG_ACT_LIM 0x728
#define NLG_ACT_PLUG 0x739

//*********************************************************************//
//Deffining Variables for Operation
//General
//*********************************************************************//

//Define Wakeup interrupt Pins
#define BUTTON_PIN_BITMASK 0x380 //IO7, IO8, IO9 https://randomnerdtutorials.com/esp32-external-wake-up-deep-sleep/

//Define Vehicle states
#define Standby 0
#define Run 1
#define Charging 2

//Define Gaspedal Chanel 
#define GASPEDAL1 0
#define GASPEDAL2 1

#define MinValPot 15568
#define MaxValPot 11200


//Pin for reverse signal 0 = forward, 1 = reverse pin 0 on AD
#define REVERSE 0
//Battery Voltage values
bool batteryArmed = 0;
#define MIN_U_BAT 360 //3.2V*104S
#define NOM_U_BAT 382 //3.67V*104S
#define MAX_U_BAT 436 //4.2V*104S

#define MAX_DMC_CURRENT 300 //A 
#define MAX_NLG_CURRENT 72 //A
#define PRECHARGE_CURRENT 20 //A Curret of BSC in boost mode

//Define Charger states
#define NLG_ACT_SLEEP 0
#define NLG_ACT_WAKEUP 1 
#define NLG_ACT_STANDBY 2
#define NLG_ACT_READY2CHARGE 3
#define NLG_ACT_CHARGE 4
#define NLG_ACT_SHUTDOWN 5


//INPUT deffinitions
#define NLG_HW_Wakeup 7  //Input for VCU
#define IGNITION 8       //Input for VCU
#define UNLCKCON 9       //Input for VCU

//define possible charger state demands
#define NLG_DEM_STANDBY 0
#define NLG_DEM_CHARGE 1
#define NLG_DEM_SLEEP 6

enum Gear { Neutral, Drive, Reverse };
Gear currentGear = Neutral; // Default to Neutral
bool shiftAttempted = false;


//BSC run modes
#define BSC6_BUCK 0
#define BSC6_BOOST 1

bool HasPrecharged = 0; //Safe when vehicle has precharged  
bool NLG_Charged = 0; //Safe when vehicle has carged 

bool CanError = false;

//driving variables
//Storage for Pedal Position
uint8_t sampleSetCounter = 0;
int16_t sampleSetPedal[5] = {0,0,0,0,0};


mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
uint8_t VehicleMode = Standby;  // Set default vehicle Mode to standby
uint8_t WakeupReason = 0;       // Set default Wakeup reason to 0
//*********************************************************************//
//Deffining Variables for Can transmission
//BMS
//*********************************************************************//
//Should be set by BMS
uint8_t  BMS_SOC = 0;
uint16_t BMS_U_BAT = 0;
int16_t  BMS_I_BAT = 0;  
uint16_t BMS_MAX_Discharge = 0;
uint8_t  BMS_MAX_Charge = 0;


//*********************************************************************//
//Deffining Variables for Can transmission
//NLG
//*********************************************************************//

//Sending Variables
//Variables for 0x711
bool NLG_C_ClrError = 0;         //Clear error latch
bool NLG_C_UnlockConRq = 1;      //Unlock connector request
bool NLG_C_VentiRq = 0;
int  NLG_DcHvVoltLimMax = MAX_U_BAT;   //Maximum HV voltage

int NLG_DcHvCurrLimMax = MAX_NLG_CURRENT;
uint8_t NLG_StateDem = 0;       //Setting State demand: 0 = Standby, 1 = Charge, 6 = Sleep
uint16_t NLG_LedDem = 0;        //Charge LED demanded See table 
uint16_t NLG_AcCurrLimMax = 32;  //Maximum AC current

bool NLG_C_EnPhaseShift = 0;
uint16_t NLG_AcPhaseShift = 0;

uint16_t NLG_AcPhaseShift_Scale = 0;
uint16_t NLG_AcCurrLimMax_Scale = 0;
int NLG_DcHvCurrLimMax_Scale = 0;
int NLG_DcHvVoltLimMax_Scale = 0;

//recive Variable
//Variables for 0x709
uint16_t NLG_AcWhAct = 0; //Actual AC kWh
uint16_t NLG_DcHvWhAct = 0; //Actual DC kWh
bool unlockPersist = false;

uint16_t NLG_MaxTempAct = 0;  //Maximum actual internal temperature
int16_t NLG_TempCon = 0;  //Actual temperatur connector
float NLG_DcHvAhAct = 0;  //Actual DC Ah
//Variables for 0x728
uint8_t NLG_StateCtrlPilot = 0; //State ControlPilot (see IEC 61851)
uint16_t NLG_DcHvVoltAct = 0; //Actual HV battery output voltage
uint8_t NLG_StateAct = 0; //0 Sleep, 1 Wakeup, 2 Standby, 3 Ready2Charge, 4 Charge, 5 Shtdown
bool NLG_S_DcHvCurrLim = 0; 
bool NLG_S_DcHvVoltLim = 0;
int16_t NLG_AcPhaseUsdNLG_S_DcHvVoltLim = 0;
bool NLG_S_ProximityLim = 0;
bool NLG_S_CtrlPilotLim = 0;
bool NLG_S_ConTempLim = 0;
bool NLG_S_IntTempLim = 0;
bool NLG_S_AcCurrLim = 0;
int16_t NLG_AcCurrMaxAct = 0;
uint16_t NLG_AcCurrHwAvl = 0;
bool NLG_S_ProximityDet = 0;
bool NLG_S_CtrlPilotDet = 0;
bool NLG_S_ConLocked = 0;
bool NLG_S_AcDet = 0;
bool NLG_S_HwWakeup = 0;
bool NLG_S_HwEnable = 0;
bool NLG_S_Err = 0;
bool NLG_S_War = 0;
uint16_t NLG_DcHvCurrAct = 0; //Actual HV battery output current
//Variables for 0x739
uint8_t NLG_ACT_PLUGCom = 0;
uint8_t NLG_StatusCP = 0;
bool NLG_S_CP_X1 = 0;
bool NLG_S_CP_SCC = 0;
uint16_t NLG_AcCurrMaxCP = 0;
uint8_t NLG_StatusPP = 0;
uint8_t NLG_AcCurrMaxPP = 0;
bool NLG_S_AcVoltDerating = 0;
bool NLG_S_AcDeratingNoisy = 0;
uint8_t NLG_AcPhaseUsd = 0;
uint8_t NLG_AcPhaseDet = 0;
uint8_t NLG_CoolingRequest = 0;
int16_t NLG_TempCoolPlate = 0;


bool conUlockInterrupt = 0; //Interrupt for connector unlock
//*********************************************************************//
//Deffining Variables for Can transmission
//DMC
//*********************************************************************//

// Add this at the global scope (outside any function)
enum DrivingMode {
    LEGACY,
    REGEN,
    OPD
};

// Add this as a global variable
DrivingMode currentDrivingMode = REGEN; // Default to LEGACY mode

#define DMC_MAXTRQ 850 //kinda should be372 but nice try
#define DMC_MAXREQTRQ 850
#define MAX_REVERSE_TRQ 220
// Variable to control OPD mode at runtime
bool isOPDEnabled = false;  // Set to true for OPD, false for original formula
bool isRegenEnabled = true; // Set true for newer regen(1/4 throttel strat)

uint16_t speed = 0;
#define NORMAL_RATIO 1.2
#define REDUCED_RATIO 2.1
#define DIFF_RATIO 3.9
#define WHEEL_CIRC 2.08
#define LowRange 0
//**********************//
//Sending Variables
//Variables for 0x210
//**********************//

bool enableDMC = 0;
bool modeDMC = 0;
bool oscLim = 1;
bool negTrqSpd = 1;
bool posTrqSpd = 1;
bool errLatch = 0;

int errorCnt = 0;

int DMC_SpdRq	 = 6000;  // Desired DMC_SpdRq	 in rpm
int DMC_TrqRq_Scale = 0;
int raw = 0;
unsigned char lowNibSpd = 0;
unsigned char highNibSpd = 0;
unsigned char lowNibTrq = 0;
unsigned char highNibTrq = 0;

//**********************//
//Sending Variables
//Variables for 0x211
//**********************//

int DMC_DcVLimMot = MIN_U_BAT; //Setting battery low voltage limit
int DMC_DcVLimGen = MAX_U_BAT; //Setting battery high voltage limit
int DMC_DcCLimMot = 600;      //Setting driving Current limit
int DMC_DcCLimGen = 420;      //Setting regen Current limit

int DMC_DcVLimMot_Scale = 0;
int DMC_DcVLimGen_Scale = 0;
int DMC_DcCLimMot_Scale = 0;
int DMC_DcCLimGen_Scale = 0;

//**********************//
//Sending Variables
//Variables for 0x212
//**********************//

int DMC_TrqSlewrate = 150;
int DMC_SpdSlewrate = 655;
int DMC_MechPwrMaxMot = 20000;
int DMC_MechPwrMaxGen = 20000;

int DMC_TrqSlewrate_Scale = 0;
int DMC_MechPwrMaxMot_Scale = 0;
int DMC_MechPwrMaxGen_Scale = 0;

//**********************//
//recive Variable
//Variables for 0x458
//**********************//

float DMC_TempInv = 0;
float DMC_TempMot = 0;
int8_t DMC_TempSys = 0;

//**********************//
//recive Variable
//Variables for 0x258
//**********************//

bool DMC_Ready = 0;
bool DMC_Running = 0;

bool DMC_SensorWarning	= 0;
bool DMC_GenErr = 0;
bool DMC_TrqLimitation = 0;

float DMC_TrqAvl = 0;
float DMC_TrqAct = 0;
float DMC_SpdAct = 0;

//**********************//
//recive Variable
//Variables for 0x259
//**********************//

float DMC_DcVltAct = 0;
float DMC_DcCurrAct = 0;
float DMC_AcCurrAct = 0;
int32_t DMC_MechPwr = 0;

//*********************************************************************//
//Deffining Variables for Can transmission
//BSC
//*********************************************************************//

//**********************//
//Sending Variables
//Variables for 0x260
//**********************//

bool enableBSC = 0;
bool modeBSC = 0;     //Buck mode = 0, Boost mode = 1
int Hvoltage = MAX_U_BAT;  
int Lvoltage = 14;
int LvoltageScale = 0;
int HvoltageScale = 0;

//**********************//
//Sending Variables
//Variables for 0x261
//**********************//

int BSC6_HVVOL_LOWLIM = MIN_U_BAT;
int BSC6_HVVOL_LOWLIM_SCALED = 0;
int BSC6_LVCUR_UPLIM_BUCK = 100;
int BSC6_HVCUR_UPLIM_BUCK = 20;
int BSC6_HVCUR_UPLIM_BUCK_SCALED = 0;
int BSC6_LVVOL_LOWLIM = 9;
int BSC6_LVVOL_LOWLIM_SCALED = 0;
int BSC6_LVCUR_UPLIM_BOOST = 100;
int BSC6_HVCUR_UPLIM_BOOST = PRECHARGE_CURRENT;
int BSC6_HVCUR_UPLIM_BOOST_SCALED = 0;

//**********************//
//Reciveing Variables
//Variables For 0x26A	
//**********************//

float BSC6_HVVOL_ACT = 0;
float BSC6_LVVOLT_ACT = 0;
float BSC6_HVCUR_ACT = 0;
float BSC6_LVCUR_ACT = 0;
unsigned char BSC6_MODE = 16;

uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr
uint8_t  len;

byte readDataBSC[MAX_DATA_SIZE] = {0}; //Storage for recived data BSC
byte readDataNLG[MAX_DATA_SIZE] = {0}; //Storage for recived data NLG
byte readDataBMS[MAX_DATA_SIZE] = {0}; //Storage for recived data NLG
unsigned char controllBufferDMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char controllBuffer2DMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char limitBufferDMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char controllBufferBSC[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //Storage for controll mesages
unsigned char limitBufferBSC[8] = {0, 0, 0, 0, 0, 0, 0, 0};    //Stroage for limit Values
unsigned char controllBufferNLG1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char controllRelayBuffer[8] = {0xFF, 0, 0, 0, 0, 0, 0, 0};

//*********************************************************************//
//Timing Variables
//CAN
//*********************************************************************//

unsigned long time10mscycle = 0;
unsigned long time50mscycle = 0;
unsigned long time100mscycle = 0;
uint16_t delay10ms = 10;
uint16_t delay50ms = 50;
uint16_t delay100ms = 100;
//Interrupt rutine for connector unlock
void IRAM_ATTR unlockCON() {
  if(NLG_S_ConLocked){
    VehicleMode = Charging;
    conUlockInterrupt = 1;
  }
  else{
    Serial.println("Standby 423");
    VehicleMode = Standby;
  }
}
void setup() {
  
  pinMode(IGNITION, INPUT); //define ignition pin as input
  pinMode(NLG_HW_Wakeup, INPUT); //define NLG_HW_Wakeup pin as input
  pinMode(UNLCKCON, INPUT); //define UNLCKCON pin as input
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);  //set up sleep interrupt rutine for ignition and NLG_HW_Wakeup and unlockcon
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);
  pinMode(18, INPUT);
  
  
  

  
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    CAN_COM,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(100); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    BACKBONE,   /* Task function. */
                    "Task2",     /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(100); 
}

//*********************************************************************//
//Core Lock 0
//CAN COM
//*********************************************************************//
void CAN_COM( void * pvParameters ){
      Serial.begin(115200);

  customSPI = new SPIClass(HSPI);
  customSPI -> begin(SCK, MISO, MOSI, SPI_CS_PIN);
  CAN.setSPI(customSPI);
  //SPI.begin(SCK, MISO, MOSI, SPI_CS_PIN);

  time10mscycle = millis();
  time50mscycle = millis();
  time100mscycle = millis();
  Serial.println("HelloVCU");
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
      Serial.println("CAN BUS Shield init fail");
      delay(100);
  }
  Serial.println("Can init OK");
  CanError = false;

  //CAN.begin(CAN_500KBPS);
  //Init the i2c bus
  Wire.begin(1,2);
  //CAN.begin(CAN_500KBPS);
  ADS.begin();
  ADS.setGain(2);
  initADAC(0b1001000, 1, 1);
  setADCpin(0);

  for(;;){
    esp_task_wdt_init(5, true);
    reciveBMS();
    switch(VehicleMode){
      case Standby:

      break;
      case Run:
        
        //BMS DMC max current
        sampleSetPedal[0] = ADS.readADC(GASPEDAL1);  //Read ADC into sampleSet
        sampleSetPedal[1] = ADS.readADC(GASPEDAL1);  //Read ADC into sampleSet
        sampleSetPedal[2] = ADS.readADC(GASPEDAL1);  //Read ADC into sampleSet
        sampleSetPedal[3] = ADS.readADC(GASPEDAL1);  //Read ADC into sampleSet
    
      /*
        if(BMS_MAX_Discharge < MAX_DMC_CURRENT){
          DMC_DcCLimMot = BMS_MAX_Discharge;
        }
        else{
         DMC_DcCLimMot = MAX_DMC_CURRENT;

        }
        */
        DMC_DcCLimMot = MAX_DMC_CURRENT;
      

        //polling CAN msgs
        reciveBMS();
        reciveBSC();
        reciveDMC(); 

        //Low latency cycle
        if(millis()>(time10mscycle + delay10ms)){
          time10mscycle = millis();
          sendDMC();
        }
        //Mid latency cycle
        if(millis()>(time50mscycle + delay50ms)){
          time50mscycle = millis();
          sendBSC();
          updateGearState(); // Add this line to call gear state update
        }
        //slow cycle for LUGA
        if(millis()>(time100mscycle + delay100ms)){
          time100mscycle = millis();
         
        }
        if(HasPrecharged){
            DMC_TrqRq_Scale = calculateTorque5S();
        }
        //Serial.println(ADS.readADC(GASPEDAL1));
        //Serial.println(DMC_TrqRq_Scale);
        /*
        Serial.print("SOC");
        Serial.println(BMS_SOC);
        Serial.print("U_BAT");
        Serial.println(BMS_U_BAT);
        Serial.print("I_BAT");
        Serial.println(BMS_I_BAT);
        Serial.print("MAX_Discharge");
        Serial.println(BMS_MAX_Discharge);
        Serial.print("MAX_Charge");
        Serial.println(BMS_MAX_Charge);
        */
        
      break;

      case Charging:
        //BMS DMC max current 
        /*
        Serial.print("SOC");
        Serial.println(BMS_SOC);
        Serial.print("U_BAT");
        Serial.println(BMS_U_BAT);
        Serial.print("I_BAT");
        Serial.println(BMS_I_BAT);
        Serial.print("MAX_Discharge");
        Serial.println(BMS_MAX_Discharge);
        Serial.print("MAX_Charge");
        Serial.println(BMS_MAX_Charge);
        
        if(BMS_MAX_Charge < MAX_NLG_CURRENT){
          NLG_DcHvCurrLimMax = BMS_MAX_Charge;
        }
        else{
          NLG_DcHvCurrLimMax = MAX_NLG_CURRENT;
        }
      */
     reciveBMS();
     NLG_DcHvCurrLimMax = BMS_MAX_Charge;
        if(millis()>(time10mscycle + delay10ms)){
          time10mscycle = millis();
          sendNLG();
        }
        //Mid latency cycle
        if(millis()>(time50mscycle + delay50ms)){
          time50mscycle = millis();
          sendBSC();
        }
        
        //polling CAN msgs
        reciveBSC();
        reciveNLG();
      break;
      default:
        Serial.println("Standby 592");
        VehicleMode = Standby;
      break;
    }

  } 
}
//*********************************************************************//
//Core Lock 1
//BACK BONE
//*********************************************************************//
void BACKBONE( void * pvParameters ){

  attachInterrupt(digitalPinToInterrupt(UNLCKCON), unlockCON, RISING);  //Interrupt for connector unlock
  //Set up Relais pins as output
  pinMode(CONTACTOR, OUTPUT);
  pinMode(PUMP, OUTPUT); 
  pinMode(PW1, OUTPUT);
  pinMode(NLGKL15, OUTPUT);
  pinMode(BCKLIGHT, OUTPUT);
  pinMode(DMCKL15, OUTPUT);
  pinMode(BSCKL15, OUTPUT);
  pinMode(LWP5, OUTPUT);
  pinMode(LWP6, OUTPUT);
  pinMode(LWP7, OUTPUT);


  //Read out the wake up reason
  WakeupReason = print_GPIO_wake_up();

  switch(WakeupReason){
    case NLG_HW_Wakeup:
      //NLG demands wakeup because type 2 charger detected
      VehicleMode = Charging;
      delay(1000);
      NLG_C_UnlockConRq = 0;
      digitalWrite(NLGKL15, HIGH);
    break;
    case IGNITION:
      //KL15 detected vehicle is in run mode
      VehicleMode = Run; 
    break;
    case UNLCKCON:
      //Connector unlock detected
      if(NLG_S_ConLocked){
        VehicleMode = Charging;
        conUlockInterrupt = 1;
        NLG_Charged =1;
        
      }
      else{
        VehicleMode = Standby;
      }
    break;
    default:
      Serial.println("Standby 645");
      VehicleMode = Standby;
    break;
  }

  for(;;){
    //reset watchdog timer its a rough gestimation
    esp_task_wdt_init(5, true);
    //Serial.println("Wake");
    switch(VehicleMode){
      case Standby:
        //No input detectet from ext sources
        digitalWrite(DMCKL15, LOW);  //DMC KL15
        digitalWrite(BSCKL15, LOW);  //BSC KL15
        digitalWrite(NLGKL15, LOW);  //NLG KL15
        Serial.println("Standby");
        NLG_Charged = 0;
        enableBSC = 0;
        enableDMC = 0;
        armBattery(0);
        armColingSys(0);
        if(digitalRead(NLG_HW_Wakeup)){VehicleMode = Charging;}
        if(digitalRead(IGNITION)){VehicleMode = Run;}
        if((!digitalRead(IGNITION)) && (!digitalRead(NLG_HW_Wakeup))){
          Serial.println("enteringSleep");
          esp_deep_sleep_start();
        }

      break;
      
  case Run:
  //Serial.println("Running");
    NLG_Charged = 0;
    //Serial.print("Torque Demand: ");
    //Serial.println(DMC_SpdAct);
    //Serial.println( ADS.readADC(GASPEDAL1));
    if (!digitalRead(IGNITION)) {
        Serial.println("Standby 680");
        VehicleMode = Standby;
    }

    armColingSys(1);
    armBattery(1);

    digitalWrite(DMCKL15, HIGH);  // DMC KL15
    digitalWrite(BSCKL15, HIGH);  // BSC KL15

    // Reverse light control based on gear state
    if (currentGear == Reverse) {
        digitalWrite(BCKLIGHT, HIGH);  // Turn on reverse light
    } else {
        digitalWrite(BCKLIGHT, LOW);   // Turn off reverse light
    }
    break;


    case Charging:
      digitalWrite(NLGKL15, HIGH);  // NLG KL15
      digitalWrite(DMCKL15, HIGH);  // BSC KL15
        //Check if Con  unlock interrupt is set
        if(conUlockInterrupt){
          if(NLG_S_ConLocked){
            NLG_C_UnlockConRq = 1;
            NLG_Charged = 1;
          }
          else{
            conUlockInterrupt = 0;
            NLG_StateDem = NLG_DEM_STANDBY;
            Serial.println("Standby 710");
            VehicleMode = Standby;
          }
        }
        else{
        //Serial.println("Charging");
        digitalWrite(NLGKL15, HIGH);
        armColingSys(1);
        
        chargeManage();
        }
      break;
      default:
        Serial.println("Standby 722");
        VehicleMode = Standby;
      break;
    } 
  }
}


//*********************************************************************//
//Functions for gen Operation
//Generic
//*********************************************************************//


//**********************//
//Charging management
//**********************//
void chargeManage() {
    static unsigned long unlockTimeout = 0;  // Timeout for unlocking
    const unsigned long unlockTimeoutDuration = 3000;  // 3 seconds timeout

    if (VehicleMode == Charging) {
        switch (NLG_StateAct) {
            case NLG_ACT_SLEEP:
                NLG_LedDem = 0;  // LED OFF
                NLG_StateDem = NLG_DEM_STANDBY;  // Demand Standby
                break;

            case NLG_ACT_STANDBY:
                NLG_LedDem = 1;  // LED red pulsing
                if (NLG_Charged) {
                    NLG_StateDem = NLG_DEM_SLEEP;
                    Serial.println("Standby 753");
                    VehicleMode = Standby;
                    NLG_C_UnlockConRq = 1;  // Initiate unlock request
                    unlockPersist = true;   // Set persistent unlock
                    unlockTimeout = millis();  // Start timeout timer
                } else {
                    
                }

                // Transition to standby immediately if the connector is disconnected

                /*
                if (!NLG_S_ConLocked) {
                    Serial.println("Standby 763");
                    VehicleMode = Standby;
                    NLG_LedDem = 0;  // LED OFF
                }
                */
                break;

            case NLG_ACT_READY2CHARGE:
                NLG_LedDem = 3;  // LED pulsating green
                if (unlockPersist || NLG_C_UnlockConRq) {
                    NLG_StateDem = NLG_DEM_STANDBY;
                } else {
                    if(HasPrecharged){
                    NLG_StateDem = NLG_DEM_CHARGE;  // Demand Charge
                    }
                    Serial.println("Precharging");
                    armBattery(1);
                }
                break;

            case NLG_ACT_CHARGE:
                NLG_LedDem = 4;  // LED green
                //NLG_Charged = 1;
                armBattery(1);
                break;

            default:
                armBattery(0);
                break;
        }

        // Persistent Unlock Connector Logic
        if (unlockPersist) {
            NLG_C_UnlockConRq = 1;  // Keep unlock request active
            if (!NLG_S_ConLocked) {  // If successfully unlocked
                NLG_StateDem = NLG_DEM_STANDBY;  // Set Standby mode
                Serial.println("Standby 797");
                VehicleMode = Standby;
                NLG_LedDem = 0;  // LED OFF to ensure no blinking continues
            } else if (millis() - unlockTimeout > unlockTimeoutDuration) {
                unlockTimeout = millis();  // Refresh timeout to maintain unlock persistently
            }
        }
    } else {
        armBattery(0);
        digitalWrite(NLGKL15, LOW);
    }
}

//**********************//
//Cooling sys manager
//**********************//

void armColingSys(bool arm){
  //switch PUMP on VCU for pump
  //switch relais3 on VCU for FAN
 
  //Serial.println("Cooling armed");
  if(batteryArmed){
    if(arm  && ((DMC_TempInv > 65)|(DMC_TempMot > 80)|(NLG_CoolingRequest > 50 ))){
      digitalWrite(LWP6, 1);
      digitalWrite(LWP7, 1);

    }
    else if(((DMC_TempInv < 40)&&(DMC_TempMot < 50)&&(NLG_CoolingRequest < 0 ))){
      digitalWrite(LWP6, 0);
      digitalWrite(LWP7, 0);

    }
 }
}


//**********************//
//Battery sys manager
//**********************//
// Add this global variable to keep track of the last mode change time
unsigned long lastModeChangeTime = 0;

void armBattery(bool arm) {
    // Check if precharge process should run
    if (arm) { 
      
        if (!HasPrecharged) {
            // Change to Boost mode with a delay if switching from Buck
            if (modeBSC != BSC6_BOOST) {
                modeBSC = BSC6_BOOST;
                lastModeChangeTime = millis();  // Record the time of mode change
                enableBSC = 0;  // Temporarily disable BSC during delay
                Serial.println("Switching to Boost mode...");
            }

            // Wait 2 seconds after switching modes before enabling BSC
            if (millis() - lastModeChangeTime >= 2000) {
                Hvoltage = BMS_U_BAT;       // Set high voltage command to BMS voltage
                enableBSC = 1;              // Enable BSC after delay
                //Serial.println("Boost mode enabled after delay.");
                
            }

            // Check if actual HV voltage is within ±20V of target voltage
            if ((BSC6_HVVOL_ACT >= (BMS_U_BAT - 20)) && (BSC6_HVVOL_ACT <= (BMS_U_BAT + 20)) && (BSC6_HVVOL_ACT > 50)) {
                HasPrecharged = 1;           // Mark precharge as complete
                digitalWrite(CONTACTOR, 1);    // Connect HV system
                digitalWrite(LWP5, 1);
                enableBSC = 0;               // Disable BSC after precharging
                Serial.println("Precharging complete. HV system connected.");
            } else {
                Serial.println(BSC6_HVVOL_ACT);
                Serial.println(BMS_U_BAT);
                //Serial.println("Precharging in progress...");
            }
        } else if (HasPrecharged) {
          batteryArmed = 1;
            errLatch = 1;
            delay(100);
            errLatch = 0;

            // Change to Buck mode with a delay if switching from Boost
            if (modeBSC != BSC6_BUCK) {
                modeBSC = BSC6_BUCK;
                lastModeChangeTime = millis();  // Record the time of mode change
                enableBSC = 0;  // Temporarily disable BSC during delay
                //Serial.println("Switching to Buck mode...");
            }

            // Wait 2 seconds after switching modes before enabling BSC
            if (millis() - lastModeChangeTime >= 1000) {
                //enableDMC = 1;          // Enable DMC for normal operation
                enableBSC = 1;          // Enable BSC after delay
                //Serial.println("Buck mode enabled after delay.");
            }
        }
    } else {
        // Disable battery and reset precharge if arm is false or BMS voltage too low
        batteryArmed = 0;
        enableBSC = 0;
        HasPrecharged = 0;
        digitalWrite(CONTACTOR, 0);            // Disconnect HV system
        digitalWrite(LWP5, 0);            // Disconnect HV system
        //Serial.println("Battery disconnected or BMS voltage too low.");
    }
}

//**********************//
//Throttle managment
//**********************//
int16_t calculateTorque5S() {
    int32_t SampledPotiValue = 0;
    int16_t DMC_TorqueCalc = 0;
    static float lastTorque = 0;
    
    Serial.println("--- Debug Info ---");
    if(LowRange){
      speed = DMC_SpdAct*60/REDUCED_RATIO/DIFF_RATIO*WHEEL_CIRC;
    }
    else{
      speed = DMC_SpdAct*60/NORMAL_RATIO/DIFF_RATIO*WHEEL_CIRC;
    }
    // Throttle Sampling and Averaging
    for (int i = 0; i < 4; i++) {
        SampledPotiValue += sampleSetPedal[i];
    }
    SampledPotiValue /= 4;

    float rawThrottle = map(SampledPotiValue, MinValPot, MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    Serial.print("Raw Throttle (%): "); Serial.println(rawThrottle);
    
    // Quick exit for legacy mode
    if (!isOPDEnabled && !isRegenEnabled && rawThrottle < 1.0f) {
        lastTorque = 0;
        enableDMC = 0;
        Serial.println("Mode: Legacy Pedal Released");
        Serial.println("Final Torque: 0");
        Serial.println("DMC Enabled: 0");
        Serial.println("---------------");
        return 0;
    }
    
    float throttlePosition = pow(rawThrottle / 100.0f, 1.5f) * 100.0f;

    if (currentGear == Neutral) {
        lastTorque = 0;
        Serial.println("Mode: Neutral");
        Serial.println("Final Torque: 0");
        Serial.println("---------------");
        return 0;
    }

    float rawSpeed = DMC_SpdAct;  // This is in RPM
    
    Serial.print("Raw Motor Speed (RPM): "); Serial.println(rawSpeed);
    Serial.print("Throttle Position: "); Serial.println(throttlePosition);

    // Determine driving mode
    enum DrivingMode {
        LEGACY,
        REGEN,
        OPD
    };

    DrivingMode currentMode;
    if (!isOPDEnabled && !isRegenEnabled) {
        currentMode = REGEN;
    } else if (isRegenEnabled) {  // You'll need to add this flag
        currentMode = REGEN;
    } else {
        currentMode = OPD;
    }

    // Common parameters
    float tau_rm = DMC_MAXREQTRQ * 0.7f;
    float tau_am = DMC_MAXTRQ * 0.9f;
    float gamma = 1.2f;
    const float REGEN_FADE_START = 400.0f;
    const float DIRECTION_PROTECT_SPEED = 0.1f;

    switch (currentMode) {
        case LEGACY: {
            float normalizedThrottle = pow(rawThrottle / 100.0f, 1.5f);
            DMC_TorqueCalc = normalizedThrottle * (currentGear == Drive ? -DMC_MAXTRQ : MAX_REVERSE_TRQ);
            Serial.println("Mode: Legacy");
            break;
        }

case REGEN: {
    // Adjusted parameters
    Serial.println("regening");
    const float ZERO_SPEED_WINDOW = 0.5f;
    const float MIN_SPEED_FOR_REGEN = 100.0f;    // Minimum speed for regen
    const float HIGH_SPEED_THRESHOLD = 1000.0f;
    const float REGEN_END_POINT = 35.0f;       // End regen at this point
    const float COAST_END_POINT = 40.0f;       // End coast at this point
    static float smoothedRegenTorque = 0.0f;
    const float SMOOTHING_FACTOR = 0.1f;
    
    if (currentGear == Drive) {
        if (abs(rawSpeed) < ZERO_SPEED_WINDOW) {
            // At standstill, only respond to acceleration commands
            if (throttlePosition > COAST_END_POINT) {
                float accelFactor = (throttlePosition - COAST_END_POINT) / (100.0f - COAST_END_POINT);
                accelFactor = pow(accelFactor, gamma);
                DMC_TorqueCalc = -accelFactor * tau_am;
            } else {
                DMC_TorqueCalc = 0;
            }
            smoothedRegenTorque = 0.0f;
            Serial.println("Mode: Standstill");
        }
        else if (rawSpeed < -MIN_SPEED_FOR_REGEN) {  // Forward motion
            // Corrected throttle mapping zones:
            // 0-35%: Regen
            // 35-40%: Coast
            // 40-100%: Acceleration
            if (throttlePosition < REGEN_END_POINT) {
                // REGEN ZONE (0% to 35% throttle)
                float regenFactor = 1.0f - (throttlePosition / REGEN_END_POINT);
                regenFactor = pow(regenFactor, 1.8f);  // Gentle curve
                
                float speedFactor = 1.0f;
                if (abs(rawSpeed) > HIGH_SPEED_THRESHOLD) {
                    speedFactor = 1.2f;
                }
                
                float targetRegen = regenFactor * tau_rm * speedFactor;
                DMC_TorqueCalc = targetRegen;  // Positive for regen
                Serial.println("Mode: Regen");
            }
            else if (throttlePosition < COAST_END_POINT) {
                // COAST ZONE (35% to 40% throttle)
                DMC_TorqueCalc = 0;
                Serial.println("Mode: Coast");
            }
            else {
                // ACCELERATION ZONE (40% to 100% throttle)
                float accelFactor = (throttlePosition - COAST_END_POINT) / (100.0f - COAST_END_POINT);
                accelFactor = pow(accelFactor, gamma);
                DMC_TorqueCalc = -accelFactor * tau_am;  // Negative for forward acceleration
                Serial.println("Mode: Acceleration");
            }
        }
        else {  // Too slow or wrong direction
            if (throttlePosition > COAST_END_POINT) {
                DMC_TorqueCalc = -tau_am * 0.3f;
                Serial.println("Mode: Direction Correction");
            } else {
                DMC_TorqueCalc = 0;
            }
        }
    }
    // ... rest of the function remains the same ...
}
case OPD: {
    // Torque limits
    const float tau_rm = DMC_MAXREQTRQ * 0.7f;  // Maximum regenerative torque
    const float tau_am = DMC_MAXTRQ * 0.9f;     // Maximum acceleration torque
    const float tau_rm_reverse = MAX_REVERSE_TRQ * 0.7f;  // Maximum reverse regenerative torque
    const float tau_am_reverse = MAX_REVERSE_TRQ * 0.9f;  // Maximum reverse acceleration torque
    
    // Control parameters
    const float ZERO_SPEED_THRESHOLD = 0.5f;    // Speed threshold in kph
    const float REGEN_FADE_START = 15.0f;       // Speed where regen starts fading (kph)
    const float DIRECTION_PROTECT_SPEED = 0.1f;  // kph
    const float gamma = 1.5f;                    // Pedal progression factor
    const float MAX_SPEED = 120.0f;             // Maximum speed in kph

    // Enhanced debug output
    Serial.println("--- Enhanced Debug Info ---");
    Serial.print("Current Gear: ");
    Serial.println(currentGear == Drive ? "Drive" : (currentGear == Reverse ? "Reverse" : "Neutral"));
    Serial.print("Raw Throttle (%): "); Serial.println(throttlePosition);
    Serial.print("Motor Speed (RPM): "); Serial.println(DMC_SpdAct);
    Serial.print("Vehicle Speed (kph): "); Serial.println(speed);

    // Protection for initial enable
    static bool wasEnabled = false;
    if (!wasEnabled && enableDMC) {
        DMC_TorqueCalc = 0;
        wasEnabled = true;
        Serial.println("Mode: OPD Enable Protection");
        break;
    }
    wasEnabled = enableDMC;

    if (!enableDMC) {
        DMC_TorqueCalc = 0;
        Serial.println("Mode: OPD Disabled");
        break;
    }

    // Calculate speed-dependent coast position
    float speedPercent = constrain(abs(speed) / MAX_SPEED, 0.0f, 1.0f);
    float coastPosition = 20.0f + (speedPercent * 30.0f);
    float normalizedPosition = (throttlePosition - coastPosition) / coastPosition;

    Serial.print("Coast Position: "); Serial.println(coastPosition);
    Serial.print("Normalized Position: "); Serial.println(normalizedPosition);

    // Main gear-based control logic
    if (currentGear == Drive) {
        // ... [Previous Drive gear code remains unchanged]
    }
    else if (currentGear == Reverse) {
        Serial.println("Processing Reverse Gear Logic");
        
        if (abs(speed) < ZERO_SPEED_THRESHOLD) {
            // At standstill
            if (throttlePosition > 5.0f) {  // Basic deadzone
                float accelFactor = pow(throttlePosition / 100.0f, gamma);
                DMC_TorqueCalc = accelFactor * tau_am_reverse;  // Positive torque for reverse
                Serial.print("Standstill Reverse Torque: "); Serial.println(DMC_TorqueCalc);
            } else {
                DMC_TorqueCalc = 0;
                Serial.println("In deadzone - zero torque");
            }
        }
        else if (speed >= 0.0f) {  // Moving in reverse (positive speed)
            if (normalizedPosition > 0) {  // Acceleration zone
                if (throttlePosition > 5.0f) {
                    float accelFactor = pow(normalizedPosition, gamma);
                    // Speed-based torque reduction for better control
                    float speedBasedTorque = tau_am_reverse * (1.0f - (speedPercent * 0.3f));
                    DMC_TorqueCalc = accelFactor * speedBasedTorque;
                    Serial.print("Reverse Acceleration Torque: "); Serial.println(DMC_TorqueCalc);
                } else {
                    DMC_TorqueCalc = 0;
                }
            }
            else {  // Regenerative zone
                float regenFactor = -normalizedPosition;
                float speedFactor = 1.0f;
                
                if (speed < REGEN_FADE_START) {
                    speedFactor = speed / REGEN_FADE_START;
                    speedFactor = constrain(speedFactor, 0.0f, 1.0f);
                }
                
                DMC_TorqueCalc = -regenFactor * tau_rm_reverse * speedFactor;
                Serial.print("Reverse Regen Torque: "); Serial.println(DMC_TorqueCalc);
            }
        }
        else {  // Wrong direction protection (negative speed in Reverse)
            DMC_TorqueCalc = tau_am_reverse * 0.3f;
            Serial.println("Reverse Direction Correction");
        }
    }
    else {  // Neutral
        DMC_TorqueCalc = 0;
        Serial.println("Neutral - Zero Torque");
    }

    // Final torque validation
    Serial.print("Pre-validation Torque: "); Serial.println(DMC_TorqueCalc);
    
    // Additional safety checks
    if (abs(throttlePosition) < 2.0f) {
        DMC_TorqueCalc = 0;
        Serial.println("Zero throttle protection applied");
    }
    
    // Torque limiting
    if (currentGear == Reverse) {
        DMC_TorqueCalc = constrain(DMC_TorqueCalc, -tau_rm_reverse, tau_am_reverse);
    }

    Serial.print("Final Torque Output: "); Serial.println(DMC_TorqueCalc);
    Serial.println("---------------");
    
    break;
}
    }

    // Asymmetric torque rate limiting
    float torqueDiff = DMC_TorqueCalc - lastTorque;
    float maxAccelStep = 8.0f;
    float maxDecelStep = 25.0f;
    
    if (abs(DMC_TorqueCalc) > abs(lastTorque)) {
        if (torqueDiff > maxAccelStep) {
            DMC_TorqueCalc = lastTorque + maxAccelStep;
        } else if (torqueDiff < -maxAccelStep) {
            DMC_TorqueCalc = lastTorque - maxAccelStep;
        }
    } else {
        if (torqueDiff > maxDecelStep) {
            DMC_TorqueCalc = lastTorque + maxDecelStep;
        } else if (torqueDiff < -maxDecelStep) {
            DMC_TorqueCalc = lastTorque - maxDecelStep;
        }
    }
    
    lastTorque = DMC_TorqueCalc;

    // Deadband with hysteresis
    static bool wasInDeadband = false;
    if (wasInDeadband) {
        if (abs(DMC_TorqueCalc) > 25) {
            wasInDeadband = false;
            enableDMC = 1;
        } else {
            DMC_TorqueCalc = 0;
            enableDMC = 1;
        }
    } else {
        if (abs(DMC_TorqueCalc) < 18) {
            wasInDeadband = true;
            DMC_TorqueCalc = 0;
            enableDMC = 1;
        } else {
            enableDMC = 1;
        }
    }

    Serial.print("Final Torque: "); Serial.println(DMC_TorqueCalc);
    Serial.print("DMC Enabled: "); Serial.println(enableDMC);
    Serial.println("---------------");
    
    return DMC_TorqueCalc;
}
void updateGearState() {
    int forwardValue = ADS.readADC(2); // Drive switch on A2
    int reverseValue = ADS.readADC(3); // Reverse switch on A3

    bool isForwardHigh = forwardValue > 200;
    bool isReverseHigh = reverseValue > 200;

    // Shift only if RPM is below threshold
    if (DMC_SpdAct < 100) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = Drive;
            shiftAttempted = false;
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = Reverse;
            shiftAttempted = false;
        } else {
            currentGear = Neutral;
            shiftAttempted = false;
        }
    } else {
        // If RPM > 100 and a shift is requested, go to Neutral
        if ((isForwardHigh && currentGear == Reverse) || (isReverseHigh && currentGear == Drive)) {
            currentGear = Neutral;
            shiftAttempted = true;
        }
    }

    // Reactivate gear if returning to the same selection
    if (shiftAttempted && DMC_SpdAct < 100) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = Drive;
            shiftAttempted = false;
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = Reverse;
            shiftAttempted = false;
        }
    }

    // Set torque to 0 in Neutral
    if (currentGear == Neutral) {
        DMC_TrqRq_Scale = 0;
    }
  /*
    // Print the current gear
    switch (currentGear) {
        case Drive:
            Serial.println("Current Gear: Drive");
            break;
        case Neutral:
            Serial.println("Current Gear: Neutral");
            break;
        case Reverse:
            Serial.println("Current Gear: Reverse");
            break;
    }
    */
}




uint8_t print_GPIO_wake_up(){
  uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  return (log(GPIO_reason))/log(2);
}

//*********************************************************************//
//CAN  functions
//Generic
//*********************************************************************//

//**********************//
//BSC
//**********************//

void sendBSC() {
    // Scaling the signals as per DBC spec
    LvoltageScale = static_cast<uint8_t>(Lvoltage * 10);  // Scale to match 0.1 V/bit with offset 8V
    HvoltageScale = static_cast<uint8_t>((Hvoltage - 220));     // Scale to match 1 V/bit with offset 220V

    // Construct control buffer for BSC6COM (ID: 0x260)
    controllBufferBSC[0] = (enableBSC << 0) | (modeBSC << 1)|0x80;  // enableBSC and modeBSC control the run and mode states
    controllBufferBSC[1] = LvoltageScale;
    controllBufferBSC[2] = HvoltageScale;

    // Scaling limit signals for BSC6LIM (ID: 0x261)
    BSC6_HVVOL_LOWLIM_SCALED = BSC6_HVVOL_LOWLIM - 220;  // Scale to match 1 V/bit with offset 220V
    BSC6_HVCUR_UPLIM_BUCK_SCALED = static_cast<uint8_t>(BSC6_HVCUR_UPLIM_BUCK * 10);  // Scale to match 0.1 A/bit
    BSC6_LVVOL_LOWLIM_SCALED = static_cast<uint8_t>(BSC6_LVVOL_LOWLIM * 10);  // Scale to match 0.1 V/bit
    BSC6_HVCUR_UPLIM_BOOST_SCALED = static_cast<uint8_t>(BSC6_HVCUR_UPLIM_BOOST * 10);  // Scale to match 0.1 A/bit

    // Construct limit buffer for BSC6LIM (ID: 0x261)
    limitBufferBSC[0] = BSC6_HVVOL_LOWLIM_SCALED;
    limitBufferBSC[1] = BSC6_LVCUR_UPLIM_BUCK;
    limitBufferBSC[2] = BSC6_HVCUR_UPLIM_BUCK_SCALED;
    limitBufferBSC[3] = BSC6_LVVOL_LOWLIM_SCALED;
    limitBufferBSC[4] = BSC6_LVCUR_UPLIM_BOOST;
    limitBufferBSC[5] = BSC6_HVCUR_UPLIM_BOOST_SCALED;

    // Sending the CAN messages
    CAN.sendMsgBuf(0x260, 0, 3, controllBufferBSC);
    CAN.sendMsgBuf(0x261, 0, 6, limitBufferBSC);
}



//**********************//
//DMC
//**********************//

void sendDMC() {
    // Scaling the signals as per DBC spec
    DMC_TrqSlewrate_Scale = DMC_TrqSlewrate * 100;  // Scale to match 0.01 Nm/s/bit
    DMC_MechPwrMaxMot_Scale = DMC_MechPwrMaxMot / 4;  // Scale to match 4 W/bit
    DMC_MechPwrMaxGen_Scale = DMC_MechPwrMaxGen / 4;

    // Construct control buffer for DMC_CTRL2 (ID: 0x212)
    controllBuffer2DMC[0] = DMC_TrqSlewrate_Scale >> 8;
    controllBuffer2DMC[1] = DMC_TrqSlewrate_Scale & 0xFF;
    controllBuffer2DMC[2] = DMC_SpdSlewrate >> 8;
    controllBuffer2DMC[3] = DMC_SpdSlewrate & 0xFF;
    controllBuffer2DMC[4] = DMC_MechPwrMaxMot_Scale >> 8;
    controllBuffer2DMC[5] = DMC_MechPwrMaxMot_Scale & 0xFF;
    controllBuffer2DMC[6] = DMC_MechPwrMaxGen_Scale >> 8;
    controllBuffer2DMC[7] = DMC_MechPwrMaxGen_Scale & 0xFF;

    // Scaling voltage and current limits for DMC_LIM (ID: 0x211)
    DMC_DcVLimMot_Scale = DMC_DcVLimMot * 10;  // Scale to match 0.1 V/bit
    DMC_DcVLimGen_Scale = DMC_DcVLimGen * 10;
    DMC_DcCLimMot_Scale = DMC_DcCLimMot * 10;  // Scale to match 0.1 A/bit
    DMC_DcCLimGen_Scale = DMC_DcCLimGen * 10;

    // Construct limit buffer for DMC_LIM (ID: 0x211)
    limitBufferDMC[0] = DMC_DcVLimMot_Scale >> 8;
    limitBufferDMC[1] = DMC_DcVLimMot_Scale & 0xFF;
    limitBufferDMC[2] = DMC_DcVLimGen_Scale >> 8;
    limitBufferDMC[3] = DMC_DcVLimGen_Scale & 0xFF;
    limitBufferDMC[4] = DMC_DcCLimMot_Scale >> 8;
    limitBufferDMC[5] = DMC_DcCLimMot_Scale & 0xFF;
    limitBufferDMC[6] = DMC_DcCLimGen_Scale >> 8;
    limitBufferDMC[7] = DMC_DcCLimGen_Scale & 0xFF;

    // Control buffer for DMC_CTRL (ID: 0x210)
    lowNibSpd = DMC_SpdRq & 0xFF;
    highNibSpd = DMC_SpdRq >> 8;
    lowNibTrq = static_cast<int16_t>(DMC_TrqRq_Scale * 10) & 0xFF;  // Scale to match 0.01 Nm/bit
    highNibTrq = static_cast<int16_t>(DMC_TrqRq_Scale * 10) >> 8;

    controllBufferDMC[0] = (enableDMC << 7) | (modeDMC << 6) | (oscLim << 5) | (negTrqSpd << 1) | posTrqSpd;
    controllBufferDMC[2] = highNibSpd;
    controllBufferDMC[3] = lowNibSpd;
    controllBufferDMC[4] = highNibTrq;
    controllBufferDMC[5] = lowNibTrq;

    // Sending the CAN messages
    CAN.sendMsgBuf(DMCCTRL, 0, 8, controllBufferDMC);
    CAN.sendMsgBuf(DMCLIM, 0, 8, limitBufferDMC);
    CAN.sendMsgBuf(0x212, 0, 8, controllBuffer2DMC);
}


//**********************//
//NLG
//**********************//

void sendNLG(){
    // Scaling values as per CAN specifications
    NLG_DcHvVoltLimMax_Scale = static_cast<int>((NLG_DcHvVoltLimMax) * 10); // Based on 0.1 V/bit factor, offset 350V
    NLG_DcHvCurrLimMax_Scale = static_cast<int>((NLG_DcHvCurrLimMax + 102.4) * 10); // Based on 0.1 A/bit factor, offset -102.4A
    NLG_AcCurrLimMax_Scale = static_cast<int>((NLG_AcCurrLimMax + 102.4) * 10); // Based on 0.1 A/bit factor, offset -102.4A
    NLG_AcPhaseShift_Scale = static_cast<int>((NLG_AcPhaseShift) * 10); // Based on 0.1 degree/bit factor

    // Constructing the buffer based on bit positioning from the specification
    controllBufferNLG1[0] = (NLG_C_ClrError << 7) | (NLG_C_UnlockConRq << 6) | (NLG_C_VentiRq << 5) | ((NLG_DcHvVoltLimMax_Scale >> 8) & 0x1F);
    controllBufferNLG1[1] = NLG_DcHvVoltLimMax_Scale & 0xFF;
    controllBufferNLG1[2] = (NLG_StateDem << 5) | ((NLG_DcHvCurrLimMax_Scale >> 8) & 0x07);
    controllBufferNLG1[3] = NLG_DcHvCurrLimMax_Scale & 0xFF;
    controllBufferNLG1[4] = (NLG_LedDem << 4) | ((NLG_AcCurrLimMax_Scale >> 8) & 0x07);
    controllBufferNLG1[5] = NLG_AcCurrLimMax_Scale & 0xFF;
    controllBufferNLG1[6] = (NLG_C_EnPhaseShift << 4) | ((NLG_AcPhaseShift_Scale >> 8) & 0x07);
    controllBufferNLG1[7] = NLG_AcPhaseShift_Scale & 0xFF;

    // Sending the CAN message
    CAN.sendMsgBuf(NLG_DEM_LIM, 0, 8, controllBufferNLG1);   
}

//NLG_C_UnlockConRq
//*********************************************************************//
//CAN recive functions
//Generic
//*********************************************************************//

//**********************//
//INFO
//**********************//

void reciveINFO(){
  Serial.println("NotDone");

}


//**********************//
//BMS
//**********************//

void reciveBMS(){
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
      return;
  }
  // read data, len: data length, buf: data buf
  CAN.readMsgBuf(&len, readDataBMS);
  id = CAN.getCanId();
  type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);
  BMS_U_BAT = 370;
  if(id == 0x001){
    BMS_SOC = readDataBMS[0]/2;
    BMS_U_BAT = 370;//(readDataBMS[1] | (readDataBMS[2] << 8))/100;
    BMS_I_BAT = (readDataBMS[3] | (readDataBMS[4] << 8))/100;
    BMS_MAX_Discharge = (readDataBMS[5] | (readDataBMS[6] << 8))/100;
    BMS_MAX_Charge = readDataBMS[7]*2;
  }   
}


//**********************//
//BSC
//**********************//


void reciveBSC() {
    // Check if data is available
    if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return;
    }

    // Read data, len: data length, buf: data buf
    CAN.readMsgBuf(&len, readDataBSC);
    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);

    if (id == 0x26A) {
        BSC6_HVVOL_ACT = ((readDataBSC[0] << 8) | readDataBSC[1]) * 0.1; // Scale to match 0.1 V/bit
        BSC6_LVVOLT_ACT = readDataBSC[2] * 0.1;                          // Scale to match 0.1 V/bit
        BSC6_HVCUR_ACT = (((readDataBSC[3] << 8) | readDataBSC[4]) * 0.1) - 25; // Scale to match 0.1 A/bit with offset -25 A
        BSC6_LVCUR_ACT = ((readDataBSC[5] << 8) | readDataBSC[6]) - 280;  // Scale to match 1 A/bit with offset -280 A
        BSC6_MODE = readDataBSC[7] >> 4;                                  // Extract 4-bit mode information
    }
}

//**********************//
//DMC
//**********************//

void reciveDMC() {
    if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return;
    }

    CAN.readMsgBuf(&len, readDataBSC);
    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);
    
    // Move variable declaration outside switch
    int16_t rawSpeed = 0;

    switch (id) {
        case 0x258:
            DMC_Ready = readDataBSC[0] & 0x80;
            DMC_Running = readDataBSC[0] & 0x40;
            DMC_TrqAvl = ((readDataBSC[2] << 8) | readDataBSC[3]) * 0.01;
            DMC_TrqAct = ((readDataBSC[4] << 8) | readDataBSC[5]) * 0.01;
            // Fix for signed speed interpretation
            rawSpeed = (readDataBSC[6] << 8) | readDataBSC[7];
            DMC_SpdAct = static_cast<float>(rawSpeed);
            break;

        case 0x259:
            DMC_DcVltAct = ((readDataBSC[0] << 8) | readDataBSC[1]) * 0.1;
            DMC_DcCurrAct = ((readDataBSC[2] << 8) | readDataBSC[3]) * 0.1;
            DMC_AcCurrAct = ((readDataBSC[4] << 8) | readDataBSC[5]) * 0.25;
            DMC_MechPwr = ((readDataBSC[6] << 8) | readDataBSC[7]) * 16;
            break;

        case 0x458:
            DMC_TempInv = ((readDataBSC[0] << 8) | readDataBSC[1]) * 0.5;
            DMC_TempMot = ((readDataBSC[2] << 8) | readDataBSC[3]) * 0.5;
            DMC_TempSys = readDataBSC[4] - 50;
            break;

        default:
            break;
    }
}

//**********************//
//NLG
//**********************//
void reciveNLG() {
    if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return;
    }

    // Read data, len: data length, buf: data buf
    CAN.readMsgBuf(&len, readDataNLG);

    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);

    // Switch for NLG message IDs
    switch (id) {
        case NLG_ACT_LIM:
            NLG_StateCtrlPilot = readDataNLG[0] >> 5; // 3 bits starting from bit 0
            NLG_DcHvVoltAct = ((readDataNLG[0] & 0x1F) << 8) | readDataNLG[1]; // 13 bits starting from bit 3
            NLG_StateAct = readDataNLG[2] >> 5; // 3 bits starting from bit 16
            NLG_S_DcHvCurrLim = (readDataNLG[2] >> 4) & 0x01; // 1 bit at bit 19
            NLG_S_DcHvVoltLim = (readDataNLG[2] >> 3) & 0x01; // 1 bit at bit 20
            NLG_DcHvCurrAct = ((readDataNLG[2] & 0x07) << 8) | readDataNLG[3]; // 11 bits starting from bit 21
            NLG_S_ProximityLim = readDataNLG[4] >> 7; // 1 bit at bit 32
            NLG_S_CtrlPilotLim = (readDataNLG[4] >> 6) & 0x01; // 1 bit at bit 33
            NLG_S_ConTempLim = (readDataNLG[4] >> 5) & 0x01; // 1 bit at bit 34
            NLG_S_IntTempLim = (readDataNLG[4] >> 4) & 0x01; // 1 bit at bit 35
            NLG_S_AcCurrLim = (readDataNLG[4] >> 3) & 0x01; // 1 bit at bit 36
            NLG_AcCurrMaxAct = ((readDataNLG[4] & 0x07) << 8) | readDataNLG[5]; // 11 bits starting from bit 37
            NLG_AcCurrHwAvl = readDataNLG[6];
            NLG_S_ProximityDet = readDataNLG[7] >> 7; // 1 bit at bit 56
            NLG_S_CtrlPilotDet = (readDataNLG[7] >> 6) & 0x01; // 1 bit at bit 57
            NLG_S_ConLocked = (readDataNLG[7] >> 5) & 0x01; // 1 bit at bit 58
            NLG_S_AcDet = (readDataNLG[7] >> 4) & 0x01; // 1 bit at bit 59
            NLG_S_HwWakeup = (readDataNLG[7] >> 3) & 0x01; // 1 bit at bit 60
            NLG_S_HwEnable = (readDataNLG[7] >> 2) & 0x01; // 1 bit at bit 61
            NLG_S_Err = (readDataNLG[7] >> 1) & 0x01; // 1 bit at bit 62
            NLG_S_War = readDataNLG[7] & 0x01; // 1 bit at bit 63
            break;
        case NLG_ACT_PLUG:
            NLG_CoolingRequest = readDataNLG[4];
          break;
        // Add other cases if needed
    }
}



//MIMIMIMIMIMIMIMI
void loop() {}
