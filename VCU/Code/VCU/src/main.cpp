i have a problem this doesnt work in the precharge stage so the bsc doesnt ron i will send you the dbc and manual
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

//Relay pinout
#define RELAIS1 11  //HV Battery
#define RELAIS2 12  //Cooling Pump
#define RELAIS3 13  //HV Battery
#define RELAIS4 14  //Charger KL15
#define RELAIS5 17  //Reverse Signal
#define RELAIS6 18  //DMC KL15
#define RELAIS7 21  //BSC KL15
#define RELAIS8 33  //Not Used

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

#define MinValPot 18410
#define MaxValPot 13240

//Pin for reverse signal 0 = forward, 1 = reverse pin 0 on AD
#define REVERSE 0
//Battery Voltage values
#define MIN_U_BAT 360 //3.4V*104S
#define NOM_U_BAT 382 //3.67V*104S
#define MAX_U_BAT 436 //4.2V*104S

#define MAX_DMC_CURRENT 600 //A 
#define MAX_NLG_CURRENT 32 //A
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
uint16_t NLG_AcCurrLimMax = 0;  //Maximum AC current

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

#define DMC_MAXTRQ 1000 //kinda should be372 but nice try
#define MAX_REVERSE_TRQ 170
//**********************//
//Sending Variables
//Variables for 0x210
//**********************//

bool enableDMC = 0;
bool modeDMC = 1;
bool oscLim = 0;
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

int DMC_TrqSlewrate = 1300;
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
  customSPI = new SPIClass(HSPI);
  customSPI -> begin(SCK, MISO, MOSI, SPI_CS_PIN);
  CAN.setSPI(customSPI);
  //SPI.begin(SCK, MISO, MOSI, SPI_CS_PIN);

  time10mscycle = millis();
  time50mscycle = millis();
  time100mscycle = millis();

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
      Serial.println("CAN BUS Shield init fail");
      delay(100);
  }
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
        delay(10);
        //BMS DMC max current
        sampleSetPedal[0] = ADS.readADC(GASPEDAL1);  //Read ADC into sampleSet
        sampleSetPedal[1] = ADS.readADC(GASPEDAL1);  //Read ADC into sampleSet
        sampleSetPedal[2] = ADS.readADC(GASPEDAL1);  //Read ADC into sampleSet
        sampleSetPedal[3] = ADS.readADC(GASPEDAL1);  //Read ADC into sampleSet
    
        if((DMC_SpdAct < 100) && sampleSetPedal[1] < 200){
          DMC_DcCLimMot = 10;
        }
        else{
          if(BMS_MAX_Discharge < MAX_DMC_CURRENT){
            DMC_DcCLimMot = BMS_MAX_Discharge;
          }
          else{
            DMC_DcCLimMot = MAX_DMC_CURRENT;
          }
        }

        //polling CAN msgs
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
        
        DMC_TrqRq_Scale = calculateTorque5S();
        //Serial.println(ADS.readADC(GASPEDAL1));
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
        
        
      break;

      case Charging:
        //BMS DMC max current 
      
        if(BMS_MAX_Charge < MAX_NLG_CURRENT){
          NLG_DcHvCurrLimMax = BMS_MAX_Charge;
        }
        else{
          NLG_DcHvCurrLimMax = MAX_NLG_CURRENT;
        }
     
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
  pinMode(RELAIS1, OUTPUT);
  pinMode(RELAIS2, OUTPUT); 
  pinMode(RELAIS3, OUTPUT);
  pinMode(RELAIS4, OUTPUT);
  pinMode(RELAIS5, OUTPUT);
  pinMode(RELAIS6, OUTPUT);
  pinMode(RELAIS7, OUTPUT);
  pinMode(RELAIS8, OUTPUT);

  
  Serial.begin(115200);
  //Read out the wake up reason
  WakeupReason = print_GPIO_wake_up();

  switch(WakeupReason){
    case NLG_HW_Wakeup:
      //NLG demands wakeup because type 2 charger detected
      VehicleMode = Charging;
      delay(1000);
      NLG_C_UnlockConRq = 0;
      digitalWrite(RELAIS4, HIGH);
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
        digitalWrite(RELAIS6, LOW);  //DMC KL15
        digitalWrite(RELAIS7, LOW);  //BSC KL15
        digitalWrite(RELAIS4, LOW);  //NLG KL15
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
    NLG_Charged = 0;

    if (!digitalRead(IGNITION)) {
        VehicleMode = Standby;
    }

    armColingSys(1);
    armBattery(1);

    digitalWrite(RELAIS6, HIGH);  // DMC KL15
    digitalWrite(RELAIS7, HIGH);  // BSC KL15

    // Reverse light control based on gear state
    if (currentGear == Reverse) {
        digitalWrite(RELAIS5, HIGH);  // Turn on reverse light
    } else {
        digitalWrite(RELAIS5, LOW);   // Turn off reverse light
    }

    enableBSC = 1;

    if (DMC_Ready) {
        enableDMC = 1;
    } else {
        enableDMC = 0;
    }

    // Error handling if required
    if (errorCnt < 40 && (DMC_SensorWarning || DMC_GenErr)) {
        enableDMC = 0;
        errorCnt++;
        errLatch = 1;
    } else {
        errorCnt = 0;
        enableDMC = 1;
    }
    break;


      case Charging:
        
        //Check if Con  unlock interrupt is set
        if(conUlockInterrupt){
          if(NLG_S_ConLocked){
            NLG_C_UnlockConRq = 1;
            NLG_Charged = 1;
          }
          else{
            conUlockInterrupt = 0;
            NLG_StateDem = NLG_DEM_STANDBY;
            VehicleMode = Standby;
          }
        }
        else{
        //Serial.println("Charging");
        digitalWrite(RELAIS4, HIGH);
        armColingSys(1);
        enableBSC = 1;
        chargeManage();
        }
      break;
      default:
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
                    VehicleMode = Standby;
                    NLG_C_UnlockConRq = 1;  // Initiate unlock request
                    unlockPersist = true;   // Set persistent unlock
                    unlockTimeout = millis();  // Start timeout timer
                } else {
                    armBattery(1);
                }

                // Transition to standby immediately if the connector is disconnected
                if (!NLG_S_ConLocked) {
                    VehicleMode = Standby;
                    NLG_LedDem = 0;  // LED OFF
                }
                break;

            case NLG_ACT_READY2CHARGE:
                NLG_LedDem = 3;  // LED pulsating green
                if (unlockPersist || NLG_C_UnlockConRq) {
                    NLG_StateDem = NLG_DEM_STANDBY;
                } else {
                    NLG_StateDem = NLG_DEM_CHARGE;  // Demand Charge
                }
                break;

            case NLG_ACT_CHARGE:
                NLG_LedDem = 4;  // LED green
                NLG_Charged = 1;
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
                VehicleMode = Standby;
                NLG_LedDem = 0;  // LED OFF to ensure no blinking continues
            } else if (millis() - unlockTimeout > unlockTimeoutDuration) {
                unlockTimeout = millis();  // Refresh timeout to maintain unlock persistently
            }
        }
    } else {
        armBattery(0);
        digitalWrite(RELAIS4, LOW);
    }
}

//**********************//
//Cooling sys manager
//**********************//

void armColingSys(bool arm){
  //switch relais2 on VCU for pump
  //switch relais3 on VCU for FAN
 
  //Serial.println("Cooling armed");
  if(arm  && ((DMC_TempInv > 50)|(DMC_TempMot > 80)|(NLG_TempCoolPlate > 50 ))){
    digitalWrite(RELAIS2, 1);
  }
  else if(((DMC_TempInv < 40)&&(DMC_TempMot < 50)&&(NLG_TempCoolPlate < 40 ))){
    digitalWrite(RELAIS2, 0);
  }
}


//**********************//
//Battery sys manager
//**********************//

void armBattery(bool arm){
  //switch relais2 on VCU
  if(arm){
    if(!HasPrecharged){
      BSC6_MODE = BSC6_BOOST;
      Hvoltage = BMS_U_BAT;
      enableBSC = 1;
      Serial.println("Precharging");
      digitalWrite(RELAIS3, 0);
      if(((BSC6_HVVOL_ACT + 20) >= BMS_U_BAT)&& BSC6_HVVOL_ACT > 50) {
        HasPrecharged = 1;
        digitalWrite(RELAIS3, 1);
        enableBSC = 0;
      }
    }
    else if(HasPrecharged){
      BSC6_MODE = BSC6_BUCK;
      enableBSC = 1;
    }
  }
  else{
    enableBSC = 0;
    digitalWrite(RELAIS3, 0);
  }
}

//**********************//
//Throttle managment
//**********************//

int16_t calculateTorque5S() {
    int32_t SampledPotiValue = 0;
    int16_t DMC_TorqueCalc = 0;

    // Zero torque demand if in Neutral
    if (currentGear == Neutral) {
        return 0;
    }

    // Sum values in sampleSetPedal (assuming sampleSetPedal[0-3] contains valid data)
    for (int i = 0; i < 4; i++) {
        SampledPotiValue += sampleSetPedal[i];
    }

    // Calculate average and map the result
    SampledPotiValue /= 4;

    // Apply different mapping and limit based on gear
    if (currentGear == Drive) {
        SampledPotiValue = map(SampledPotiValue, MinValPot, MaxValPot, 0, DMC_MAXTRQ);
        DMC_TorqueCalc = static_cast<int16_t>(constrain(SampledPotiValue, 0, DMC_MAXTRQ));
    } 
    else if (currentGear == Reverse) {
        SampledPotiValue = map(SampledPotiValue, MinValPot, MaxValPot, 0, MAX_REVERSE_TRQ);
        DMC_TorqueCalc = -static_cast<int16_t>(constrain(SampledPotiValue, 0, MAX_REVERSE_TRQ));
    }

    // Apply deadband of ±14
    if (DMC_TorqueCalc > -14 && DMC_TorqueCalc < 14) {
        DMC_TorqueCalc = 0;
    }

    //Serial.print("Torque Demand: ");
    //Serial.println(DMC_TorqueCalc);
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
//CAN send functions
//Generic
//*********************************************************************//

//**********************//
//BSC
//**********************//

void sendBSC(){
  //Sending Can
  LvoltageScale = Lvoltage * 10;
  HvoltageScale = Hvoltage - 220; 

  controllBufferBSC[0] = modeBSC << 1 | enableBSC;
  controllBufferBSC[1] = LvoltageScale;
  controllBufferBSC[2] = HvoltageScale;

  BSC6_HVVOL_LOWLIM_SCALED = BSC6_HVVOL_LOWLIM - 220;
  BSC6_HVCUR_UPLIM_BUCK_SCALED = BSC6_HVCUR_UPLIM_BUCK * 10;
  BSC6_LVVOL_LOWLIM_SCALED = BSC6_LVVOL_LOWLIM * 10;
  BSC6_HVCUR_UPLIM_BOOST_SCALED = BSC6_HVCUR_UPLIM_BOOST * 10;

  limitBufferBSC[0] = BSC6_HVVOL_LOWLIM_SCALED;
  limitBufferBSC[1] = BSC6_LVCUR_UPLIM_BUCK;
  limitBufferBSC[2] = BSC6_HVCUR_UPLIM_BUCK_SCALED;
  limitBufferBSC[3] = BSC6_LVVOL_LOWLIM_SCALED;
  limitBufferBSC[4] = BSC6_LVCUR_UPLIM_BOOST;
  limitBufferBSC[5] = BSC6_HVCUR_UPLIM_BOOST_SCALED;

  CAN.sendMsgBuf(BSC_COMM, 0, 3, controllBufferBSC);
  CAN.sendMsgBuf(BSC_LIM, 0, 6, limitBufferBSC);
}


//**********************//
//DMC
//**********************//

void sendDMC(){
  DMC_TrqSlewrate_Scale = DMC_TrqSlewrate * 50;
  DMC_MechPwrMaxMot_Scale = DMC_MechPwrMaxMot / 4;
  DMC_MechPwrMaxGen_Scale = DMC_MechPwrMaxGen / 4;

  controllBuffer2DMC[0] = DMC_TrqSlewrate_Scale >> 8;
  controllBuffer2DMC[1] = DMC_TrqSlewrate_Scale & 0x00FF;
  controllBuffer2DMC[2] = DMC_SpdSlewrate >> 8;
  controllBuffer2DMC[3] = DMC_SpdSlewrate & 0x00FF;
  controllBuffer2DMC[4] = DMC_MechPwrMaxMot_Scale >> 8;
  controllBuffer2DMC[5] = DMC_MechPwrMaxMot_Scale & 0x00FF;
  controllBuffer2DMC[6] = DMC_MechPwrMaxGen_Scale >> 8;
  controllBuffer2DMC[7] = DMC_MechPwrMaxGen_Scale & 0x00FF;

  DMC_DcVLimMot_Scale = DMC_DcVLimMot * 10;
  DMC_DcVLimGen_Scale = DMC_DcVLimGen * 10;
  DMC_DcCLimMot_Scale = DMC_DcCLimMot * 10;
  DMC_DcCLimGen_Scale = DMC_DcCLimGen * 10;
  
  limitBufferDMC[0] = DMC_DcVLimMot_Scale >> 8;
  limitBufferDMC[1] = DMC_DcVLimMot_Scale & 0x00FF;
  limitBufferDMC[2] = DMC_DcVLimGen_Scale >> 8;
  limitBufferDMC[3] = DMC_DcVLimGen_Scale & 0x00FF;
  limitBufferDMC[4] = DMC_DcCLimMot_Scale >> 8;
  limitBufferDMC[5] = DMC_DcCLimMot_Scale & 0x00FF;
  limitBufferDMC[6] = DMC_DcCLimGen_Scale >> 8;
  limitBufferDMC[7] = DMC_DcCLimGen_Scale & 0x00FF;


  lowNibSpd = DMC_SpdRq	 & 0x00FF;
  highNibSpd = DMC_SpdRq	 >> 8;
  lowNibTrq = DMC_TrqRq_Scale & 0x00FF;
  highNibTrq = DMC_TrqRq_Scale >> 8;

  controllBufferDMC[0] = enableDMC << 7 | modeDMC << 6 | oscLim << 5 | negTrqSpd << 1 | posTrqSpd;
  controllBufferDMC[2] = highNibSpd;
  controllBufferDMC[3] = lowNibSpd;
  controllBufferDMC[4] = highNibTrq;
  controllBufferDMC[5] = lowNibTrq;

  CAN.sendMsgBuf(DMCCTRL, 0, 8, controllBufferDMC);
  CAN.sendMsgBuf(DMCLIM, 0, 8, limitBufferDMC);
}


//**********************//
//NLG
//**********************//

void sendNLG(){
  NLG_DcHvVoltLimMax_Scale = NLG_DcHvVoltLimMax * 10;
  NLG_DcHvCurrLimMax_Scale = NLG_DcHvCurrLimMax * 10;
  NLG_DcHvCurrLimMax_Scale -= 1024;
  NLG_AcCurrLimMax_Scale = NLG_AcCurrLimMax * 10;
  NLG_AcCurrLimMax_Scale -= 1024;
  NLG_AcPhaseShift_Scale = NLG_AcPhaseShift * 10;
  controllBufferNLG1[0] = (NLG_C_ClrError << 7) |  NLG_C_UnlockConRq << 6 | (NLG_C_VentiRq << 5) | ((NLG_DcHvVoltLimMax_Scale >> 8)&0x1F);
  controllBufferNLG1[1] = NLG_DcHvVoltLimMax_Scale & 0x00FF;
  controllBufferNLG1[2] = (NLG_StateDem << 5)  | (NLG_DcHvCurrLimMax_Scale >> 8)& 0x07;
  controllBufferNLG1[3] = NLG_DcHvCurrLimMax_Scale & 0x00FF;
  controllBufferNLG1[4] = NLG_LedDem << 4 | (NLG_AcCurrLimMax_Scale << 8) & 0x07;
  controllBufferNLG1[5] = NLG_AcCurrLimMax_Scale & 0x00FF;
  controllBufferNLG1[6] = NLG_C_EnPhaseShift << 4 | (NLG_AcPhaseShift_Scale >> 8) & 0x07;
  controllBufferNLG1[7] = NLG_AcPhaseShift_Scale & 0x00FF;
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
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
      return;
  }


  // read data, len: data length, buf: data buf
  CAN.readMsgBuf(&len, readDataBMS);
  Serial.println("reading");
  id = CAN.getCanId();
  type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);
  
  if(id == 0x553){
    BMS_SOC = readDataBMS[7];
    BMS_U_BAT = readDataBMS[6] | (readDataBMS[5] << 8);
    BMS_I_BAT = readDataBMS[4] | (readDataBMS[3] << 8);
    BMS_MAX_Discharge = readDataBMS[2] | (readDataBMS[1] << 8);
    BMS_MAX_Charge = readDataBMS[0];
  } 
  
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
  
  if(id == 0x001){
    BMS_SOC = readDataBMS[0];
    BMS_U_BAT = readDataBMS[1] | (readDataBMS[2] << 8);
    BMS_I_BAT = readDataBMS[3] | (readDataBMS[4] << 8);
    BMS_MAX_Discharge = readDataBMS[5] | (readDataBMS[6] << 8);
    BMS_MAX_Charge = readDataBMS[7];
  }   
}


//**********************//
//BSC
//**********************//

void reciveBSC(){
 //Reciveing Can
    // check if data coming
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
      return;
  }
  // read data, len: data length, buf: data buf
  CAN.readMsgBuf(&len, readDataBSC);

  id = CAN.getCanId();
  type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);
  /*
    * MCP2515(or this driver) could not handle properly
    * the data carried by remote frame
    */

  
  if(id == 0x26A){
    BSC6_HVVOL_ACT = readDataBSC[0] | (readDataBSC[1] << 8);
    BSC6_HVVOL_ACT = BSC6_HVVOL_ACT / 10;

    BSC6_LVVOLT_ACT = readDataBSC[2];
    BSC6_LVVOLT_ACT = BSC6_LVVOLT_ACT / 10;

    BSC6_HVCUR_ACT = readDataBSC[3];
    BSC6_HVCUR_ACT = BSC6_HVCUR_ACT / 10;
    BSC6_HVCUR_ACT = BSC6_HVCUR_ACT - 25;

    BSC6_LVCUR_ACT = readDataBSC[4] | (readDataBSC[5] << 8);
    BSC6_LVCUR_ACT = BSC6_LVCUR_ACT - 280;

    BSC6_MODE = readDataBSC[7] >> 4;

  }  
}


//**********************//
//DMC
//**********************//

void reciveDMC(){
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return;
  }
  CAN.readMsgBuf(&len, readDataBSC);

  id = CAN.getCanId();
  type = (CAN.isExtendedFrame() << 0) |
          (CAN.isRemoteRequest() << 1);
  
  
  
  switch (id){
    case 0x259:
      DMC_DcVltAct = readDataBSC[1] | (readDataBSC[0] << 8);
      DMC_DcVltAct = DMC_DcVltAct / 10;

      DMC_DcCurrAct = readDataBSC[3] | (readDataBSC[2] << 8);
      DMC_DcCurrAct = DMC_DcCurrAct / 10;

      DMC_AcCurrAct = readDataBSC[5] | (readDataBSC[4] << 8);
      DMC_AcCurrAct = DMC_AcCurrAct / 4;

      DMC_MechPwr = readDataBSC[7] | (readDataBSC[6] << 8);
      DMC_MechPwr = DMC_MechPwr * 16;

    break;
    case 0x258:
      DMC_Ready = readDataBSC[0] & 0x80;
      DMC_Running = readDataBSC[0] & 0x40;
      DMC_SensorWarning = readDataBSC[0] & 0x04;
      DMC_GenErr = readDataBSC[0] & 0x02;
      DMC_TrqLimitation = readDataBSC[0] & 0x01;

      DMC_TrqAvl = readDataBSC[3] | (readDataBSC[2] << 8);
      DMC_TrqAvl = DMC_TrqAvl / 100;

      DMC_TrqAct = readDataBSC[5] | (readDataBSC[4] << 8);
      DMC_TrqAct = DMC_TrqAct / 100;

      DMC_SpdAct = readDataBSC[7] | (readDataBSC[6] << 8);
      
    break;
    case 0x458:
      DMC_TempInv = readDataBSC[1] | (readDataBSC[0] << 8);
      DMC_TempInv = DMC_TempInv / 2;

      DMC_TempMot = readDataBSC[3] | (readDataBSC[2] << 8);
      DMC_TempMot = DMC_TempMot / 2;

      DMC_TempSys = readDataBSC[4];
      DMC_TempSys = DMC_TempSys -50;
      
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
            NLG_StateCtrlPilot = readDataNLG[0] >> 5;
            NLG_DcHvVoltAct = (readDataNLG[0] << 8) & 0x1F00;
            NLG_DcHvVoltAct |= readDataNLG[1];
            NLG_StateAct = readDataNLG[2] >> 5;
            NLG_S_DcHvCurrLim = readDataNLG[2] >> 4 & 0x01;
            NLG_S_DcHvVoltLim = readDataNLG[2] >> 3 & 0x01;
            NLG_DcHvCurrAct = (readDataNLG[2] << 8) & 0x07;
            NLG_DcHvCurrAct |= readDataNLG[3];
            NLG_S_ProximityLim = readDataNLG[4] >> 7;
            NLG_S_CtrlPilotLim = (readDataNLG[4] >> 6) & 0x01;
            NLG_S_ConTempLim = (readDataNLG[4] >> 5) & 0x01;
            NLG_S_IntTempLim = (readDataNLG[4] >> 4) & 0x01;
            NLG_S_AcCurrLim= (readDataNLG[4] >> 5) & 0x01;
            NLG_AcCurrMaxAct = (readDataNLG[4] << 8) & 0x07;
            NLG_AcCurrMaxAct |= readDataNLG[5];
            NLG_AcCurrHwAvl = readDataNLG[6];
            NLG_S_ProximityDet = readDataNLG[7] >> 7;
            NLG_S_CtrlPilotDet = (readDataNLG[7] >> 6) & 0x01;
            NLG_S_ConLocked = (readDataNLG[7] >> 5) & 0x01;
            NLG_S_AcDet = (readDataNLG[7] >> 4) & 0x01;
            NLG_S_HwWakeup = (readDataNLG[7] >> 3) & 0x01;
            NLG_S_HwEnable = (readDataNLG[7] >> 2) & 0x01;
            NLG_S_Err = (readDataNLG[7] >> 1) & 0x01;
            NLG_S_War = readDataNLG[7] & 0x01;
            break;

        case NLG_ACT_PLUG:
            NLG_StatusCP = (readDataNLG[0] >> 5) & 0x07;
            NLG_S_CP_X1 = (readDataNLG[0] >> 4) & 0x01;
            NLG_S_CP_SCC = (readDataNLG[0] >> 3) & 0x01;
            NLG_AcCurrMaxCP = (readDataNLG[0] << 8) & 0x03;
            NLG_AcCurrMaxCP |= readDataNLG[1];
            NLG_StatusPP = (readDataNLG[2] >> 4) & 0x07;
            NLG_AcCurrMaxPP = readDataNLG[2] & 0x0F;
            NLG_S_AcVoltDerating = readDataNLG[3] >> 7;
            NLG_S_AcDeratingNoisy = (readDataNLG[3] >> 6) & 0x01;
            NLG_AcPhaseUsd = (readDataNLG[3] >> 5) & 0x01;
            NLG_AcPhaseDet = (readDataNLG[3] >> 2) & 0x01;
            NLG_CoolingRequest = (readDataNLG[4] >> 6) & 0x01;
            NLG_ACT_PLUGCom = readDataNLG[4];
            break;

        case NLG_ACT_ERR:
            // Handle error conditions if needed
            break;
    }
}


//MIMIMIMIMIMIMIMI
void loop() {}
