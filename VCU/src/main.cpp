// Author: Christian Obrecht
// Date: 07.11.2023
// Description: Communication with BSC and DMC over CAN and Controll of ADAC


//Libraries
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include "mcp2515_can.h"

//Create Tasks for each Core
void CAN_COM (void * pvParameters);
void BACKBONE (void * pvParameters);

//Task Handles
TaskHandle_t Task1;
TaskHandle_t Task2;
//Test für PAPA

//Function Declarations
void sendBSC();
void sendDMC();
void reciveBSC();
void reciveDMC();
void setLCDBSC();
void setLCDDMC();
void reciveNLG();
void sendNLG();

void startBSC();
void chargeManage();

void armBattery(bool arm);
void armNLG(bool arm);
void armBSC(bool arm);
void armDMC(bool arm);

void initADAC();
void setVref(bool enable);
void setupDAC();
void setupADC();
void setupGPO();
void setupGPI();
void setDACVal(uint8_t DACnum, uint16_t DACvalue);
void setDACGain(bool gain);
void setLDAC(uint8_t LDAC);
uint16_t readADC(uint8_t DACnum);

int16_t calculateTorque5S(bool reverseSig);


//Pinout
#define SCK 4
#define MOSI 6
#define MISO 5
const int SPI_CS_PIN = 36;
const int CAN_INT_PIN = 11;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#define MAX_DATA_SIZE 8

//Defining CAN Indexes
#define CAN_2515


#define BSC_COMM 0x260
#define BSC_LIM 0x261

#define DMCCTRL 0x210
#define DMCLIM 0x211
#define DMCCTRL2 0x212

#define NLG_DEM_LIM 0x711
#define NLG_ACT_ERR 0x799
#define NLG_ACT_LIM 0x728
#define NLG_ACT_PLUG 0x739


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//*********************************************************************//
//Deffining Variables for ADAC
//Inputs
//*********************************************************************//

int dacAdress = 0x10;
#define _ADAC_NULL           B00000000
#define _ADAC_ADC_SEQUENCE   B00000010 // ADC sequence register - Selects ADCs for conversion
#define _ADAC_GP_CONTROL     B00000011 // General-purpose control register - DAC and ADC control register
#define _ADAC_ADC_CONFIG     B00000100 // ADC pin configuration - Selects which pins are ADC inputs
#define _ADAC_DAC_CONFIG     B00000101 // DAC pin configuration - Selects which pins are DAC outputs
#define _ADAC_PULL_DOWN      B00000110 // Pull-down configuration - Selects which pins have an 85 kO pull-down resistor to GND
#define _ADAC_LDAC_MODE      B00000111 // LDAC mode - Selects the operation of the load DAC
#define _ADAC_GPIO_WR_CONFIG B00001000 // GPIO write configuration - Selects which pins are general-purpose outputs
#define _ADAC_GPIO_WR_DATA   B00001001 // GPIO write data - Writes data to general-purpose outputs
#define _ADAC_GPIO_RD_CONFIG B00001010 // GPIO read configuration - Selects which pins are general-purpose inputs
#define _ADAC_POWER_REF_CTRL B00001011 // Power-down/reference control - Powers down the DACs and enables/disables the reference
#define _ADAC_OPEN_DRAIN_CFG B00001100 // Open-drain configuration - Selects open-drain or push-pull for geeral-purpose outputs
#define _ADAC_THREE_STATE    B00001101 // Three-state pins - Selects which pins are three-stated
#define _ADAC_RESERVED       B00001110 // Reserved
#define _ADAC_SOFT_RESET     B00001111 // Software reset - Resets the AD5593R
 
 
#define _ADAC_VREF_ON        B00000010 // VREF on
#define _ADAC_SEQUENCE_ON    B00000010 // Sequence on
 
#define _ADAC_DAC_WRITE      B00010000 // DAC write
#define _ADAC_ADC_READ       B01000000 // ADC read
#define _ADAC_DAC_READ       B01010000 // DAC read
#define _ADAC_GPIO_READ      B01110000 // GPIO read
#define _ADAC_REG_READ       B01100000 // Register read

                  //DAC0                 DAC7
bool DACenable[8] = {1, 1, 1, 0, 0, 0, 0, 0};
bool ADCenable[8] = {0, 0, 0, 0, 0, 1, 1, 1};
bool GPIenable[8] = {0, 0, 0, 0, 1, 0, 0, 0};
bool GPOenable[8] = {0, 0, 0, 1, 0, 0, 0, 0};
 
#define ADCPoti 7

float voltage = 2.65;

int value = 0;


//*********************************************************************//
//Deffining Variables for Operation
//General
//*********************************************************************//
#define Standby 0
#define Run 1
#define Charging 2

#define NLG_ACT_SLEEP 0
#define NLG_ACT_WAKEUP 1 
#define NLG_ACT_STANDBY 2
#define NLG_ACT_READY2CHARGE 3
#define NLG_ACT_CHARGE 4
#define NLG_ACT_SHUTDOWN 5

#define NLG_HW_Wakeup 9 //Input for VCU
#define IGNITION 99 //Input for VCU

#define NLG_DEM_STANDBY 0
#define NLG_DEM_CHARGE 1
#define NLG_DEM_SLEEP 6

bool NLG_Charged = 0;

uint8_t sampleSetCounter = 0;
int16_t sampleSetPedal[5] = {0,0,0,0,0};
bool reversSig = 0;
uint8_t VehicleMode = Run;

//*********************************************************************//
//Deffining Variables for Can transmission
//NLG
//*********************************************************************//

//Sending Variables
//Variables for 0x711
bool NLG_C_ClrError = 0;         //Clear error latch
bool NLG_C_UnlockConRq = 0;      //Unlock connector request
bool NLG_C_VentiRq = 0;
int  NLG_DcHvVoltLimMax = 400;   //Maximum HV voltage

int NLG_DcHvCurrLimMax = 50;
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

//*********************************************************************//
//Deffining Variables for Can transmission
//DMC
//*********************************************************************//
//Sending Variables
//Variables for 0x210
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
//Variables for 0x211
int DMC_DcVLimMot = 380;
int DMC_DcVLimGen = 422;
int DMC_DcCLimMot = 200;
int DMC_DcCLimGen = 100;

int DMC_DcVLimMot_Scale = 0;
int DMC_DcVLimGen_Scale = 0;
int DMC_DcCLimMot_Scale = 0;
int DMC_DcCLimGen_Scale = 0;
//Variables for0x212
int DMC_TrqSlewrate = 1300;
int DMC_SpdSlewrate = 655;
int DMC_MechPwrMaxMot = 20000;
int DMC_MechPwrMaxGen = 20000;

int DMC_TrqSlewrate_Scale = 0;
int DMC_MechPwrMaxMot_Scale = 0;
int DMC_MechPwrMaxGen_Scale = 0;

//recive Variable
//Variables for 0x458
float DMC_TempInv = 0;
float DMC_TempMot = 0;
int8_t DMC_TempSys = 0;

//Variables for 0x258
bool DMC_Ready = 0;
bool DMC_Running = 0;

bool DMC_SensorWarning	= 0;
bool DMC_GenErr = 0;
bool DMC_TrqLimitation = 0;

float DMC_TrqAvl = 0;
float DMC_TrqAct = 0;
float DMC_SpdAct = 0;
//Variables for 0x259
float DMC_DcVltAct = 0;
float DMC_DcCurrAct = 0;
float DMC_AcCurrAct = 0;
int32_t DMC_MechPwr = 0;



//SendingVariables
//Variables for 0x260
bool enableBSC = 0;
bool modeBSC = 1;
int Hvoltage = 400;  
int Lvoltage = 8;
int LvoltageScale = 0;
int HvoltageScale = 0;

//Variables for 0x261
int BSC6_HVVOL_LOWLIM = 240;
int BSC6_HVVOL_LOWLIM_SCALED = 0;
int BSC6_LVCUR_UPLIM_BUCK = 240;
int BSC6_HVCUR_UPLIM_BUCK = 11;
int BSC6_HVCUR_UPLIM_BUCK_SCALED = 0;
int BSC6_LVVOL_LOWLIM = 4;
int BSC6_LVVOL_LOWLIM_SCALED = 0;
int BSC6_LVCUR_UPLIM_BOOST = 20;
int BSC6_HVCUR_UPLIM_BOOST = 11;
int BSC6_HVCUR_UPLIM_BOOST_SCALED = 0;

//Reciveing Variables
//Variables For 0x26A	
float BSC6_HVVOL_ACT = 0;
float BSC6_LVVOLT_ACT = 0;
float BSC6_HVCUR_ACT = 0;
float BSC6_LVCUR_ACT = 0;
unsigned char BSC6_MODE = 16;

uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr
uint8_t  len;

byte readDataBSC[MAX_DATA_SIZE] = {0}; //Storage for recived data BSC
byte readDataNLG[MAX_DATA_SIZE] = {0}; //Storage for recived data NLg
unsigned char controllBufferDMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char controllBuffer2DMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char limitBufferDMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char controllBufferBSC[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //Storage for controll mesages
unsigned char limitBufferBSC[8] = {0, 0, 0, 0, 0, 0, 0, 0};    //Stroage for limit Values
unsigned char controllBufferNLG1[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);
  pinMode(IGNITION, INPUT);
  pinMode(NLG_HW_Wakeup, INPUT);
  pinMode(18, INPUT);
  //Init the i2c bus
  Wire.begin(1,2);
  //Init the LCD
  initADAC();
  lcd.init();                      
  lcd.init();
  lcd.backlight();

  Serial.begin(115200);
 

  #if MAX_DATA_SIZE > 8
  CAN.setMode(CAN_NORMAL_MODE);
  #endif
  CAN.begin(CAN_500KBPS);            // init can bus : baudrate = 500k
  
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    CAN_COM,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    BACKBONE,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500); 
}

//Can Com on Core 0
void CAN_COM( void * pvParameters ){

  for(;;){
    esp_task_wdt_init(5, true);
    switch(VehicleMode){
      case Standby:

      break;
      case Run:
       
        DMC_TrqRq_Scale = calculateTorque5S(reversSig);
        Serial.println(DMC_TrqRq_Scale);
  
        sendBSC();
        sendDMC();
        reciveBSC();
        reciveDMC();  
        
      break;

      case Charging:
        sendBSC();
        sendNLG();
        reciveBSC();
        reciveNLG();
      break;
      default:
        VehicleMode = Standby;
      break;
    }

  } 
}
//Backbone on Core 1
void BACKBONE( void * pvParameters ){
  
  for(;;){
    esp_task_wdt_init(5, true);
    switch(VehicleMode){
      case Standby:
        armBattery(0);
        armNLG(0);
        armBSC(0);
        armDMC(0);
        if(digitalRead(NLG_HW_Wakeup)){VehicleMode = Charging;}
        if(digitalRead(IGNITION)){VehicleMode = Run;}
        enableBSC = 0;
        enableDMC = 0;
      break;
      
      case Run:
        if(digitalRead(NLG_HW_Wakeup)){VehicleMode = Charging;}
        if(!digitalRead(IGNITION)){VehicleMode = Standby;}
          armBattery(1);
          armNLG(0);
          armBSC(1);
          armDMC(1);
        for(sampleSetCounter = 0; sampleSetCounter < 5; sampleSetCounter ++){  
        sampleSetPedal[sampleSetCounter] = readADC(ADCPoti); //Read ADC into sampleSet
        }
        enableBSC = 1;
        enableDMC = 1;
        setLCDBSC();
        setLCDDMC();
        lcd.setCursor(0,2);
        lcd.print(readADC(ADCPoti));
        if(errorCnt < 40 && DMC_SensorWarning | DMC_GenErr){
          errorCnt ++;
          errLatch = 1;
        }
        else{
          errorCnt = 0;
        }
      break;
      case Charging:
        chargeManage();
        if(!digitalRead(NLG_HW_Wakeup)){VehicleMode = Standby;}
        
      break;
      default:
        VehicleMode = Standby;
      break;
    } 
  }
}
void chargeManage(){
  if(VehicleMode == Charging){
    switch (NLG_StateAct){
    case  NLG_ACT_SLEEP :
      NLG_StateDem = NLG_DEM_SLEEP;   //Demand Standby
      break;
    case NLG_ACT_STANDBY:
      armBattery(1);
      armNLG(1);
      armBSC(1);
      armDMC(0);
      break;
    case NLG_ACT_READY2CHARGE:
      NLG_StateDem = NLG_DEM_CHARGE;  //Demand Charge
      startBSC();
      break;
    default:
      armBattery(0);
      armNLG(0);
      break;
    }
  }
  else {
    armBattery(0);
    armNLG(0);
    armBSC(0);
    armDMC(0);
    }
  
}
void startBSC(){

}
void armBattery(bool arm){
  //TODO
  Serial.println("Not DONE Battery ARM/DISARM");
}
void armBSC(bool arm){
  //TODO
  Serial.println("Not DONE BSC ARM/DISARM");
}
void armDMC(bool arm){
  //TODO
  Serial.println("Not DONE DMC ARM/DISARM");
}
void armNLG(bool arm){
  //TODO  !! PRECHARGE nicht vergessen
  Serial.println("Not DONE NLG ARM/DISARM");
}
void setLCDBSC(){
  lcd.setCursor(0,0);
  lcd.print("BSC6");
  lcd.setCursor(5,0);
  
  switch (BSC6_MODE){
    case 0:
      lcd.print("Ready.."); 
    break;
    case 1:
      lcd.print("Running");
    break;
    case 6:
      lcd.print("Running");
    break;
    case 16:
      lcd.print("NoCAN");
    break;
    default:
      lcd.print("ERROR");
    break;
  }
  lcd.setCursor(13,0);
  lcd.print(BSC6_LVVOLT_ACT);
  lcd.setCursor(19,0);
  lcd.print("V");
}
void setLCDDMC(){
  lcd.setCursor(0,1);
  lcd.print("DMC5");
  lcd.setCursor(5,1);
  if(DMC_SensorWarning | DMC_GenErr){
    lcd.print("ERROR");
  }
  else if (DMC_Ready){
    lcd.print("Ready.."); 
  }
  else if (DMC_Running){
    lcd.print("Running");
  }
  else{
    lcd.print("ERROR");
  }
  lcd.setCursor(13,1);
  lcd.print(DMC_DcCurrAct);
  lcd.setCursor(19,1);
  lcd.print("A");
}
void reciveBSC(){
 //Reciveing Can
    // check if data coming
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
    
    
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
}
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
void reciveDMC(){
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
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
}

void sendNLG(){
    NLG_DcHvVoltLimMax_Scale = NLG_DcHvVoltLimMax * 10;
    NLG_DcHvCurrLimMax_Scale = NLG_DcHvCurrLimMax * 10;
    NLG_DcHvCurrLimMax_Scale -= 1024;
    NLG_AcCurrLimMax_Scale = NLG_AcCurrLimMax * 10;
    NLG_AcCurrLimMax_Scale -= 1024;
    NLG_AcPhaseShift_Scale = NLG_AcPhaseShift * 10;
    controllBufferNLG1[0] = NLG_C_ClrError << 8 | NLG_C_UnlockConRq << 7 | NLG_C_VentiRq << 6 | (NLG_DcHvVoltLimMax_Scale >> 8) & 0x1F;
    controllBufferNLG1[1] = NLG_DcHvVoltLimMax_Scale & 0x00FF;
    controllBufferNLG1[2] = (NLG_StateDem << 5)  | (NLG_DcHvCurrLimMax_Scale >> 8)& 0x07;
    controllBufferNLG1[3] = NLG_DcHvCurrLimMax_Scale & 0x00FF;
    controllBufferNLG1[4] = NLG_LedDem << 4 | (NLG_AcCurrLimMax_Scale << 8) & 0x07;
    controllBufferNLG1[5] = NLG_AcCurrLimMax_Scale & 0x00FF;
    controllBufferNLG1[6] = NLG_C_EnPhaseShift << 4 | (NLG_AcPhaseShift_Scale >> 8) & 0x07;
    controllBufferNLG1[7] = NLG_AcPhaseShift_Scale & 0x00FF;
    CAN.sendMsgBuf(NLG_DEM_LIM, 0, 8, controllBufferNLG1);
    delay(100);                       // send data per 100ms
   
}
void reciveNLG(){

    if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return;
    }



   
    // read data, len: data length, buf: data buf
    CAN.readMsgBuf(&len, readDataNLG);

    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) |
           (CAN.isRemoteRequest() << 1);
    
    
    
    switch (id){
      case NLG_ACT_LIM:
        NLG_StateCtrlPilot = readDataNLG[0] >> 5;
        NLG_DcHvVoltAct = (readDataNLG[0] << 8) & 0x1F00;
        NLG_DcHvVoltAct = NLG_DcHvVoltAct | readDataNLG[1];
        NLG_StateAct = readDataNLG[2] >> 5;
        NLG_S_DcHvCurrLim = readDataNLG[2] >> 4 & 0x01;
        NLG_S_DcHvVoltLim = readDataNLG[2] >> 3 & 0x01;
        NLG_DcHvCurrAct = (readDataNLG[2] << 8) & 0x07;
        NLG_DcHvCurrAct = NLG_DcHvCurrAct | readDataNLG[3];
        NLG_S_ProximityLim = readDataNLG[4] >> 7;
        NLG_S_CtrlPilotLim = (readDataNLG[4] >> 6) & 0x01;
        NLG_S_ConTempLim = (readDataNLG[4] >> 5) & 0x01;
        NLG_S_IntTempLim = (readDataNLG[4] >> 4) & 0x01;
        NLG_S_AcCurrLim= (readDataNLG[4] >> 5) & 0x01;
        NLG_AcCurrMaxAct = (readDataNLG[4] << 8) & 0x07;
        NLG_AcCurrMaxAct = NLG_AcCurrMaxAct | readDataNLG[5];
        NLG_AcCurrHwAvl = readDataNLG[6];
        NLG_S_ProximityDet = readDataNLG[7] >> 7;
        NLG_S_CtrlPilotDet = (readDataNLG[7] >> 6) & 0x01;
        NLG_S_ConLocked = (readDataNLG[7] >> 5) & 0x01;
        NLG_S_AcDet = (readDataNLG[7] >> 4) & 0x01;
        NLG_S_HwWakeup = (readDataNLG[7] >> 3) & 0x01;
        NLG_S_HwEnable = (readDataNLG[7] >> 2) & 0x01;
        NLG_S_Err = (readDataNLG[7] >> 1) & 0x01;
        NLG_S_War = readDataNLG[7]& 0x01;
      break;
      case NLG_ACT_PLUG:
        NLG_StatusCP = (readDataNLG[0] >> 5) & 0x07;
        NLG_S_CP_X1 = (readDataNLG[0] >> 4) & 0x01;
        NLG_S_CP_SCC = (readDataNLG[0] >> 3) & 0x01;
        NLG_AcCurrMaxCP = (readDataNLG[0] << 8) & 0x03;
        NLG_AcCurrMaxCP = NLG_AcCurrMaxCP | readDataNLG[1];
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

      break;
    }
}
//*********************************************************************//
//Functions for ADAC
//Inputs
//*********************************************************************//
void initADAC(){
  voltage = voltage + 0.001;
  value = (voltage / 5)*pow(2,12);
  setVref(1);
 
  setupDAC();
  setupADC();
  setupGPI();
  setupGPO();
 
  setLDAC(0);
 
  setDACGain(1);
}
 
void setVref(bool enable){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_POWER_REF_CTRL);  // [D0]
  uint8_t setup = enable << 1;
  Wire.write(setup);  // [D0]
  Wire.write(B00000000);  // [D0]
 
  Wire.endTransmission();
}
 
void setupDAC(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_DAC_CONFIG);
  uint8_t setup = 0;
  for(int i = 0; i <= 7; i++){
    setup = setup | DACenable[i] << i;
  }
  Wire.write(0x00);  // [D0]
  Wire.write(setup);  // [D0]
 
  Wire.endTransmission();
}
 
void setupADC(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_ADC_CONFIG);
  uint8_t setup = 0;
  for(int i = 0; i <= 7; i++){
    setup = setup | ADCenable[i] << i;
  }
  Wire.write(0x00);  // [D0]
  Wire.write(setup);  // [D0]
 
  Wire.endTransmission();
}
 
void setupGPO(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_GPIO_WR_CONFIG);
  uint8_t setup = 0;
  for(int i = 0; i <= 7; i++){
    setup = setup | GPOenable[i] << i;
  }
  
  Wire.write(0x00);  // [D0]
  Wire.write(setup);  // [D0]
 
  Wire.endTransmission();
}
 
void setupGPI(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_GPIO_RD_CONFIG);
  uint8_t setup = 0;
  for(int i = 0; i <= 7; i++){
    setup = setup | GPIenable[i] << i;
  }
  Wire.write(0x00);  // [D0]
  Wire.write(setup);  // [D0]
 
  Wire.endTransmission();
}
 
void setDACVal(uint8_t DACnum, uint16_t DACvalue){
  DACnum = DACnum & 0x0F;
  DACvalue = DACvalue & 0x0FFF;
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_DAC_WRITE|DACnum);
  Wire.write(DACvalue >> 8);  // [D0]
  Wire.write(DACvalue & 0x00FF);  // [D0]
 
  Wire.endTransmission();
}

uint16_t readADC(uint8_t DACnum){
  uint16_t ADCvalue = 0;
  DACnum = DACnum & 0x0F;

  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_ADC_SEQUENCE);
  Wire.write(0x02);
  Wire.write(byte(1 << DACnum));
  Wire.endTransmission();

  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_ADC_READ);
  Wire.endTransmission();
 
  Wire.requestFrom(int(dacAdress), int(2), int(1));
  if (Wire.available()) ADCvalue = (Wire.read() & 0x0f) << 8;
  if (Wire.available()) ADCvalue = ADCvalue | Wire.read();

  Wire.endTransmission();
  return ADCvalue;
}

void setDACGain(bool gain){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_GP_CONTROL);
  Wire.write(0x00);  // [D0]
  Wire.write(gain << 4);  // [D0]
 
 Wire.endTransmission();
}
 
void setLDAC(uint8_t LDAC){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_LDAC_MODE);
  Wire.write(0x00);  // [D0]
  Wire.write(LDAC & 0x03);  // [D0]
 
  Wire.endTransmission();
}

int16_t calculateTorque5S(bool reverseSig){
  int16_t SampeldPotiValue = 0;
  int16_t DMC_TorqueCalc = 0;

  for(int i = 0; i < 5; i++){
    SampeldPotiValue = SampeldPotiValue + sampleSetPedal[i];
  }
  SampeldPotiValue = SampeldPotiValue / 5;
  DMC_TorqueCalc = map(SampeldPotiValue, 0, 4096, 0, 32767);
  if(reverseSig){
    DMC_TorqueCalc = 0 - DMC_TorqueCalc;
  }
  return DMC_TorqueCalc;
}
void loop() {}
