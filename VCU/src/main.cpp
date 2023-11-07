#include <Arduino.h>
#include <esp_task_wdt.h>

void Task1code (void * pvParameters);
void Task2code (void * pvParameters);

void sendBSC();
void sendDMC();
void reciveBSC();
void reciveDMC();
void setLCDBSC();
void setLCDDMC();

TaskHandle_t Task1;
TaskHandle_t Task2;

#define SCK 4
#define MOSI 6
#define MISO 5
// Git tests 2


#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#define BSC_COMM 0x260
#define BSC_LIM 0x261
#define CAN_2515
#define DMCCTRL 0x210
#define DMCLIM 0x211
#define DMCCTRL2 0x212
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)


const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

const int SPI_CS_PIN = 36;
const int CAN_INT_PIN = 11;
#endif

#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin

#define MAX_DATA_SIZE 8

#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#define MAX_DATA_SIZE 8
#endif
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
void setup() {
  pinMode(18, INPUT);
  Wire.begin(1,2);
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
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500); 
}
bool VehicleMode = 0;
//DMC
//0x210
bool enableDMC = 0;
bool modeDMC = 1;
bool oscLim = 0;
bool negTrqSpd = 1;
bool posTrqSpd = 1;
bool errLatch = 0;

int errorCnt = 0;

int DMC_SpdRq	 = 10000;  // Desired DMC_SpdRq	 in rpm
int DMC_TrqRq = 20;
int DMC_TrqRq_Scale = 0;
int raw = 0;
unsigned char lowNibSpd = 0;
unsigned char highNibSpd = 0;
unsigned char lowNibTrq = 0;
unsigned char highNibTrq = 0;
//0x211
int DMC_DcVLimMot = 380;
int DMC_DcVLimGen = 422;
int DMC_DcCLimMot = 200;
int DMC_DcCLimGen = 100;

int DMC_DcVLimMot_Scale = 0;
int DMC_DcVLimGen_Scale = 0;
int DMC_DcCLimMot_Scale = 0;
int DMC_DcCLimGen_Scale = 0;
//0x212
int DMC_TrqSlewrate = 1300;
int DMC_SpdSlewrate = 655;
int DMC_MechPwrMaxMot = 20000;
int DMC_MechPwrMaxGen = 20000;

int DMC_TrqSlewrate_Scale = 0;
int DMC_MechPwrMaxMot_Scale = 0;
int DMC_MechPwrMaxGen_Scale = 0;
//recive Variable
//Variables For 0x458
float DMC_TempInv = 0;
float DMC_TempMot = 0;
int8_t DMC_TempSys = 0;

//Variables For 0x258
bool DMC_Ready = 0;
bool DMC_Running = 0;

bool DMC_SensorWarning	= 0;
bool DMC_GenErr = 0;
bool DMC_TrqLimitation = 0;

float DMC_TrqAvl = 0;
float DMC_TrqAct = 0;
float DMC_SpdAct = 0;
//Variables For 0x259
float DMC_DcVltAct = 0;
float DMC_DcCurrAct = 0;
float DMC_AcCurrAct = 0;
int32_t DMC_MechPwr = 0;

//BSC
//SendingVariables
bool enableBSC = 0;
bool modeBSC = 1;

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


int Hvoltage = 400;  
int Lvoltage = 8;
int LvoltageScale = 0;
int HvoltageScale = 0;

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

byte readDataBSC[MAX_DATA_SIZE] = {0}; //Storage for recived data
unsigned char controllBufferDMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char controllBuffer2DMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char limitBufferDMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char controllBufferBSC[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //Storage for controll mesages
unsigned char limitBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};    //Storage for limit Values


//Can Com on Core 0
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    esp_task_wdt_init(5, true);
    if(VehicleMode){

      sendBSC();
      // sendDMC();
      reciveBSC();
      reciveDMC();
    }
  } 
}
//Backbone on Core 1
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    esp_task_wdt_init(5, true);
    if(digitalRead(39)){
      enableBSC = 1;
      enableDMC = 1;
    }
    else{
      enableBSC = 0;
      enableDMC = 0;
    }
    setLCDBSC();
    setLCDDMC();
    if(errorCnt < 40 && DMC_SensorWarning | DMC_GenErr){
      errorCnt ++;
      errLatch = 1;
    }
    else{
      errorCnt = 0;
    }
  } 
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

  limitBuffer[0] = BSC6_HVVOL_LOWLIM_SCALED;
  limitBuffer[1] = BSC6_LVCUR_UPLIM_BUCK;
  limitBuffer[2] = BSC6_HVCUR_UPLIM_BUCK_SCALED;
  limitBuffer[3] = BSC6_LVVOL_LOWLIM_SCALED;
  limitBuffer[4] = BSC6_LVCUR_UPLIM_BOOST;
  limitBuffer[5] = BSC6_HVCUR_UPLIM_BOOST_SCALED;

  CAN.sendMsgBuf(BSC_COMM, 0, 3, controllBufferBSC);
  CAN.sendMsgBuf(BSC_LIM, 0, 6, limitBuffer);
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

  DMC_TrqRq_Scale = DMC_TrqRq * 100;
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
void loop() {}
