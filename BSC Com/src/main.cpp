// Author: Christian Obrecht
// Date: 07.11.2023
// Description: Communication with BSC over CAN
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#define BSC_COMM 0x260
#define BSC_LIM 0x261
#define CAN_2515
#define DMCCTRL 0x210
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)

const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
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
  Wire.begin(21,20);
  lcd.init();                      
  lcd.init();
  lcd.backlight();
  Serial.begin(115200);
  #if MAX_DATA_SIZE > 8
  CAN.setMode(CAN_NORMAL_MODE);
  #endif
  CAN.begin(CAN_500KBPS);            // init can bus : baudrate = 500k
  lcd.setCursor(0,0);
  lcd.print("BSC6");

}
bool enable = 1;
bool mode = 1;
bool oscLim = 0;
bool negTrqSpd = 1;
bool posTrqSpd = 1;
bool errLatch = 0;

int speed = 10000;  // Desired speed in rpm
int torque = 20;
int torqueScaled = 0;
int raw = 0;
unsigned char lowNibSpd = 0;
unsigned char highNibSpd = 0;
unsigned char lowNibTrq = 0;
unsigned char highNibTrq = 0;

//SendingVariables
bool enable = 0;
bool mode = 1;

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


int Hvoltage = 370;  
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
unsigned char controllBufferBSC[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //Storage for controll mesages
unsigned char limitBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};    //Storage for limit Values


void loop() {
  setLCDBSC();
  sendBSC();
  reciveBSC();
  
  delay(10);
}
void setLCDBSC(){
  lcd.setCursor(5,0);
  Serial.println(BSC6_MODE);
  switch (BSC6_MODE){
    case 0:
      lcd.print("Ready"); 
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
  lcd.setCursor(17,0);
  lcd.print("V");
}
void reciveBSC(){
 //Reciveing Can
    // check if data coming
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
       
    char prbuf[32 + MAX_DATA_SIZE * 3];
    int i, n;

    unsigned long t = millis();
    // read data, len: data length, buf: data buf
    CAN.readMsgBuf(&len, readDataBSC);

    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);
    /*
     * MCP2515(or this driver) could not handle properly
     * the data carried by remote frame
     */

    n = sprintf(prbuf, "%04lu.%03d ", t / 1000, int(t % 1000));
    
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

  controllBufferBSC[0] = mode << 1 | enable;
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
