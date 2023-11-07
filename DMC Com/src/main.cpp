#include <SPI.h>
#define DMCCTRL 0x210
#define CAN_2515
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
//sendVariable
bool enable = 1;
bool mode = 1;
bool oscLim = 0;
bool negTrqSpd = 1;
bool posTrqSpd = 1;
bool errLatch = 0;

int speed = 10000;  // Desired speed in rpm
int torque = 200;
int torqueScaled = 0;
int raw = 0;
unsigned char lowNibSpd = 0;
unsigned char highNibSpd = 0;
unsigned char lowNibTrq = 0;
unsigned char highNibTrq = 0;
unsigned char controllBufferDMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
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

uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr
uint8_t  len;
byte readDataBSC[MAX_DATA_SIZE] = {0};

void setup() {
    //send 
    Serial.begin(115200);
    #if MAX_DATA_SIZE > 8
    CAN.setMode(CAN_NORMAL_MODE);
    #endif
    CAN.begin(CAN_500KBPS);            // init can bus : baudrate = 500k

}

void loop() {
    // Set the speed request value in the stmp array
    //raw = analogRead(A6);
     //speed = map(raw, 0, 255, 0, 20000);
    torqueScaled = torque * 100;
    lowNibSpd = speed & 0x00FF;
    highNibSpd = speed >> 8;
    lowNibTrq = torqueScaled & 0x00FF;
    highNibTrq = torqueScaled >> 8;

    controllBufferDMC[0] = enable << 7 | mode << 6 | oscLim << 5 | negTrqSpd << 1 | posTrqSpd;
    controllBufferDMC[2] = highNibSpd;
    controllBufferDMC[3] = lowNibSpd;
    controllBufferDMC[4] = highNibTrq;
    controllBufferDMC[5] = lowNibTrq;

    CAN.sendMsgBuf(DMCCTRL, 0, 8, controllBufferDMC);
    delay(100);                       // send data per 100ms
   

    if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return;
    }



   
    // read data, len: data length, buf: data buf
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
    /*
    Serial.print("Id:");
    Serial.print(id, HEX);
    for (i = 0; i < len; i++) {
        Serial.print("(");
        Serial.print(readDataBSC[i], HEX);
        Serial.print(")");
    }
    */
    Serial.println(DMC_TempSys);
}
