#include <SPI.h>
//Send
#define NLG_DEM_LIM 0x711
//recive
#define NLG_ACT_ERR 0x799
#define NLG_ACT_LIM 0x728
#define NLG_ACT_PLUG 0x739

#define SLEEP 6
#define STANDBY 0
#define Charge
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


uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr
uint8_t  len;
byte readDataNLG[MAX_DATA_SIZE] = {0};
//send Var 0x711
bool NLG_C_ClrError = 0;
bool NLG_C_UnlockConRq = 0;
bool NLG_C_VentiRq = 0;
int  NLG_DcHvVoltLimMax = 400;
int NLG_DcHvVoltLimMax_Scale = 0;
int NLG_DcHvCurrLimMax = 50;
int NLG_DcHvCurrLimMax_Scale = 0;
uint16_t NLG_AcCurrLimMax_Scale = 0;
uint8_t NLG_StateDem = 0;
uint16_t NLG_LedDem = 0;
uint16_t NLG_AcCurrLimMax = 0;

bool NLG_C_EnPhaseShift = 0;
uint16_t NLG_AcPhaseShift = 0;
uint16_t NLG_AcPhaseShift_Scale = 0;
uint16_t prep = 0;
//recive 0x709
uint16_t NLG_AcWhAct = 0;
uint16_t NLG_DcHvWhAct = 0;
uint16_t NLG_MaxTempAct = 0;
int16_t NLG_TempCon = 0;
float NLG_DcHvAhAct = 0;
//recive 728
uint8_t NLG_StateCtrlPilot = 0;
uint16_t NLG_DcHvVoltAct = 0;
uint8_t NLG_StateAct = 0;
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
uint16_t NLG_DcHvCurrAct = 0;
//recive 0x739
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

unsigned char controllBufferNLG1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void setup() {
    //send 
    Serial.begin(115200);
    #if MAX_DATA_SIZE > 8
    CAN.setMode(CAN_NORMAL_MODE);
    #endif
    CAN.begin(CAN_500KBPS);            // init can bus : baudrate = 500k

}

void loop() {

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
