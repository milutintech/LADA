
//Function to calculate the torque request for the DMC with 5 samples

int16_t calculateTorque5S(int16_t sensorSim[5],bool reverseSig){
  int16_t SampeldPotiValue = 0;
  int16_t DMC_TorqueCalc = 0;

  for(int i = 0; i < 5; i++){
    SampeldPotiValue = SampeldPotiValue + sensorSim[i];
  }
  SampeldPotiValue = SampeldPotiValue / 5;
  DMC_TorqueCalc = map(SampeldPotiValue, 0, 4096, 0, 32767);
  if(reverseSig){
    DMC_TorqueCalc = 0 - DMC_TorqueCalc;
  }
  return DMC_TorqueCalc;
}


// ADAC
#include <Wire.h>
#include <math.h>
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
 
 
#define _ADAC_VREF_ON     B00000010
#define _ADAC_SEQUENCE_ON B00000010
 
#define _ADAC_DAC_WRITE       B00010000
#define _ADAC_ADC_READ        B01000000
#define _ADAC_DAC_READ        B01010000
#define _ADAC_GPIO_READ       B01110000
#define _ADAC_REG_READ        B01100000
                  //DAC0                 DAC7
bool DACenable[8] = {1, 1, 1, 0, 0, 0, 0, 0};
bool ADCenable[8] = {0, 0, 0, 0, 0, 1, 1, 1};
bool GPIenable[8] = {0, 0, 0, 0, 1, 0, 0, 0};
bool GPOenable[8] = {0, 0, 0, 1, 0, 0, 0, 0};
 
float voltage = 2.65;
 
int value = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin(1,2);
  voltage = voltage + 0.001;
  value = (voltage / 5)*pow(2,12);
  initDAC();
  setDACVal(1, value);
 
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_ADC_READ);
  Serial.print("request");
  Serial.println(Wire.requestFrom(dacAdress, 2, true));
  Serial.println( Wire.endTransmission());
 
 
 
}
 
void loop() {
 
 
}
 
void initDAC(){
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
 
  Serial.println( Wire.endTransmission());
}
 
void setupDAC(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_DAC_CONFIG);
  uint8_t setup = 0;
  for(int i = 0; i <= 7; i++){
    setup = setup | DACenable[i] << i;
  }
  Serial.println(setup);
  Wire.write(0x00);  // [D0]
  Wire.write(setup);  // [D0]
 
  Serial.println( Wire.endTransmission());
}
 
void setupADC(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_ADC_CONFIG);
  uint8_t setup = 0;
  for(int i = 0; i <= 7; i++){
    setup = setup | ADCenable[i] << i;
  }
  Serial.println(setup);
  Wire.write(0x00);  // [D0]
  Wire.write(setup);  // [D0]
 
  Serial.println( Wire.endTransmission());
}
 
void setupGPO(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_GPIO_WR_CONFIG);
  uint8_t setup = 0;
  for(int i = 0; i <= 7; i++){
    setup = setup | GPOenable[i] << i;
  }
  Serial.println(setup);
  Wire.write(0x00);  // [D0]
  Wire.write(setup);  // [D0]
 
  Serial.println( Wire.endTransmission());
}
 
void setupGPI(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_GPIO_RD_CONFIG);
  uint8_t setup = 0;
  for(int i = 0; i <= 7; i++){
    setup = setup | GPIenable[i] << i;
  }
  Serial.println(setup);
  Wire.write(0x00);  // [D0]
  Wire.write(setup);  // [D0]
 
  Serial.println( Wire.endTransmission());
}
 
void setDACVal(uint8_t DACnum, uint16_t DACvalue){
  DACnum = DACnum & 0x0F;
  DACvalue = DACvalue & 0x0FFF;
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_DAC_WRITE|DACnum);
  Wire.write(DACvalue >> 8);  // [D0]
  Wire.write(DACvalue & 0x00FF);  // [D0]
 
  Serial.println( Wire.endTransmission());
}
/*
void readADC(){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_DAC_WRITE|DACnum);
  Wire.write(DACvalue >> 8);  // [D0]
  Wire.write(DACvalue & 0x00FF);  // [D0]
 
  Serial.println( Wire.endTransmission());
}
*/
void setDACGain(bool gain){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_GP_CONTROL);
  Wire.write(0x00);  // [D0]
  Wire.write(gain << 4);  // [D0]
 
  Serial.println( Wire.endTransmission());
}
 
void setLDAC(uint8_t LDAC){
  Wire.beginTransmission(dacAdress);
  Wire.write(_ADAC_LDAC_MODE);
  Wire.write(0x00);  // [D0]
  Wire.write(LDAC & 0x03);  // [D0]
 
  Serial.println( Wire.endTransmission());
}
 
 
