
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
