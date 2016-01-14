#include "DAC.h"

#ifdef DAC_STEPPER_CURRENT

void DAC_begin(DAC *dac)
{
  //TODO
}

uint8_t DAC_reset(DAC *dac) {
  return DAC_simpleCommand(dac, RESET);
}

uint8_t DAC_writeGain() {
  //TODO
  return 0;
}

uint8_t DAC_writeVref() {
  //TODO
  return 0;
}

uint8_t DAC_setVref(DAC *dac, uint8_t channel, uint8_t value) {
  dac->_intVref[channel] = value;
  return DAC_writeVref(dac);
}

uint8_t DAC_setGain(DAC *dac, uint8_t channel, uint8_t value) {
  dac->_gain[channel] = value;
  return DAC_writeGain();
}

uint8_t DAC_fastWrite(DAC *dac) {
  //TODO
}

uint8_t DAC_analogWrite(DAC *dac, uint16_t value1, uint16_t value2, uint16_t value3, uint16_t value4) {
  dac->_values[0] = value1;
  dac->_values[1] = value2;
  dac->_values[2] = value3;
  dac->_values[3] = value4;
  return fastWrite();
}

uint8_t DAC_simpleCommand(DAC *dac, byte simpleCommand) {
  //TODO
  return 0;
}

#endif
