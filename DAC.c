#include "DAC.h"

#ifdef DAC_STEPPER_CURRENT

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

#endif
