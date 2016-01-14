#ifndef DAC_h
#define DAC_h

#include "Configuration.h"
#include "Configuration_adv.h"

#ifdef DAC_STEPPER_CURRENT

#define defaultVDD 5000
#define BASE_ADDR 0x60
#define RESET 0B00000110
#define WAKE 0B00001001
#define UPDATE 0B00001000
#define MULTIWRITE 0B01000000
#define SINGLEWRITE 0B01011000
#define SEQWRITE 0B01010000
#define VREFWRITE 0B10000000
#define GAINWRITE 0B11000000
#define POWERDOWNWRITE 0B10100000
#define GENERALCALL 0B0000000
#define GAINWRITE 0B11000000

typedef struct _DAC
{
  uint8_t      _dev_address;
  uint8_t      _deviceID;
  uint8_t      _intVref[4];
  uint8_t      _gain[4];
  uint8_t      _powerDown[4];
  uint16_t     _values[4];
  uint16_t     _valuesEp[4];
  uint8_t      _intVrefEp[4];
  uint8_t      _gainEp[4];
  uint8_t      _powerDownEp[4];
  uint16_t     _vOut[4];
  uint16_t     _vdd;
} DAC;

uint8_t DAC_fastWrite(DAC *dac);
uint8_t DAC_analogWrite(DAC *dac, uint16_t value1, uint16_t value2, uint16_t value3, uint16_t value4);

#endif
#endif
