/*
  This code contibuted by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#ifndef _FASTIO_ARDUINO_H
#define _FASTIO_ARDUINO_H

#include <avr/io.h>

/*
  utility functions
*/

#ifndef MASK
/// MASKING- returns \f$2^PIN\f$
#define MASK(PIN)  (1 << PIN)
#endif


#define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)

#define _WRITE_C(IO, v)   do { if (v) { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); }\
                                         CRITICAL_SECTION_END; \
                                       }\
                                       else {\
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }\
                                         CRITICAL_SECTION_END; \
                                       }\
                                     }\
                                     while (0)




#define _WRITE(IO, v)  do {  if (&(DIO ##  IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)


#define WRITE(IO, v)  _WRITE(IO, v)





#endif
