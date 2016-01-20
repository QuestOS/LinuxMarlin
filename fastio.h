/*
  This code contibuted by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#ifndef _FASTIO_ARDUINO_H
#define _FASTIO_ARDUINO_H

//#include <avr/io.h>

/*
  utility functions
*/

#define NGPIO 26

void WRITE(unsigned IO, int v);
int READ(unsigned IO); 

void SET_INPUT(unsigned IO);
void SET_OUTPUT(unsigned IO);

static const int minnow_pin_mapping[NGPIO+1] = {
	-1, -1, -1, -1, -1, 476, 481,
	477, 480, 478, 483, 479, 482,
	499, 472, 498, 473, 485, 475,
	484, 474, 338, 504, 339, 505,
	340, 464, 
};

#endif
