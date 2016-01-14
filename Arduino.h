#ifndef ARDUINO_H
#define ARDUINO_H

//TODO: value may need to be changed
#define HIGH 1
#define LOW -1

unsigned long millis();
int timeInit();

int digitalRead(int pin);
void digitalWrite(int pin, int val);

#endif
