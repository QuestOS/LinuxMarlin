
CC=gcc
CFLAGS=-I. -std=c99 -lm
DEPS = Marlin.h


%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)


all: Marlin_main.o Arduino.o planner.o stepper.o
	$(CC) -o marlin Marlin_main.o Arduino.o planner.o stepper.o $(CFLAGS)

fornow:
	$(CC) -Wall -o marlin Marlin_main.c


clean:
	rm -f *.o marlin
