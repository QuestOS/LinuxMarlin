
CC=gcc
CFLAGS=-I. -std=c99 -lm
DEPS = Marlin.h


%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)


all: Marlin_main.o Arduino.o planner.o stepper.o temperature.o vector_3.o
	$(CC) -o marlin Marlin_main.o Arduino.o planner.o stepper.o temperature.o vector_3.o $(CFLAGS)

clean:
	rm -f *.o marlin
