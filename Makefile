
CC=gcc
CFLAGS=-I.
DEPS = Marlin.h


%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)


all: Marlin_main.o
	$(CC) -o marlin Marlin_main.o $(CFLAGS)

fornow:
	$(CC) -Wall -o marlin Marlin_main.c


clean:
	rm *.o marlin