#include "Marlin.h"
#include "fastio.h"
#include <mraa.h>

mraa_gpio_context gpio_cxt[NGPIO];

void SET_OUTPUT(unsigned inIO)
{
	DEBUG_PRINT("setting up pin %u\n", inIO);
	int IO = minnow_pin_mapping[inIO];
	if (!gpio_cxt[inIO]) {
		gpio_cxt[inIO] = mraa_gpio_init_raw(IO);
		if (!gpio_cxt[inIO]) {
			errExit("mraa_gpio_init_raw");
		}
	}
	mraa_gpio_dir(gpio_cxt[inIO], MRAA_GPIO_OUT);
}

void SET_INPUT(unsigned inIO)
{
	int IO = minnow_pin_mapping[inIO];
	if (!gpio_cxt[inIO]) {
		gpio_cxt[inIO] = mraa_gpio_init_raw(IO);
		if (!gpio_cxt[inIO]) {
			errExit("mraa_gpio_init_raw");
		}
	}
	mraa_gpio_dir(gpio_cxt[inIO], MRAA_GPIO_IN);
}

void WRITE(unsigned IO, int v)
{
	if (!gpio_cxt[IO]) {
		errExit("write to uninitialized gpio");
	}
	mraa_gpio_write(gpio_cxt[IO], v);
}

int READ(unsigned IO)
{
	if (!gpio_cxt[IO]) {
		errExit("read from uninitialized gpio");
	}
	return mraa_gpio_read(gpio_cxt[IO]);
}
