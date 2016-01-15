#include "Marlin.h"
#include <mraa.h>

extern mraa_gpio_context gpio_cxt[];

void SET_OUTPUT(unsigned IO)
{
	if (!gpio_cxt[IO]) {
		gpio_cxt[IO] = mraa_gpio_init_raw(IO);
		if (!gpio_cxt[IO]) {
			errExit("mraa_gpio_init_raw");
		}
	}
	mraa_gpio_dir(gpio_cxt[IO], MRAA_GPIO_OUT);
}

void SET_INPUT(unsigned IO)
{
	if (!gpio_cxt[IO]) {
		gpio_cxt[IO] = mraa_gpio_init_raw(IO);
		if (!gpio_cxt[IO]) {
			errExit("mraa_gpio_init_raw");
		}
	}
	mraa_gpio_dir(gpio_cxt[IO], MRAA_GPIO_IN);
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
