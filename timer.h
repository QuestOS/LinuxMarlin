#ifndef __TIME_H__
#define  __TIME_H__

#include <time.h>
#include <stdbool.h>

struct timer_info {
	timer_t timer_sys_id;
	struct itimerspec its;
	void (*handler)(void);
	bool enable;
};

extern void timer_init();
extern void enable_timer(int);
extern void disable_timer(int);
extern void set_time(int, int, long);
extern int create_timer(void (*handler)(void));

#endif
