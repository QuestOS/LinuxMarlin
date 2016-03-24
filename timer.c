#include <signal.h>
#include "Marlin.h"
#include "timer.h"

#define NTIMER 2

static int n_timer, n_enabled_timer;
struct timer_info timer_list[2];
sigset_t mask;

static void timer_handler(int sig, siginfo_t *si, void *uc)
{
	timer_t *tidp;
	struct timer_info tinfo;
	int i;

	tidp = si->si_value.sival_ptr;
	for (i = 0; i < n_timer; i++) {
		if (timer_list[i].timer_sys_id == *tidp) {
			tinfo = timer_list[i];
			break;
		}
	}

	if (tinfo.enable) {
		//call the registered handler
		tinfo.handler();
	}
}

//register a generic timer handler to SIGALRM
void timer_init()
{
	/* establish handler for timer signal */
	struct sigaction sa;
	sa.sa_flags = SA_SIGINFO;
  sa.sa_sigaction = timer_handler;
  sigemptyset(&sa.sa_mask);
  if (sigaction(SIGALRM, &sa, NULL) == -1)
    errExit("sigaction");

	/* block timer signal temporarily */
	sigemptyset(&mask);
	sigaddset(&mask, SIGALRM);
	if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
		errExit("sigprocmask");
}

static inline void _enable_timer(struct timer_info * tinfo)
{
	if (tinfo->enable == 1) {
		return;
	} else {
		tinfo->enable = 1;
		if (!n_enabled_timer) {
			//no timer was active before this one
			//so the global alarm signal must be masked off
			//now enable it
			n_enabled_timer++;
			sigprocmask(SIG_UNBLOCK, &mask, NULL);
		}
	}
}

void enable_timer(int timerid)
{
	_enable_timer(&(timer_list[timerid]));
}

static inline void _disable_timer(struct timer_info * tinfo)
{
	if (tinfo->enable == 0) {
		return;
	} else {
		tinfo->enable = 0;
		n_enabled_timer--;
		if (!n_enabled_timer) {
			//no enabled timer in the process, disable the global alarm signal
			sigprocmask(SIG_BLOCK, &mask, NULL);
		}
	}
}

void disable_timer(int timerid)
{
	_disable_timer(&(timer_list[timerid]));
}

int create_timer(void (*handler)(void))
{
  struct sigevent sev;
	struct timer_info * tinfo = &(timer_list[n_timer]);

  /* create the timer */
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = SIGALRM;
  sev.sigev_value.sival_ptr = &(tinfo->timer_sys_id);
  if (timer_create(CLOCK_REALTIME, &sev, &(tinfo->timer_sys_id)) == -1)
    errExit("timer_create");
  DEBUG_PRINT("timer ID is 0x%lx\n", (long) tinfo->timer_sys_id);

	tinfo->handler = handler;
  memset(&(tinfo->its), 0, sizeof(struct itimerspec));
	//default to disabled
	tinfo->enable = 0;

	return n_timer++;
}

//mode: 0-one shot, 1-periodic
void set_time(int timerid, int mode, long nanosecs)
{
  //DEBUG_PRINT("set timer %id to %lfus\n", timerid, nanosecs / 1000.0);
	struct timer_info * tinfo = &(timer_list[timerid]);

	if (mode == 0) {
		tinfo->its.it_value.tv_nsec = nanosecs;
		if (timer_settime(tinfo->timer_sys_id, 0, &(tinfo->its), NULL) == -1)
			errExit("timer_settime");
	} else {
		tinfo->its.it_value.tv_nsec = nanosecs;
		tinfo->its.it_interval.tv_nsec = nanosecs;
		if (timer_settime(tinfo->timer_sys_id, 0, &(tinfo->its), NULL) == -1)
			errExit("timer_settime");
	}
}
