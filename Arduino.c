#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <sys/sysinfo.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "fastio.h"

static uint64_t tsc_init = 0;
static float clocks_per_ns = 0;
float cpufreq = 0;

/* PWM for fan */
mraa_pwm_context pwm_cxt;

static inline uint64_t rdtsc(void)
{
    uint32_t lo, hi;
    uint64_t returnVal;
    /* We cannot use "=A", since this would use %rax on x86_64 */
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    returnVal = hi;
    returnVal <<= 32;
    returnVal |= lo;

    return returnVal;
}

unsigned long millis( void )
{
    /* similar to the micros() function, it returns ms since sketch start up time.
     The underlying counter is a 64 bit value, but the representation of millis
     as unsigned 32-bits means it recycles in ~ 1190 hours.*/

    uint64_t tsc_cur = rdtsc(), diff = 0, divisor = 0;
    divisor = (cpufreq * 1000);
    diff = tsc_cur - tsc_init;

    return (unsigned long) ( (diff / divisor) );

}

/* TSC snapshot */
int timeInit(void)
{
    int cpufreq_fd, ret;
    char buf[0x400];
    char * str = 0, * str2 = 0;
    char * mhz_str = "cpu MHz\t\t: ";

    /* Grab initial TSC snapshot */
    tsc_init = rdtsc();

    cpufreq_fd = open("/proc/cpuinfo", O_RDONLY);
    if( cpufreq_fd < 0){
        fprintf(stderr, "unable to open /proc/cpuinfo\n");
        return -1;
    }
    memset(buf, 0x00, sizeof(buf));
    ret = read(cpufreq_fd, buf, sizeof(buf));
    if ( ret < 0 ){
        fprintf(stderr, "unable to read cpuinfo !\n");
        close(cpufreq_fd);
        return -1;
    }
    close(cpufreq_fd);
    str = strstr(buf, mhz_str);
    if (!str){
        fprintf(stderr, "Buffer %s does not contain CPU frequency info !\n", buf);
        return -1;
    }

    str += strlen(mhz_str);
    str2 = str;

    while(str2 < buf  + sizeof(buf)-1 && *str2 != '\n'){
        str2++;
    }
    if(str2 == buf + sizeof(buf-1) && *str2 !='\n'){
        fprintf(stderr, "malformed cpufreq string %s\n", str);
        return -1;
    }
    *str2 = '\0';
    cpufreq = atof(str);


    printf("cpufrequency is %f mhz\n", cpufreq);

    /* Calculate nanoseconds per clock */
    clocks_per_ns = 1000/cpufreq;

    printf("nanoseconds per clock %f\n", clocks_per_ns);
}

int digitalRead(int pin)
{
  return READ(pin);
}

void digitalWrite(int pin, int val)
{
  WRITE(pin, val);
}

void analogWrite(int pin, int val)
{
#ifdef DEBUG
  if (pin != FAN_PIN)
    DEBUG_PRINT("analogWrite: trying to use invalid analog pin!");
#endif
  if (!pwm_cxt) {
    pwm_cxt = mraa_pwm_init(GET_OS_MAPPING(pin));
    if (!pwm_cxt) {
      errExit("mraa_pwm_init");
    }

    mraa_pwm_period_us(pwm, 1);
    mraa_pwm_enable(pwm, 1);
  }
  mraa_pwm_write(pwm_cxt, (float)val / (float)255);
}
