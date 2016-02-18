/*
  temperature.c - temperature control
  Part of Marlin
  
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */

#include <signal.h>
#include <time.h>
#include <inttypes.h>
#include "Marlin.h"
#include "temperature.h"
#include "thermistortables.h"
#include "Arduino.h"

//===========================================================================
//=============================public variables============================
//===========================================================================
static int timerid;

int target_temperature[EXTRUDERS] = { 0 };
//int target_temperature_bed = 0;
int current_temperature_raw[EXTRUDERS] = { 0 };
float current_temperature[EXTRUDERS] = { 0.0 };
//int current_temperature_bed_raw = 0;
//float current_temperature_bed = 0.0;
#ifdef PIDTEMP
  float Kp=DEFAULT_Kp;
  float Ki=(DEFAULT_Ki*PID_dT);
  float Kd=(DEFAULT_Kd/PID_dT);
  #ifdef PID_ADD_EXTRUSION_RATE
    float Kc=DEFAULT_Kc;
  #endif
#endif //PIDTEMP

unsigned char soft_pwm_bed;
  
//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

#ifdef PIDTEMP
  //static cannot be external:
  static float temp_iState[EXTRUDERS] = { 0 };
  static float temp_dState[EXTRUDERS] = { 0 };
  static float pTerm[EXTRUDERS];
  static float iTerm[EXTRUDERS];
  static float dTerm[EXTRUDERS];
  //int output;
  static float pid_error[EXTRUDERS];
  static float temp_iState_min[EXTRUDERS];
  static float temp_iState_max[EXTRUDERS];
  // static float pid_input[EXTRUDERS];
  // static float pid_output[EXTRUDERS];
  static bool pid_reset[EXTRUDERS];
#endif //PIDTEMP

static unsigned long  previous_millis_bed_heater;
static unsigned char soft_pwm[EXTRUDERS];

# define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP );
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP );
static int minttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );
//static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP; /* No bed mintemp error implemented?!? */

static void *heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );
static uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );

static float analog2temp(int raw, uint8_t e);
//static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#ifndef SOFT_PWM_SCALE
#define SOFT_PWM_SCALE 0
#endif

//===========================================================================
//=============================   functions      ============================
//===========================================================================
static void handler(int sig, siginfo_t *si, void *uc);
    
void updatePID()
{
#ifdef PIDTEMP
	int e;
  for(e = 0; e < EXTRUDERS; e++) { 
     temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;  
  }
#endif
#ifdef PIDTEMPBED
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;  
#endif
}
  
int getHeaterPower(int heater) {
	if (heater<0)
		return soft_pwm_bed;
  return soft_pwm[heater];
}

void manage_heater()
{
  float pid_input;
  float pid_output;

  if(temp_meas_ready != true)   //better readability
    return; 

  updateTemperaturesFromRawValues();

	int e;
  for(e = 0; e < EXTRUDERS; e++) 
  {

  #ifdef PIDTEMP
    pid_input = current_temperature[e];

    #ifndef PID_OPENLOOP
        pid_error[e] = target_temperature[e] - pid_input;
        if(pid_error[e] > PID_FUNCTIONAL_RANGE) {
          pid_output = BANG_MAX;
          pid_reset[e] = true;
        }
        else if(pid_error[e] < -PID_FUNCTIONAL_RANGE || target_temperature[e] == 0) {
          pid_output = 0;
          pid_reset[e] = true;
        }
        else {
          if(pid_reset[e] == true) {
            temp_iState[e] = 0.0;
            pid_reset[e] = false;
          }
          pTerm[e] = Kp * pid_error[e];
          temp_iState[e] += pid_error[e];
          temp_iState[e] = constrain(temp_iState[e], temp_iState_min[e], temp_iState_max[e]);
          iTerm[e] = Ki * temp_iState[e];

          //K1 defined in Configuration.h in the PID settings
          #define K2 (1.0-K1)
          dTerm[e] = (Kd * (pid_input - temp_dState[e]))*K2 + (K1 * dTerm[e]);
          pid_output = constrain(pTerm[e] + iTerm[e] - dTerm[e], 0, PID_MAX);
        }
        temp_dState[e] = pid_input;
    #else 
          pid_output = constrain(target_temperature[e], 0, PID_MAX);
    #endif //PID_OPENLOOP
    #ifdef PID_DEBUG
    SERIAL_ECHO_START(" PIDDEBUG ");
    SERIAL_ECHO(e);
    SERIAL_ECHO(": Input ");
    SERIAL_ECHO(pid_input);
    SERIAL_ECHO(" Output ");
    SERIAL_ECHO(pid_output);
    SERIAL_ECHO(" pTerm ");
    SERIAL_ECHO(pTerm[e]);
    SERIAL_ECHO(" iTerm ");
    SERIAL_ECHO(iTerm[e]);
    SERIAL_ECHO(" dTerm ");
    SERIAL_ECHOLN(dTerm[e]);  
    #endif //PID_DEBUG
  #else /* PID off */
    pid_output = 0;
    if(current_temperature[e] < target_temperature[e]) {
      pid_output = PID_MAX;
    }
  #endif

    // Check if temperature is within the correct range
    if((current_temperature[e] > minttemp[e]) && (current_temperature[e] < maxttemp[e])) 
    {
      soft_pwm[e] = (int)pid_output >> 1;
    }
    else {
      soft_pwm[e] = 0;
    }
  } // End extruder for loop

  #ifndef PIDTEMPBED
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();
  #endif
}

// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw, uint8_t e) {
#ifdef TEMP_SENSOR_1_AS_REDUNDANT
  if(e > EXTRUDERS)
#else
  if(e >= EXTRUDERS)
#endif
  {
      SERIAL_ERROR_START;
      SERIAL_ERROR((int)e);
      SERIAL_ERRORLNPGM(" - Invalid extruder number !");
      ikill();
  } 
  #ifdef HEATER_0_USES_MAX6675
    if (e == 0)
    {
      return 0.25 * raw;
    }
  #endif

  if(heater_ttbl_map[e] != NULL)
  {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if ((*tt)[i][0] > raw)
      {
        celsius = (*tt)[i-1][1] + 
          (raw - (*tt)[i-1][0]) * 
          (float)((*tt)[i][1] - (*tt)[i-1][1]) /
          (float)((*tt)[i][0] - (*tt)[i-1][0]);
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = (*tt)[i-1][1];

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{
		uint8_t e;
    for(e=0;e<EXTRUDERS;e++)
    {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
    }

    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}

void tp_init()
{
  int e;
  // Finish init of mult extruder arrays 
  for(e = 0; e < EXTRUDERS; e++) {
    // populate with the first value 
    maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    temp_iState_min[e] = 0.0;
    temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif //PIDTEMP
  }

  SET_OUTPUT(HEATER_0_PIN);

  #if defined(FAN_PIN) && (FAN_PIN > -1)  
    #ifdef FAST_PWM_FAN
    SET_OUTPUT(FAN_PIN);
    setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #ifdef FAN_SOFT_PWM
    SET_OUTPUT(FAN_PIN);
    soft_pwm_fan = fanSpeedSoftPwm / 2;
    #endif
  #endif  

  // --TOM--: we are reading temperature using I2C-ADC instead of analog pin.
  // Legitimately, i2c should be initialized here. But it is actually initialized 
  // in setup() because we want to cluster all board specific initializations
  
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millis interrupt
  /*OCR0B = 128;
  TIMSK0 |= (1<<OCIE0B);  */

  //Timer0 is already set up to generate a millisecond (1KHz) interrupt to 
  //update the millisecond counter reported by millis() in Arduino
  /* establish handler for timer signal */
  timerid = create_timer(handler);

  /* start the periodic timer */
  /* normal mode, so no matter what's in OCR0B, frequency remains 1KHz */
  set_time(timerid, 1, 1000000);
  enable_timer(timerid);

  // Wait for temperature measurement to settle
  delay(250);

#ifdef HEATER_0_MINTEMP
  minttemp[0] = HEATER_0_MINTEMP;
  while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    minttemp_raw[0] += OVERSAMPLENR;
#else
    minttemp_raw[0] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp[0] = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw[0] -= OVERSAMPLENR;
#else
    maxttemp_raw[0] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP
}

void setWatch() 
{  
#ifdef WATCH_TEMP_PERIOD
  for (int e = 0; e < EXTRUDERS; e++)
  {
    if(degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp[e] = degHotend(e);
      watchmillis[e] = millis();
    } 
  }
#endif 
}

void disable_heater()
{
	int i;
  for(i=0;i<EXTRUDERS;i++)
    setTargetHotend(0,i);
  //setTargetBed(0);
  #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
  target_temperature[0]=0;
  soft_pwm[0]=0;
   #if defined(HEATER_0_PIN) && HEATER_0_PIN > -1  
     WRITE(HEATER_0_PIN,LOW);
   #endif
  #endif
}

void max_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    fprintf(stderr, "Extruder %d switched off. MAXTEMP triggered !\n", e);
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

void min_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    fprintf(stderr, "Extruder %d switched off. MINTEMP triggered !\n", e);
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

// Timer 0 is shared with millis
//ISR(TIMER0_COMPB_vect)
static void
handler(int sig, siginfo_t *si, void *uc)
{
  //DEBUG_PRINT("temperature handler\n");
  //these variables are only accesible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned char temp_state = 0;
  static unsigned char pwm_count = (1 << SOFT_PWM_SCALE);
  static unsigned char soft_pwm_0;
  
  if(pwm_count == 0){
    soft_pwm_0 = soft_pwm[0];
    if(soft_pwm_0 > 0) 
      WRITE(HEATER_0_PIN,1);
    else
      WRITE(HEATER_0_PIN,0);
	
  }
  if(soft_pwm_0 < pwm_count)
    WRITE(HEATER_0_PIN,0);
  
  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;
  
  //--TOM-- single-ended channel 0
  const uint8_t cmd = 0x80;
  uint8_t res[2];
  uint16_t final_res;
  extern mraa_i2c_context temp_sensor;

  //--TOM-- modified based on Marlin firmware
  //read temperature from TEMP_0_PIN every 8 interrupts
  if (++temp_state % 8 == 0) {
    #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
    if (mraa_i2c_write_byte(temp_sensor, cmd) != MRAA_SUCCESS)
      errExit("mraa_i2c_write_byte");
    mraa_i2c_read(temp_sensor, &res[0], 2);
    final_res = res[0];
    final_res = final_res << 8;
    final_res |= res[1];
    DEBUG_PRINT("read word: %u\n", final_res);
    raw_temp_0_value += final_res >> 2;
    #endif
    temp_state = 0;
    temp_count++;
  }
  
  if(temp_count >= 16) { // 8 ms * 16 = 128ms.
    //Only update the raw values if they have been read. Else we could be updating them during reading.
    if (!temp_meas_ready) {
      current_temperature_raw[0] = raw_temp_0_value;
    }
    
    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;

#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw[0] <= maxttemp_raw[0]) {
#else
    if(current_temperature_raw[0] >= maxttemp_raw[0]) {
#endif
        max_temp_error(0);
    }
#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw[0] >= minttemp_raw[0]) {
#else
    if(current_temperature_raw[0] <= minttemp_raw[0]) {
#endif
        min_temp_error(0);
    }
  }
}

#ifdef PIDTEMP
// Apply the scale factors to the PID values


float scalePID_i(float i)
{
	return i*PID_dT;
}

float unscalePID_i(float i)
{
	return i/PID_dT;
}

float scalePID_d(float d)
{
    return d/PID_dT;
}

float unscalePID_d(float d)
{
	return d*PID_dT;
}

#endif //PIDTEMP


/* vi: set et sw=2 sts=2: */
