/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
//#include "temperature.h"
#include "language.h"
#include "speed_lookuptable.h"
#include <mraa.h>
#include "fastio.h"
#include "timer.h"
//#include "DAC.h"
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif


//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced

static int timerid;
#define ENABLE_STEPPER_DRIVER_INTERRUPT()   enable_timer(timerid)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  disable_timer(timerid)

//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y,
            counter_z,
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
#ifdef ADVANCE
  static long advance_rate, advance, final_advance = 0;
  static long old_advance = 0;
  static long e_steps[3];
#endif
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
bool abort_on_endstop_hit = false;
#endif
#ifdef MOTOR_CURRENT_PWM_XY_PIN
  int motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;
#endif

static bool old_x_min_endstop=false;
static bool old_x_max_endstop=false;
static bool old_y_min_endstop=false;
static bool old_y_max_endstop=false;
static bool old_z_min_endstop=false;
static bool old_z_max_endstop=false;

static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

//===========================================================================
//=============================functions         ============================
//===========================================================================

#undef FORCE_INLINE
#define FORCE_INLINE static __attribute__((always_inline)) inline

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
// XXX: to be tested!!!
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
intRes = (uint16_t)(((uint32_t)charIn1 * (uint32_t)intIn2) >> 16)
/*asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %A1, %A2 \n\t" \
"add %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r0 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (charIn1), \
"d" (intIn2) \
: \
"r26" \
)*/

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2) \
intRes = (uint16_t)(((uint64_t)longIn1 * (uint64_t)longIn2) >> 24)
/*asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"mov r27, r1 \n\t" \
"mul %B1, %C2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %C1, %C2 \n\t" \
"add %B0, r0 \n\t" \
"mul %C1, %B2 \n\t" \
"add %A0, r0 \n\t" \
"adc %B0, r1 \n\t" \
"mul %A1, %C2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %B2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %C1, %A2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %A2 \n\t" \
"add r27, r1 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r27 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (longIn1), \
"d" (longIn2) \
: \
"r26" , "r27" \
)*/

// Some useful constants

void checkHitEndstops()
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   SERIAL_ECHO_START;
   SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     SERIAL_ECHOPAIR(" X:",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
   }
   if(endstop_y_hit) {
     SERIAL_ECHOPAIR(" Y:",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
   }
   if(endstop_z_hit) {
     SERIAL_ECHOPAIR(" Z:",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
   }
   SERIAL_ECHOLN("");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
   if (abort_on_endstop_hit)
   {
     card.sdprinting = false;
     card.closefile();
     quickStop();
     setTargetHotend0(0);
     setTargetHotend1(0);
     setTargetHotend2(0);
   }
#endif
 }
}

void endstops_hit_on_purpose()
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void step_wait(){
  int8_t i;
   for(i=0; i < 6; i++){
   }
}


FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000);
  step_rate -= (F_CPU/500000); // Correct for minimal speed
  if(step_rate >= (8*256)){ // higher step rate
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)speed_lookuptable_fast[(unsigned char)(step_rate>>8)][1];
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0] - timer;
  }
  else { // lower step rates
    timer = (unsigned short)speed_lookuptable_slow[(unsigned char)(step_rate>>3)][0];
    timer -= (((unsigned short)speed_lookuptable_slow[(unsigned char)(step_rate>>3)][1] * (unsigned char)(step_rate & 0x0007))>>3);
  }
  if(timer < 100) { timer = 100; SERIAL_PROTOCOL(MSG_STEPPER_TOO_HIGH); SERIAL_PROTOCOLLN(step_rate); }//(20kHz this should never happen)
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {
  #ifdef ADVANCE
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
    old_advance = advance >>8;
  #endif
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  //OCR1A = acceleration_time;
  set_time(timerid, 0, 500 * acceleration_time);

//    SERIAL_ECHO_START;
//    SERIAL_ECHOPGM("advance :");
//    SERIAL_ECHO(current_block->advance/256.0);
//    SERIAL_ECHOPGM("advance rate :");
//    SERIAL_ECHO(current_block->advance_rate/256.0);
//    SERIAL_ECHOPGM("initial advance :");
//  SERIAL_ECHO(current_block->initial_advance/256.0);
//    SERIAL_ECHOPGM("final advance :");
//    SERIAL_ECHOLN(current_block->final_advance/256.0);

}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
//ISR(TIMER1_COMPA_vect)
static void handler(void)
{
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      current_block->busy = true;
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;

      #ifdef Z_LATE_ENABLE
        if(current_block->steps_z > 0) {
          enable_z();
          /*
          OCR1A = 2000; //1ms wait
          */
          return;
        }
      #endif
    }
    else {
        //OCR1A=2000; // 1kHz.
        // 1kHz
        //DEBUG_PRINT("no move\n");
        set_time(timerid, 0, 500 * 2000);
    }
  }

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;

    // Set the direction bits (X_AXIS=A_AXIS and Y_AXIS=B_AXIS for COREXY)
    if((out_bits & (1<<X_AXIS))!=0){
      WRITE(X_DIR_PIN, INVERT_X_DIR);
      count_direction[X_AXIS]=-1;
    } else{
      WRITE(X_DIR_PIN, !INVERT_X_DIR);
      count_direction[X_AXIS]=1;
    }
    if((out_bits & (1<<Y_AXIS))!=0){
      WRITE(Y_DIR_PIN, INVERT_Y_DIR);
      count_direction[Y_AXIS]=-1;
    } else{
      WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
      count_direction[Y_AXIS]=1;
    }
    if ((out_bits & (1<<Z_AXIS)) != 0) {   
      WRITE(Z_DIR_PIN,INVERT_Z_DIR);
      count_direction[Z_AXIS]=-1;
    } else {
      WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
      count_direction[Z_AXIS]=1;
    }

    // Set direction en check limit switches
    if (check_endstops) {
      if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
        #if defined(X_MIN_PIN) && X_MIN_PIN > -1
          bool x_min_endstop=(READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING);
          if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
            DEBUG_PRINT("X_MIN_ENDSTOP hit\n");
          }
          old_x_min_endstop = x_min_endstop;
        #endif
      }
      else { // +direction
        #if defined(X_MAX_PIN) && X_MAX_PIN > -1
          bool x_max_endstop=(READ(X_MAX_PIN) != X_MAX_ENDSTOP_INVERTING);
          if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
            DEBUG_PRINT("X_MAX_ENDSTOP hit\n");
          }
          old_x_max_endstop = x_max_endstop;
        #endif
      }

      if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
        #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
          bool y_min_endstop=(READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING);
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
            DEBUG_PRINT("Y_MIN_ENDSTOP hit\n");
          }
          old_y_min_endstop = y_min_endstop;
        #endif
      }
      else { // +direction
        #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
          bool y_max_endstop=(READ(Y_MAX_PIN) != Y_MAX_ENDSTOP_INVERTING);
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
            DEBUG_PRINT("Y_MAX_ENDSTOP hit\n");
          }
          old_y_max_endstop = y_max_endstop;
        #endif
      }

      if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
        #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
          bool z_min_endstop=(READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
            DEBUG_PRINT("Z_MIN_ENDSTOP hit\n");
          }
          old_z_min_endstop = z_min_endstop;
        #endif
      }
      else { // +direction
        #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
          bool z_max_endstop=(READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING);
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
            DEBUG_PRINT("Z_MAX_ENDSTOP hit\n");
          }
          old_z_max_endstop = z_max_endstop;
        #endif
      }
    }

    if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
      REV_E_DIR();
      count_direction[E_AXIS]=-1;
    }
    else { // +direction
      NORM_E_DIR();
      count_direction[E_AXIS]=1;
    }


    int8_t i;
    for(i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves)
      #ifndef AT90USB
      MSerial.checkRx(); // Check for serial chars.
      #endif

      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
        counter_x -= current_block->step_event_count;
        count_position[X_AXIS]+=count_direction[X_AXIS];   
        WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
        counter_y -= current_block->step_event_count;
        count_position[Y_AXIS]+=count_direction[Y_AXIS];
        WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
		  
      }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
        counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
        WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
        
      }

      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        WRITE_E_STEP(!INVERT_E_STEP_PIN);
        counter_e -= current_block->step_event_count;
        count_position[E_AXIS]+=count_direction[E_AXIS];
        WRITE_E_STEP(INVERT_E_STEP_PIN);
      }
      step_events_completed += 1;
      if(step_events_completed >= current_block->step_event_count) break;
    }
    // Calculare new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {

      MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;

      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      //OCR1A = timer;
      set_time(timerid, 0, 500 * timer);
      acceleration_time += timer;
    }
    else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {
      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);

      if(step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if(step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      //OCR1A = timer;
      set_time(timerid, 0, 500 * timer);
      deceleration_time += timer;
    }
    else {
      //OCR1A = OCR1A_nominal;
      set_time(timerid, 0, 500 * OCR1A_nominal);
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

   // DEBUG_PRINT("step_events_completed: %lu, step_event_count: %lu\n",
   //     step_events_completed, current_block->step_event_count);

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }
  }
}

void st_init()
{
  //digipot_init(); //Initialize Digipot Motor Current
  //microstep_init(); //Initialize Microstepping Pins

  //Initialize Dir Pins
  SET_OUTPUT(X_DIR_PIN);
  SET_OUTPUT(Y_DIR_PIN);
  SET_OUTPUT(Z_DIR_PIN);
  SET_OUTPUT(E0_DIR_PIN);

  //Initialize Enable Pins - steppers default to disabled.
  SET_OUTPUT(X_ENABLE_PIN);
  if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  SET_OUTPUT(Y_ENABLE_PIN);
  if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  SET_OUTPUT(Z_ENABLE_PIN);
  if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
  SET_OUTPUT(E0_ENABLE_PIN);
  if(!E_ENABLE_ON) WRITE(E0_ENABLE_PIN,HIGH);

  //endstops and pullups
  #if defined(X_MIN_PIN) && X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);
    #ifdef ENDSTOPPULLUP_XMIN
      WRITE(X_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);
    #ifdef ENDSTOPPULLUP_YMIN
      WRITE(Y_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);
    #ifdef ENDSTOPPULLUP_ZMIN
      WRITE(Z_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(X_MAX_PIN) && X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN);
    #ifdef ENDSTOPPULLUP_XMAX
      WRITE(X_MAX_PIN,HIGH);
    #endif
  #endif

  #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN);
    #ifdef ENDSTOPPULLUP_YMAX
      WRITE(Y_MAX_PIN,HIGH);
    #endif
  #endif

  #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z_MAX_PIN,HIGH);
    #endif
  #endif

  //Initialize Step Pins
  SET_OUTPUT(X_STEP_PIN);
  WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);
  disable_x();

  SET_OUTPUT(Y_STEP_PIN);
  WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
  disable_y();

  SET_OUTPUT(Z_STEP_PIN);
  WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
  disable_z();

  SET_OUTPUT(E0_STEP_PIN);
  WRITE(E0_STEP_PIN,INVERT_E_STEP_PIN);
  disable_e0();

/*
  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11);
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0);
  TCCR1A &= ~(3<<COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
*/

  timerid = create_timer(handler);

  /* start the one-shot timer */
  set_time(timerid, 0, 500 * 0x4000);

  ENABLE_STEPPER_DRIVER_INTERRUPT();

  enable_endstops(true); // Start with endstops active. After homing they can be disabled
  sei();
}

// Block until all buffered steps are executed
void st_synchronize()
{
  while( blocks_queued()) {
    manage_heater();
    manage_inactivity();
    //lcd_update();
  }
}

void st_set_position(const long x, const long y, const long z, const long e)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const long e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis)
{
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

#ifdef ENABLE_AUTO_BED_LEVELING
float st_get_position_mm(uint8_t axis)
{
  float steper_position_in_steps = st_get_position(axis);
  return steper_position_in_steps / axis_steps_per_unit[axis];
}
#endif  // ENABLE_AUTO_BED_LEVELING

void finishAndDisableSteppers()
{
  st_synchronize();
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
}

void quickStop()
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/* vi: set et sw=2 sts=2: */
