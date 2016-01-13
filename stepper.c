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
#include "temperature.h"


//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


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


// Block until all buffered steps are executed
void st_synchronize()
{
  while( blocks_queued()) {
    //manage_heater();
    //manage_inactivity();
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


//TODO
#define ENABLE_STEPPER_DRIVER_INTERRUPT() ;// TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() ;// TIMSK1 &= ~(1<<OCIE1A)


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

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
ISR(TIMER1_COMPA_vect)
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
          OCR1A = 2000; //1ms wait
          return;
        }
      #endif

//      #ifdef ADVANCE
//      e_steps[current_block->active_extruder] = 0;
//      #endif
    }
    else {
        OCR1A=2000; // 1kHz.
    }
  }

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;


    // Set the direction bits (X_AXIS=A_AXIS and Y_AXIS=B_AXIS for COREXY)
    if((out_bits & (1<<X_AXIS))!=0){
      WRITE(X_DIR_PIN, INVERT_X_DIR);
      count_direction[X_AXIS]=-1;
    }
    else{
      WRITE(X_DIR_PIN, !INVERT_X_DIR);
      count_direction[X_AXIS]=1;
    }
    if((out_bits & (1<<Y_AXIS))!=0){
      WRITE(Y_DIR_PIN, INVERT_Y_DIR);
	  
	  #ifdef Y_DUAL_STEPPER_DRIVERS
	    WRITE(Y2_DIR_PIN, !(INVERT_Y_DIR == INVERT_Y2_VS_Y_DIR));
	  #endif
	  
      count_direction[Y_AXIS]=-1;
    }
    else{
      WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
	  
	  #ifdef Y_DUAL_STEPPER_DRIVERS
	    WRITE(Y2_DIR_PIN, (INVERT_Y_DIR == INVERT_Y2_VS_Y_DIR));
	  #endif
	  
      count_direction[Y_AXIS]=1;
    }

    // Set direction en check limit switches
    #ifndef COREXY
    if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
    #else
    if ((((out_bits & (1<<X_AXIS)) != 0)&&(out_bits & (1<<Y_AXIS)) != 0)) {   //-X occurs for -A and -B
    #endif
      CHECK_ENDSTOPS
      {
        {
          #if defined(X_MIN_PIN) && X_MIN_PIN > -1
            bool x_min_endstop=(READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING);
            if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
              endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
              endstop_x_hit=true;
              step_events_completed = current_block->step_event_count;
            }
            old_x_min_endstop = x_min_endstop;
          #endif
        }
      }
    }
    else { // +direction
      CHECK_ENDSTOPS
      {
        {
          #if defined(X_MAX_PIN) && X_MAX_PIN > -1
            bool x_max_endstop=(READ(X_MAX_PIN) != X_MAX_ENDSTOP_INVERTING);
            if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
              endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
              endstop_x_hit=true;
              step_events_completed = current_block->step_event_count;
            }
            old_x_max_endstop = x_max_endstop;
          #endif
        }
      }
    }

    #ifndef COREXY
    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
    #else
    if ((((out_bits & (1<<X_AXIS)) != 0)&&(out_bits & (1<<Y_AXIS)) == 0)) {   // -Y occurs for -A and +B
    #endif
      CHECK_ENDSTOPS
      {
        #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
          bool y_min_endstop=(READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING);
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_min_endstop = y_min_endstop;
        #endif
      }
    }
    else { // +direction
      CHECK_ENDSTOPS
      {
        #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
          bool y_max_endstop=(READ(Y_MAX_PIN) != Y_MAX_ENDSTOP_INVERTING);
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_max_endstop = y_max_endstop;
        #endif
      }
    }

    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      WRITE(Z_DIR_PIN,INVERT_Z_DIR);
      
      #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_DIR_PIN,INVERT_Z_DIR);
      #endif

      count_direction[Z_AXIS]=-1;
      CHECK_ENDSTOPS
      {
        #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
          bool z_min_endstop=(READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_min_endstop = z_min_endstop;
        #endif
      }
    }
    else { // +direction
      WRITE(Z_DIR_PIN,!INVERT_Z_DIR);

      #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_DIR_PIN,!INVERT_Z_DIR);
      #endif

      count_direction[Z_AXIS]=1;
      CHECK_ENDSTOPS
      {
        #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
          bool z_max_endstop=(READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING);
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_max_endstop = z_max_endstop;
        #endif
      }
    }

    #ifndef ADVANCE
      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
        REV_E_DIR();
        count_direction[E_AXIS]=-1;
      }
      else { // +direction
        NORM_E_DIR();
        count_direction[E_AXIS]=1;
      }
    #endif //!ADVANCE



    for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves)
      #ifndef AT90USB
      MSerial.checkRx(); // Check for serial chars.
      #endif

      #ifdef ADVANCE
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        counter_e -= current_block->step_event_count;
        if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
          e_steps[current_block->active_extruder]--;
        }
        else {
          e_steps[current_block->active_extruder]++;
        }
      }
      #endif //ADVANCE

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
		  
		  #ifdef Y_DUAL_STEPPER_DRIVERS
			WRITE(Y2_STEP_PIN, !INVERT_Y_STEP_PIN);
		  #endif
		  
          counter_y -= current_block->step_event_count;
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
		  
		  #ifdef Y_DUAL_STEPPER_DRIVERS
			WRITE(Y2_STEP_PIN, INVERT_Y_STEP_PIN);
		  #endif
        }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
        
        #ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_STEP_PIN, !INVERT_Z_STEP_PIN);
        #endif

        counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
        WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
        
        #ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_STEP_PIN, INVERT_Z_STEP_PIN);
        #endif
      }

      #ifndef ADVANCE
        counter_e += current_block->steps_e;
        if (counter_e > 0) {
          WRITE_E_STEP(!INVERT_E_STEP_PIN);
          counter_e -= current_block->step_event_count;
          count_position[E_AXIS]+=count_direction[E_AXIS];
          WRITE_E_STEP(INVERT_E_STEP_PIN);
        }
      #endif //!ADVANCE
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
      OCR1A = timer;
      acceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance += advance_rate;
        }
        //if(advance > current_block->advance) advance = current_block->advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;

      #endif
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
      OCR1A = timer;
      deceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance -= advance_rate;
        }
        if(advance < final_advance) advance = final_advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;
      #endif //ADVANCE
    }
    else {
      OCR1A = OCR1A_nominal;
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }
  }
}

//TODO: signal for driving stepper

/* vi: set et sw=2 sts=2: */
