/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
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

#include "Marlin.h"
#include "Configuration.h"
#include "ConfigurationStore.h"
#include "Arduino.h"
#include "planner.h"
#include "stepper.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/sysinfo.h>

#include <fcntl.h>
#include <sys/stat.h>

//===========================================================================
//=============================public variables=============================
//===========================================================================
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
#ifdef DELTA
float endstop_adj[3]={0,0,0};
#endif
float min_pos[3] = { X_MIN_POS_DEFAULT, Y_MIN_POS_DEFAULT, Z_MIN_POS_DEFAULT };
float max_pos[3] = { X_MAX_POS_DEFAULT, Y_MAX_POS_DEFAULT, Z_MAX_POS_DEFAULT };
bool axis_known_position[3] = {false, false, false};

// Extruder offset
#if EXTRUDERS > 1
#ifndef DUAL_X_CARRIAGE
  #define NUM_EXTRUDER_OFFSETS 2 // only in XY plane
#else
  #define NUM_EXTRUDER_OFFSETS 3 // supports offsets in XYZ plane
#endif
float extruder_offset[NUM_EXTRUDER_OFFSETS][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
};
#endif
char active_extruder = 0;
int fanSpeed=0;
#ifdef SERVO_ENDSTOPS
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
  bool autoretract_enabled=true;
  bool retracted=false;
  float retract_length=3, retract_feedrate=17*60, retract_zlift=0.8;
  float retract_recover_length=0, retract_recover_feedrate=8*60;
#endif

#ifdef ULTIPANEL
  #ifdef PS_DEFAULT_OFF
    bool powersupply = false;
  #else
	  bool powersupply = true;
  #endif
#endif

#ifdef DELTA
float delta[3] = {0.0, 0.0, 0.0};
#endif

  
//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[MAX_CMD_SIZE];
//static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static bool comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;


#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

bool CooldownNoWait = true;
bool target_direction;
bool Stopped = false;

/*typedef struct line {
	char *gcode;
	char *params;
	line *next;
} line;*/

char *file_buf = NULL;
int file_size;
int current_read = 0;

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

//block all signals to imitate disabling interrupts
inline void
cli()
{
  sigprocmask(SIG_SETMASK, &global_interrupt, &old_global_interrupt);
}

inline void
sei()
{
  sigprocmask(SIG_SETMASK, &old_global_interrupt, NULL);
}

float code_value()
{
  return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer, code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void ikill()
{
  DEBUG_PRINT("kill\n");
  exit(1);
  //TODO
}

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return array##_P[axis]; }

#define XYZ_DYN_FROM_CONFIG(type, array, CONFIG)	\
static inline type array(int axis)			\
    { type temp[3] = { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };\
      return temp[axis];}

float base_min_pos[3] = { X_MIN_POS_DEFAULT, Y_MIN_POS_DEFAULT, Z_MIN_POS_DEFAULT };
float base_max_pos[3] = { X_MAX_POS_DEFAULT, Y_MAX_POS_DEFAULT, Z_MAX_POS_DEFAULT };
#ifdef ENABLE_AUTO_BED_LEVELING
float bed_level_probe_offset[3] = {X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT,
	Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT, -Z_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT};
#endif

XYZ_DYN_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_DYN_FROM_CONFIG(float, max_length, MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

/***********************************/
//int parse(char *line, line *l);

int main(int argc, char *argv[]) {

  if (argc != 2) {
    printf("Wrong number of arguments provided... Provided %d instead \
            of 2\nmarlin /path/to/file\n", argc);
    exit(1);
  }

  int file = setup(argv[1]);

  loop(file);

  return 0;
}

int setup(char *path)
{
  int file;
  struct stat s;

  if (stat(path, &s) == -1) {
    printf("Error stating %s\n", path);
    exit(1);
  }

  file_size = s.st_size;

  DEBUG_PRINT("opening file %s\n", path);
  if (!(file = open(path, O_RDONLY))) {
    perror("open file");
    exit(1);
  }

  file_buf = (char *)malloc(file_size);
  if (read(file, file_buf, file_size) < file_size) {
    printf("Error reading file\n");
    exit(1);
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  DEBUG_PRINT("loading data\n");
  Config_RetrieveSettings();

  //init timer
  DEBUG_PRINT("initializing timer\n");
  if(timeInit() < 0) {
    fprintf(stderr, "Failed to init timer\n");
    exit(-1);
  }

  //tp_init();    // Initialize temperature loop
  DEBUG_PRINT("initializing planner\n");
  plan_init();  // Initialize planner;
#ifdef DAC_STEPPER_CURRENT
  //dac_init(); //Initialize DAC to set stepper current
#endif

  //init board specific data
  DEBUG_PRINT("initializing board specific data\n");
  mraa_init();
  minnowmax_gpio_init();

  //init stepper
  DEBUG_PRINT("initializing stepper\n");
  st_init();

  //init global_interrupt
  sigfillset(&global_interrupt);
  sigemptyset(&old_global_interrupt);

  return file;
}

void loop(int fd)
{
  while (1) {
    if (get_command()) {
      DEBUG_PRINT("==========================================\n");
      DEBUG_PRINT("%s\n", cmdbuffer);
      process_commands();
    }

    //check heater every n milliseconds
    //TODO
    manage_heater();
    manage_inactivity();
    //checkHitEndstops();
  }
}

bool get_command()
{
  while (current_read < file_size) {
    serial_char = file_buf[current_read++];
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        return false;
      }
      cmdbuffer[serial_count] = 0;  //terminate string
      if(!comment_mode){
        comment_mode = false; //for new command

        if(strchr(cmdbuffer, 'N') != NULL)
        {
          printf("line number support needed\n");
          exit(1);
        }
        if((strchr(cmdbuffer, '*') != NULL))
        {
          printf("checksum support needed\n");
          exit(1);
        }

        if((strchr(cmdbuffer, 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer, 'G');
          switch((int)((strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
            }
            else {
              printf("MSG_ERR_STOPPED\n");
            }
            break;
          default:
            break;
          }
        }
        serial_count = 0; //clear buffer
      }
      else
      {
        if(serial_char == ';') comment_mode = true;
        if(!comment_mode) cmdbuffer[serial_count++] = serial_char;
      }
      serial_count = 0; //clear buffer
      return true;
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[serial_count++] = serial_char;
    }
  }

  return false;
}

static void axis_is_at_home(int axis) {
  current_position[axis] = base_home_pos(axis) + add_homeing[axis];
  min_pos[axis] =          base_min_pos[axis] + add_homeing[axis];
  max_pos[axis] =          base_max_pos[axis] + add_homeing[axis];
}

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {
    int axis_home_dir = home_dir(axis);

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
	

    // Engage Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        if (axis==Z_AXIS) {
          engage_z_probe();
        }
	    else
      #endif
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
      }
    #endif

    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
#ifdef DELTA
    feedrate = homing_feedrate[axis]/10;
#else
    feedrate = homing_feedrate[axis]/2 ;
#endif
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
#ifdef DELTA
    // retrace by the amount specified in endstop_adj
    if (endstop_adj[axis] * axis_home_dir < 0) {
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      destination[axis] = endstop_adj[axis];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
    }
#endif
    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_known_position[axis] = true;

    // Retract Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
      }
    #endif
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
    if (axis==Z_AXIS) retract_z_probe();
#endif
    
  }
}
void homing()
{
  int8_t i;
#ifdef ENABLE_AUTO_BED_LEVELING
  //TODO
  //plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
#endif //ENABLE_AUTO_BED_LEVELING


  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;
  previous_millis_cmd = millis();

  enable_endstops(true);

  for(i=0; i < NUM_AXIS; i++) {
    destination[i] = current_position[i];
  }
  feedrate = 0.0;


  home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

  if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
  {
    HOMEAXIS(X);
  }

  if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
    HOMEAXIS(Y);
  }

  if(code_seen(axis_codes[X_AXIS]))
  {
    if(code_value_long() != 0) {
      current_position[X_AXIS]=code_value()+add_homeing[0];
    }
  }

  if(code_seen(axis_codes[Y_AXIS])) {
    if(code_value_long() != 0) {
      current_position[Y_AXIS]=code_value()+add_homeing[1];
    }
  }
  
  #ifndef Z_SAFE_HOMING
  if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
  #if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
    // Set destination away from bed
    destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);   
    feedrate = max_feedrate[Z_AXIS];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
        destination[E_AXIS], feedrate, active_extruder);
    st_synchronize();
  #endif
    HOMEAXIS(Z);
  }
  #else                      // Z Safe mode activated. 
  if(home_all_axis) {
    destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - bed_level_probe_offset[0]);
    destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - bed_level_probe_offset[1]);
    // Set destination away from bed
    destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);  
    feedrate = XY_TRAVEL_SPEED;
    current_position[Z_AXIS] = 0;
	
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
        current_position[Z_AXIS], current_position[E_AXIS]);
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
        destination[E_AXIS], feedrate, active_extruder);
    st_synchronize();
    current_position[X_AXIS] = destination[X_AXIS];
    current_position[Y_AXIS] = destination[Y_AXIS];
    HOMEAXIS(Z);
  }
  // Let's see if X and Y are homed and probe is inside bed area.
  if(code_seen(axis_codes[Z_AXIS])) {
    if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
      && (current_position[X_AXIS]+bed_level_probe_offset[0] >= min_pos[0]) \
      && (current_position[X_AXIS]+bed_level_probe_offset[0] <= max_pos[0]) \
      && (current_position[Y_AXIS]+bed_level_probe_offset[1] >= min_pos[1]) \
      && (current_position[Y_AXIS]+bed_level_probe_offset[1] <= max_pos[1])) {

      current_position[Z_AXIS] = 0;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
          current_position[Z_AXIS], current_position[E_AXIS]);			  
      // Set destination away from bed
      destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);   
      feedrate = max_feedrate[Z_AXIS];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
          destination[E_AXIS], feedrate, active_extruder);
      st_synchronize();
      HOMEAXIS(Z);
    } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
      //TODO
      //LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      //SERIAL_ECHO_START;
      //SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
    } else {
      //TODO
      //LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
      //SERIAL_ECHO_START;
      //SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
      }
    }
  #endif

  if(code_seen(axis_codes[Z_AXIS])) {
    if(code_value_long() != 0) {
      current_position[Z_AXIS]=code_value()+add_homeing[2];
    }
  }
  #ifdef ENABLE_AUTO_BED_LEVELING
  if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
    current_position[Z_AXIS] += -bed_level_probe_offset[2];  //Add Z_Probe offset (the distance is negative)
  }
  #endif
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
      current_position[Z_AXIS], current_position[E_AXIS]);
  #ifdef ENDSTOPS_ONLY_FOR_HOMING
  enable_endstops(false);
  #endif

  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
  previous_millis_cmd = millis();
  endstops_hit_on_purpose();
}

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
#ifdef ENABLE_AUTO_BED_LEVELING
  float x_tmp, y_tmp, z_tmp, real_z;
#endif
  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      //DEBUG_PRINT("G0 or G1 found\n");
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
        //ClearToSend();
        return;
      }
      //break;
    case 28: //G28 Home all Axis one at a time
      homing();
    }
  }
}

void get_coordinates()
{
  int8_t i;
  bool seen[4]={false,false,false,false};
  for(i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
  #ifdef FWRETRACT
  if(autoretract_enabled)
  if( !(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
  {
    float echange=destination[E_AXIS]-current_position[E_AXIS];
    if(echange<-MIN_RETRACT) //retract
    {
      if(!retracted)
      {

      destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
      //if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
      float correctede=-echange-retract_length;
      //to generate the additional steps, not the destination is changed, but inversely the current position
      current_position[E_AXIS]+=-correctede;
      feedrate=retract_feedrate;
      retracted=true;
      }

    }
    else
      if(echange>MIN_RETRACT) //retract_recover
    {
      if(retracted)
      {
      //current_position[Z_AXIS]+=-retract_zlift;
      //if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
      float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
      current_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
      feedrate=retract_recover_feedrate;
      retracted=false;
      }
    }

  }
  #endif //FWRETRACT
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_cmd = millis();

  DEBUG_PRINT("destination: (%lf, %lf, %lf)\n",
      destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS]);
  DEBUG_PRINT("current: (%lf, %lf, %lf)\n",
      current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
  int8_t i;
  for(i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void manage_inactivity()
{
  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      ikill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }
  check_axes_activity();
}


/* vi: set et sw=2 sts=2: */
