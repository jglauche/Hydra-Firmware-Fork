
//Hydra_Firmware_fork

#include "configuration.h"
#include "pins.h"
#include "ThermistorTable.h"
//#include <cpwStepper.h> // custom library for stepper motor driving, comment this out if it is not required to reduce program size

// unit systems
#define INCHES 20
#define MILLIMETERS 21

// extruder direction
#define FORWARD 23
#define BACKWARD 24

// temperature control
#define PID_CONTROL 0 // 1 for on, 0 for off
#if PID_CONTROL
  #include <PID_Beta6.h> // library for PID temperature control
#endif

const boolean INVERT_X_DIR = false;
const int X_MIN_ENDSTOP = X_MIN_PIN;
const int X_MAX_ENDSTOP = X_MAX_PIN; 
const boolean INVERT_X_MIN_ENDSTOP = true; // true: low means switch is activated, false: high means switch is activated
const boolean INVERT_X_MAX_ENDSTOP = false; // true: low means switch is activated, false: high means switch is activated

const boolean INVERT_Y_DIR = false;
const int Y_MIN_ENDSTOP = Y_MIN_PIN;
const int Y_MAX_ENDSTOP = Y_MAX_PIN;
const boolean INVERT_Y_MIN_ENDSTOP = true; // true: low means switch is activated, false: high means switch is activated
const boolean INVERT_Y_MAX_ENDSTOP = false; // true: low means switch is activated, false: high means switch is activated

const boolean INVERT_Z_DIR = false;
const int Z_DIR_PIN[4] = {Z_DIR_PINN, Z_DIR_PINN, Z_DIR_PINN, Z_DIR_PINN};
const int Z_STEP_PIN[4] = {Z_STEP_PINN, Z_STEP_PINN, Z_STEP_PINN, Z_STEP_PINN};
const int Z_MIN_ENDSTOP = Z_MIN_PIN; // note that all the Z min endstops can be wired together to reduce pin usage
const int Z_MAX_ENDSTOP = Z_MAX_PIN; // note that all the Z min endstops can be wired together to reduce pin usage
const boolean INVERT_Z_MIN_ENDSTOP = true; // true: low means switch is activated, false: high means switch is activated
const boolean INVERT_Z_MAX_ENDSTOP = false; // true: low means switch is activated, false: high means switch is activated

const boolean INVERT_E_DIR = false;


// accessory declarations
const int KILLPIN_DIGITAL = 9;
const int KILLPIN_INTERRUPT = 8;

// toolhead specifications
const int NUMBER_OF_TOOLS = 1; // change this to the number of tools the machine has hooked up
// do not change first number (tool reference), syntax is tool #, X offset from tool 0, Y offset from tool 0, Z offset from tool 0 
float toolchanger[4][4] = {{0, 0, 0, 0}, // this is the tool that all other tools are referenced from
                           {1, 1.0, 1.0, 0}, // optional tool 1, x, y, and z offset from tool 0
                           {2, 2.0, 1.0, 0}, // optional tool 2, x, y, and z offset from tool 0
                           {3, 1.0, -1.0, 0}}; // optional tool 3, x, y, and z offset from tool 0
float x0, y0, z0;
int current_tool = 0; // default is tool 0
int current_tool_previous;
float toolhead_clearance = 0.5; // distance to move other tools away from active tool (for milling deep into parts, make this larger)


// logic and option variables
boolean min_software_endstops = false; // using hardware endstops would be better in case you miss steps somewhere
boolean max_software_endstops = true;
boolean comment_mode = false;
boolean absolute_positioning = true; // absolute coordinate system is used by default to get rid of round-off error
boolean extruding = false; // variable to know when we are supposed to be extruding or not
int extruder_dir = FORWARD; // variable to know when we are supposed to be extruding or not
boolean spindle_on = false;
int unit_system = MILLIMETERS; //INCHES; // Re-defined to MM by default
boolean acceleration = true;
boolean debugging = false; // use this to control using serial.print statements so they don't interfere with GUI auth

// speed, timing, and position variables
float absolute_x = 0.0, absolute_y = 0.0, absolute_z[4] = {0.0, 0.0, 0.0, 0.0}, absolute_e; // need to add absolute tracking (so G92 doesn't screw up endstops)
float current_x = 0.0, current_y = 0.0, current_z = 0.0, current_e = 0.0;
float destination_x, destination_y, destination_z = 0.0, destination_e = 0.0;
float x1, y1, z1, x2, y2, z2;
int x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take;


const float ZEROING_IPM = 30.0;  //Feedrate

float maximum_user_feedrate_IPM = MAXIMUM_FEEDRATE_IPM;
float feedrate_IPM;

float x_interval, y_interval, z_interval, e_interval; // for speed delay
unsigned long previous_micros, previous_micros_x, previous_micros_y, previous_micros_z, previous_micros_e;
float time_for_move;
float dwell_time;

// acceleration
const float MAX_ACCEL = 100000.0; // in/min^2
const float STARTING_SPEED = 20.0; // startup speed, test to make sure motors can instantly jump to this speed, this must be greater than zero to avoid divide by zero error
const int MAX_STEPS_TO_ACCEL = 200; // max size of array below, if you make the acceleration slower or the max speed higher, this number will need to increase
int accel_interval_scaler[MAX_STEPS_TO_ACCEL]; // using int data type for this so we only use 2 Bytes * MAX_STEPS_TO_ACCEL instead of 4 Bytes * MAX_STEPS_TO_ACCEL
float x_distance, y_distance, z_distance, total_distance;
float x_move_fraction, y_move_fraction, z_move_fraction;
float x_velocity, y_velocity, z_velocity;
float t_accel;
float x_accel, y_accel, z_accel;
float current_velocity;
float accel_interval;
int steps_to_accel;
// end speed, timing, and position variables


// comm variables
const int MAX_CMD_SIZE = 256;
char buffer[MAX_CMD_SIZE]; // buffer for serial commands
char serial_char; // value for each byte read in from serial comms
int serial_count = 0; // current length of command
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
int codenum;
// end comm variables


// motor drive types, see #defines above for options
// if you are using external motor drivers (ie use_dirstep == true) then make sure the modes below match what your external driver is set to
// drive_types: FULL 1, HALF 2, MICRO4 4, MICRO8 8, MICRO16 16, MICRO64 64
int x_drive_type = 2;
int y_drive_type = 2;
int z_drive_type = 2;
int e_drive_type = 2;

// setup all motors through library class if we need them
#if !X_USE_DIRSTEP
  cpwStepper cpwStepper_x(X_COIL1A, X_COIL1B, X_COIL2A, X_COIL2B, x_drive_type);
#endif
#if !Y_USE_DIRSTEP
  cpwStepper cpwStepper_y(Y_COIL1A, Y_COIL1B, Y_COIL2A, Y_COIL2B, y_drive_type);
#endif
#if !Z_USE_DIRSTEP
  cpwStepper cpwStepper_z(Z_COIL1A, Z_COIL1B, Z_COIL2A, Z_COIL2B, z_drive_type);
#endif
#if !E_USE_DIRSTEP and !E_USE_DC
  cpwStepper cpwStepper_e(E_COIL1A, E_COIL1B, E_COIL2A, E_COIL2B, e_drive_type);
#endif

#if E_USE_DC
  byte dc_extrusion_speed = 0;
#endif

// temperature control variables

int thermocouplePin = TEMP_0_PIN;
int heaterPin = HEATER_0_PIN;
#if PID_CONTROL
  double Setpoint, Input, Output;
  PID myPID(&Input, &Output, &Setpoint,2,5,1);
  float Setpoint_max = 550.0;
  float Setpoint_min = 30.0;
  //float current_temp;
  //int current_temp_int;
#endif



void setup() // initialization loop for pin types and initial values
{
  pinMode(KILLPIN_DIGITAL, INPUT);
  digitalWrite(KILLPIN_DIGITAL, HIGH); // enable 20k pullup resistor
  attachInterrupt(KILLPIN_INTERRUPT, kill_all, FALLING); // interrupt 2, digital pin 21
  
  pinMode(X_MIN_ENDSTOP, INPUT);
  digitalWrite(X_MIN_ENDSTOP, HIGH); // enable 20k pullup resistor
  pinMode(Y_MIN_ENDSTOP, INPUT);
  digitalWrite(Y_MIN_ENDSTOP, HIGH); // enable 20k pullup resistor
  pinMode(Z_MIN_ENDSTOP, INPUT);
  digitalWrite(Z_MIN_ENDSTOP, HIGH); // enable 20k pullup resistor
  
  pinMode(X_MAX_ENDSTOP, INPUT);
  digitalWrite(X_MAX_ENDSTOP, HIGH); // enable 20k pullup resistor
  pinMode(Y_MAX_ENDSTOP, INPUT);
  digitalWrite(Y_MAX_ENDSTOP, HIGH); // enable 20k pullup resistor
  pinMode(Z_MAX_ENDSTOP, INPUT);
  digitalWrite(Z_MAX_ENDSTOP, HIGH); // enable 20k pullup resistor
  
  #if X_USE_DIRSTEP
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(X_STEP_PIN, OUTPUT);
    if(X_ENABLE_PIN > 0){
      pinMode(X_ENABLE_PIN, OUTPUT);
      digitalWrite(X_ENABLE_PIN, !ENABLE_ON);
    }
  #else // we are using internal stepping
    cpwStepper_x.takestep(INVERT_X_DIR); // startup x motor
    cpwStepper_x.disable(); // used while debuggingg to save power, can delete later
  #endif
  
  #if Y_USE_DIRSTEP
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Y_STEP_PIN, OUTPUT);
    if(Y_ENABLE_PIN > 0){
      pinMode(Y_ENABLE_PIN, OUTPUT);
      digitalWrite(Y_ENABLE_PIN, !ENABLE_ON);
    } 
  #else // we are using internal stepping
    cpwStepper_y.takestep(INVERT_Y_DIR); // startup y motor
    cpwStepper_y.disable(); // used while debuggingg to save power, can delete later
  #endif
  
  #if Z_USE_DIRSTEP
    pinMode(Z_DIR_PINN, OUTPUT);
    pinMode(Z_STEP_PINN, OUTPUT);
    if(Z_ENABLE_PIN > 0){
      pinMode(Z_ENABLE_PIN, OUTPUT);
      digitalWrite(Z_ENABLE_PIN, !ENABLE_ON);
    }

  #else // we are using internal stepping
    cpwStepper_z.takestep(INVERT_Z_DIR); // startup z motor
    cpwStepper_z.disable(); // used while debuggingg to save power, can delete later
  #endif
  
  #if E_USE_DIRSTEP
    pinMode(E_DIR_PIN, OUTPUT);
    pinMode(E_STEP_PIN, OUTPUT);
    if(E_ENABLE_PIN > 0){
      pinMode(E_ENABLE_PIN, OUTPUT);
      digitalWrite(E_ENABLE_PIN, !ENABLE_ON);
    } 
  #elif !E_USE_DC // we are using internal stepping
    cpwStepper_e.takestep(INVERT_E_DIR); // startup extruder motor
    cpwStepper_e.disable(); // used while debuggingg to save power, can delete later
  #endif
  
#ifdef SPINDLEPIN
  pinMode(SPINDLEPIN, OUTPUT);
  digitalWrite(SPINDLEPIN, LOW);
#endif
#ifdef FANPIN
  pinMode(FANPIN, OUTPUT);
  digitalWrite(FANPIN, LOW);
#endif
#ifdef LEDPIN
  pinMode(LEDPIN, OUTPUT); // debuggingg, delete later
  digitalWrite(LEDPIN, LOW);
#endif
#ifdef LIGHTPIN
  pinMode(LIGHTPIN, OUTPUT);
  digitalWrite(LIGHTPIN, LOW);
#endif
#ifdef PS_ON_PIN
  pinMode(PS_ON_PIN, OUTPUT);
  digitalWrite(PS_ON_PIN, HIGH);
#endif

pinMode(X_ENABLE_PIN, OUTPUT);
digitalWrite(X_ENABLE_PIN, !ENABLE_ON);

pinMode(Y_ENABLE_PIN, OUTPUT);
digitalWrite(Y_ENABLE_PIN, !ENABLE_ON);

pinMode(Z_ENABLE_PIN, OUTPUT);
digitalWrite(Z_ENABLE_PIN, !ENABLE_ON);

pinMode(E_ENABLE_PIN, OUTPUT);
digitalWrite(E_ENABLE_PIN, !ENABLE_ON);




  Serial.begin(19200); // initialize serial interface for debugging
  
  clear_buffer();
  
  #if PID_CONTROL
    //initialize temperature variables
    Input = analogRead(thermocouplePin);
    Setpoint = 40;
    myPID.SetMode(AUTO); //turn the PID on
  #endif
  
  
  establishContact();  // send a byte to establish contact until receiver responds
}

void loop() // input loop, looks for manual input and then checks to see if and serial commands are coming in
{
  //previous_micros = micros(); // time at beginning of loop, for optimizationn
  digitalWrite(LEDPIN, HIGH); // turn on LED, debuggingg, delete later
  
  
  #if PID_CONTROL
    // handle PID temperature control
    Input = analogRead(thermocouplePin);
    myPID.Compute();
    analogWrite(heaterPin,Output);
  #endif
  
  
  get_command(); // check for Gcodes
  
  ////Serial.println(micros()-previous_micros); // used for code optimizationn
}



void get_command() // gets commands from serial connection and then calls up subsequent functions to deal with them
{
  if (Serial.available() > 0) // each time we see something
  {
    serial_char = Serial.read(); // read individual byte from serial connection
    
    if (serial_char == '\n') // end of a command character, could change to \n for carriage return if needed, but this is what works with Hydra GUI
    { 
      process_commands(buffer, serial_count);
      clear_buffer();
      comment_mode = false; // reset comment mode before each new command is processed
    }
    else // not end of command
    {
      if (serial_char == ';') // semicolon signifies start of comment
      {
        comment_mode = true;
      }
      
      if (comment_mode != true) // ignore if a comment has started
      {
        buffer[serial_count] = serial_char; // add byte to buffer string
        serial_count++;
        if (serial_count > MAX_CMD_SIZE) // overflow, dump and restart
        {
          if (debugging == true) Serial.println("command buffer overflow encountered");
          clear_buffer();
          Serial.flush();
        }
      }
    }
  }
}


void clear_buffer() // empties command buffer from serial connection
{
  serial_count = 0; // reset buffer placement
}


void startup() {
  zero_xy();
  
  zero_all_z();
  
  compensate_toolhead_length();
  
  set_toolhead_clearance();
}

  
void zero_xy() {
  // zero xy axes
  destination_x = -X_STROKE*2;
  destination_y = -Y_STROKE*2;
  destination_z = current_z;
  
  x_steps_to_take = abs(destination_x - current_x)*x_steps_per_inch;
  y_steps_to_take = abs(destination_y - current_y)*y_steps_per_inch;
  z_steps_to_take = 0;
  e_steps_to_take = 0;
  
  feedrate_IPM = ZEROING_IPM;
  
  // find time for move by moving largest distance at max speed
  if (x_steps_to_take >= y_steps_to_take) {
    time_for_move = (float)x_steps_to_take*(1/(x_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
  }
  else {
    time_for_move = (float)y_steps_to_take*(1/(y_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
  }
  x_interval = time_for_move/x_steps_to_take;
  y_interval = time_for_move/y_steps_to_take;
  
  linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
  
  current_x = 0.0;
  current_y = 0.0;
}


void zero_all_z() {
  // zero all z axes
  current_tool_previous = current_tool;
  
  for (int n = 0; n <= NUMBER_OF_TOOLS; n++) {
    
    current_tool = n;
    
    destination_x = current_x; // don't move
    destination_y = current_y; // don't move
    destination_z = -Z_STROKE*2;
    
    x_steps_to_take = 0;
    y_steps_to_take = 0;
    z_steps_to_take = abs(destination_z - current_z)*z_steps_per_inch;
    e_steps_to_take = 0;
    
    feedrate_IPM = ZEROING_IPM;
    
    time_for_move = (float)z_steps_to_take*(1/(z_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
    z_interval = time_for_move/z_steps_to_take;
    
    linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
  }
  
  current_tool = current_tool_previous;
  current_z = 0.0;
}


void compensate_toolhead_length() {
  // compensate for z tool lengths, at end all tools should be at same z height
  
  current_tool_previous = current_tool;
  
  for (int n = 0; n <= NUMBER_OF_TOOLS; n++) {
    
    current_tool = n;
    
    destination_x = current_x; // don't move
    destination_y = current_y; // don't move
    destination_z = toolchanger[0][3] - toolchanger[current_tool][3];
    
    x_steps_to_take = 0;
    y_steps_to_take = 0;
    z_steps_to_take = abs(destination_z - current_z)*z_steps_per_inch;
    e_steps_to_take = 0;
    
    feedrate_IPM = ZEROING_IPM;
    
    time_for_move = (float)z_steps_to_take*(1/(z_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
    z_interval = time_for_move/z_steps_to_take;
    
    linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
  }
  
  current_tool = current_tool_previous;
  
}


void set_toolhead_clearance() {
  // compensate for z tool lengths, at end all tools should be at same z height
  
  current_tool_previous = current_tool;
  
  for (int n = 0; n <= NUMBER_OF_TOOLS; n++) {
    if (n != current_tool_previous) {
      
      current_tool = n;
      
      destination_x = current_x; // don't move
      destination_y = current_y; // don't move
      destination_z = current_z + toolhead_clearance;
      
      x_steps_to_take = 0;
      y_steps_to_take = 0;
      z_steps_to_take = abs(destination_z - current_z)*z_steps_per_inch;
      e_steps_to_take = 0;
      
      feedrate_IPM = ZEROING_IPM;
      
      time_for_move = (float)z_steps_to_take*(1/(z_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
      z_interval = time_for_move/z_steps_to_take;
      
      linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
    }
  }
  
  current_tool = current_tool_previous;
  
}


void process_commands(char command[], int command_length) // deals with standardized input from serial connection
{
  command[command_length] = 0;
  Serial.println(command);
  if (command[0] == 'G') // G code
  {
    codenum = (int)strtod(&command[1], NULL);
    switch(codenum)
    {
      // look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
      
      case 0: // G0, rapid positioning
        if (debugging == true) Serial.println("G0: rapid positioing");
        get_coordinates(command);
        
        // move to the coordinates, don't worry about direction, decide that later
        x_steps_to_take = abs(destination_x - current_x)*x_steps_per_inch;
        y_steps_to_take = abs(destination_y - current_y)*y_steps_per_inch;
        z_steps_to_take = abs(destination_z - current_z)*z_steps_per_inch;
        if (extruding == true) {
          e_steps_to_take = sqrt((destination_x - current_x)*(destination_x - current_x) + (destination_y - current_y)*(destination_y - current_y) + (destination_z - current_z)*(destination_z - current_z))*e_steps_per_inch;;
        }
        else {
          e_steps_to_take = 0;
        }
        
        feedrate_IPM = maximum_user_feedrate_IPM;
        
        // find time for move by moving largest distance at max speed
        if (x_steps_to_take >= y_steps_to_take && x_steps_to_take >= z_steps_to_take) {
          time_for_move = (float)x_steps_to_take*(1/(x_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        else if (y_steps_to_take >= x_steps_to_take && y_steps_to_take >= z_steps_to_take) {
          time_for_move = (float)y_steps_to_take*(1/(y_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        else {
          time_for_move = (float)z_steps_to_take*(1/(z_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        x_interval = time_for_move/x_steps_to_take;
        y_interval = time_for_move/y_steps_to_take;
        z_interval = time_for_move/z_steps_to_take; // this should probably have it's own max, different from XY
        e_interval = time_for_move/e_steps_to_take;
        
        linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
        
        break;
      
      case 1: // G1, linear interpolation at specified speed
        if (debugging == true) Serial.println("G1: linear interpolation");
        get_coordinates(command);
        get_feedrate(command);
        
        // move to the coordinates, don't worry about direction, decide that later
        x_steps_to_take = abs(destination_x - current_x)*x_steps_per_inch;
        y_steps_to_take = abs(destination_y - current_y)*y_steps_per_inch;
        z_steps_to_take = abs(destination_z - current_z)*z_steps_per_inch;
        if (extruding == true) {
          e_steps_to_take = sqrt((destination_x - current_x)*(destination_x - current_x) + (destination_y - current_y)*(destination_y - current_y) + (destination_z - current_z)*(destination_z - current_z))*e_steps_per_inch;;
        }
        else {
          //e_steps_to_take = 0;
          e_steps_to_take = abs(destination_e - current_e)*e_steps_per_inch;
        }
        
        // find time for move by moving largest distance at defined feedrate
        if (x_steps_to_take >= y_steps_to_take && x_steps_to_take >= z_steps_to_take) {
          time_for_move = (float)x_steps_to_take*(1/(x_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        else if (y_steps_to_take >= x_steps_to_take && y_steps_to_take >= z_steps_to_take) {
          time_for_move = (float)y_steps_to_take*(1/(y_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        else {
          time_for_move = (float)z_steps_to_take*(1/(z_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        x_interval = time_for_move/x_steps_to_take;
        y_interval = time_for_move/y_steps_to_take;
        z_interval = time_for_move/z_steps_to_take; // this should probably have it's own max, different from XY
        e_interval = time_for_move/e_steps_to_take;
        
        // set debugging to true above and use Arduino serial monitor to send commands followed with a colon
        // ie  "G1 X1 Y1 F10 :" and then you can viewed the returned data
        if (debugging == true) {
          Serial.println();
          Serial.print("x/in: ");
          Serial.print(x_steps_per_inch);
          Serial.print(" y/in: ");
          Serial.print(y_steps_per_inch);
          Serial.print(" z/in: ");
          Serial.print(z_steps_per_inch);
          Serial.print(" e/in: ");
          Serial.println(e_steps_per_inch);
          
          Serial.print("Time for move: ");
          Serial.println(time_for_move);
          
          Serial.print("X dest: ");
          Serial.print(destination_x);
          Serial.print(" X steps: ");
          Serial.print(x_steps_to_take);
          Serial.print(" X int: ");
          Serial.println(x_interval);
          
          Serial.print("Y dest: ");
          Serial.print(destination_y);
          Serial.print(" Y steps: ");
          Serial.print(y_steps_to_take);
          Serial.print(" Y int: ");
          Serial.println(y_interval);
          
          Serial.print("Z dest: ");
          Serial.print(destination_z);
          Serial.print(" Z steps: ");
          Serial.print(z_steps_to_take);
          Serial.print(" Z int: ");
          Serial.println(z_interval);
          
          Serial.print("E steps: ");
          Serial.print(e_steps_to_take);
          Serial.print(" E int: ");
          Serial.println(e_interval);
        }
        
        linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
        
        break;
        
      case 2: // G2, CW circular interpolation
        
        break;
      case 3: // G3, CCW circular interpolation
        
        break;
      case 4: // G4, dwell
        strchr_pointer = strchr(buffer, 'P'); // P parameter holds delay time in seconds
        if (strchr_pointer != NULL) // We found a P value
        {
          dwell_time = (float)strtod(&command[strchr_pointer - command + 1], NULL);
          if (dwell_time > 0)
          {
            //Serial.print("G4: dwell, pausing for ");
            //Serial.print(dwell_time);
            //Serial.println(" ms");
            delay(dwell_time); // delay function takes input in ms
          }
          else
          {
            //Serial.println("Invalid dwell time (G4), use P parameter for delay time (sec)");
          }
        }
        
        break;
      case 20: // G20, inch unit system
        unit_system = INCHES;
        break;
      case 21: // G21, millimeter unit system
        unit_system = MILLIMETERS;
        break;
      case 30: // G30, set reference 1
        x1 = current_x;
        y1 = current_y;
        z1 = current_z;
        break;
      case 31: // G31, return to reference 1
        destination_x = x1;
        destination_y = y1;
        destination_z = z1;
        
        // move to the coordinates, don't worry about direction, decide that later
        x_steps_to_take = abs(destination_x - current_x)*x_steps_per_inch;
        y_steps_to_take = abs(destination_y - current_y)*y_steps_per_inch;
        z_steps_to_take = abs(destination_z - current_z)*z_steps_per_inch;
        if (extruding == true) {
          e_steps_to_take = sqrt((destination_x - current_x)*(destination_x - current_x) + (destination_y - current_y)*(destination_y - current_y) + (destination_z - current_z)*(destination_z - current_z))*e_steps_per_inch;;
        }
        else {
          e_steps_to_take = 0;
        }
        
        feedrate_IPM = maximum_user_feedrate_IPM;
        
        // find time for move by moving largest distance at max speed
        if (x_steps_to_take >= y_steps_to_take && x_steps_to_take >= z_steps_to_take) {
          time_for_move = (float)x_steps_to_take*(1/(x_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        else if (y_steps_to_take >= x_steps_to_take && y_steps_to_take >= z_steps_to_take) {
          time_for_move = (float)y_steps_to_take*(1/(y_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        else {
          time_for_move = (float)z_steps_to_take*(1/(z_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        x_interval = time_for_move/x_steps_to_take;
        y_interval = time_for_move/y_steps_to_take;
        z_interval = time_for_move/z_steps_to_take; // this should probably have it's own max, different from XY
        e_interval = time_for_move/e_steps_to_take;
        
        linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
        
        break;
      case 32: // G32, set reference 2
        x2 = current_x;
        y2 = current_y;
        z2 = current_z;
        break;
      case 33: // G33, return to reference 2
        destination_x = x2;
        destination_y = y2;
        destination_z = z2;
        
        // move to the coordinates, don't worry about direction, decide that later
        x_steps_to_take = abs(destination_x - current_x)*x_steps_per_inch;
        y_steps_to_take = abs(destination_y - current_y)*y_steps_per_inch;
        z_steps_to_take = abs(destination_z - current_z)*z_steps_per_inch;
        if (extruding == true) {
          e_steps_to_take = sqrt((destination_x - current_x)*(destination_x - current_x) + (destination_y - current_y)*(destination_y - current_y) + (destination_z - current_z)*(destination_z - current_z))*e_steps_per_inch;;
        }
        else {
          e_steps_to_take = 0;
        }
        
        feedrate_IPM = maximum_user_feedrate_IPM;
        
        // find time for move by moving largest distance at max speed
        if (x_steps_to_take >= y_steps_to_take && x_steps_to_take >= z_steps_to_take) {
          time_for_move = (float)x_steps_to_take*(1/(x_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        else if (y_steps_to_take >= x_steps_to_take && y_steps_to_take >= z_steps_to_take) {
          time_for_move = (float)y_steps_to_take*(1/(y_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        else {
          time_for_move = (float)z_steps_to_take*(1/(z_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
        }
        x_interval = time_for_move/x_steps_to_take;
        y_interval = time_for_move/y_steps_to_take;
        z_interval = time_for_move/z_steps_to_take; // this should probably have it's own max, different from XY
        e_interval = time_for_move/e_steps_to_take;
        
        linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
        
        break;
      case 90: // G90, absolute coordinate system
        // currently everything is defaulted to this and all calculations use this system
        //
        // all movements are relative to absolute coordinate datums, round-off errors do not accumulate
        // and mid program starts are possible
        //Serial.println("G90: absolute coordinate system selected");
        absolute_positioning = true;
        break;
      case 91: // G91, incremental coordinate system
        // with incremental programming the control moves the machine to the next point relative
        // to the present position.  This can cause accumulated error and you can't start mid program
        // wherever the machine moves to is assumed to be equal to the final destination
        //Serial.println("G91: incremental coordinate system selected");
        absolute_positioning = false;
        break;
      case 92: // G92, program absoulte zero, sets current position as home
        // note that doing this with software endstops enabled will decrease available workspace
        get_coordinates(command);
        
        current_x = destination_x;
        current_y = destination_y;
        current_z = destination_z;
        
        break;
    }
  }
  
  if (command[0] == 'M') // M code
  {
    codenum = (int)strtod(&command[1], NULL);
    switch(codenum)
    {
      // look here for descriptions of mcodes: http://linuxcnc.org/handbook/gcode/g-code.html
      // and reprap specific mcodes: http://reprap.org/bin/view/Main/MCodeReference
      
      case 0: // M0, program stop
        kill_all(); // this will turn everything off
        break;
      case 2: // M2, program end
        kill_all(); // this will turn everything off
        break;
      case 3: // M3, turn spindle on, CW
        if (debugging == true) Serial.println("M3: turn spindle on, CW");
	  #ifdef SPINDLEPIN
          digitalWrite(SPINDLEPIN, HIGH);
          spindle_on = true;
        #endif
        break;
      case 4: // M4, turn spindle on, CCW
        if (debugging == true) Serial.println("M4: turn spindle on, CCW");
	  #ifdef SPINDLEPIN
          digitalWrite(SPINDLEPIN, HIGH);
          spindle_on = true;
        #endif
        break;
      case 5: // M5, turn spindle off
        if (debugging == true) Serial.println("M5: turn spindle off");
        #ifdef SPINDLEPIN
          digitalWrite(SPINDLEPIN, LOW);
          spindle_on = false;
        #endif
        break;
      case 6: // M6, tool change
        
        break;
      case 7: // M7, mist coolant on
        
        break;
      case 8: // M8, flood coolant on
        
        break;
      case 9: // M9, turn off mist and flood coolants
        
        break;
      case 44: // M44, enable min software endstops
        min_software_endstops = true;
        if (debugging == true) Serial.println("M44: min software endstops enabled");
        break;
      case 45: // M45, disable min software endstops
        min_software_endstops = false;
        if (debugging == true) Serial.println("M45: min software endstops disabled");
        break;
      case 46: // M46, enable max software endstops
        max_software_endstops = true;
        if (debugging == true) Serial.println("M46: max software endstops enabled");
        break;
      case 47: // M47, disable max software endstops
        max_software_endstops = false;
        if (debugging == true) Serial.println("M47: max software endstops disabled");
        break;
      case 48: // M48, enable speed override
        
        strchr_pointer = strchr(buffer, 'F'); // F parameter holds new max speed
        if (strchr_pointer != NULL) // We found a F value
        {
          maximum_user_feedrate_IPM = (float)strtod(&command[strchr_pointer - command + 1], NULL);
          if (unit_system == MILLIMETERS) {
            maximum_user_feedrate_IPM = maximum_user_feedrate_IPM / 25.4;
          }
          
          if (maximum_user_feedrate_IPM > 0)
          {
            if (debugging == true) {
              Serial.print("M48: speed override, max feedrate set to ");
              Serial.print(maximum_user_feedrate_IPM);
              Serial.println("IPM");
            }
          }
          else
          {
            if (debugging == true) Serial.println("Invalid speed override (M48), use F parameter for new max feedrate (IPM)");
          }
        }
        
        break;
      case 49: // M49, disable speed override
        maximum_user_feedrate_IPM = MAXIMUM_FEEDRATE_IPM;
        
        if (debugging == true) {
          Serial.print("M49: speed override disabled, max feedrate returned to ");
          Serial.print(maximum_user_feedrate_IPM);
          Serial.println("IPM");
        }
        
        break;
      case 50: // M50, change stepping mode
        // find X coordinate
        strchr_pointer = strchr(buffer, 'X');
        if (strchr_pointer != NULL) // We found an X value
        {
          int old_drive_type = x_drive_type;
          x_drive_type = (int)strtod(&command[strchr_pointer - command + 1], NULL);
          if (x_drive_type == 1 || x_drive_type == 2 || x_drive_type == 4 || x_drive_type == 8 || x_drive_type == 16 || x_drive_type == 32 || x_drive_type == 64) { // it is a valid type
            x_steps_per_inch = x_steps_per_inch*x_drive_type/old_drive_type;
            #if !X_USE_DIRSTEP
              cpwStepper_x.steptype(x_drive_type);
            #endif
          }
        }
        
        // find Y coordinate
        strchr_pointer = strchr(buffer, 'Y');
        if (strchr_pointer != NULL) // We found a Y value
        {
          int old_drive_type = y_drive_type;
          y_drive_type = (int)strtod(&command[strchr_pointer - command + 1], NULL);
          if (y_drive_type == 1 || y_drive_type == 2 || y_drive_type == 4 || y_drive_type == 8 || y_drive_type == 16 || y_drive_type == 32 || y_drive_type == 64) { // it is a valid type
            y_steps_per_inch = y_steps_per_inch*y_drive_type/old_drive_type;
            #if !Y_USE_DIRSTEP
              cpwStepper_y.steptype(y_drive_type);
            #endif
          }
        }
        
        // find Z coordinate
        strchr_pointer = strchr(buffer, 'Z');
        if (strchr_pointer != NULL) // We found a Z value
        {
          int old_drive_type = z_drive_type;
          z_drive_type = (int)strtod(&command[strchr_pointer - command + 1], NULL);
          if (z_drive_type == 1 || z_drive_type == 2 || z_drive_type == 4 || z_drive_type == 8 || z_drive_type == 16 || z_drive_type == 32 || z_drive_type == 64) { // it is a valid type
            z_steps_per_inch = z_steps_per_inch*z_drive_type/old_drive_type;
            #if !Z_USE_DIRSTEP
              cpwStepper_z.steptype(z_drive_type);
            #endif
          }
        }
        
        // find E coordinate
        strchr_pointer = strchr(buffer, 'E');
        if (strchr_pointer != NULL) // We found an E value
        {
          int old_drive_type = e_drive_type;
          e_drive_type = (int)strtod(&command[strchr_pointer - command + 1], NULL);
          if (e_drive_type == 1 || e_drive_type == 2 || e_drive_type == 4 || e_drive_type == 8 || e_drive_type == 16 || e_drive_type == 32 || e_drive_type == 64) { // it is a valid type
            e_steps_per_inch = e_steps_per_inch*e_drive_type/old_drive_type;
            #if !E_USE_DIRSTEP and !E_USE_DC
              cpwStepper_x.steptype(x_drive_type);
            #endif
          }
        }
        break;
      case 60: // M60, turn acceleration on
        acceleration = true;
        break;
      case 61: // M61, turn acceleration off
        acceleration = false;
        break;
      case 70: // M70, debugging mode turned on
        debugging = true;
        break;
      case 71: // M71, debugging mode turned off
        debugging = false;
        break;
      case 80: // M80, Custom code to turn on ATX PSU
        if (debugging == true) Serial.println("M80: turn on ATX PSU");
        #ifdef PS_ON_PIN
          digitalWrite(PS_ON_PIN, LOW);
        #endif
        break;
      case 81: // M80, Custom code to turn off ATX PSU
        if (debugging == true) Serial.println("M81: turn off ATX PSU");
        #ifdef PS_ON_PIN
          digitalWrite(PS_ON_PIN, HIGH);
        #endif
        break;
      case 101: // M101, reprap specific, extruder on, forward
        extruding = true;
        extruder_dir = FORWARD;
#if E_USE_DC
        digitalWrite(E_DIR_PIN, extruder_dir);
        analogWrite(E_STEP_PIN, dc_extrusion_speed);
#endif
        break;
      case 102: // M102, reprap specific, extruder on, reverse
        extruding = true; // need to look at how reprap and skeinforge handle reverse extrusion, command turns it on, but then is a dwell command given to actually pull filament back up??
        extruder_dir = BACKWARD;
#if E_USE_DC
        digitalWrite(E_DIR_PIN, extruder_dir);
        analogWrite(E_STEP_PIN, dc_extrusion_speed);
#endif
        break;
      case 103: // M103, reprap specific, extruder off
        extruding = false;
#if E_USE_DC
        analogWrite(E_STEP_PIN, 0);
#endif

        break;
      case 104: // M104, reprap specific, set extruder temperature
        
        #if PID_CONTROL
          // P parameter contains temperature in degrees C, ie M104 P230 is set temp to 230C
          if (debugging == true) Serial.println("M104: set extruder temp");
          
          // find P coordinate
          strchr_pointer = strchr(buffer, 'S');
          if (strchr_pointer != NULL) // We found a S value
          {
            Setpoint = temp2analog( ((float)strtod(&command[strchr_pointer - command + 1], NULL)) ); // convert from deg C to setpoint (0-1023)
            Serial.print("temp2analog: ");
            Serial.println(Setpoint);
            if (Setpoint > Setpoint_max) { // it is a valid type
              Setpoint = Setpoint_max;
            }
            else if (Setpoint < Setpoint_min) {
              Setpoint = Setpoint_min;
            }
          }
        #endif
        
        break;
      case 105: // M105, reprap specific, get extruder temperature
          if (debugging == true) Serial.println("M105: get extruder temp");
          Serial.print("T:");
          Serial.println(  analog2temp( analogRead(thermocouplePin) )  );
      //    Serial.print("analogRead: ");
      //    Serial.println( analogRead(thermocouplePin) );
        break;
      case 106: // M106, reprap specific, turn fan on
        if (debugging == true) Serial.println("M106: turn fan on");
        #ifdef FANPIN
          digitalWrite(FANPIN, HIGH);
        #endif
        break;
      case 107: // M107, reprap specific, turn fan off
        if (debugging == true) Serial.println("M107: turn fan off");
        #ifdef FANPIN
          digitalWrite(FANPIN, LOW);
        #endif
        break;
      case 108: // extruder speed for DC motor
#if E_USE_DC
          strchr_pointer = strchr(buffer, 'S');
          if (strchr_pointer != NULL) // We found a S value
          {
            dc_extrusion_speed = (byte)strtod(&command[strchr_pointer - command + 1], NULL);  
          }
#endif
        break;
      case 201: // M201, turn light on
        #ifdef LIGHTPIN
          if (debugging == true) Serial.println("M201: turn light on");
          digitalWrite(LIGHTPIN, HIGH);
        #endif
        break;
      case 202: // M202, turn light off
        #ifdef LIGHTPIN
          if (debugging == true) Serial.println("M202: turn light off");
          digitalWrite(LIGHTPIN, LOW);
        #endif
        break;
        
    }
  }
  
  
  else if (command[0] == 'T') // T code
  {
    codenum = (int)strtod(&command[1], NULL);
    switch(codenum)
    {
      // codes for managing multiple toolheads
      
      case 0: // T0, use tool 0
        switch_tool(0);
        break;
      case 1: // T1, use tool 1
        switch_tool(1);
        break;
      case 2: // T2, use tool 2
        switch_tool(2);
        break;
      case 3: // T3, use tool 3
        switch_tool(3);
        break;
      case 10: // T10, home all z axes
        zero_all_z();
        break;
      case 11: // T11, level all z axes
        compensate_toolhead_length();
        break;
      case 14: // T14, set desired active toolhead clearance
        // P parameter contains desired clearance
        
        // find P coordinate
        strchr_pointer = strchr(buffer, 'P');
        if (strchr_pointer != NULL) // We found a P value
        {
          toolhead_clearance = ((float)strtod(&command[strchr_pointer - command + 1], NULL));
          if (unit_system == MILLIMETERS) {
            toolhead_clearance = toolhead_clearance / 25.4;
          }
        }
        break;
      case 15: // T15, move all non active toolheads to achieve appropriate active toolhead clearance
        set_toolhead_clearance();
        break;
      case 20: // T20, move all non active toolheads to achieve appropriate active toolhead clearance
        startup();
        break;
      
    } 
  }
  
  
  // other code types?
  
  
  // done processing commands
  if (Serial.available() <= 0) {
    Serial.println("ok");   // send a capital A
  }
  
}



void switch_tool(int new_tool) {
  x0 = current_x; // storage of original positions
  y0 = current_y;
  z0 = current_z;
  
  destination_x = current_x + (toolchanger[new_tool][1] - toolchanger[current_tool][1]);
  destination_y = current_y + (toolchanger[new_tool][2] - toolchanger[current_tool][2]);
  destination_z = current_z + (toolchanger[new_tool][3] - toolchanger[current_tool][3]);
  
   // move to the coordinates, don't worry about direction, decide that later
  x_steps_to_take = abs(destination_x - current_x)*x_steps_per_inch;
  y_steps_to_take = abs(destination_y - current_y)*y_steps_per_inch;
  z_steps_to_take = abs(destination_z - current_z)*z_steps_per_inch;
  e_steps_to_take = 0;
  
  feedrate_IPM = maximum_user_feedrate_IPM;
  
  // find time for move by moving largest distance at max speed
  if (x_steps_to_take >= y_steps_to_take && x_steps_to_take >= z_steps_to_take) {
    time_for_move = (float)x_steps_to_take*(1/(x_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
  }
  else if (y_steps_to_take >= x_steps_to_take && y_steps_to_take >= z_steps_to_take) {
    time_for_move = (float)y_steps_to_take*(1/(y_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
  }
  else {
    time_for_move = (float)z_steps_to_take*(1/(z_steps_per_inch*feedrate_IPM/60000000)); // time in microseconds
  }
  x_interval = time_for_move/x_steps_to_take;
  y_interval = time_for_move/y_steps_to_take;
  z_interval = time_for_move/z_steps_to_take; // this should probably have it's own max, different from XY
  
  linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
  
  current_x = x0; // restore original positioning
  current_y = y0;
  
  current_tool = new_tool;
}


void get_coordinates(char command[])
{
  // initialize destination to current position in case we don't get new coordinates for some axes
  destination_x = current_x;
  destination_y = current_y;
  destination_z = current_z;
  destination_e = current_e;
  
  // find X coordinate
  strchr_pointer = strchr(buffer, 'X');
  if (strchr_pointer != NULL) // We found an X value
  {
    destination_x = (float)strtod(&command[strchr_pointer - command + 1], NULL);
    if (unit_system == MILLIMETERS) {
      destination_x = destination_x / 25.4;
    }
  }
  
  // find Y coordinate
  strchr_pointer = strchr(command, 'Y');
  if (strchr_pointer != NULL) // We found a Y value
  {
    destination_y = (float)strtod(&command[strchr_pointer - command + 1], NULL);
    if (unit_system == MILLIMETERS) {
      destination_y = destination_y / 25.4;
    }
  }
  
  // find E coordinate
  strchr_pointer = strchr(buffer, 'E');
  if (strchr_pointer != NULL) // We found an X value
  {
    destination_e = (float)strtod(&command[strchr_pointer - command + 1], NULL);
    if (unit_system == MILLIMETERS) {
      destination_e = destination_e / 25.4;
    }
  }
  
  // find Z coordinate
  strchr_pointer = strchr(command, 'Z');
  if (strchr_pointer != NULL) // We found a Z value
  {
    destination_z = (float)strtod(&command[strchr_pointer - command + 1], NULL);
    if (unit_system == MILLIMETERS) {
      destination_z = destination_z / 25.4;
    }
  }
  
  // handle software endstops
  if (min_software_endstops == true)
  {
    // if corrdinates are < 0, set them to 0 to avoid going too far (software endstop)
    // if software endstops are disabled just let the machine keep moving until it hits hardware endstops
    if (destination_x < 0) {
      destination_x = 0.0;
    }
    if (destination_y < 0) {
      destination_y = 0.0;
    }
    if (destination_z < 0) {
      destination_z = 0.0;
    }
  }
  
  if (max_software_endstops == true)
  {
    // if corrdinates are < 0, set them to 0 to avoid going too far (software endstop)
    // if software endstops are disabled just let the machine keep moving until it hits hardware endstops
    if (destination_x > X_STROKE) {
      destination_x = X_STROKE;
    }
    if (destination_y > Y_STROKE) {
      destination_y = Y_STROKE;
    }
    if (destination_z > Z_STROKE) {
      destination_z = Z_STROKE;
    }
  }
  
}


void get_feedrate(char command[])
{
  // initialize feedrate to default feed just in case we don't get a speed parameter
  feedrate_IPM = DEFAULT_FEEDRATE_IPM;
  
  // find F parameter
  strchr_pointer = strchr(buffer, 'F');
  if (strchr_pointer != NULL) // We found an F value
  {
    feedrate_IPM = (float)strtod(&command[strchr_pointer - command + 1], NULL);
    if (unit_system == MILLIMETERS) {
      feedrate_IPM = feedrate_IPM / 25.4;
    }
    if (feedrate_IPM > maximum_user_feedrate_IPM) { // if it wants us to go too fast
      feedrate_IPM = maximum_user_feedrate_IPM;
    }
  }
  
}




void get_acceleration_parameters(int x_steps_remaining, int y_steps_remaining, int z_steps_remaining, int e_steps_remaining)
{
  x_distance = abs(destination_x - current_x);
  y_distance = abs(destination_y - current_y);
  z_distance = abs(destination_z - current_z);
  total_distance = sqrt(x_distance*x_distance + y_distance*y_distance + z_distance*z_distance);
  
  x_move_fraction = x_distance/total_distance;
  y_move_fraction = y_distance/total_distance;
  z_move_fraction = z_distance/total_distance;
  
  x_velocity = feedrate_IPM*x_move_fraction;
  y_velocity = feedrate_IPM*y_move_fraction;
  z_velocity = feedrate_IPM*z_move_fraction;
  
  if (x_distance >= y_distance && x_distance >= z_distance) {
    t_accel = x_velocity/MAX_ACCEL;
  }
  else if (y_distance >= x_distance && y_distance >= z_distance) {
    t_accel = y_velocity/MAX_ACCEL;
  }
  else {
    t_accel = z_velocity/MAX_ACCEL;
  }
  
  x_accel = x_velocity/t_accel;
  y_accel = y_velocity/t_accel;
  z_accel = z_velocity/t_accel;
  
  if (debugging == true) {
    Serial.println();
    Serial.print("x steps remaining: "); Serial.println(x_steps_remaining);
    Serial.print("y steps remaining: "); Serial.println(y_steps_remaining);
    Serial.print("z steps remaining: "); Serial.println(z_steps_remaining);
    
    Serial.print("x_accel: "); Serial.println(x_accel);
    Serial.print("y_accel: "); Serial.println(y_accel);
    Serial.print("z_accel: "); Serial.println(z_accel);
    Serial.println();
  }
  
}


void get_acceleration_interval(int x_steps_remaining, int y_steps_remaining, int z_steps_remaining, int e_steps_remaining)
{
  steps_to_accel = 0;
  
  if (x_distance >= y_distance && x_distance >= z_distance)
  {
    current_velocity = STARTING_SPEED*x_move_fraction; // start-up speed
    
    while (x_steps_remaining > 0) {
      if (x_steps_remaining == x_steps_to_take) { // we haven't taken any steps yet
        // take first step
        accel_interval = (1.0/x_steps_per_inch)/(STARTING_SPEED*x_move_fraction);
        current_velocity = (1.0/x_steps_per_inch)/accel_interval;
        
        if (current_velocity > x_velocity) {
          if (debugging == true) Serial.println("velocity exceeded desired in first step");
          accel_interval = (1.0/x_steps_per_inch)/x_velocity;
          current_velocity = x_velocity;
        }
        
        steps_to_accel++;
        accel_interval_scaler[steps_to_accel] = int((current_velocity - STARTING_SPEED*x_move_fraction)/(x_velocity - STARTING_SPEED*x_move_fraction)*10000.0);
      }
      else { // not the first step, check how many steps we have left and determine what to do
        if (x_steps_remaining > steps_to_accel) { // dont need to decel yet
          if (current_velocity < x_velocity) { // still accelerating
            accel_interval = (-current_velocity + sqrt((current_velocity*current_velocity) + 2.0*x_accel*(1.0/x_steps_per_inch)))/(x_accel);
            current_velocity = (1.0/x_steps_per_inch)/accel_interval;
            
            if (current_velocity > x_velocity) { // if we calculated a speed above our desired speed, exit loop as we are done finding scalers
              accel_interval = (1.0/x_steps_per_inch)/x_velocity;
              current_velocity = x_velocity;
              x_steps_remaining = 0;
              break;
            }
            
            steps_to_accel++;
            accel_interval_scaler[steps_to_accel] = int((current_velocity - STARTING_SPEED*x_move_fraction)/(x_velocity - STARTING_SPEED*x_move_fraction)*10000.0);
          }
          else { // coasting, exit loop as we are done finding scalers
            x_steps_remaining = 0;
            break;
          }
        }
        else { // need to start decel, exit loop as we are done finding scalers
          x_steps_remaining = 0;
          break;
        }
      }
      
      x_steps_remaining--;
      
      if (steps_to_accel >= MAX_STEPS_TO_ACCEL) { // we are taking too many steps to speed up and the array is going to overflow
        if (debugging == true) {
          Serial.println("array overflow for acceleration calculations, need to increase array size or increase max acceleration");
          Serial.println("acceleration is now being disabled.");
        }
        acceleration = false;
        break;
      }
    }
    
    if (debugging == true) {
      for (int j = 1; j <= steps_to_accel; j++) {
        Serial.print("  ");
        Serial.println(float(accel_interval_scaler[j]/10000.0), 4);
        Serial.println();
      }
    }
  }
  
  else if (y_distance >= x_distance && y_distance >= z_distance)
  {
    current_velocity = STARTING_SPEED*y_move_fraction; // start-up speed
    
    while (y_steps_remaining > 0) {
      if (y_steps_remaining == y_steps_to_take) { // we haven't taken any steps yet
        // take first step
        accel_interval = (1.0/y_steps_per_inch)/(STARTING_SPEED*y_move_fraction);
        current_velocity = (1.0/y_steps_per_inch)/accel_interval;
        
        if (current_velocity > y_velocity) {
          if (debugging == true) Serial.println("velocity exceeded desired in first step");
          accel_interval = (1.0/y_steps_per_inch)/y_velocity;
          current_velocity = y_velocity;
        }
        
        steps_to_accel++;
        accel_interval_scaler[steps_to_accel] = int((current_velocity - STARTING_SPEED*y_move_fraction)/(y_velocity - STARTING_SPEED*y_move_fraction)*10000.0);
      }
      else { // not the first step, check how many steps we have left and determine what to do
        if (y_steps_remaining > steps_to_accel) { // dont need to decel yet
          if (current_velocity < y_velocity) { // still accelerating
            accel_interval = (-current_velocity + sqrt((current_velocity*current_velocity) + 2.0*y_accel*(1.0/y_steps_per_inch)))/(y_accel);
            current_velocity = (1.0/y_steps_per_inch)/accel_interval;
            
            if (current_velocity > y_velocity) { // if we calculated a speed above our desired speed, exit loop as we are done finding scalers
              accel_interval = (1.0/y_steps_per_inch)/y_velocity;
              current_velocity = y_velocity;
              y_steps_remaining = 0;
              break;
            }
            
            steps_to_accel++;
            accel_interval_scaler[steps_to_accel] = int((current_velocity - STARTING_SPEED*y_move_fraction)/(y_velocity - STARTING_SPEED*y_move_fraction)*10000.0);
          }
          else { // coasting, exit loop as we are done finding scalers
            y_steps_remaining = 0;
            break;
          }
        }
        else { // need to start decel, exit loop as we are done finding scalers
          y_steps_remaining = 0;
          break;
        }
      }
      
      y_steps_remaining--;
      
      if (steps_to_accel >= MAX_STEPS_TO_ACCEL) { // we are taking too many steps to speed up and the array is going to overflow
        if (debugging == true) {
          Serial.println("array overflow for acceleration calculations, need to increase array size or increase max acceleration");
          Serial.println("acceleration is now being disabled.");
        }
        acceleration = false;
        break;
      }
    }
    
    if (debugging == true) {
      for (int j = 1; j <= steps_to_accel; j++) {
        Serial.print("  ");
        Serial.println(float(accel_interval_scaler[j]/10000.0), 4);
        Serial.println();
      }
    }
  }
  
  else // z is largest distance
  {
    current_velocity = STARTING_SPEED*z_move_fraction; // start-up speed
    
    while (z_steps_remaining > 0) {
      if (z_steps_remaining == z_steps_to_take) { // we haven't taken any steps yet
        // take first step
        accel_interval = (1.0/z_steps_per_inch)/(STARTING_SPEED*z_move_fraction);
        current_velocity = (1.0/z_steps_per_inch)/accel_interval;
        
        if (current_velocity > z_velocity) {
          if (debugging == true) Serial.println("velocity exceeded desired in first step");
          accel_interval = (1.0/z_steps_per_inch)/z_velocity;
          current_velocity = z_velocity;
        }
        
        steps_to_accel++;
        accel_interval_scaler[steps_to_accel] = int((current_velocity - STARTING_SPEED*z_move_fraction)/(z_velocity - STARTING_SPEED*z_move_fraction)*10000.0);
      }
      else { // not the first step, check how many steps we have left and determine what to do
        if (z_steps_remaining > steps_to_accel) { // dont need to decel yet
          if (current_velocity < z_velocity) { // still accelerating
            accel_interval = (-current_velocity + sqrt((current_velocity*current_velocity) + 2.0*z_accel*(1.0/z_steps_per_inch)))/(z_accel);
            current_velocity = (1.0/z_steps_per_inch)/accel_interval;
            
            if (current_velocity > z_velocity) { // if we calculated a speed above our desired speed, exit loop as we are done finding scalers
              accel_interval = (1.0/z_steps_per_inch)/z_velocity;
              current_velocity = z_velocity;
              z_steps_remaining = 0;
              break;
            }
            
            steps_to_accel++;
            accel_interval_scaler[steps_to_accel] = int((current_velocity - STARTING_SPEED*z_move_fraction)/(z_velocity - STARTING_SPEED*z_move_fraction)*10000.0);
          }
          else { // coasting, exit loop as we are done finding scalers
            z_steps_remaining = 0;
            break;
          }
        }
        else { // need to start decel, exit loop as we are done finding scalers
          z_steps_remaining = 0;
          break;
        }
      }
      
      z_steps_remaining--;
      
      if (steps_to_accel >= MAX_STEPS_TO_ACCEL) { // we are taking too make steps to speed up and the array is going to overflow
        if (debugging == true) {
          Serial.println("array overflow for acceleration calculations, need to increase array size or increase max acceleration");
          Serial.println("acceleration is now being disabled.");
        }
        acceleration = false;
        break;
      }
    }
    
    if (debugging == true) {
      for (int j = 1; j <= steps_to_accel; j++) {
        Serial.print("  ");
        Serial.println(float(accel_interval_scaler[j]/10000.0), 4);
        Serial.println();
      }
    }
  }
}


void linear_move(int x_steps_remaining, int y_steps_remaining, int z_steps_remaining, int e_steps_remaining) // make linear move with preset speeds and destinations, see G0 and G1
{
  if (acceleration == true) {
    get_acceleration_parameters(x_steps_remaining, y_steps_remaining, z_steps_remaining, e_steps_remaining);
    get_acceleration_interval(x_steps_remaining, y_steps_remaining, z_steps_remaining, e_steps_remaining); // make array of stepping intervals for smooth acceleration
  }
  
    // Adding enable support for the steppers
  if (x_steps_remaining > 0 && X_ENABLE_PIN > 0){
    digitalWrite(X_ENABLE_PIN, ENABLE_ON);
  }

  if (y_steps_remaining > 0 && Y_ENABLE_PIN > 0){
    digitalWrite(Y_ENABLE_PIN, ENABLE_ON);
  }

  if (z_steps_remaining > 0 && Z_ENABLE_PIN > 0){
    digitalWrite(Z_ENABLE_PIN, ENABLE_ON);
  }
  
  if (e_steps_remaining > 0 && E_ENABLE_PIN > 0){
    digitalWrite(E_ENABLE_PIN, ENABLE_ON);
  }
  
  while(x_steps_remaining > 0 || y_steps_remaining > 0 || z_steps_remaining > 0 || e_steps_remaining > 0) // move until no more steps remain
  {
    if (x_steps_remaining > 0)
    {
      
      if (acceleration == true) {
        // update x interval
        if ((x_steps_to_take - x_steps_remaining) + 1 < steps_to_accel) { // still accelerating
          x_interval = (1.0/x_steps_per_inch)/(STARTING_SPEED*x_move_fraction + (x_velocity - STARTING_SPEED*x_move_fraction)*(accel_interval_scaler[(x_steps_to_take - x_steps_remaining) + 1]/10000.0));
        }
        else if (x_steps_remaining < steps_to_accel) { // decelerating
          x_interval = (1.0/x_steps_per_inch)/(STARTING_SPEED*x_move_fraction + (x_velocity - STARTING_SPEED*x_move_fraction)*(accel_interval_scaler[x_steps_remaining]/10000.0));
        }
        else { // coasting
          x_interval = (1.0/x_steps_per_inch)/x_velocity;
        }
        
        x_interval = x_interval*60000000; // convert from minutes to microseconds
        if (debugging == true) {
          Serial.print("  ");
          Serial.println(x_interval);
        }
      }
      
      
      if (destination_x > current_x) // X CW, this is where motor direction is handled
      {
        if (digitalRead(X_MAX_ENDSTOP) == INVERT_X_MAX_ENDSTOP) {
          if (debugging == true) Serial.println("x max endstop triggered");
          x_steps_remaining = 0;
          //current_x = X_STROKE;
        }
        else if ((micros()-previous_micros_x) >= x_interval) // if we have waited long enough
        {
          #if X_USE_DIRSTEP
            digitalWrite(X_DIR_PIN, INVERT_X_DIR);
            digitalWrite(X_STEP_PIN, HIGH);
            previous_micros_x = micros();
            delayMicroseconds(3);
            digitalWrite(X_STEP_PIN, LOW);
          #else
            cpwStepper_x.takestep(INVERT_X_DIR);
            previous_micros_x = micros();
          #endif
          x_steps_remaining = x_steps_remaining - 1;
          if (x_steps_remaining == 0 && X_ENABLE_PIN > 0){
              digitalWrite(X_ENABLE_PIN, !ENABLE_ON);
          }
        }
      }
      else // X CCW
      {
        if (digitalRead(X_MIN_ENDSTOP) == INVERT_X_MIN_ENDSTOP) {
          if (debugging == true) Serial.println("x min endstop triggered");
          x_steps_remaining = 0;
          //current_x = 0.0;
        }
        else if ((micros()-previous_micros_x) >= x_interval) // if we have waited long enough
        {
          #if X_USE_DIRSTEP
            digitalWrite(X_DIR_PIN, !INVERT_X_DIR);
            digitalWrite(X_STEP_PIN, HIGH);
            previous_micros_x = micros();
            delayMicroseconds(3);
            digitalWrite(X_STEP_PIN, LOW);
          #else
            cpwStepper_x.takestep(!INVERT_X_DIR);
            previous_micros_x = micros();
          #endif
          x_steps_remaining = x_steps_remaining - 1;
          if (x_steps_remaining == 0 && X_ENABLE_PIN > 0){
              digitalWrite(X_ENABLE_PIN, !ENABLE_ON);
          }
        }
      }
    }
    
    if (y_steps_remaining > 0)
    {
      
      if (acceleration == true) {
        // update y interval
        if ((y_steps_to_take - y_steps_remaining) + 1 < steps_to_accel) { // still accelerating
          y_interval = (1.0/y_steps_per_inch)/(STARTING_SPEED*y_move_fraction + (y_velocity - STARTING_SPEED*y_move_fraction)*(accel_interval_scaler[(y_steps_to_take - y_steps_remaining) + 1]/10000.0));
        }
        else if (y_steps_remaining < steps_to_accel) { // decelerating
          y_interval = (1.0/y_steps_per_inch)/(STARTING_SPEED*y_move_fraction + (y_velocity - STARTING_SPEED*y_move_fraction)*(accel_interval_scaler[y_steps_remaining]/10000.0));
        }
        else { // coasting
          y_interval = (1.0/y_steps_per_inch)/y_velocity;
        }
        
        y_interval = y_interval*60000000; // convert from minutes (used in velocity calcs) to microseconds (used for timing)
        if (debugging == true) {
          Serial.print("  ");
          Serial.println(y_interval);
        }
      }
      
      if (destination_y > current_y) // Y CW, this is where motor direction is handled
      {
        if (digitalRead(Y_MAX_ENDSTOP) == INVERT_Y_MAX_ENDSTOP) {
          if (debugging == true) Serial.println("y max endstop triggered");
          y_steps_remaining = 0;
          //current_y = Y_STROKE;
        }
        else if ((micros()-previous_micros_y) >= y_interval) // if we have waited long enough
        {
          #if Y_USE_DIRSTEP
            digitalWrite(Y_DIR_PIN, INVERT_Y_DIR);
            digitalWrite(Y_STEP_PIN, HIGH);
            previous_micros_y = micros();
            delayMicroseconds(3);
            digitalWrite(Y_STEP_PIN, LOW);
          #else
            cpwStepper_y.takestep(INVERT_Y_DIR);
            previous_micros_y = micros();
          #endif
          y_steps_remaining = y_steps_remaining - 1;
          if (y_steps_remaining == 0 && Y_ENABLE_PIN > 0){
              digitalWrite(Y_ENABLE_PIN, !ENABLE_ON);
          }
        }
      }
      else // Y CCW
      {
        if (digitalRead(Y_MIN_ENDSTOP) == INVERT_Y_MIN_ENDSTOP) {
          if (debugging == true) Serial.println("y min endstop triggered");
          y_steps_remaining = 0;
          //current_y = 0.0;
        }
        else if ((micros()-previous_micros_y) >= y_interval) // if we have waited long enough
        {
          #if Y_USE_DIRSTEP
            digitalWrite(Y_DIR_PIN, !INVERT_Y_DIR);
            digitalWrite(Y_STEP_PIN, HIGH);
            previous_micros_y = micros();
            delayMicroseconds(3);
            digitalWrite(Y_STEP_PIN, LOW);
          #else
            cpwStepper_y.takestep(!INVERT_Y_DIR);
            previous_micros_y = micros();
          #endif
          y_steps_remaining = y_steps_remaining - 1;
          if (y_steps_remaining == 0 && Y_ENABLE_PIN > 0){
              digitalWrite(Y_ENABLE_PIN, !ENABLE_ON);
          }
        }
      }
    }
    
    if (z_steps_remaining > 0)
    {
      
      if (acceleration == true) {
        // update z interval
        if ((z_steps_to_take - z_steps_remaining) + 1 < steps_to_accel) { // still accelerating
          z_interval = (1.0/z_steps_per_inch)/(STARTING_SPEED*z_move_fraction + (z_velocity - STARTING_SPEED*z_move_fraction)*(accel_interval_scaler[(z_steps_to_take - z_steps_remaining) + 1]/10000.0));
        }
        else if (z_steps_remaining < steps_to_accel) { // decelerating
          z_interval = (1.0/z_steps_per_inch)/(STARTING_SPEED*z_move_fraction + (z_velocity - STARTING_SPEED*z_move_fraction)*(accel_interval_scaler[z_steps_remaining]/10000.0));
        }
        else { // coasting
          z_interval = (1.0/z_steps_per_inch)/z_velocity;
        }
        
        z_interval = z_interval*60000000; // convert from minutes to microseconds
        if (debugging == true) {
          Serial.print("  ");
          Serial.println(z_interval);
        }
      }
      
      if (destination_z > current_z) // Z CW, this is where motor direction is handled
      {
        if (digitalRead(Z_MAX_ENDSTOP) == INVERT_Z_MAX_ENDSTOP) {
          if (debugging == true) Serial.println("z max endstop triggered");
          z_steps_remaining = 0;
          //current_z = Z_STROKE;
        }
        else if ((micros()-previous_micros_z) >= z_interval) // if we have waited long enough
        {
          #if Z_USE_DIRSTEP
            digitalWrite(Z_DIR_PIN[current_tool], INVERT_Z_DIR);
            digitalWrite(Z_STEP_PIN[current_tool], HIGH);
            previous_micros_z = micros();
            delayMicroseconds(3);
            digitalWrite(Z_STEP_PIN[current_tool], LOW);
          #else
            cpwStepper_z.takestep(INVERT_Z_DIR);
            previous_micros_z = micros();
          #endif
          z_steps_remaining = z_steps_remaining - 1;
          if (z_steps_remaining == 0 && Z_ENABLE_PIN > 0){
              digitalWrite(Z_ENABLE_PIN, !ENABLE_ON);
          }
        }
      }
      else // Z CCW
      {
        if (digitalRead(Z_MIN_ENDSTOP) == INVERT_Z_MIN_ENDSTOP) {
          if (debugging == true) Serial.println("z min endstop triggered");
          z_steps_remaining = 0;
          //current_z = 0.0;
        }
        else if ((micros()-previous_micros_z) >= z_interval) // if we have waited long enough
        {
          #if Z_USE_DIRSTEP
            digitalWrite(Z_DIR_PIN[current_tool], !INVERT_Z_DIR);
            digitalWrite(Z_STEP_PIN[current_tool], HIGH);
            previous_micros_z = micros();
            delayMicroseconds(3);
            digitalWrite(Z_STEP_PIN[current_tool], LOW);
          #else
            cpwStepper_z.takestep(!INVERT_Z_DIR);
            previous_micros_z = micros();
          #endif
          z_steps_remaining = z_steps_remaining - 1;
          if (z_steps_remaining == 0 && Z_ENABLE_PIN > 0){
              digitalWrite(Z_ENABLE_PIN, !ENABLE_ON);
          }
        }
      }
    }
    
    if (e_steps_remaining > 0)
    {
      if (extruder_dir == FORWARD) // extruder forward, this is where motor direction is handled
      {
        if ((micros()-previous_micros_e) >= e_interval) // if we have waited long enough
        {
          #if E_USE_DIRSTEP
            digitalWrite(E_DIR_PIN, INVERT_E_DIR);
            digitalWrite(E_STEP_PIN, HIGH);
            previous_micros_e = micros();
            delayMicroseconds(3);
            digitalWrite(E_STEP_PIN, LOW);
          #elif !E_USE_DC
            cpwStepper_e.takestep(INVERT_E_DIR);
            previous_micros_e = micros();
          #endif
          e_steps_remaining = e_steps_remaining - 1;
          if (e_steps_remaining == 0 && E_ENABLE_PIN > 0){
              digitalWrite(E_ENABLE_PIN, !ENABLE_ON);
          }
        }
      }
      else if (extruder_dir == BACKWARD) // extruder forward, this is where motor direction is handled
      {
        if ((micros()-previous_micros_e) >= e_interval) // if we have waited long enough
        {
          #if E_USE_DIRSTEP
            digitalWrite(E_DIR_PIN, !INVERT_E_DIR);
            digitalWrite(E_STEP_PIN, HIGH);
            previous_micros_e = micros();
            delayMicroseconds(3);
            digitalWrite(E_STEP_PIN, LOW);
          #elif !E_USE_DC
            cpwStepper_e.takestep(!INVERT_E_DIR);
            previous_micros_e = micros();
          #endif
          e_steps_remaining = e_steps_remaining - 1;
          if (e_steps_remaining == 0 && E_ENABLE_PIN > 0){
              digitalWrite(E_ENABLE_PIN, !ENABLE_ON);
          }
        }
      }
    }
  }
  
  // update positioning
  if (absolute_positioning == true) { // use absolute programming
    if (destination_x > current_x) { // forward
      current_x = current_x + x_steps_to_take/x_steps_per_inch;
    }
    else { // going backwards
      current_x = current_x - x_steps_to_take/x_steps_per_inch;
    }
    if (destination_y > current_y) { // forward
      current_y = current_y + y_steps_to_take/y_steps_per_inch;
    }
    else { // going backwards
      current_y = current_y - y_steps_to_take/y_steps_per_inch;
    }
    if (destination_z > current_z) { // forward
      current_z = current_z + z_steps_to_take/z_steps_per_inch;
    }
    else { // going backwards
      current_z = current_z - z_steps_to_take/z_steps_per_inch;
    }
  }
  else { // use incremental programming
    current_x = destination_x;
    current_y = destination_y;
    current_z = destination_z;
  }
  
  // done moving and position is updated, clear all steps
  x_steps_to_take = 0;
  y_steps_to_take = 0;
  z_steps_to_take = 0;
  e_steps_to_take = 0;
  
  if (debugging == true) {
    Serial.println("in linear move fxn after position update");
    Serial.print("X pos: ");
    Serial.print(current_x);
    Serial.print(" X dest: ");
    Serial.print(destination_x);
    Serial.print(" X steps: ");
    Serial.print(x_steps_to_take);
    Serial.print(" X int: ");
    Serial.println(x_interval);
    
    Serial.print("Y pos: ");
    Serial.print(current_y);
    Serial.print(" Y dest: ");
    Serial.print(destination_y);
    Serial.print(" Y steps: ");
    Serial.print(y_steps_to_take);
    Serial.print(" Y int: ");
    Serial.println(y_interval);
  }
}


void establishContact() {
  while (Serial.available() <= 0) { // as long as we still don't see any data
    Serial.println("ok");   // send a capital A
    delay(300);
  }
}


void kill_all() // activated through estop
{
  if (debugging == true) Serial.println("emergency stop activated!");
  
  #if !X_USE_DIRSTEP
    cpwStepper_x.disable();
  #endif
  #if !Y_USE_DIRSTEP
    cpwStepper_y.disable();
  #endif
  #if !Z_USE_DIRSTEP
    cpwStepper_z.disable();
  #endif
  #if !E_USE_DIRSTEP and !E_USE_DC
    cpwStepper_e.disable();
  #endif
  
  #ifdef SPINDLEPIN
  digitalWrite(SPINDLEPIN, LOW); // turn off spindle
  #endif
  
  #ifdef LEDPIN
  digitalWrite(LEDPIN, LOW); // turn off LED, debuggingg, delete later
  #endif
  
  #if PID_CONTROL
    analogWrite(heaterPin, 0); // turn off heater
  #endif
  
  // turn off coolant
  // turn off fan
  
  while (digitalRead(KILLPIN_DIGITAL) == LOW) // wait for kill switch to be released and then resume
  {
    delay(1);
  }
  if (debugging == true) Serial.println("resuming operation...");
}

// Takes temperature value as input and returns corresponding analog value from RepRap thermistor temp table.
// This is needed because PID in hydra firmware hovers around a given analog value, not a temp value.
// This function is derived from inversing the logic from a portion of getTemperature() in FiveD RepRap firmware.
int temp2analog(int celsius)
{
#ifdef USE_THERMISTOR
  int raw = 0;
  byte i;

  for (i=1; i<NUMTEMPS; i++)
  {
    if (temptable[i][1] < celsius)
    {
      raw = temptable[i-1][0] + 
        (celsius - temptable[i-1][1]) * 
        (temptable[i][0] - temptable[i-1][0]) /
        (temptable[i][1] - temptable[i-1][1]);

      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == NUMTEMPS) raw = temptable[i-1][0];

  return raw;
#else
  return celsius * ((1024.0/(5.0*100.0));
#endif
}

// Derived from RepRap FiveD extruder::getTemperature()
int analog2temp(int raw)
{
#ifdef USE_THERMISTOR

  int celsius = 0;
  byte i;

  for (i=1; i<NUMTEMPS; i++)
  {
    if (temptable[i][0] > raw)
    {
      celsius  = temptable[i-1][1] + 
        (raw - temptable[i-1][0]) * 
        (temptable[i][1] - temptable[i-1][1]) /
        (temptable[i][0] - temptable[i-1][0]);

      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == NUMTEMPS) celsius = temptable[i-1][1];

  return celsius;
#else
  return analogRead(thermocouplePin) * ((5.0*100.0)/1024.0);
#endif
}

