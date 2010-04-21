#ifndef PARAMETERS_H
#define PARAMETERS_H

// NO RS485/EXTRUDER CONTROLLER SUPPORT
#define MOTHERBOARD 1 // SANGUINO 1, MOTHERBOARD12 = 2, MEGA 3, ATMEGA328 4, HYDRA 64

#define USE_THERMISTOR //Comment out if using thermocouple

#define  E_STEPS_PER_MM 0.706
#define  X_STEPS_PER_MM 10.047
#define  Y_STEPS_PER_MM 10.047
#define  Z_STEPS_PER_MM 833.398

// machine specifications
const float X_STROKE = (200 / 25.4); // inches of x axis travel
const float Y_STROKE = (200 / 25.4); // inches of y axis travel
const float Z_STROKE = (120 / 25.4); // inches of z axis travel

const float DEFAULT_FEEDRATE_IPM = (1500 / 25.4); // MilliMeters per min / ConversionRatio
const float MAXIMUM_FEEDRATE_IPM = (2000 / 25.4); // MilliMeters per min / ConversionRatio

float x_steps_per_inch = X_STEPS_PER_MM / 25.4;
float y_steps_per_inch = Y_STEPS_PER_MM / 25.4;
float z_steps_per_inch = Z_STEPS_PER_MM / 25.4;
float e_steps_per_inch = E_STEPS_PER_MM / 25.4;

#endif
