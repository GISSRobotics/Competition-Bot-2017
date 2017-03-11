#ifndef ROBOTMAP_H
#define ROBOTMAP_H

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// Motors

const int DRIVE_MOTOR_LF = 0;
const int DRIVE_MOTOR_LR = 1;
const int DRIVE_MOTOR_RF = 2;
const int DRIVE_MOTOR_RR = 3;
//const int GEAR_SERVO = 4;
const int GEAR_MOTOR = 4;
const int WINCH_MOTOR = 5;

// Sensors and Inputs

const int DRIVE_ENCODER_LA = 0;
const int DRIVE_ENCODER_LB = 1;
const int DRIVE_ENCODER_RA = 2;
const int DRIVE_ENCODER_RB = 3;
const int GEAR_STATUS_SWITCH = 4;
const int GEAR_LIMIT_SWITCH_DOWN = 5;
const int GEAR_LIMIT_SWITCH_UP = 6;

#endif  // ROBOTMAP_H
