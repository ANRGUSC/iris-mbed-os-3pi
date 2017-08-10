//Sets up MINIMU-9 and allows for m3pi movement guided by MINIMU-9 sensors

#ifndef MBED_MOVEMENT_H
#define MBED_MOVEMENT_H

#include "mbed.h"
#include "rtos.h"
#include "m3pi.h"
#include "LIS3MDL.h"
#include "LSM6.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//Use to check bits of data ready registers
#define DATA_MASK	0x03
//Acceleration and Gyro data ready pins
#define XLDA	0x01
#define GDA		0x02
//Compass x and y data ready pins
#define XDA 	0x01
#define YDA		0x02

//Gyro scaling factor and approximate z axis drift
#define G_FACTOR 0.00875
#define DRIFT 	 2.6

//PID control constants
#define KP		 5.0
#define KD		 0.5
#define KI		 0.0

//movement_t which includes the type of movement and any parameters
//(time, degrees, speed) needed to perform the movement
typedef struct __attribute__((packed))
{
	uint8_t move_type;
	uint16_t time;
	int16_t degrees;
	int8_t speed;
} movement_t;

//Types of mbed movement commands
typedef enum
{
	INIT_IMU			  = 0,
	CALIBRATE			  = 1,
	ROTATE				  = 2,
	DRIVE_SPEED			  = 3,
	DRIVE_PID			  = 4
} movement_funct_t;

//init_minimu()
//Initializes the MINIMU-9 so that data from gyroscope, accelerometer,
//and magnetometer can be read
//Parameters: none
//Returns: true or false (whether initialization succeeded)
bool init_minimu(void);

//calibrate_compass
//Sets all of the Calibration scales and coefficients for compass
//Rotates a couple times in order to measure range of compass values
//Accurate in one place, may not be if compass is moved to a new location
//Parameters: none
//Returns: none
void calibrate_compass(void);

//rotate_degrees
//Rotates the m3pi a certain number of degrees at a certain speed
//Uses the magnetometer to determine the position
//Checks angle and predicts at what time it will have to stop
//Has been tested for speeds 15-55. Slower speeds usually stall the m3pi,
//while higher speeds are unlikely to be needed for rotation purposes
//Accuracy +- 15 degrees, decreases at slow speeds and large rotations
//	(once rotation approaches multiple full rotations)
//Parameters: int16_t degrees: gives number of degrees to rotate (pos or neg)
//			  int8_t speed: speed at which to rotate (-127 - 127)
//Returns: none
void rotate_degrees(int16_t degrees, int8_t speed);

//drive_forward()
//Moves the m3pi forward, using the compass to stay on a straight heading
//Parameters: int16_t time: time in ms to move forward
//			  int8_t speed: speed at which to move forward
//Returns: none
void drive_forward(uint16_t time, int8_t speed);

//drive_forward()
//Runs like drive_forward above, but at fixed speed of 40
//Uses a PID controller for increased accuracy
//If accuracy is poor, may need to be retuned to properly estimate error
//Parameters; uint16_t time: time in ms to move forward
//Returns: none
void drive_forward(uint16_t time);

#endif