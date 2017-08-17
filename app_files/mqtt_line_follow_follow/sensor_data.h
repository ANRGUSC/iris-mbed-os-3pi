#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_
/**
 *  A single length queue for sending and receiving the sensor data
 */
#include "mbed.h"
#include "rtos.h"

/**
 * @brief clears the telemetry buffer/queue
 */
void clear_telemetry();

/**
 * @brief returns the current telemetry value
 * @param millisec      wait time in milisecond
 * @return              [the current telemetry]
 */
float get_telemetry();

/**
 * @brief returns the current telemetry value
 * @param millisec      wait time in milisecond
 * @return              [the current telemetry]
 */
float get_telemetry(int millisec);

/**
 * @brief puts the recent telemetry value in the buffer
 * @param telemetry          telemetry
 * @return              [ ]
 */
void put_telemetry(float telemetry);

#endif