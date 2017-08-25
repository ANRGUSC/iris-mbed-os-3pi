#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_
/**
 *  A single length queue for sending and receiving the sensor data
 */
#include "mbed.h"
#include "rtos.h"
typedef struct control_data{
    float speed_l;
    float speed_r;
}control_data_t;

/**
 * @brief clears the telemetry buffer/queue
 */
void clear_telemetry();

/**
 * @brief returns the current telemetry value
 * @param millisec      wait time in milisecond
 * @return              [the current telemetry]
 */
void get_telemetry(control_data_t *result);

/**
 * @brief returns the current telemetry value
 * @param millisec      wait time in milisecond
 * @return              [the current telemetry]
 */
void get_telemetry(int millisec, control_data_t *result);

/**
 * @brief puts the recent telemetry value in the buffer
 * @param telemetry          telemetry
 * @return              [ ]
 */
void put_telemetry(char telemetry[]);

#endif