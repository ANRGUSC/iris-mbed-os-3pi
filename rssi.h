#ifndef RSSI_H_
#define RSSI_H_
/**
 *  A single length queue for sending and receiving the rssi
 */
#include "mbed.h"
#include "rtos.h"

/**
 * @brief clears the rssi buffer/queue
 */
void clear_rssi();


/**
 * @brief returns the current rssi value
 * @param  millisec wait time in milisecond
 * @return          [description]
 */
float get_rssi(uint32_t millisec = osWaitForever);

/**
 * @brief puts the recent rssi value in the buffer
 * @param rssi          rssi_value
 * @return              [ ]
 */
void put_rssi(float rssi);

/**
 * @brief clears the range buffer/queue
 */
void clear_range();

/**
 * @brief returns the current range value
 * @param millisec      wait time in milisecond
 * @return              [the current range]
 */

float get_range(uint32_t millisec);

/**
 * @brief puts the recent range value in the buffer
 * @param range          range
 * @return              [ ]
 */
void put_range(float range);

#endif