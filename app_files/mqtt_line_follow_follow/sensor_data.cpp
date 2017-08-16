#include "sensor_data.h"

Mail<float, 10> telemetry_mailbox;

/**
 * @brief clears the telemetry buffer/queue
 */
void clear_telemetry()
{
    osEvent evt = telemetry_mailbox.get(1); // Clear the queue 
    if(evt.status != osEventMail) 
        return;
    float *message = (float*)evt.value.p;
    telemetry_mailbox.free(message);
}

/**
 * @brief returns the current telemetry value
 * @param millisec      wait time in milisecond
 * @return              [the current telemetry]
 */

float get_telemetry()
{ 
    osEvent evt = telemetry_mailbox.get(); // Clear the queue 
    if(evt.status != osEventMail) 
        return -1;

    float *message = (float*)evt.value.p;
    float telemetry_data= *message; 
    telemetry_mailbox.free(message);

    return telemetry_data;
}

/**
 * @brief puts the recent telemetry value in the buffer
 * @param telemetry          telemetry
 * @return              [ ]
 */
void put_telemetry(float telemetry)
{ 
    clear_telemetry();

    float *message = telemetry_mailbox.alloc();
    if(message == NULL)
        return;
    *message = telemetry; 
    telemetry_mailbox.put(message); 
}