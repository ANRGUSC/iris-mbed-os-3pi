#include "sensor_data.h"

Mail<control_data_t, 10> telemetry_mailbox;

/**
 * @brief clears the telemetry buffer/queue
 */
void clear_telemetry()
{
    osEvent evt = telemetry_mailbox.get(1); // Clear the queue 
    if(evt.status != osEventMail) 
        return;
    control_data_t *message = (control_data_t*)evt.value.p;
    telemetry_mailbox.free(message);
}

/**
 * @brief returns the current telemetry value
 * @param millisec      wait time in milisecond
 * @return              [the current telemetry]
 */

void get_telemetry(control_data_t *result)
{ 
    osEvent evt = telemetry_mailbox.get(); // Clear the queue 
    if(evt.status != osEventMail) 
        return;

    control_data_t *telemetry_data = (control_data_t*)evt.value.p;
    memcpy(result, telemetry_data, sizeof(control_data_t));
    telemetry_mailbox.free(telemetry_data);
}

void get_telemetry(int millisec, control_data_t *result)
{ 
    osEvent evt = telemetry_mailbox.get(millisec); // Clear the queue 
    if(evt.status != osEventMail) 
        return;

    control_data_t *telemetry_data = (control_data_t*)evt.value.p;
    memcpy(result, telemetry_data, sizeof(control_data_t));
    telemetry_mailbox.free(telemetry_data);;
}


/**
 * @brief puts the recent telemetry value in the buffer
 * @param telemetry          telemetry
 * @return              [ ]
 */
void put_telemetry(char telemetry[])
{ 
    clear_telemetry();

    control_data_t *message = telemetry_mailbox.alloc();
    if(message == NULL)
        return;

    sscanf(telemetry, "%f %f", &message->speed_l, &message->speed_r);
    printf("%f %f \n",message->speed_l, message->speed_r);
    telemetry_mailbox.put(message); 
}