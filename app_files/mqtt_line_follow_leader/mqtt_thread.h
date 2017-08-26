#ifndef _MQTT_THREAD_H
#define _MQTT_THREAD_H

#define EMCUTE_ID_LEN       (8)

/**
 * @brief      This lists the states of the mqtt_control_thread in the sequence
 */
enum mqtt_states
{
    MQTT_DISCON              = 0,
    MQTT_RECV_MQTT_GO        = 1,
    MQTT_RECV_HW_ADDR        = 2,
    MQTT_MBED_INIT_DONE      = 3,
    MQTT_CONTROL_GO          = 4,
    MQTT_CONTROL_GO_WAIT     = 5    
};

/**
 * @brief      Intialized the mqtt Thread
 * @param[in]  priority  The priority of the mqtt thread
 * @return     pointer to the mqtt mailbox
 */
Mail<msg_t, HDLC_MAILBOX_SIZE> *mqtt_init(osPriority priority);
/**
 * @brief      Returns a pointer to the mqtt mailbox.
 * @return     The mqtt mailbox.
 */
Mail<msg_t, HDLC_MAILBOX_SIZE> *get_mqtt_mailbox();

int get_mqtt_state (void);
void set_mqtt_state (int state);

void reset_system(void);

#endif /*  _MQTT_THREAD_H */ 