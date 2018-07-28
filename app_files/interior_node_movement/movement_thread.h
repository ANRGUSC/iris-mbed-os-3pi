#ifndef _MOVEMENT_THREAD_H
#define _MOVEMENT_THREAD_H


/**
 * @brief      This lists the states of the movement_control_thread in the sequence
 */
enum movement_states
{
    NO_INFO              = 0,
};

/**
 * @brief      Intialized the movement Thread
 * @param[in]  priority  The priority of the movement thread
 * @return     pointer to the movement mailbox
 */
Mail<msg_t, HDLC_MAILBOX_SIZE> *movement_init(osPriority priority);
/**
 * @brief      Returns a pointer to the movement mailbox.
 * @return     The movement mailbox.
 */
Mail<msg_t, HDLC_MAILBOX_SIZE> *get_movement_mailbox();

/**
 * @brief      Gets the movement state.
 *
 * @return     The movement state.
 */
int get_movement_state (void);

/**
 * @brief      Sets the movement state.
 *
 * @param[in]  state  The state
 */
void set_movement_state (int state);

/**
 * @brief      Gets the node identifier.
 *
 * @param      ret   The ret
 */
void get_node_id (char *ret);


#endif /*  _MOVEMENT_THREAD_H */ 
