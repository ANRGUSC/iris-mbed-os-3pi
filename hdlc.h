/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal
 * with the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * - Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimers.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimers in the 
 *     documentation and/or other materials provided with the distribution.
 * - Neither the names of Autonomous Networks Research Group, nor University of 
 *     Southern California, nor the names of its contributors may be used to 
 *     endorse or promote products derived from this Software without specific 
 *     prior written permission.
 * - A citation to the Autonomous Networks Research Group must be included in 
 *     any publications benefiting from the use of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH 
 * THE SOFTWARE.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Full duplex hdlc implementation.
 *
 * This implementation leverages yahdlc, an open source library. The current 
 * implementation is stop & wait.
 *
 * @author      Pradipta Ghosh  <pradiptg@usc.edu>
 * @author      Jason A. Tran   <jasontra@usc.edu>
 *
 * @}
 */

#ifndef HDLC_H_
#define HDLC_H_

#include "yahdlc.h"
#include "rtos.h"
#include "mbed.h"
#include "uart_pkt.h"


/**
 * Define your custom parameter values under "hdlc_custom.h" header file
 */
#include "main-conf.h"


/**
 * The follwing macros define the speed of operation. 
 */

#ifndef RTRY_TIMEO_USEC
    #define RTRY_TIMEO_USEC         100000
#endif

#ifndef RETRANSMIT_TIMEO_USEC
    #define RETRANSMIT_TIMEO_USEC   50000
#endif

#ifndef HDLC_MAX_PKT_SIZE
    #define HDLC_MAX_PKT_SIZE       64
#endif

#ifndef HDLC_MAILBOX_SIZE
    #define HDLC_MAILBOX_SIZE       100 
#endif

/**
 *  This structure is used for the hdlc packets
 */
typedef struct {
    yahdlc_control_t control; // CRC check related stuff
    char *data;               // Data section of a HDLC packet
    unsigned int length;      // Length of the HDLC packet
} hdlc_buf_t;

/**
 * This structure is used for the inter-thread messaging.
 */
typedef struct {
    osThreadId sender_pid;    
    void *source_mailbox;
    uint16_t type;              /**< Type field. */
    union {
        void *ptr;              /**< Pointer content field. */
        uint32_t value;         /**< Value content field. */
    } content;                  /**< Content of the message. */
} msg_t;

/**
 * Struct for other threads to pass to hdlc thread via IPC 
 */
typedef struct {
    char *data;
    unsigned int length;
} hdlc_pkt_t;

/**
 * HDLC inter-thread message types 
 */
enum {
    HDLC_MSG_REG_DISPATCHER,
    HDLC_MSG_RECV,
    HDLC_MSG_SND,
    HDLC_MSG_RESEND,
    HDLC_MSG_SND_ACK,
    HDLC_RESP_RETRY_W_TIMEO,
    HDLC_RESP_SND_SUCC,
    HDLC_PKT_RDY
};

/**
 * 
 * This structure is used in the linked list of mapping between port number and the respective threads.
 * Each thread can register to multiple port numbers.
 * but a port number can belong to only one thread.
 * 
 */
typedef struct hdlc_entry {
    struct hdlc_entry *next;
    uint16_t port;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *mailbox;
} hdlc_entry_t;


/**
 * @brief      Releases the buffer mutex so that the next hdlc packet can be processed
 * @param      buf   The buffer pointer
 * @return     status
 */
int hdlc_pkt_release(hdlc_buf_t *buf);

/**
 * @brief      Intialized the HDLC Thread
 * @param[in]  priority  The priority of the hdlc thread
 * @return     pointer to the hdlc mailbox
 */
Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_init(osPriority priority);
/**
 * @brief      Returns a pointer to the hdlc mailbox.
 * @return     The hdlc mailbox.
 */
Mail<msg_t, HDLC_MAILBOX_SIZE> *get_hdlc_mailbox();

/**
 * @brief      replicates the entire hdlc_buf_t
 *
 * @param      dst   The destination
 * @param      src   The source
 */
void buffer_cpy(hdlc_buf_t* dst, hdlc_buf_t* src);
/**
 * @brief      adds a thread to port mapping
 *
 * @param      entry  The entry
 */
void hdlc_register(hdlc_entry_t *entry);
/**
 * @brief      deletes a thread to port mapping
 *
 * @param      entry  The entry
 */
void hdlc_unregister(hdlc_entry_t *entry);
/**
 * @brief Send @p pkt as an hdlc command packet over serial. This function blocks.
 * @param  pkt            Packet to be sent.
 * @param  sender_mailbox Pointer to sender's mailbox.
 * @return                [description]
 */
int hdlc_send_command(hdlc_pkt_t *pkt, Mail<msg_t, HDLC_MAILBOX_SIZE> *sender_mailbox, riot_to_mbed_t reply);
/**
 * @brief      Sends a hdlc packet.
 *
 * @param      msg             The return packet
 * @param[in]  type            The hdlc msg type
 * @param      sender_mailbox  The sender mailbox
 * @param      ptr             The pointer to the actual data
 * 
 * @return     status
 */
int send_hdlc_mail(msg_t *msg, uint8_t type,
                        Mail<msg_t, HDLC_MAILBOX_SIZE> *sender_mailbox, void *ptr);
/**
 * @brief      Sends a hdlc retry message.
 *
 * @param      msg             The hdlc message
 * @param      sender_mailbox  The sender mailbox
 *
 * @return     status
 */
int send_hdlc_retry_mail(msg_t *msg, Mail<msg_t, HDLC_MAILBOX_SIZE> *sender_mailbox);

#endif /* HDLC_H_ */
