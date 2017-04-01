/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
 * Pradipta Ghosh
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
 * @file        main.cpp
 * @brief       Example using hdlc
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * 
 */

#include "mbed.h"
#include "rtos.h"
#include "hdlc.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "yahdlc.h"
#include "fcs16.h"
#include "uart_pkt.h"
#include "dispatcher.h"
#include "utlist.h"

#define DEBUG   0

#if (DEBUG) 
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */

Mail<msg_t, HDLC_MAILBOX_SIZE> dispatcher_mailbox;

static unsigned char DISPACHER_STACK[DEFAULT_STACK_SIZE/2];
Thread dispatcher(osPriorityNormal, 
    (uint32_t) DEFAULT_STACK_SIZE/2, (unsigned char *)DISPACHER_STACK); 


static dispatcher_entry_t *dispatcher_reg;


void dispatcher_register(dispatcher_entry_t *entry)
{
    LL_PREPEND(dispatcher_reg, entry);
}
void dispatcher_unregister(dispatcher_entry_t *entry)
{
    LL_DELETE(dispatcher_reg, entry);

}

void process_received_data(riot_to_mbed_t type, char *data)
{
    int value;
    char range[5];
    switch (type){
        
        case RSSI_DATA_PKT:
            value = (int8_t)(*data);
            PRINTF("RSSI is %d\n", value);
            put_rssi((float)value - 73);
            break;

        case SOUND_RANGE_DONE:
            memcpy(range,data,4);
            value = *(uint32_t *)data;
            PRINTF("TDoA: %ld\n", value);
            range[4] = '\0';
            // sscanf(range,"%d",&value);
            put_range((float)value);
            break;

        default:
            break;
    }
}

static void _dispatcher(void)
{
    dispatcher_entry_t *entry;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;

    hdlc_mailbox_ptr = get_hdlc_mailbox();
    msg_t *msg, *msg2;
    char *recv_data;
    hdlc_buf_t *buf;
    uart_pkt_hdr_t hdr;

    PRINTF("In dispatcher");

    msg = hdlc_mailbox_ptr->alloc();
    while(msg == NULL)
    {
        Thread::wait(50);    
        msg = hdlc_mailbox_ptr->alloc();
    }
    msg->type = HDLC_MSG_REG_DISPATCHER;
    msg->sender_pid = osThreadGetId();
    msg->source_mailbox = &dispatcher_mailbox;
    hdlc_mailbox_ptr->put(msg);

    PRINTF("dispatcher pid is %d \n", osThreadGetId());

    osEvent evt;
    while(1)
    {
        evt = dispatcher_mailbox.get();
        if (evt.status == osEventMail) 
        {
            msg = (msg_t*)evt.value.p;
            switch (msg->type)
            {
                case HDLC_PKT_RDY:
                    buf = (hdlc_buf_t *)msg->content.ptr;   
                    uart_pkt_parse_hdr(&hdr, (void *)buf->data, (size_t) (buf->length));
                    LL_SEARCH_SCALAR(dispatcher_reg, entry, port, hdr.dst_port);
                    recv_data = buf->data + UART_PKT_DATA_FIELD; 
                    process_received_data((riot_to_mbed_t)hdr.pkt_type, recv_data);

                    if (entry) {
                        msg2 = entry->mailbox->alloc();
                        if( msg2 == NULL)
                            break;
                        memcpy(msg2, msg, sizeof(msg_t));
                        entry->mailbox->put(msg2);
                        dispatcher_mailbox.free(msg);
                        // PRINTF("dispatcher: received pkt %d; thread %d\n", recv_data[0],recv_data[1]);
                    } else {
                        PRINTF("dispatcher: received pkt; thread %d\n", hdr.dst_port);
                        dispatcher_mailbox.free(msg);
                        hdlc_pkt_release(buf);
                    }
                    break;
                default:
                    dispatcher_mailbox.free(msg);
                    /* error */
                    break;
            }    

        }

    }
    PRINTF("Reached Exit");
    /* should be never reached */
}

/**
 * @brief returns the mailbox mailbox of the dispatcher
 */
Mail<msg_t, HDLC_MAILBOX_SIZE>* get_dispatcher_mailbox()
{
    return &dispatcher_mailbox;
}

/**
 * @brief Initializes the dispatcher thread
 */
Mail<msg_t, HDLC_MAILBOX_SIZE>* dispatcher_init() 
{
  
    dispatcher.start(_dispatcher);
    PRINTF("dispatcher: thread  id %d\n",dispatcher.gettid());
    // dispacher_ready=0;
    return &dispatcher_mailbox;
}
