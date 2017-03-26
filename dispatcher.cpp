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
#include "dispatcher.h"
#include "utlist.h"

#define DEBUG   1

#if (DEBUG) 
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */

Mail<msg_t, HDLC_MAILBOX_SIZE> dispatcher_mailbox;

static unsigned char DISPACHER_STACK[DEFAULT_STACK_SIZE];
Thread dispatcher(osPriorityNormal, 
    (uint32_t) DEFAULT_STACK_SIZE, (unsigned char *)DISPACHER_STACK); 


static dispatcher_entry_t *dispatcher_reg;


void dispatcher_register(dispatcher_entry_t *entry)
{
    LL_PREPEND(dispatcher_reg, entry);
}

void dispatcher_unregister(dispatcher_entry_t *entry)
{
    LL_DELETE(dispatcher_reg, entry);
}

void _dispatcher(void)
{
    dispatcher_entry_t *entry;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;

    hdlc_mailbox_ptr = get_hdlc_mailbox();
    msg_t *msg, *msg2;
    char recv_data[HDLC_MAX_PKT_SIZE];

    hdlc_buf_t *buf;
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
                    memcpy(recv_data, buf->data, buf->length);
                    // PRINTF("dispatcher: received pkt %d; thread %d\n", recv_data[0],recv_data[1]);

                    if (recv_data[1] > 0)
                    {
                        LL_SEARCH_SCALAR(dispatcher_reg, entry, port, recv_data[1]);
                        if (entry != NULL)
                        {
                            msg2 = entry->mailbox->alloc();
                            if( msg2 == NULL)
                                break;
                            memcpy(msg2, msg, sizeof(msg_t));
                            entry->mailbox->put(msg2);
                            // PRINTF("dispatcher: received pkt %d; thread %d\n", recv_data[0],recv_data[1]);
                        }    
                        dispatcher_mailbox.free(msg);

                    }
                    else
                    {
                        PRINTF("dispatcher1: received pkt %d; thread %d\n", recv_data[0],recv_data[1]);
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
