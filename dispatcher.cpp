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
#include <iostream>
#include <map>
#include <string>
#include "dispatcher.h"

#define DEBUG   0

#if (DEBUG) 
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */

DigitalOut led1(LED1);
Mail<msg_t, HDLC_MAILBOX_SIZE> dispatcher_mailbox;
Thread dispatcher; 
std::map <char, void*> mailbox_list;
static int registered_thr_cnt=0;

void _dispatcher(void)
{
    led1=1;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = get_hdlc_mailbox();
    msg_t *msg, *msg2;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
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


    // Thread thread1(_thread1);
    // dispacher_ready=1;
    PRINTF("dispatcher pid is %d \n", osThreadGetId());

    osEvent evt;
    while(1)
    {
        led1=!led1;
        Thread::wait(1000);
        evt = dispatcher_mailbox.get();

        if (evt.status == osEventMail) 
        {
            msg = (msg_t*)evt.value.p;
            switch (msg->type)
            {
                case HDLC_PKT_RDY:
                    buf = (hdlc_buf_t *)msg->content.ptr;   
                    memcpy(recv_data, buf->data, buf->length);
                    hdlc_pkt_release(buf);
                    dispatcher_mailbox.free(msg);
                    printf("dispatcher: received pkt %d\n", recv_data[0]);
                    break;
                default:
                    dispatcher_mailbox.free(msg);
                    /* error */
                    //LED3_ON;
                    break;
            }    

        }

    }
    PRINTF("Reached Exit");
    /* should be never reached */
    // return 0;
}
Mail<msg_t, HDLC_MAILBOX_SIZE>* get_dispacther_mailbox()
{
    return &dispatcher_mailbox;
}


Mail<msg_t, HDLC_MAILBOX_SIZE>* dispacher_init() 
{
  
    dispatcher.start(_dispatcher);
    PRINTF("dispatcher: thread  id %d\n",dispatcher.gettid());
    // dispacher_ready=0;
    return &dispatcher_mailbox;
}
