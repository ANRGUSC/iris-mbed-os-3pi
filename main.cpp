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
#define DEBUG   1

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */
Serial pc(USBTX,USBRX,115200);
DigitalOut myled3(LED3); //to notify when a character was received on mbed

DigitalOut myled(LED1);
Mail<msg_t, HDLC_MAILBOX_SIZE> dispatcher_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE> thread1_mailbox;

void _thread1()
{
/* Initial Direction of the Antenna*/

    char thread1_frame_no = 0;
    msg_t *msg, *msg2;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t *pkt= new hdlc_pkt_t;
    pkt->data = send_data;
    pkt->length = 0;
    hdlc_buf_t *buf;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr=get_hdlc_mailbox();
    int exit = 0;
    osEvent evt;

    while (true) 
    {

        myled3=!myled3;
        pkt->data[0] = thread1_frame_no;

        for(int i = 1; i < HDLC_MAX_PKT_SIZE; i++) {
            pkt->data[i] = (char) ( rand() % 0x7E);
        }

        pkt->length = HDLC_MAX_PKT_SIZE;

        /* send pkt */
        msg = hdlc_mailbox_ptr->alloc();
        msg->type = HDLC_MSG_SND;
        msg->content.ptr = pkt;
        msg->sender_pid = osThreadGetId();
        msg->source_mailbox = &thread1_mailbox;
        hdlc_mailbox_ptr->put(msg);
        PRINTF("thread1: sending pkt no %d \n", thread1_frame_no);

        while(1)
        {
            evt = thread1_mailbox.get();
            if (evt.status == osEventMail) 
            {
                msg = (msg_t*)evt.value.p;
                switch (msg->type)
                {
                    case HDLC_RESP_SND_SUCC:
                        PRINTF("thread1: sent frame_no %d!\n", thread1_frame_no);
                        exit = 1;
                        thread1_mailbox.free(msg);
                        break;    
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg->content.value/1000);
                        PRINTF("thread1: retry frame_no %d \n", thread1_frame_no);
                        msg2 = hdlc_mailbox_ptr->alloc();
                        while (msg2 == NULL) {
                            Thread::wait(50);
                            msg2 = thread1_mailbox.alloc();  
                            msg2->type = HDLC_RESP_RETRY_W_TIMEO;
                            msg2->content.value = (uint32_t) RTRY_TIMEO_USEC;
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &thread1_mailbox;
                            thread1_mailbox.put(msg2);
                            thread1_mailbox.free(msg);
                            break;
                        }
                        msg2->type = HDLC_MSG_SND;
                        msg2->content.ptr = pkt;
                        msg2->sender_pid = osThreadGetId();
                        msg2->source_mailbox = &thread1_mailbox;
                        hdlc_mailbox_ptr->put(msg2);
                        thread1_mailbox.free(msg);
                        break;
                    case HDLC_PKT_RDY:
                        buf = (hdlc_buf_t *)msg->content.ptr;   
                        memcpy(recv_data, buf->data, buf->length);
                        hdlc_pkt_release(buf);
                        thread1_mailbox.free(msg);
                        PRINTF("thread1: received pkt %d\n", recv_data[0]);
                        break;
                    default:
                        thread1_mailbox.free(msg);
                        /* error */
                        //LED3_ON;
                        break;
                }
            }    
            if(exit) {
                exit = 0;
                break;
            }
        }

        thread1_frame_no++;
        Thread::wait(1000);

    }
}

int main(void)
{
    myled=1;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    PRINTF("In main");
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
    msg_t *msg, *msg2;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t *pkt= new hdlc_pkt_t;
    pkt->data = send_data;
    pkt->length = 0;
    hdlc_buf_t *buf;
    PRINTF("In main");

    msg = hdlc_mailbox_ptr->alloc();
    msg->type = HDLC_MSG_REG_DISPATCHER;
    msg->sender_pid = osThreadGetId();
    msg->source_mailbox = &dispatcher_mailbox;
    hdlc_mailbox_ptr->put(msg);

    PRINTF("dispatcher pid is %d \n", osThreadGetId());

    Thread thread1(_thread1);

    int exit = 0;
    osEvent evt;
    while(1)
    {
        myled=!myled;
        pkt->data[0] = frame_no;
        pkt->data[1] = frame_no;

        for(int i = 1; i < HDLC_MAX_PKT_SIZE; i++) {
            pkt->data[i] = (char) ( rand() % 0x7E);
        }

        pkt->length = HDLC_MAX_PKT_SIZE;

        /* send pkt */
        msg = hdlc_mailbox_ptr->alloc();
        msg->type = HDLC_MSG_SND;
        msg->content.ptr = pkt;
        msg->sender_pid = osThreadGetId();
        msg->source_mailbox = &dispatcher_mailbox;
        hdlc_mailbox_ptr->put(msg);

        printf("dispatcher: sending pkt no %d \n", frame_no);

        while(1)
        {
            myled=!myled;
            evt = dispatcher_mailbox.get();

            if (evt.status == osEventMail) 
            {
                msg = (msg_t*)evt.value.p;

                switch (msg->type)
                {
                    case HDLC_RESP_SND_SUCC:
                        printf("dispatcher: sent frame_no %d!\n", frame_no);
                        exit = 1;
                        dispatcher_mailbox.free(msg);
                        break;
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg->content.value/1000);
                        PRINTF("dispatcher: retry frame_no %d \n", frame_no);
                        msg2 = hdlc_mailbox_ptr->alloc();
                        while (msg2 == NULL) {
                            Thread::wait(50);
                            msg2 = dispatcher_mailbox.alloc();  
                            msg2->type = HDLC_RESP_RETRY_W_TIMEO;
                            msg2->content.value = (uint32_t) RTRY_TIMEO_USEC;
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &dispatcher_mailbox;
                            dispatcher_mailbox.put(msg2);
                            dispatcher_mailbox.free(msg);
                            break;
                        }
                        msg2->type = HDLC_MSG_SND;
                        msg2->content.ptr = pkt;
                        msg2->sender_pid = osThreadGetId();
                        msg2->source_mailbox = &dispatcher_mailbox;
                        hdlc_mailbox_ptr->put(msg2);
                        dispatcher_mailbox.free(msg);
                        break;
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
            if(exit) {
                exit = 0;
                break;
            }
        }

        frame_no++;
        Thread::wait(1000);

    }
    PRINTF("Reached Exit");
    /* should be never reached */
    return 0;
}
