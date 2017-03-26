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
#include "main-conf.h"

#define DEBUG   1

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */
Serial                          pc(USBTX,USBRX,115200);
DigitalOut                      myled3(LED3); //to notify when a character was received on mbed
DigitalOut                      myled(LED1);

Mail<msg_t, HDLC_MAILBOX_SIZE>  thread1_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;


void _thread1()
{
/* Initial Direction of the Antenna*/

    char thread1_frame_no = 0;
    msg_t *msg, *msg2;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t pkt;
    pkt.data = send_data;
    pkt.length = 0;
    hdlc_buf_t *buf;
    uart_pkt_hdr_t recv_hdr;
    uart_pkt_hdr_t send_hdr = { THREAD1_PORT, THREAD1_PORT, NULL_PKT_TYPE };
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = get_hdlc_mailbox();
    int exit = 0;
    dispatcher_entry_t thread1 = { NULL, THREAD1_PORT, &thread1_mailbox };
    dispatcher_register(&thread1);

    osEvent evt;

    while (1) 
    {

        myled3 =! myled3;
        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 

        pkt.data[UART_PKT_DATA_FIELD] = thread1_frame_no;
        for(int i = UART_PKT_DATA_FIELD + 1; i < HDLC_MAX_PKT_SIZE; i++) {
            pkt.data[i] = (char) ( rand() % 0x7E);
        }

        pkt.length = HDLC_MAX_PKT_SIZE;

        /* send pkt */
        msg = hdlc_mailbox_ptr->alloc();
        msg->type = HDLC_MSG_SND;
        msg->content.ptr = &pkt;
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
                        PRINTF("main_thr: retry frame_no %d \n", thread1_frame_no);
                        msg2 = hdlc_mailbox_ptr->alloc();
                        if (msg2 == NULL) {
                            Thread::wait(50);
                            while (msg2 == NULL)
                            {
                                msg2 = thread1_mailbox.alloc();  
                                Thread::wait(10);
                            }
                            msg2->type = HDLC_RESP_RETRY_W_TIMEO;
                            msg2->content.value = (uint32_t) RTRY_TIMEO_USEC;
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &thread1_mailbox;
                            thread1_mailbox.put(msg2);
                            thread1_mailbox.free(msg);
                            break;
                        }
                        msg2->type = HDLC_MSG_SND;
                        msg2->content.ptr = &pkt;
                        msg2->sender_pid = osThreadGetId();
                        msg2->source_mailbox = &thread1_mailbox;
                        hdlc_mailbox_ptr->put(msg2);
                        thread1_mailbox.free(msg);
                        break;
                    case HDLC_PKT_RDY:
                        buf = (hdlc_buf_t *)msg->content.ptr;   
                        uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
                        if (recv_hdr.pkt_type == NULL_PKT_TYPE) {
                            memcpy(recv_data, buf->data, buf->length);
                            printf("thread1: received pkt %d ; dst_port %d\n", 
                            recv_data[UART_PKT_DATA_FIELD], recv_hdr.dst_port);
                        }
                        thread1_mailbox.free(msg);
                        hdlc_pkt_release(buf);
                        break;
                    default:
                        thread1_mailbox.free(msg);
                        /* error */
                        break;
                }
            }    
            if(exit) {
                exit = 0;
                break;
            }
        }

        thread1_frame_no++;
        Thread::wait(500);
    }
}


int main(void)
{
    myled = 1;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *dispatch_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    dispatch_mailbox_ptr = dispatcher_init();

    msg_t *msg, *msg2;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t pkt;
    pkt.data = send_data;
    pkt.length = 0;
    hdlc_buf_t *buf;
    uart_pkt_hdr_t recv_hdr;
    uart_pkt_hdr_t send_hdr = { MAIN_THR_PORT, MAIN_THR_PORT, NULL_PKT_TYPE };
    PRINTF("In main\n");
    dispatcher_entry_t main_thr = { NULL, MAIN_THR_PORT, &main_thr_mailbox };
    dispatcher_register(&main_thr);

    Thread thr;
    thr.start(_thread1);

    int exit = 0;
    osEvent evt;
    while(1)
    {
        myled=!myled;
        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 

        pkt.data[UART_PKT_DATA_FIELD] = frame_no;
        for(int i = UART_PKT_DATA_FIELD + 1; i < HDLC_MAX_PKT_SIZE; i++) {
            pkt.data[i] = (char) ( rand() % 0x7E);
        }

        pkt.length = HDLC_MAX_PKT_SIZE;

        /* send pkt */
        msg = hdlc_mailbox_ptr->alloc();
        msg->type = HDLC_MSG_SND;
        msg->content.ptr = &pkt;
        msg->sender_pid = osThreadGetId();
        msg->source_mailbox = &main_thr_mailbox;
        hdlc_mailbox_ptr->put(msg);

        PRINTF("main_thread: sending pkt no %d \n", frame_no);

        while(1)
        {
            myled=!myled;
            evt = main_thr_mailbox.get();

            if (evt.status == osEventMail) 
            {
                msg = (msg_t*)evt.value.p;

                switch (msg->type)
                {
                    case HDLC_RESP_SND_SUCC:
                        PRINTF("main_thread: sent frame_no %d!\n", frame_no);
                        exit = 1;
                        main_thr_mailbox.free(msg);
                        break;
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg->content.value/1000);
                        PRINTF("main_thr: retry frame_no %d \n", frame_no);
                        msg2 = hdlc_mailbox_ptr->alloc();
                        if (msg2 == NULL) {
                            // Thread::wait(50);
                            while(msg2==NULL)
                            {
                                msg2 = main_thr_mailbox.alloc();  
                                Thread::wait(10);
                            }
                            msg2->type = HDLC_RESP_RETRY_W_TIMEO;
                            msg2->content.value = (uint32_t) RTRY_TIMEO_USEC;
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &main_thr_mailbox;
                            main_thr_mailbox.put(msg2);
                            main_thr_mailbox.free(msg);
                            break;
                        }
                        msg2->type = HDLC_MSG_SND;
                        msg2->content.ptr = &pkt;
                        msg2->sender_pid = osThreadGetId();
                        msg2->source_mailbox = &main_thr_mailbox;
                        hdlc_mailbox_ptr->put(msg2);
                        main_thr_mailbox.free(msg);
                        break;
                    case HDLC_PKT_RDY:
                        buf = (hdlc_buf_t *)msg->content.ptr;   
                        uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
                        if (recv_hdr.pkt_type == NULL_PKT_TYPE) {
                            memcpy(recv_data, buf->data, buf->length);
                            printf("main_thr: received pkt %d ; dst_port %d\n", 
                            recv_data[UART_PKT_DATA_FIELD], recv_hdr.dst_port);
                        }
                        main_thr_mailbox.free(msg);
                        hdlc_pkt_release(buf);
                        break;
                    default:
                        main_thr_mailbox.free(msg);
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
        Thread::wait(360);

    }
    PRINTF("Reached Exit");
    /* should be never reached */
    return 0;
}
