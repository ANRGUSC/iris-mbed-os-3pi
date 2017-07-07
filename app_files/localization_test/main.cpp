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
 * @brief       Full-duplex hdlc test using a single thread (run on both sides).
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * 
 * In this test, the main thread and thread2 thread will contend for the same
 * UART line to communicate to another MCU also running a main and thread2 
 * thread. It seems as though stability deteriorates if the hdlc thread is given
 * a higher priority than the two application threads (RIOT's MAC layer priority
 * is well below the default priority for the main thread. Note that two threads
 * are equally contending for the UART line, one thread may starve the other to 
 * the point where the other thread will continue to retry. Increasing the msg 
 * queue size of hdlc's thread may also increase stability. Since this test can
 * easily stress the system, carefully picking the transmission rates (see below)
 * and tuning the RTRY_TIMEO_USEC and RETRANSMIT_TIMEO_USEC timeouts in hdlc.h
 * may lead to different stability results. The following is one known stable
 * set of values for running this test:
 *
 * -100ms interpacket intervals in xtimer_usleep() below
 * -RTRY_TIMEO_USEC = 100000
 * -RETRANSMIT_TIMEO_USEC 50000
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
#include "main-conf.h"

#define DEBUG   0

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */
Serial                          pc(USBTX,USBRX,115200);
DigitalOut                      myled3(LED3); //to notify when a character was received on mbed
DigitalOut                      myled(LED1);

Mail<msg_t, HDLC_MAILBOX_SIZE>  thread2_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;


#define MAIN_THR_PORT   5678    
#define NULL_PKT_TYPE   0xFF 
#define PKT_FROM_MAIN_THR   0
#define RANGE_PKT           1
#define RANGE_REQ           100

#define ONE_SENSOR_MODE       0x60 // 96
#define TWO_SENSOR_MODE       0x61 // 97
#define XOR_SENSOR_MODE       0x62 // 98

int main(void)
{
    myled = 1;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    msg_t *msg, *msg2;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t pkt;
    pkt.data = send_data;
    pkt.length = 0;
    hdlc_buf_t *buf;
    uart_pkt_hdr_t recv_hdr;
    uart_pkt_hdr_t send_hdr = { MAIN_THR_PORT, MAIN_THR_PORT, RANGE_REQ };
    PRINTF("In main\n");
    hdlc_entry_t main_thr = { NULL, MAIN_THR_PORT, &main_thr_mailbox };
    hdlc_register(&main_thr);


    int exit = 0;
    osEvent evt;
    const char* input;
    uint32_t ranging_type;
    uint32_t* time_diffs;

    while(1)
    {
        
        ranging_type = XOR_SENSOR_MODE;

        myled=!myled;

        pkt.length = HDLC_MAX_PKT_SIZE;        

        uart_pkt_insert_hdr(pkt.data, pkt.length, &send_hdr); 
        uart_pkt_cpy_data(pkt.data, pkt.length, &ranging_type, sizeof(uint32_t));

        /* send pkt */
        msg = hdlc_mailbox_ptr->alloc();
        msg->type = HDLC_MSG_SND;
        msg->content.ptr = &pkt;
        msg->sender_pid = osThreadGetId();
        msg->source_mailbox = &main_thr_mailbox;
        hdlc_mailbox_ptr->put(msg);

        PRINTF("main_thread: sending pkt \n");

        while(1)
        {
            myled=!myled;
            PRINTF("Waiting for response\n");
            evt = main_thr_mailbox.get();
            
            if (evt.status == osEventMail) 
            {
                PRINTF("Got a response\n");
                msg = (msg_t*)evt.value.p;

                switch (msg->type)
                {
                    case HDLC_RESP_SND_SUCC:
                        PRINTF("main_thread: sent frame!\n");
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
                        if (recv_hdr.pkt_type == RANGE_PKT) {
                            memcpy(recv_data, buf->data, buf->length);
                            PRINTF("main_thr: received range pkt\n");
                            time_diffs= (uint32_t *) uart_pkt_get_data(buf->data, buf->length);

                            printf("TDoA = %lu\n", time_diffs[0]);
                            switch (ranging_type){
                                case ONE_SENSOR_MODE:
                                    break;

                                case TWO_SENSOR_MODE:
                                    if(time_diffs[2]!=0){
                                        printf("Missed pin %lu\n", time_diffs[2]);
                                    } else{
                                        printf("OD = %lu\n", time_diffs[1]);
                                    }
                                    break;

                                case XOR_SENSOR_MODE:
                                    printf("OD = %lu\n", time_diffs[1]);
                                    break;
                            }
                        }
                        else{
                            printf("main_thr: recieved non-range pkt\n");
                        }
                        main_thr_mailbox.free(msg);
                        hdlc_pkt_release(buf);
                        exit = 1;
                        break;
                    default:
                        PRINTF("Reached default case\n");
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
        PRINTF("Reached end of loop\n");
        //Thread::wait(100);
    }
    PRINTF("Reached Exit");
    /* should be never reached */
    return 0;
}
