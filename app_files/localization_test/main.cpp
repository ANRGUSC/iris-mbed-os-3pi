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
 * @author      Yutong Gu <yutonggu@usc.edu>
 * 
 * In this test, the mbed will repeated send packets to a dedicated thread for 
 * ranging on the openmote requesting range data. The packets sent will contain 
 * information on the mode to range with. Available options are ONE_SENSOR_MODE, 
 * TWO_SENSOR_MODE, and XOR_SENSOR_MODE. The loop will alternate through all 
 * three options, taking a specified sample number, SAMPS_PER_MODE, at a delay 
 * of LOOP_DELAY. The fastest this system can range at is 100 ms and this is due 
 * to the hardware limitations of the ultrasound sensors.
 * 
 *
 **/

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
#include "data_conv.h"

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

#define LOOP_DELAY            0
#define SAMPS_PER_MODE        1

// ???
// // mbed will pass an instance of this to the openmote.
// typedef struct range_params
// {
//     uint16_t runs = 10;
//     uint32_t ranging_type_data;
//     // add more options in the future?
// };


int main(void)
{
    /* mbed setup */
    myled = 1;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
    int pkt_size = sizeof(uart_pkt_hdr_t) + sizeof(range_params_t);
    int end_of_series = 0;
    // int pkt_size = sizeof(uart_pkt_hdr_t) + sizeof(range_params); ???

    /* openmote setup */
    msg_t *msg, *msg2;
    char frame_no = 0;
    char send_data[pkt_size];
    hdlc_pkt_t pkt;
    pkt.data = send_data;
    pkt.length = 0;
    hdlc_buf_t *buf;
    uart_pkt_hdr_t recv_hdr;
    uart_pkt_hdr_t send_hdr = { MAIN_THR_PORT, MAIN_THR_PORT, SOUND_RANGE_REQ };
    PRINTF("In main\n");
    hdlc_entry_t main_thr = { NULL, MAIN_THR_PORT, &main_thr_mailbox };
    hdlc_register(&main_thr);

    /* misc */
    int exit = 0;
    osEvent evt;
    range_params_t params;
    range_data_t* time_diffs;

    int tdoa_a;
    int tdoa_b;
    float dist_a;
    float dist_b;
    float dist;
    float angle;

    // struct range_params rparams; ???

    int i = 0;

    while(1)
    {
       
        i%=3;
        
        // if(i == 0){                
        //     printf("******************ONE SENSOR MODE*******************\n");
        //     params.ranging_mode = ONE_SENSOR_MODE;                
        // } 
        // else if(i == 1){                
        //     printf("******************TWO SENSOR MODE*******************\n");
        //     params.ranging_mode = TWO_SENSOR_MODE;            
        // } 
        // else if(i == 2){                
        //     printf("******************XOR SENSOR MODE*******************\n");
        //     params.ranging_mode = XOR_SENSOR_MODE;                
        // }

         /** 
         * Change this as needed.
         * Options:
         *      ONE_SENSOR_MODE for one sensor.
         *      TWO_SENSOR_MODE for two sensors.
         *      XOR_SENSOR_MODE for two sensors XOR'd together.
         */
         params.ranging_mode = TWO_SENSOR_MODE;  
    
        i++;

        params.num_samples = SAMPS_PER_MODE;

        /* Blinks the led. */
        myled = !myled;

        // rparams.ranging_type_data = ranging_type; ???
        pkt.length = pkt_size;        

        uart_pkt_insert_hdr(pkt.data, pkt.length, &send_hdr); 
        uart_pkt_cpy_data(pkt.data, pkt.length, &params, sizeof(range_params_t));
        // uart_pkt_cpy_data(pkt.data, pkt.length, &rparams, sizeof(rparams)); ???

        /* send pkt */
        PRINTF("main_thread: sending pkt \n");
        msg = hdlc_mailbox_ptr->alloc();
        msg->type = HDLC_MSG_SND;
        msg->content.ptr = &pkt;
        msg->sender_pid = osThreadGetId();
        msg->source_mailbox = &main_thr_mailbox;
        hdlc_mailbox_ptr->put(msg);

        while(1)
        {
            /* Blinks the led. */
            myled = !myled;

            PRINTF("Waiting for response\n");
            evt = main_thr_mailbox.get();
            
            if(evt.status == osEventMail)
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

                        /* Tries to allocate memory for msg2. */
                        msg2 = hdlc_mailbox_ptr->alloc();
                        if(msg2 == NULL) 
                        {
                            /* Blocking call until the memory has been allocated. */
                            // Thread::wait(50);
                            while(msg2 == NULL)
                            {
                                msg2 = main_thr_mailbox.alloc();  
                                Thread::wait(10);
                            }
                            msg2->type = HDLC_RESP_RETRY_W_TIMEO;
                            msg2->content.value = (uint32_t) RTRY_TIMEO_USEC;
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &main_thr_mailbox;
                            main_thr_mailbox.put(msg2);
                        }
                        else
                        {
                            msg2->type = HDLC_MSG_SND;
                            msg2->content.ptr = &pkt;
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &main_thr_mailbox;
                            hdlc_mailbox_ptr->put(msg2);
                        }
                        main_thr_mailbox.free(msg);
                        break;

                    case HDLC_PKT_RDY:
                        /* Setting up buf, making it easier to access data from the msg. */ 
                        buf = (hdlc_buf_t *)msg->content.ptr;   
                        uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);

                        if(recv_hdr.pkt_type == SOUND_RANGE_DONE) 
                        {
                            PRINTF("main_thr: received range pkt\n");
                            time_diffs = (range_data_t *)uart_pkt_get_data(buf->data, buf->length);
                            while(!end_of_series){
                                tdoa_a = 0;
                                tdoa_b = 0;
                                if(time_diffs->error > 2){
                                    time_diffs->error-=10;
                                    end_of_series = 1;
                                }
                                /* Displaying results. */
                                if(time_diffs->tdoa > 0){
                                    tdoa_a = time_diffs->tdoa;
                                    dist_a = tdoa_to_dist(tdoa_a);
                                    //printf("TDoA = %lu\n", tdoa_a);

                                    switch (params.ranging_mode)
                                    {
                                        case ONE_SENSOR_MODE:
                                            dist = dist_a;
                                            break;
                                        case TWO_SENSOR_MODE:
                                            if(time_diffs->error != 0)
                                            {
                                                printf("Missed pin %lu\n", time_diffs->error);
                                            } 
                                            else
                                            {
                                                tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                                                dist_b = tdoa_to_dist(tdoa_b);
                                                //printf("OD = %lu\n", tdoa_b);
                                            }
                                            break;
                                        case XOR_SENSOR_MODE:
                                            tdoa_b = tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                                            dist_b = tdoa_to_dist(tdoa_b);
                                            //printf("OD = %lu\n", tdoa_b);
                                            break;
                                    }

                                    //printf("\n******************************\n", dist);
                                    if(tdoa_b != 0){
                                        dist = calc_x(dist_a, dist_b);
                                        angle = od_to_angle(dist_a, dist_b);
                                        printf("Distance: %.2f\n", dist);
                                        printf("Angle : %.2f\n", angle);
                                    }
                                    else{
                                        printf("Distance: %.2f\n", dist);
                                    }
                                     printf("******************************\n", dist);

                                } else{
                                    printf("Ultrsnd Ping missed\n");
                                }
                                time_diffs++;
                            }
                            end_of_series = 0;
                            if(recv_hdr.msg_complete == 1){
                                printf("All data recieved\n");
                                exit = 1;
                            }
                        }
                        else
                        {
                            printf("main_thr: recieved non-range pkt\n");
                            exit = 1;
                        }
                        hdlc_pkt_release(buf);
                        main_thr_mailbox.free(msg);
                        break;
                    default:
                        PRINTF("Reached default case\n");
                        /* error */
                        //LED3_ON;
                        main_thr_mailbox.free(msg);
                        break;
                }
            }    
            if(exit) 
            {
                exit = 0;
                break;
            }
        }
        frame_no++;
        PRINTF("Reached end of loop\n");
        if(LOOP_DELAY != 0){
            Thread::wait(LOOP_DELAY);
        }
        
    }
    PRINTF("Reached Exit");
    /* should be never reached */
    return 0;
}
