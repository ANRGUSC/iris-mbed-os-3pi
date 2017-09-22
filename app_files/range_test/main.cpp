/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Yutong Gu
 * Richard Kim
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
 * @brief       Ultrasound ranging test using packets passed over hdlc
 * 
 * In this test, the mbed will repeated send request packets to a dedicated 
 * thread for ranging on the openmote requesting range data. Therequest packets
 * sent will contain information on the mode to range with and number of 
 * samples to take at once. Available modes are ONE_SENSOR_MODE, 
 * TWO_SENSOR_MODE, and XOR_SENSOR_MODE. The ranging thread will automatically 
 * split the data into packets to send to the openmote. The mbed will alternate 
 * through all three options or repeatedly use one, taking a specified sample 
 * number SAMPS_PER_MODE, at a delay of LOOP_DELAY. The fastest this system can
 * range at is 100 ms and this is due to the hardware limitations of the 
 * ultrasound sensors.
 * 
 * Corresponding openmote program can be found in the 
 * examples/mbed_riot/tests/localization_test.c folder in anrg-riot repository 
 * on the localization branch.
 *
 * @author      Yutong Gu <yutonggu@usc.edu>
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

Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

#define LOOP_DELAY            0
#define SAMPS_PER_MODE        11

#define MAIN_THR_PORT   5678    
#define NULL_PKT_TYPE   0xFF 
#define PKT_FROM_MAIN_THR   0

#define DATA_PER_PKT        ((HDLC_MAX_PKT_SIZE - UART_PKT_HDR_LEN - 1) / RANGE_DATA_LEN)

typedef struct __attribute__((packed)) {
    uint8_t         last_pkt;      
    range_data_t    data[DATA_PER_PKT];                  
} range_hdr_t;

int main(void)
{
    /* mbed setup */
    myled = 1;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
    int pkt_size = sizeof(uart_pkt_hdr_t) + sizeof(range_params_t);

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
    range_hdr_t* range_hdr;

    int tdoa_a;
    int tdoa_b;
    float dist_a;
    float dist_b;
    float dist;
    float angle;

    // struct range_params rparams; ???

    int i = 0;
    int j = 0;
    int data_per_pkt;

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
          *     OMNI_SENSOR_MODE for omni_directional sensor system.
         */
        params.ranging_mode = OMNI_SENSOR_MODE;  
    
        i++;

        params.node_id = -1;

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
                            msg2->content.value = (uint32_t) HDLC_RTRY_TIMEO_USEC;
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
                            range_hdr = (range_hdr_t *)uart_pkt_get_data(buf->data, buf->length);
                            time_diffs = (range_data_t *)range_hdr->data;
                            
                            data_per_pkt = (buf->length - sizeof(uart_pkt_hdr_t) - sizeof(uint8_t))/sizeof(range_data_t);
                            PRINTF("There should be %d ranges in this pkt\n",data_per_pkt);

                            for(j = 0; j < data_per_pkt; j++){
                                PRINTF ("%d:\n", j);
                                tdoa_a = 0;
                                tdoa_b = 0;
                                /* Displaying results. */

                                if(time_diffs->status == RF_MISSED){
                                    printf("RF Ping missed\n");
                                    time_diffs++;
                                    continue;
                                }
                                else if(time_diffs->status == ULTRSND_MISSED){
                                    printf("Ultrsnd Ping missed\n");
                                    time_diffs++;
                                    continue;
                                }
                                
                                
                                tdoa_a = time_diffs->tdoa;
                                dist_a = get_dist(tdoa_a);
                                printf("TDoA = %lu\n", tdoa_a);

                                switch (params.ranging_mode)
                                {
                                    case ONE_SENSOR_MODE:
                                        dist = dist_a;
                                        break;
                                    case TWO_SENSOR_MODE:
                                        if(time_diffs->status > 2)
                                        {
                                            printf("Missed pin %lu\n", MISSED_PIN_UNMASK - time_diffs-> status); 
                                        } 
                                        else
                                        {
                                            tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                                            dist_b = get_dist(tdoa_b);
                                            printf("OD = %lu\n", time_diffs-> orient_diff);
                                        }
                                        break;
                                    case XOR_SENSOR_MODE:
                                        tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                                        dist_b = get_dist(tdoa_b);
                                        printf("OD = %lu\n", time_diffs-> orient_diff);
                                        break;
                                    case OMNI_SENSOR_MODE:
                                        dist = dist_a;
                                        break;
                                }

                                //printf("\n******************************\n", dist);
                               if(tdoa_b != 0){
                                    dist = get_mid_dist(dist_a, dist_b);
                                    if(time_diffs->status == 2){
                                        angle = get_angle(dist_b, dist_a);
                                    }
                                    else{
                                        angle = get_angle(dist_a, dist_b);
                                    }
                                    printf("Distance: %.2f\n", dist);
                                    printf("Angle : %.2f\n", angle);
                                }
                                else{
                                    printf("Distance: %.2f\n", dist);
                                }
                                 printf("******************************\n");

                            
                                time_diffs++;
                            }

                            if(range_hdr->last_pkt == 1){
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
            else{
                printf("Range: Didn't get mail: %02x\n",evt.status);
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
