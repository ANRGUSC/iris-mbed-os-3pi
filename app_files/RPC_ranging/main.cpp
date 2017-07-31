/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Yutong Gu
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
 * In this test, the mbed is being directly controlled via RPC to transmit range
 * requests to the openmote and outputting the data onto the terminal. Because
 * the RPC function runs in an IRQ context, our mailbox classes will not work 
 * properly unless put in a separate thread. That is the function of range_thread.
 * Range_thread listens for a message from the RPC function to initiate and the 
 * RPC function is not completed until range_thread is complete and recieves a 
 * message confirming its completion.
 * 
 * Corresponding openmote program can be found in the 
 * examples/mbed_riot/tests/localization_test.c folder in anrg-riot repository 
 * on the localization branch.
 * 
 * The SerialRPCInterface code was borrowed from the following link: 
 * https://developer.mbed.org/users/MichaelW/code/RPCInterface/docs/9d82e28ffaea/classmbed_1_1SerialRPCInterface.html
 * 
 * An example of its use can be found here:
 * https://developer.mbed.org/cookbook/RPC-Interface-Library
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
#include "SerialRPCInterface.h"

#define DEBUG   0 

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */
/**
* Copyright (c)2010 ARM Ltd.
* Released under the MIT License: http://mbed.org/license/mit
*/

Serial                          pc(USBTX,USBRX,115200);

#define RANGE_THR_PORT   5678    
#define NULL_PKT_TYPE   0xFF 
#define PKT_FROM_MAIN_THR   0

#define DATA_PER_PKT        ((HDLC_MAX_PKT_SIZE - UART_PKT_HDR_LEN - 1) / RANGE_DATA_LEN)

#define LOOP_DELAY            0
#define SAMPS_PER_MODE        1

typedef struct __attribute__((packed)) {
    uint8_t         last_pkt;      
    range_data_t    data[DATA_PER_PKT];                  
} range_hdr_t;
 
using namespace mbed;
Mail<msg_t, HDLC_MAILBOX_SIZE>  range_thr_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE>  RPC_mailbox;
 
//Create the interface on the USB Serial Port
SerialRPCInterface RPC(USBTX, USBRX);

void range_RPC(Arguments * input, Reply * output);


RPCFunction range_rx(&range_RPC, "range_rx");

DigitalOut                      myled3(LED3); //to notify when a character was received on mbed
DigitalOut                      myled(LED1);

static uint8_t mode = ONE_SENSOR_MODE;
static uint16_t samples = 1;
volatile uint8_t loopbegin = 0;

/**
 * @brief      This is the thread that will communicate with hdlc and request ranging data
 * 
 * This thread will wait until it recieves a message from the range_RPC function which is only called
 * when SerialRPCInterface recieves the appropriate call from the pc. 
 * 
 */
void range_thread(){

    PRINTF("Range: Now in ranging thread\n");
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
    uart_pkt_hdr_t send_hdr = { RANGE_THR_PORT, RANGE_THR_PORT, SOUND_RANGE_REQ };
    PRINTF("In main\n");
    hdlc_entry_t main_thr = { NULL, RANGE_THR_PORT, &range_thr_mailbox };
    hdlc_register(&main_thr);


    /* misc */
    int exit = 0;
    int j = 0;
    osEvent evt;
    range_params_t params;
    range_data_t* time_diffs;
    range_hdr_t* range_hdr;

    int tdoa_a = 0;
    int tdoa_b = 0;
    float dist_a = 0;
    float dist_b = 0;
    float dist = 0;
    float angle = 0;
    int data_per_pkt;
    while(1){
        PRINTF("Range: Waiting for RANGE_THR_START\n");
        evt = range_thr_mailbox.get();
        
        if(evt.status == osEventMail){
            msg = (msg_t*)evt.value.p;
            if(msg->type == RANGE_THR_START){
                PRINTF("Range: Got start message! Starting...\n");
                range_thr_mailbox.free(msg);
            } else{
                PRINTF("Range: Got something other than start message\n");
                range_thr_mailbox.free(msg);
                continue;
            }
        }

        params.ranging_mode = mode;  
        
        params.num_samples = samples;

        /* Blinks the led. */
        myled = !myled;

        // rparams.ranging_type_data = ranging_type; ???
        pkt.length = pkt_size;        

        uart_pkt_insert_hdr(pkt.data, pkt.length, &send_hdr); 
        uart_pkt_cpy_data(pkt.data, pkt.length, &params, sizeof(range_params_t));
        // uart_pkt_cpy_data(pkt.data, pkt.length, &rparams, sizeof(rparams)); ???

        /* send pkt */
        PRINTF("Range: sending pkt \n");
        msg = hdlc_mailbox_ptr->alloc();
        msg->type = HDLC_MSG_SND;
        msg->content.ptr = &pkt;
        msg->sender_pid = osThreadGetId();
        msg->source_mailbox = &range_thr_mailbox;
        hdlc_mailbox_ptr->put(msg);

        while(1)
        {
           /* Blinks the led. */
            myled = !myled;

            PRINTF("Range: Waiting for response\n");

            evt = range_thr_mailbox.get();

            if(evt.status == osEventMail)
            {
                PRINTF("Range: Got a response\n");
                msg = (msg_t*)evt.value.p;

                switch (msg->type)
                {
                    case HDLC_RESP_SND_SUCC:
                        PRINTF("Range: sent frame!\n");
                        range_thr_mailbox.free(msg);
                        break;
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg->content.value/1000);
                        PRINTF("Range: retry frame_no %d \n", frame_no);

                        /* Tries to allocate memory for msg2. */
                        msg2 = hdlc_mailbox_ptr->alloc();
                        if(msg2 == NULL) 
                        {
                            /* Blocking call until the memory has been allocated. */
                            // Thread::wait(50);
                            while(msg2 == NULL)
                            {
                                msg2 = range_thr_mailbox.alloc();  
                                Thread::wait(10);
                            }
                            msg2->type = HDLC_RESP_RETRY_W_TIMEO;
                            msg2->content.value = (uint32_t) RTRY_TIMEO_USEC;
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &range_thr_mailbox;
                            range_thr_mailbox.put(msg2);
                        }
                        else
                        {
                            msg2->type = HDLC_MSG_SND;
                            msg2->content.ptr = &pkt;
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &range_thr_mailbox;
                            hdlc_mailbox_ptr->put(msg2);
                        }
                        range_thr_mailbox.free(msg);
                        break;

                    case HDLC_PKT_RDY:
                        /* Setting up buf, making it easier to access data from the msg. */ 
                        buf = (hdlc_buf_t *)msg->content.ptr;   
                        uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);

                        if(recv_hdr.pkt_type == SOUND_RANGE_DONE) 
                        {
                            PRINTF("Range: received range pkt\n");
                            range_hdr = (range_hdr_t *)uart_pkt_get_data(buf->data, buf->length);
                            time_diffs = (range_data_t *)range_hdr->data;
                            
                            data_per_pkt = (buf->length - sizeof(uart_pkt_hdr_t) - sizeof(uint8_t))/sizeof(range_data_t);
                            PRINTF("Range: There should be %d ranges in this pkt\n",data_per_pkt);

                            for(j = 0; j < data_per_pkt; j++){
                                PRINTF ("Range: %d\n", j);
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
                                dist_a = tdoa_to_dist(tdoa_a);
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
                                            dist_b = tdoa_to_dist(tdoa_b);
                                            printf("OD = %lu\n", time_diffs-> orient_diff);
                                        }
                                        break;
                                    case XOR_SENSOR_MODE:
                                        tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                                        dist_b = tdoa_to_dist(tdoa_b);
                                        printf("OD = %lu\n", time_diffs-> orient_diff);
                                        break;
                                }

                                //printf("\n******************************\n", dist);
                                if(tdoa_b != 0){
                                    dist = calc_x(dist_a, dist_b);
                                    if(time_diffs->status == 2){
                                        angle = od_to_angle(dist_b, dist_a);
                                    }
                                    else{
                                        angle = od_to_angle(dist_a, dist_b);
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
                                PRINTF("Range: All data recieved\n");
                                exit = 1;
                            }
                        }
                        else
                        {
                            printf("Range: recieved non-range pkt %d\n",recv_hdr.pkt_type);
                            exit = 1;
                        }
                        hdlc_pkt_release(buf);
                        range_thr_mailbox.free(msg);
                        break;
                    default:
                        PRINTF("Range: Reached default case\n");
                        /* error */
                        //LED3_ON;
                        range_thr_mailbox.free(msg);
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
        msg2 = RPC_mailbox.alloc();
        msg2->type = RANGE_THR_COMPLETE;
        msg2->content.ptr = NULL;
        msg2->sender_pid = osThreadGetId();
        msg2->source_mailbox = &range_thr_mailbox;
        RPC_mailbox.put(msg2);
    }

}


/**
 * @brief      This function will trigger the range_thread upon being called from the RPC
 * 
 * When SerialRPCInterface recieves a command of the form /range_rx/run arg1 arg2, it will run this function which will trigger 
 * the range_thread to request data over hdlc from the openmote. This is neccessary because range_rx will be run in an irq context
 * which causes hdlc and mailbox to run into issues.
 *
 * @param      input   The arguments input from the pc as an Arguments struct put together by the RPC
 * @param      output  The response after completing this call as a Reply struct (not used in this function)
 */
void range_RPC(Arguments* input, Reply* output){
    msg_t *tx_msg, *rx_msg;
    osEvent RPC_evt;

    if(input->argc != 2){
        printf("usage: <num_samples> <ranging_mode>\n");
        return;
    }

    mode = atoi(input->argv[1]);
    samples = atoi(input->argv[0]);
    switch (mode){
        case 0:
            mode = ONE_SENSOR_MODE;
            printf("ONE_SENSOR_MODE:\n");
            break;
        case 1:
            mode = TWO_SENSOR_MODE;
            printf("TWO_SENSOR_MODE:\n");
            break;
        case 2:
            mode = XOR_SENSOR_MODE;
            printf("XOR_SENSOR_MODE:\n");
            break;
        default:
            printf("Invalid ranging mode entry\nValid entries are:\n0: ONE_SENSOR_MODE\n1: TWO_SENSOR_MODE\n2: XOR_SENSOR_MODE\n");
            return;
    }

    if(samples <= 0){
        printf("Error: num_samples must be greater than 0\n");
        return;
    }

    PRINTF("RPC: Starting ranging thread\n");
    tx_msg = range_thr_mailbox.alloc();
    tx_msg->type = RANGE_THR_START;
    tx_msg->content.ptr = NULL;
    tx_msg->sender_pid = osThreadGetId();
    tx_msg->source_mailbox = &RPC_mailbox;
    range_thr_mailbox.put(tx_msg);

    RPC_evt = RPC_mailbox.get();
    if(RPC_evt.status == osEventMail){
        rx_msg = (msg_t*)RPC_evt.value.p;
        if(rx_msg->type == RANGE_THR_COMPLETE){
            PRINTF("RPC: Ranging complete\n");
        } else{
            PRINTF("RPC: Ranging failed\n");
        }
    }
    RPC_mailbox.free(rx_msg);

    return;

}

int main() {
    Thread thr;
    thr.start(range_thread);
    while(1) {
        wait(0.4);
    }
}