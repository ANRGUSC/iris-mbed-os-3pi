/**
 * Copyright (c) 2018, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
 * Pradipta Ghosh
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
 * @brief       Chain follower example for IROS submission.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Yutong Gu <yutonggu@usc.edu>
 */

#include "mbed.h"
#include "rtos.h"
#include "hdlc.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "fcs16.h"
#include "uart_pkt.h"
#include "main-conf.h"
#include "mqtt.h"
#include "range.h"
#include "controller.h"
#include "m3pi.h"

#define START_RANGE_MSG   ("INIT_RANGE")

#define DEBUG   1
#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

// the only instance of pc -- debug statements in other files depend on it
Serial                          pc(USBTX,USBRX,115200);
DigitalOut                      myled3(LED3); //to notify when a character was received on mbed
DigitalOut                      myled(LED1);

Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

#define NULL_PKT_TYPE       0xFF 
#define PKT_FROM_MAIN_THR   0
#define MBED_MAIN_PORT      6000

//uncomment for the first robot (leading robot is actually last robot)
// #define LEADER_ROBOT
// #define LEADING_ROBOT_IPV6_ADDR     "ff02"
// #define FOLLOWING_ROBOT_IPV6_ADDR   "ff02"

//uncomment for the second robot
// #define SECOND_ROBOT
// #define LEADING_ROBOT_IPV6_ADDR     "ff02"
// #define FOLLOWING_ROBOT_IPV6_ADDR   "ff02"

//uncomment for the third robot
// #define END_ROBOT
// #define LEADING_ROBOT_IPV6_ADDR     "ff02"
// #define FOLLOWING_ROBOT_IPV6_ADDR   "ff02"

int main(void)
{
    static Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    PRINTF("Starting mqtt thread\n");
    Thread mqtt_thr;
    mqtt_thr.start(_mqtt_thread);

    PRINTF("Starting range thread\n");
    init_range_thread();
    
    PRINTF("Starting controller thread\n");
    controller_init(osPriorityNormal);
    PRINTF("Started controller thread\n");

    /* no need for a separate ranging thread here */

    hdlc_entry_t main = { NULL, MBED_MAIN_PORT, &main_thr_mailbox };
    hdlc_register(&main);

    myled = 1;
    range_params_t params;
    range_data_t range_data;
    dist_angle_t range_dist_angle;

    float dist_estimate = 10, angle_estimate = 10;
    while(1)
    {
        myled =! myled;
        switch(state) {
            case STOP_BEACONS:
                //ack any stop messages
                //wait for a RANGE_ME_MSG 
                //when RANGE_ME_MSG received, request range and change state
                state = RANGING_STATE;
                break;
            case RANGING_STATE:
                //ack any RANGE_ME_MSG
                //wait for ranging thread to tell you ranging is done (repeat req is failed)
                //when ranging is done, send movement request to thread and change state
                state = SEND_STOP_STATE;
                break;
            case SEND_STOP_STATE:
                //keep sending stop signals
                //when stop ack is received, turn on beaconing and change state
                state = RANGE_ME_ACK_STATE;
                break;
            case RANGE_ME_ACK_STATE:
                //repeat "go range me" messages
                //if a "go range me" ack is received, go to range_me_no_ack state
                state = RANGE_ME_NO_ACK_STATE;
                //if a stop beacon msg received, go to stop_beacon state
                state = STOP_BEACONS;
            case RANGE_ME_NO_ACK_STATE:
                //if a A msg received, send stop-beacon-ack and change to stop-beacon state
        }

        // Wait for instruction from the leader node.
        
        
        // request ranging from the RIOT device
        if(!is_ranging()){
            PRINTF("main_thr: requesting RIOT device to range\n");
            params.node_id = 0; //TODO
            params.ranging_mode = TWO_SENSOR_MODE;

            // get_range_data() should block, but keep in mind this function is
            // currently NOT thread safe. Make the thread calling this is not 
            // receiving any other kind of messages before calling this
            range_data = get_range_data(range_params_t); 
            range_dist_angle = get_dist_angle(&range_data, TWO_SENSOR_MODE); 
            // TODO: error handling in case of ranging failure
        }

        PRINTF("main_thr: ranging done\n");

        // now that ranging is done, send message to the leading robot in front 
        // of me to stop beaconing rf/ultrasound pings and wait for ack
        send_beacon_stop(LEADING_ROBOT_IPV6_ADDR, port, &main_thr_mailbox, 
                            MBED_MAIN_PORT);
        
        

        Thread::wait(100);

        // Communicate with the following robot to start their range thread


        // Start Beaconing

        // Wait for ack from follow node


        Thread::wait(100);
        // Execute the movement based on the ranging data. 
        int a  = start_movement(NORMAL_MOV, dist_estimate, angle_estimate);

        // PRINTF("Success %d\n", a);
    }

    // should be never reached
    return 0;
}
