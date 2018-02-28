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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "mbed.h"
#include "rtos.h"
#include "hdlc.h"
#include "fcs16.h"
#include "uart_pkt.h"
#include "main-conf.h"
#include "range.h"
#include "controller.h"
#include "m3pi.h"

#define DEBUG   1
#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */
DigitalOut reset_xbee(p26);

m3pi m3pi(p23, p9, p10);

// the only instance of pc -- debug statements in other files depend on it
Serial pc(USBTX,USBRX,115200);

Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

#define MBED_MAIN_PORT      6000

#define MOVEMENT_ENABLED //define this to enable m3pi movements

typedef enum {
    STOP_BEACONS_STATE,
    RANGING_STATE,
    SEND_STOP_BEACONS_STATE,
    SEND_RANGE_ME_STATE,
    RANGE_ME_STATE,
    SEND_STOP_BEACONS_ACK_STATE
} chain_follower_state_t;

//all helper functions and message types in this header
#include "chain_follower.h"

int main(void)
{

    reset_xbee = 0;
    Thread::wait(100);
    reset_xbee = 1;

    Thread::wait(500);

    PRINTF("Starting hdlc thread\n");
    hdlc_init(osPriorityRealtime);
   
    PRINTF("Starting range thread\n");
    set_range_riot_port(RANGE_SLAVE_PORT);
    init_range_thread();
    
    PRINTF("Starting controller thread\n");
    controller_init(osPriorityNormal);

    PRINTF("Starting network helper thread\n");
    network_helper_init(osPriorityNormal);

    hdlc_entry_t main = { NULL, MBED_MAIN_PORT, &main_thr_mailbox };
    hdlc_register(&main);

    msg_t *msg;
    char data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_pkt;
    hdlc_pkt.data = data;
    range_params_t params;
    range_data_t range_data;
    dist_angle_t range_res;
    hdlc_buf_t *buf;
    uart_pkt_hdr_t uart_hdr;
    int ranging_is_done = 1; //initial state
    bool ranging_is_successful = false;
    uint8_t net_msg = INVALID_NET_MSG;

    //set initial state depending on robot position
#ifdef LEADER_ROBOT
    chain_follower_state_t state = SEND_RANGE_ME_STATE; 
    start_sending_range_me_msgs();
    int8_t my_node_id = 1;
    tdoa_beacons_on(my_node_id, &main_thr_mailbox, MBED_MAIN_PORT, &hdlc_pkt);
    bool sending_hdlc = true;
#else
    chain_follower_state_t state = STOP_BEACONS_STATE;
    bool sending_hdlc = false;
#endif

    float min_distance = 10; //minimum distance in mm between robots
    float dist_estimate = min_distance, angle_estimate = min_distance;
    char speed = ROBOT_MAX_SPEED;
    float dist_thr = 1;
    uint32_t timeout = 1000;

    while(1)
    {
        // continue looping until any outgoing hdlc packets are successful
HERE:
        do
        {
            osEvent evt = main_thr_mailbox.get(timeout);
            if(evt.status == osEventTimeout) {
            //send range or stop beacons 
                if (state == RANGING_STATE) {
                    PRINTF("main_thr: trigger_range_routine() again\n");

#ifdef SECOND_ROBOT
                    int8_t target_node_id = 1;
#else //END_ROBOT defined
                    int8_t target_node_id = 2;
#endif

                    params.node_id = target_node_id; 
                    params.ranging_mode = TWO_SENSOR_MODE;
                    trigger_range_routine(&params, msg, &main_thr_mailbox);
                }
                goto HERE;
            }
            // if (evt.status != osEventMail) 
            //     continue; 

            msg = (msg_t*)evt.value.p;
            switch (msg->type)
            {
                case HDLC_RESP_SND_SUCC:
                    sending_hdlc = false;
                    main_thr_mailbox.free(msg);
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    Thread::wait(msg->content.value/1000);
                    main_thr_mailbox.free(msg);

                    while (send_hdlc_mail(msg, HDLC_MSG_SND, &main_thr_mailbox, (void*) &hdlc_pkt) < 0)
                    {
                        Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                    }
                    break;
                case HDLC_PKT_RDY:
                    buf = (hdlc_buf_t *)msg->content.ptr;   
                    uart_pkt_parse_hdr(&uart_hdr, buf->data, buf->length);
                    if (uart_hdr.pkt_type == NET_SLAVE_RECEIVE) {
                        //actual data of the network packet is a single byte 
                        //representing the message type
                        net_msg = buf->data[buf->length-1];

                        PRINTF("net_msg received %d is %d\n", buf->data[0],net_msg);
                    }
                    main_thr_mailbox.free(msg);
                    hdlc_pkt_release(buf);
                    break;
                case RANGING_DONE: // Not gonna go out of this loop. probably need to send sending_hdlc
                    PRINTF("RANGING_DONE\n");
                    ranging_is_done = 1;
                    memcpy(&range_data, (range_data_t *)msg->content.ptr, 
                           sizeof(range_data_t));
                    //if tdoa is NOT zero and status is 11 or 12, then you can
                    //find out which pin received by taking 
                    //range_data.status - 10 (which gives you 1 or 2)
                    PRINTF("range_data.status = %d\n", range_data.status);
                    if (range_data.status > 2)
                        ranging_is_successful = false;
                    else
                        ranging_is_successful = true;

                    main_thr_mailbox.free(msg);
                    break;
                default:
                    main_thr_mailbox.free(msg); //error!
                    break;
            } // switch
        } while (sending_hdlc); //true if there's an outgoing HDLC packet

        PRINTF("going into state machine\n");
        // state machine logic (it's important to only send one HDLC pkt per 
        // iteration because of this program's design)
        switch(state) {
            case STOP_BEACONS_STATE:
                //passively waiting for a command
                PRINTF("in STOP_BEACONS_STATE\n");

                //ack any STOP_BEACONS messages
                if (net_msg == STOP_BEACONS) {
                    net_send_udp(FOLLOWING_ROBOT_IPV6_ADDR, FORWARD_TO_MBED_MAIN_PORT,
                    STOP_BEACONS_ACK, &main_thr_mailbox, MBED_MAIN_PORT, &hdlc_pkt); 
                    sending_hdlc = true;
                }

                //when RANGE_ME received, ack it and change state
                if (net_msg == RANGE_ME) {
                    net_send_udp(LEADING_ROBOT_IPV6_ADDR, FORWARD_TO_MBED_MAIN_PORT,
                    RANGE_ME_ACK, &main_thr_mailbox, MBED_MAIN_PORT, &hdlc_pkt); 
                    sending_hdlc = true;

#ifdef LEADER_ROBOT
                    //leader doesn't range so send STOP_BEACONS and change to
                    //SEND_STOP_BEACONS_STATE immediately
                    start_sending_stop_beacons_msgs();
                    state = SEND_STOP_BEACONS_STATE;
                    break;
#endif

                    state = RANGING_STATE;
                    ranging_is_done = 0; //set flag
                    PRINTF("main_thr: trigger_range_routine()\n");

#ifdef SECOND_ROBOT
                    int8_t target_node_id = 1;
#else //END_ROBOT defined
                    int8_t target_node_id = 2;
#endif

                    params.node_id = target_node_id; //TODO
                    params.ranging_mode = TWO_SENSOR_MODE;
                    trigger_range_routine(&params, msg, &main_thr_mailbox); 
                }
                break;
            case RANGING_STATE:
                //currently ranging (leader robot never enters this state)
                PRINTF("in RANGING_STATE (leader robot never enters this)\n");

                //ack any RANGE_ME messages
                if (net_msg == RANGE_ME) {
                    net_send_udp(LEADING_ROBOT_IPV6_ADDR, FORWARD_TO_MBED_MAIN_PORT,
                    RANGE_ME_ACK, &main_thr_mailbox, MBED_MAIN_PORT, &hdlc_pkt);
                    sending_hdlc = true;
                }

                if(ranging_is_done){
                    PRINTF("isdone: %d, success: %d\n", ranging_is_done, ranging_is_successful);
                    if (ranging_is_successful) {
                        range_res = get_dist_angle(&range_data, TWO_SENSOR_MODE);
                        PRINTF("distance: %f, angle: %f\n", range_res.distance, 
                               range_res.angle);
#ifdef MOVEMENT_ENABLED
                        
                        float a_kalman =  0.3820; 
                        float b_kalman =  0.6180;
                        // float c_LQG =  0.6180;

                        float dist_to_travel =  range_res.distance - dist_thr;

                        if (dist_to_travel < 0) {
                            dist_to_travel = 0;
                        }

                        //TODO: BUG IN CODE. need to negate here. fix this.
                        float angle_to_rotate = -range_res.angle;

                        // TODO? Controller decides how much to travel and how much to rotate
               
                        PRINTF("Distance = %f, Angle = %f\n", dist_to_travel, angle_to_rotate);

                        //convert to mm
                        dist_to_travel = dist_to_travel * 0.3048 * 1000.0;

                        dist_to_travel = dist_to_travel / 0.111701; 

                        /**
                         * 8952.5 ticks is equal to 1 meter. 
                         * To get 8952.5, you take 1000mm, which equals 1 meter, and divide
                         * it by 0.111701mm, which should be how much the robot travels after one
                         * encoder tick. 
                         */


                        //-361.00 is reported when it's a bad data piece
                        if (angle_to_rotate > 180 || angle_to_rotate < -180) {
                            angle_to_rotate = 0;
                        }

                        PRINTF("moving %f ticks\n", dist_to_travel);
                        m3pi.rotate_degrees_blocking((char) fabs(angle_to_rotate), signbit(angle_to_rotate)? -1 : 1, 30);
                        Thread::wait(200);
                        m3pi.move_straight_distance_blocking(30, (uint16_t) dist_to_travel);
                        Thread::wait(200);
#endif 

                        start_sending_stop_beacons_msgs();
                        state = SEND_STOP_BEACONS_STATE;
                    } else {

                        ranging_is_done = 0; //set flag
                        PRINTF("main_thr: trigger_range_routine() again\n");

#ifdef SECOND_ROBOT
                        int8_t target_node_id = 1;
#else //END_ROBOT defined
                        int8_t target_node_id = 2;
#endif

                        params.node_id = target_node_id; 
                        params.ranging_mode = TWO_SENSOR_MODE;
                        trigger_range_routine(&params, msg, &main_thr_mailbox); 
                    }
                }
                break;
            case SEND_STOP_BEACONS_STATE:
                // sending STOP_BEACONS messages every second
                PRINTF("in SEND_STOP_BEACONS_STATE\n");

                //when stop ack is received, start tdoa beacons and change state
                if (net_msg == STOP_BEACONS_ACK) {
                    stop_sending_stop_beacons_msgs();

#ifndef END_ROBOT
#ifdef LEADER_ROBOT
                    int8_t my_node_id = 1;
#else //SECOND_ROBOT defined
                    int8_t my_node_id = 2;
#endif
                    tdoa_beacons_on(my_node_id, &main_thr_mailbox, 
                                    MBED_MAIN_PORT, &hdlc_pkt);
                    sending_hdlc = true;
#endif

                    start_sending_range_me_msgs();
                    state = SEND_RANGE_ME_STATE;
                }
                break;
            case SEND_RANGE_ME_STATE: 
                //sending RANGE_ME messages every second and tdoa beacons on
                PRINTF("in SEND_RANGE_ME_STATE\n");
                
                //if a RANGE_ME_ACK is received, stop sending RANGE_ME messages
                //and go to RANGE_ME_STATE
                if (net_msg == RANGE_ME_ACK) {
                    stop_sending_range_me_msgs();
                    state = RANGE_ME_STATE;
                }

                //if a stop beacon msg received, turn tdoa beacons off and go straight to
                //SEND_STOP_BEACONS_ACK_STATE (skipping RANGE_ME_STATE)
                if (net_msg == STOP_BEACONS) {
                    stop_sending_range_me_msgs();
                    tdoa_beacons_off(&main_thr_mailbox, MBED_MAIN_PORT, &hdlc_pkt);
                    sending_hdlc = true;
                    state = SEND_STOP_BEACONS_ACK_STATE;
                }
                break;
            case RANGE_ME_STATE: 
                //tdoa beacons on
                PRINTF("in RANGE_ME_STATE\n");
                
                //if STOP_BEACONS received, turn tdoa beacons off and change state
                if (net_msg == STOP_BEACONS) {
                    tdoa_beacons_off(&main_thr_mailbox, MBED_MAIN_PORT, &hdlc_pkt); 
                    sending_hdlc = true;
                    state = SEND_STOP_BEACONS_ACK_STATE;
                }
                break;
            case SEND_STOP_BEACONS_ACK_STATE: 
                //send STOP_BEACONS_ACK once and change state immediately
                PRINTF("in SEND_STOP_BEACONS_ACK_STATE\n");
                net_send_udp(LEADING_ROBOT_IPV6_ADDR, FORWARD_TO_MBED_MAIN_PORT,
                STOP_BEACONS_ACK, &main_thr_mailbox, MBED_MAIN_PORT, &hdlc_pkt);
                state = STOP_BEACONS_STATE;
                break;
            default:
                //should not be reached
                break;
        } // switch

        //reset net message
        net_msg = INVALID_NET_MSG; 
    } // while

    // should be never reached
    return 0;
}
