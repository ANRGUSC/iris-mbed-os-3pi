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

//all message and state types are defined in here
#include "chain_follower.h"


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
#define RANGE_BEACONER_PORT 5003

enum {
    RANGE_ME_STATE,
    SEND_STOP_STATE,
    SEND_RANGE_ME_STATE

} chain_follower_state_t;

int main(void)
{
    static Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr = 
        hdlc_init(osPriorityRealtime);
   
    PRINTF("Starting range thread\n");
    init_range_thread();
    
    PRINTF("Starting controller thread\n");
    controller_init(osPriorityNormal);

    PRINTF("Starting network helper thread\n");
    network_helper_init(osPriorityNormal);

    //set initial state depending on robot position
#ifdef LEADER_ROBOT
    chain_follower_state_t state = SEND_RANGE_ME_STATE; 
#else
    chain_follower_state_t state = STOP_BEACONS_STATE;
#endif

    hdlc_entry_t main = { NULL, MBED_MAIN_PORT, &main_thr_mailbox };
    hdlc_register(&main);

    myled = 1;
    msg_t *msg;
    range_params_t params;
    range_data_t range_data;
    dist_angle_t range_res;
    hdlc_buf_t *buf;
    uart_pkt_hdr_t uart_hdr;
    int ranging_is_done = 1; //initial state

    float dist_estimate = 10, angle_estimate = 10;

    while(1)
    {
        // continue looping until any outgoing hdlc packets are successful
        do
        {
            osEvent evt = main_thr_mailbox.get();

            if (evt.status != osEventMail) 
                continue; 

            msg = (msg_t*)evt.value.p;
            switch (msg->type)
            {
                case HDLC_RESP_SND_SUCC:
                    /* done, exit loop! */
                    sending_hdlc = 0;
                    main_thr_mailbox.free(msg);
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    Thread::wait(msg->content.value/1000);
                    main_thr_mailbox.free(msg);

                    while (send_hdlc_mail(msg, HDLC_MSG_SND, main_thr_mailbox, (void*) &hdlc_pkt) < 0)
                    {
                        Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                    }
                    break;
                case HDLC_PKT_RDY:
                    buf = (hdlc_buf_t *)msg->content.ptr;   
                    uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
                    data_ptr = buf->data;
                    if (uart_hdr.pkt_type == NET_SLAVE_RECEIVE) {
                        //scroll pointer beyond ipaddr
                        while(data_ptr++ != '\0') {}
                        net_msg = *data_ptr;
                    }
                    main_thr_mailbox.free(msg);
                    hdlc_pkt_release(buf);
                    break;
                case RANGING_DONE:
                    memcpy(&range_data, (range_data_t *)msg->content.ptr, 
                           sizeof(range_data_t));
                    //if tdoa is NOT zero and status is 11 or 12, then you can
                    //find out which pin received by taking 
                    //range_data.status - 10 (which gives you 1 or 2)
                    if (range_data.status > 2)
                        ranging_is_successful = 0;

                    ranging_is_done = 1;
                    main_thr_mailbox.free(msg);
                    break;
                default:
                    main_thr_mailbox.free(msg); //error!
                    break;
            } // switch
        } while (sending_hdlc) //true if there's an outgoing HDLC packet

        // state machine logic (it's important to only send one HDLC pkt per 
        // iteration because of this program's design)
        switch(state) {
            case STOP_BEACONS_STATE:
                if (net_msg == STOP_BEACONS) {
                    send_ack();
                    sending_hdlc = 1;
                }
                //when RANGE_ME_MSG received, ack it and change state
                if (net_msg == RANGE_ME) {
                    send_range_me_ack();
                    sending_hdlc = 1;
                    state = RANGING_STATE;
#ifndef LEADER_ROBOT
                    range_is_done = 0; //set flag
                    PRINTF("main_thr: requesting range thread to trigger range routine\n");
#ifdef SECOND_ROBOT
                    int8_t target_node_id = 1;
#else //END_ROBOT
                    int8_t target_node_id = 2;
#endif
                    params.node_id = target_node_id; //TODO
                    params.ranging_mode = TWO_SENSOR_MODE;
                    trigger_range_routine(); 
#endif
                }
                break;
            case RANGING_STATE:
                //ack any RANGE_ME_MSG
                if (net_msg == RANGE_ME) {
                    send_range_me_ack();
                    sending_hdlc = 1;
                }
                // trigger range routine
                if(ranging_is_done){
                    if (ranging_is_successful) {
                        //ranging done so send movement request to thread and change state
                        int a  = start_movement(NORMAL_MOV, range_res.distance,
                                                range_res.angle);
                        state = SEND_STOP_STATE;
                        send_stop_beacons();
                    } else {
#ifndef LEADER_ROBOT
                        range_is_done = 0; //set flag
                        PRINTF("main_thr: requesting range thread to trigger range routine\n");
#ifdef SECOND_ROBOT
                        int8_t target_node_id = 1;
#else //END_ROBOT
                        int8_t target_node_id = 2;
#endif
                        range_is_done = 0; //reset flag
                        PRINTF("main_thr: requesting ranging again\n");
                        params.node_id = target_node_id; 
                        params.ranging_mode = TWO_SENSOR_MODE;
                        trigger_range_routine(); 
#endif 
                    }
                }
                break;
            case SEND_STOP_STATE:
                start_sending_stop_beacons();
                //when stop ack is received, change state
                if (net_msg == STOP_BEACONS_ACK) {
                    stop_sending_stop_beacons();
                    turn_on_beacons();
                    sending_hdlc = 1;
                    state = SEND_RANGE_ME_STATE;
                }
                break;
            case SEND_RANGE_ME_STATE: //beacons are on and RANGE_ME messages are being sent
                start_sending_range_me();
                //if a RANGE_ME_ACK is received, go to RANGE_ME_STATE
                if (net_msg == RANGE_ME_ACK) {
                    stop_sending_range_me();
                    state = RANGE_ME_STATE;
                }
                //if a stop beacon msg received, stop beacons and go straight to stop_beacon state
                if (net_msg == STOP_BEACONS) {
                    //TODO: send two hdlc messages?!
                    stop_sending_range_me();
                    stop_beaconing();
                    state = SEND_STOP_BEACONS_ACK;
                }
                break;
            case SEND_STOP_BEACONS_ACK: //send STOP_BEACONS_ACK once
                send_stop_beacons_ack();
                state = STOP_BEACONS_STATE;
                break;
            case RANGE_ME_STATE: //beacons are on and RANGE_ME messages are NOT being sent
                //if STOP_BEACONS received, stop beacons, change state to send stop beacons ack
                if (net_msg == STOP_BEACONS) {
                    stop_beaconing(); 
                    state = SEND_STOP_BEACONS_ACK;
                }
                break;
            default:
                //should not be reached
        } // switch

        //reset net message
        net_msg = 0; 
    } // while

    // should be never reached
    return 0;
}
