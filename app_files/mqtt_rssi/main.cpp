/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Pradipta Ghosh
 * Daniel Dsouza
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
 * @author      Daniel Dsouza <dmdsouza@usc.edu>
 * 
 */

#include "mbed.h"
#include "rtos.h"
// #include "m3pi.h"
#include "hdlc.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "yahdlc.h"
#include "fcs16.h"
#include "uart_pkt.h"
#include "main-conf.h"
#include "mqtt.h"
#include "mqtt_thread.h"
#include "app-conf.h"
//to reset the mbed
extern "C" void mbed_reset();


Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

#define DEBUG   1
#define TEST_TOPIC   ("init_info")

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */
Serial      pc(USBTX,USBRX,115200);
DigitalOut  myled(LED1);


// m3pi m3pi;


int main(void)
{
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);

    Mail<msg_t, HDLC_MAILBOX_SIZE> *mqtt_thread_mailbox;
    mqtt_thread_mailbox = mqtt_init(osPriorityNormal);
   
    msg_t *msg, *msg2;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t pkt = { send_data, 0 };
    
    char self_node_id[9];

    int8_t rssi_value;
    char rssi_str_value;
    char speed = 40;
    int delta_t = 100;
    float time_value;

    hdlc_buf_t *buf;
    uart_pkt_hdr_t recv_hdr;
    uart_pkt_hdr_t send_hdr = { MAIN_THR_PORT, MAIN_THR_PORT, RSSI_SCAN_STOPPED };

    hdlc_entry_t main_thr = { NULL, RSSI_MBED_DUMP_PORT, &main_thr_mailbox };
    hdlc_register(&main_thr);
    
    char data_pub[32];
    mqtt_pkt_t  mqtt_send;
    bool recv_rssi = 0;
    int  exit = 0;

    osEvent  evt;
    myled = 1;

    while ( get_mqtt_state() != MQTT_GOT_CLIENTS )
    {
        Thread::wait(100);
    }

    get_node_id(self_node_id);
    sprintf(data_pub,"%d",SERVER_SEND_RSSI);
    strcat(data_pub, self_node_id);   
    
    if (strcmp(self_node_id, PRIORITY_NODE) == 0)
    {
        PRINTF("rssi_thread: Inittiating the PING PONG.\n");
        build_mqtt_pkt_pub(TEST_TOPIC, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt);                        
        if (send_hdlc_mail(msg2, HDLC_MSG_SND, &main_thr_mailbox, (void*) &pkt))
            PRINTF("rssi_thread: sending pkt no %d \n", frame_no); 
        else
            PRINTF("rssi_thread: failed to send pkt no\n");
        frame_no++;
    }

    while (1) 
    {
        // PRINTF("In mqtt_thread");
        myled =! myled;
        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 
        pkt.length = HDLC_MAX_PKT_SIZE;        

        while(1)
        {
            evt = main_thr_mailbox.get(5000);
            if (evt.status == osEventMail) 
            {
                msg = (msg_t*)evt.value.p;
                switch (msg->type)
                {               
                    
                    case HDLC_RESP_SND_SUCC:
                        PRINTF("rssi_thread: sent frame_no %d!\n", frame_no);
                        exit = 1;
                        main_thr_mailbox.free(msg);
                        break;    
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg->content.value/1000);
                        PRINTF("rssi_thread: retry frame_no %d \n", frame_no);
                        if (send_hdlc_mail(msg2, HDLC_MSG_SND, &main_thr_mailbox, (void*) &pkt) < 0)
                        {
                            while (send_hdlc_retry_mail (msg2, &main_thr_mailbox) < 0)
                                Thread::wait(10);
                        }
                        main_thr_mailbox.free(msg);
                        break;
                    case HDLC_PKT_RDY:
                        PRINTF("rssi_thread: RSSI packet received\n");
                        buf = (hdlc_buf_t *)msg->content.ptr;   
                        uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
                        switch (recv_hdr.pkt_type)
                        {
                            case RSSI_DATA_PKT:                                
                                PRINTF("******************\n"); 
                                // PRINTF("7\n");  
                                // PRINTF("******************\n");
                                // //handles the movement of the robot
                                rssi_value = (int8_t)(* ((char *)uart_pkt_get_data(buf->data, buf->length)));
                                rssi_value = rssi_value - 73;
                                PRINTF("rssi_thread: RSSI is %d\n", rssi_value); 
                                recv_rssi = 1;                               
                                break;
                            default:
                                /* error */
                                break;
                        }
                        main_thr_mailbox.free(msg);
                        hdlc_pkt_release(buf);     
                        break;
                    default:
                        main_thr_mailbox.free(msg);
                        /* error */
                        break;
                }
            }
            
            if (evt.status == osEventTimeout || recv_rssi){

                recv_rssi = 0;
                PRINTF("rssi_thread: sending periodic keep alive msg.\n");
                build_mqtt_pkt_pub(TEST_TOPIC, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt);                        
                if (send_hdlc_mail(msg2, HDLC_MSG_SND, &main_thr_mailbox, (void*) &pkt)){
                    PRINTF("rssi_thread: sending pkt no %d \n", frame_no); 
                }
                else{
                    PRINTF("rssi_thread: failed to send pkt no\n");
                }                        
            }

            if(exit) {
                exit = 0;
                break;
            }
        }

        frame_no++;
        Thread::wait(100);
    }

    PRINTF("Reached Exit");
    /* should be never reached */
    return 0;
}
