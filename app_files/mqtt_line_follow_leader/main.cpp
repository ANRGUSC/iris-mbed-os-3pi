/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
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
 * @brief       Full-duplex hdlc with mqtt.
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Daniel Dsouza <dmdsouza@usc.edu>
 * 
 *
 */

#include "mbed.h"
#include "m3pi.h"
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
#include "mqtt.h"
#include "mqtt_thread.h"
#include "app-conf.h"

#define DEBUG   1
#define TEST_TOPIC   ("test/trial")

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */
Serial pc(USBTX,USBRX,115200);

DigitalOut  myled(LED1);

m3pi m3pi;

typedef struct control_data{
    float speed_l;
    float speed_r;
}control_data_t;

control_data_t sample_control;
// volatile bool  go_flag = 0;
/**
 * @brief      This is the MQTT thread on MBED
 */
Mail<msg_t, 10>  cont_data_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE>  cont_thr_mailbox;


Mutex       data_mutex; 
float       sense_position_of_line;

void _cont_thread()
{
    hdlc_entry_t contr_thr = { NULL, CONT_THR_PORT, &cont_thr_mailbox };
    hdlc_register(&contr_thr);

    char frame_no = 0;
    char topic_pub[16] = SENSOR_DATA_TOPIC;
    char data_pub[32];
    char send_data[HDLC_MAX_PKT_SIZE];
    int  status = 0;
    
    mqtt_pkt_t mqtt_send;
    hdlc_pkt_t pkt = {send_data, HDLC_MAX_PKT_SIZE};
    
    msg_t *msg, *msg2;
    osEvent evt;
    uart_pkt_hdr_t send_hdr = { 0, 0, 0};
    int exit = 0;
    control_data_t *control_ptr;
    
    while(1)
    {
        evt = cont_data_mailbox.get();
        if (evt.status == osEventMail) 
        {
            msg = (msg_t*)evt.value.p;
            switch (msg->type)
            {
                case INTER_THREAD:
                    control_ptr = (control_data_t *) msg->content.ptr;
                    sprintf(data_pub, "%d %0.2f %0.2f %d",SENSOR_DATA, control_ptr->speed_l, control_ptr->speed_r, frame_no);                                                         
                    build_mqtt_pkt_pub(topic_pub, data_pub, CONT_THR_PORT, &mqtt_send, &pkt);
                    PRINTF("_cont_thread: sending update %s\n", mqtt_send.data);
                    if (hdlc_send_command_wo_resp(&pkt, &cont_thr_mailbox))
                        PRINTF("_cont_thread: sending pkt no %d \n", frame_no); 
                    else
                        PRINTF("_cont_thread: failed to send pkt no\n"); 
                    
                    frame_no++;
                    cont_data_mailbox.free(msg);
                    break;
                default:
                    cont_data_mailbox.free(msg);
                    /* error */
                    break;
            }
        }    
    }
}

#define STEP_SIZE 100


int main(void)
{
    // reset_openmote();
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    Mail<msg_t, HDLC_MAILBOX_SIZE> *mqtt_thread_mailbox;
    mqtt_thread_mailbox = mqtt_init(osPriorityNormal);

    msg_t *msg;

    Thread contr_thr;
    contr_thr.start(_cont_thread);
    PRINTF("Starting the MBED\n");

    // Parameters that affect the performance
    float speed = 0.1;
    float correction = 0.05;   
    float threshold = 0.5;
 
    
    m3pi.locate(0,1);
    m3pi.printf("Line Flw");
 
    wait(2.0);
    
    m3pi.sensor_auto_calibrate();
    int countt = 0;
    int countt1 = 0;

    while ( get_mqtt_state() != MQTT_CONTROL_GO )
    {
        Thread::wait(100);
    }

    PRINTF("main_thr: go received \n");
    int mqtt_counter = 1;
    float speed_l = speed;
    float speed_r = speed;
    float position_of_line = m3pi.line_position();

    while (1) 
    {
        PRINTF("main_th: in the control thread\n");
        mqtt_counter ++;
        // m3pi.locate(0,0);
        // m3pi.printf("%d",mqtt_counter);
        // -1.0 is far left, 1.0 is far right, 0.0 in the middle


        // Line is more than the threshold to the right, slow the left motor
        if (position_of_line > threshold) {
            // m3pi.right_motor(speed);
            speed_r = speed;
            m3pi.left_motor(speed - correction);
            speed_l = speed - correction;
            // PRINTF("main_thr: case 1\n");           
        }
 
        // Line is more than 50% to the left, slow the right motor
        else if (position_of_line < -threshold) {
            m3pi.left_motor(speed);
            speed_l = speed;
            m3pi.right_motor(speed - correction);
            speed_r = speed - correction;
        }
 
        // Line is in the middle
        else {
            m3pi.forward(speed);
            speed_l = speed;
            speed_r = speed;         
        }
        Thread::wait(MOVEMENT_GRAN);    
        m3pi.stop();
        Thread::wait(WAIT_GRAN);    

        if (1) //mqtt_counter == STEP_SIZE)
        {
            set_mqtt_state(MQTT_CONTROL_GO_WAIT);
            // m3pi.stop();
            PRINTF("main_th: the m3pi is stopped\n");
            m3pi.locate(0,0);
            m3pi.printf("stopping");

            // while(get_mqtt_state() != MQTT_CONTROL_GO){
            //     Thread::wait(100);
            // }

            position_of_line = m3pi.line_position();
            mqtt_counter = 0;
            msg = cont_data_mailbox.alloc(); 
            while (msg == NULL){
                PRINTF("main_th: No space in control thread mailbox\n");
                Thread::wait(100);
                msg = cont_data_mailbox.alloc(); 
                m3pi.locate(0,1);
                m3pi.printf("no space");
            }
            {
                sample_control.speed_r = speed_r;
                sample_control.speed_l = speed_l;
                msg->type = INTER_THREAD;
                msg->content.ptr = &sample_control;
                cont_data_mailbox.put(msg); 
            }
        }


    }
    /* should be never reached */
    return 0;
}
