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
#include "sensor_data.h"
#include "mqtt_thread.h"
#include "app-conf.h"

#define DEBUG   1
#define TEST_TOPIC   ("test/trial")
#define SUB_TOPIC   ("line")

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */
Serial pc(USBTX,USBRX,115200);
DigitalOut myled(LED1);

m3pi  m3pi;
// volatile bool  go_flag = 0;
/**
 * @brief      This is the MQTT thread on MBED
 */
Mail<msg_t, HDLC_MAILBOX_SIZE>  cont_thr_mailbox;
Mail<float, HDLC_MAILBOX_SIZE>  sensor_data_mailbox;


// Mutex       data_mutex; 
// float       sense_position_of_line;

#define STEP_SIZE 100

int main(void)
{
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
    
    Mail<msg_t, HDLC_MAILBOX_SIZE> *mqtt_thread_mailbox;
    mqtt_thread_mailbox = mqtt_init(osPriorityNormal);

    control_data_t new_data;
    
    msg_t *msg;

  
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

   
    int mqtt_counter = 0;
    float position_of_line;
    while (1) 
    {
        mqtt_counter ++;

        // -1.0 is far left, 1.0 is far right, 0.0 in the middle
        // position_of_line = m3pi.line_position();

        // if (mqtt_counter == 1){
            // m3pi.stop();
            get_telemetry(&new_data);
        // }

        if (mqtt_counter == STEP_SIZE){
            mqtt_counter = 0;
        }

        // PRINTF("main_thr: speed data %f, %f\n", new_data.speed_l, new_data.speed_r);
        
        m3pi.right_motor (new_data.speed_r);
        m3pi.left_motor (new_data.speed_l);
        Thread::wait(25);    
        m3pi.stop();

    }
    /* should be never reached */
    return 0;
}
