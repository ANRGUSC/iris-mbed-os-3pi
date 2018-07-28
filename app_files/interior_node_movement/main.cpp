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
#include "movement_thread.h"
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

#define SPEED               0.15
#define DELTA_T             100
#define DELTA_T_ANGULAR     50

/* the only instance of pc -- debug statements in other files depend on it */
Serial      pc(USBTX,USBRX,115200);
DigitalOut  myled(LED1);




int main(void)
{

    PRINTF("Welcome to the interior node movement app's main function.\n");
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    Mail<msg_t, HDLC_MAILBOX_SIZE> *mqtt_thread_mailbox;
    mqtt_thread_mailbox = mqtt_init(osPriorityNormal);

    Mail<msg_t, HDLC_MAILBOX_SIZE> *movement_thread_mailbox;
    movement_thread_mailbox = movement_init(osPriorityNormal);

    char *value_ptr;
    float* currentCoordinates;
    float* targetCoordinates;
    
    msg_t *msg;
    msg_t *msg_for_movement = movement_thread_mailbox->alloc();

    osEvent evt;   
    myled = 1;

    while(1)
    {
        myled =! myled;
        evt = main_thr_mailbox.get();
        if (evt.status == osEventMail )
        {
            msg = (msg_t*)evt.value.p;
            switch (msg->type)
            {
                case INTER_THREAD:
                    value_ptr = (char *)msg->content.ptr; 
                    PRINTF("Value ptr = %s\n", value_ptr);
                    break;
                case CURRENT_LOCATION:
                    value_ptr = (char *)msg->content.ptr;
                    PRINTF("Received current location update: %s\n", value_ptr);
                    //Remove the tag
                    value_ptr++;

                    msg_for_movement->type = CURRENT_LOCATION;
                    msg_for_movement->content.ptr = msg->content.ptr;
                    msg_for_movement->sender_pid = osThreadGetId();
                    movement_thread_mailbox->put(msg_for_movement);
                    PRINTF("Passed current coordinates off to the movement thread.\n");
                    
                    break;
                case TARGET_LOCATION:
                    value_ptr = (char *)msg->content.ptr;
                    PRINTF("Received target coordinates location: %s\n", value_ptr);
                    msg_for_movement->type = TARGET_LOCATION;
                    msg_for_movement->content.ptr = msg->content.ptr;
                    msg_for_movement->sender_pid = osThreadGetId();
                    movement_thread_mailbox->put(msg_for_movement);
                    PRINTF("Passed target off to the movement thread.\n");
                    break;   
                default :
                    PRINTF("rmt_ctrl_thread: Error message\n");
                    break;

            }
            main_thr_mailbox.free(msg);
        }
        
    }
    free(currentCoordinates);
    free(targetCoordinates);

    PRINTF("Reached Exit");
    /* should be never reached */
    return 0;
}

/*
    ui = -kf(pi - pt)
    orientation = (-Op) + atan2((tx - ix), (ty - iy)) (math.h atan2(double, double))
*/