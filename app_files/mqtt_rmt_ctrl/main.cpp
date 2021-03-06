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
#include "app-conf.h"
#include "m3pi.h"
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

m3pi    m3pi;

/**
 * @brief      controls movement of the 3pi
 *
 * @param[in]  command  The movement command
 * @param[in]  speed    The speed of the movement
 * @param[in]  delta_t  The time for each movement
 *
 * @return     { void }
 */
void movement(char command, float speed, int delta_t)
{
    if (command == 's')
    {
        m3pi.forward(speed);
        Thread::wait(delta_t);
        m3pi.stop();
    }    
    else if (command == 'a')
    {
        m3pi.left(speed);
        Thread::wait(delta_t);
        m3pi.stop();
    }   
    else if (command == 'w')
    {
        m3pi.backward(speed);
        Thread::wait(delta_t);
        m3pi.stop();
    }
    else if (command == 'd')
    {
        m3pi.right(speed);
        Thread::wait(delta_t);
        m3pi.stop();
    }
}


int main(void)
{
    //initialization of the hdlc thread
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    //initialization of the mqtt thread
    Mail<msg_t, HDLC_MAILBOX_SIZE> *mqtt_thread_mailbox;
    mqtt_thread_mailbox = mqtt_init(osPriorityNormal);

    //receives the rmt control message from the mqtt thread
    char *rmt_ctrl_command;
    
    //assigning the speed and time for movements
    float speed = SPEED;
    int delta_t = DELTA_T;
    int delta_t_angular = DELTA_T_ANGULAR;

    msg_t *msg;

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
                    rmt_ctrl_command = (char *)msg->content.ptr ; 
                    PRINTF("rmt_ctrl_thread: %c\n", rmt_ctrl_command[0]);
                    if (rmt_ctrl_command[0] == 's' || 'w')
                    {
                        movement(rmt_ctrl_command[0], speed, delta_t);
                    }
                    else if (rmt_ctrl_command[0] == 'a' || 'd')
                    {
                        movement(rmt_ctrl_command[0], speed, delta_t_angular);
                    }
                    break;
                default :
                    PRINTF("rmt_ctrl_thread: Error message\n");
                    break;

            }
            main_thr_mailbox.free(msg);
        }
        
    }

    PRINTF("Reached Exit");
    /* should be never reached */
    return 0;
}
