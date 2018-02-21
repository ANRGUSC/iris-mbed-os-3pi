/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Pradipta Ghosh
 * Yutong Gu
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
 * @brief       Full-duplex hdlc with mqtt for ranging.
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Yutong Gu <yutonggu@usc.edu>
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
#include "range.h"


#define TEST_TOPIC   ("test/trial")

#define START_RANGE_MSG   ("INIT_RANGE")

#define DEBUG   1
#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */
Serial                          pc(USBTX,USBRX,115200);
DigitalOut                      myled3(LED3); //to notify when a character was received on mbed
DigitalOut                      myled(LED1);

bool mqtt_go = 0;

static char* pub_msg;
static Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;

Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

#define NULL_PKT_TYPE   0xFF 
#define PKT_FROM_MAIN_THR   0

m3pi m3pi(p23, p9, p10);


int main(void)
{
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);

    msg_t           *msg = NULL;

    hdlc_buf_t      *buf = NULL;
    uart_pkt_hdr_t  recv_hdr;
    
    int             exit = 0;
    hdlc_entry_t    main_thread = { NULL, MBED_MQTT_PORT, &main_thr_mailbox };
    hdlc_register(&main_thread);

    osEvent         evt;
   
    PRINTF("Starting mqtt thread\n");

    PRINTF("Starting range thread\n");
    init_range_thread();
   

    /**
     * Check if the MQTT coneection is established by the openmote. If not,
     * DO NOT Proceed further. The openmote sends a MQTT_GO msg once the mqtt connection is properly setup.
     */

    // while(1)
    // {
    //     evt = main_thr_mailbox.get();
    //     if (evt.status == osEventMail) 
    //     {
    //         msg = (msg_t*)evt.value.p;
    //         if (msg->type == HDLC_PKT_RDY)
    //         {
    //             buf = (hdlc_buf_t *) msg->content.ptr;   
    //             uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
    //             if (recv_hdr.pkt_type == MQTT_GO){
    //                 mqtt_go = 1;
    //                 range_load_id(buf);
    //                 PRINTF("mqtt_thread: the node is conected to the broker \n");
    //                 main_thr_mailbox.free(msg);
    //                 hdlc_pkt_release(buf);  
    //                 break;
    //             }
    //         }
    //         main_thr_mailbox.free(msg);
    //         hdlc_pkt_release(buf);  
    //     }
    // }


    PRINTF("A\n");
    m3pi.move_straight_distance_blocking(50, 2 * 2238);
    m3pi.rotate_degrees(90, 1, 50);
    PRINTF("B\n");
    wait(1);
    
    m3pi.move_straight_distance_blocking(50, 2 * 2238);
    m3pi.rotate_degrees(180, 1, 50);
    PRINTF("C\n");
    wait(1);
    
    m3pi.move_straight_distance_blocking(50, 2 * 4476);
    PRINTF("D\n");
    wait(1);

    myled = 1;
    while(1)
    {
        myled =! myled;
        Thread::wait(9000);
    }

    PRINTF("Reached Exit");
    /* should be never reached */
    return 0;
}
