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
 * @brief       Full-duplex hdlc with mqtt.
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Daniel Dsouza <dmdsouza@usc.edu>
 * 
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
#include "mbed_movement.h"

#define DEBUG   1
#define TEST_TOPIC   ("test/trial")

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

Mail<msg_t, HDLC_MAILBOX_SIZE>  mqtt_thread_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

/**
 * @brief      parse mqtt_data_t for move command information
 *             stored as hex values
 *
 * @param      movement  Pointer to store movement_t data
 * @param      data_pkt  The parsed data
 */
void mqtt_data_to_movement_t(movement_t *movement, mqtt_data_t *data_pkt)
{
    char m_type[3] = "\0\0";
    char time[5] = "\0\0\0\0";
    char degrees[5] = "\0\0\0\0";
    char speed[3] ="\0\0";
    memcpy(m_type, data_pkt->data, 2);
    memcpy(time, data_pkt->data + 2, 4);
    memcpy(degrees, data_pkt->data + 6, 4);
    memcpy(speed, data_pkt->data + 10, 2);
    
    movement->move_type = (uint8_t)strtol(m_type, NULL, 16);
    movement->time = (uint16_t)strtol(time, NULL, 16);
    movement->degrees = (int16_t)strtol(degrees, NULL, 16);
    movement->speed = (int8_t)strtol(speed, NULL, 16);

}

/**
 * @brief      This is the MQTT thread on MBED
 */
void _mqtt_thread()
{
    int             pub_length;
    char            mqtt_thread_frame_no = 0;
    msg_t           *msg, *msg2;
    char            send_data[HDLC_MAX_PKT_SIZE];
    char            recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t      pkt;
    pkt.data        = send_data;  
    pkt.length      = 0;

    mqtt_pkt_t      *mqtt_recv;
    mqtt_pkt_t      mqtt_send;
    mqtt_data_t     mqtt_recv_data;
    char            *test_str;

    movement_t      movement;
    
    uart_pkt_hdr_t  send_hdr = { 0, 0, 0};
    hdlc_buf_t      *buf;
    uart_pkt_hdr_t  recv_hdr;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = get_hdlc_mailbox();
    
    int             exit = 0;
    hdlc_entry_t    mqtt_thread = {NULL, MBED_MQTT_PORT, &mqtt_thread_mailbox};
    hdlc_register(&mqtt_thread);
    
    char            test_pub[] = "Hello world";
    char            topic_pub[16];
    char            data_pub[32];

    osEvent         evt;

    /**
     * Check if the MQTT coneection is established by the openmote. If not,
     * DO NOT Proceed further. The openmote sends a MQTT_GO msg once the mqtt connection is properly setup.
     */

    while(1)
    {
        evt = mqtt_thread_mailbox.get();
        if (evt.status == osEventMail) 
        {
            msg = (msg_t*)evt.value.p;
            if (msg->type == HDLC_PKT_RDY)
            {
                buf = (hdlc_buf_t *) msg->content.ptr;   
                uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
                if (recv_hdr.pkt_type == MQTT_GO){
                    mqtt_go = 1;
                    PRINTF("mqtt_thread: the node is conected to the broker \n");
                    mqtt_thread_mailbox.free(msg);
                    hdlc_pkt_release(buf);  
                    break;
                }
            }
            mqtt_thread_mailbox.free(msg);
            hdlc_pkt_release(buf);  
        }
    }


    while (1) 
    {
        // PRINTF("In mqtt_thread");
        myled3 =! myled3;
        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 
        pkt.length = HDLC_MAX_PKT_SIZE;        

        while(1)
        {
            evt = mqtt_thread_mailbox.get();
            if (evt.status == osEventMail) 
            {
                msg = (msg_t*)evt.value.p;
                switch (msg->type)
                {
                    case HDLC_RESP_SND_SUCC:
                        PRINTF("mqtt_thread: sent frame_no %d!\n",
                            mqtt_thread_frame_no);
                        exit = 1;
                        mqtt_thread_mailbox.free(msg);
                        break;    
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg->content.value/1000);
                        PRINTF("mqtt_thread: retry frame_no %d \n",
                            mqtt_thread_frame_no);
                        if (send_hdlc_mail(msg2, HDLC_MSG_SND,
                            &mqtt_thread_mailbox, (void*) &pkt) < 0)
                        {
                            while (send_hdlc_retry_mail (msg2,
                                &mqtt_thread_mailbox) < 0)
                                Thread::wait(10);
                        }
                        mqtt_thread_mailbox.free(msg);
                        break;
                    case HDLC_PKT_RDY:

                        buf = (hdlc_buf_t *)msg->content.ptr;   
                        uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
                        switch (recv_hdr.pkt_type)
                        {
                            case MQTT_PKT_TYPE:                                
                                // rcv_data= (mqtt_pkt_t *) uart_pkt_get_data(buf->data, buf->length);
                                mqtt_recv = (mqtt_pkt_t *) uart_pkt_get_data(
                                    buf->data, buf->length);
                                PRINTF("The data is %s \n",
                                    mqtt_recv->data);
                                PRINTF("The topic received is %s \n",
                                    mqtt_recv->topic); 
                                process_mqtt_pkt(mqtt_recv, &mqtt_recv_data);
                                switch (mqtt_recv_data.data_type){
                                    case NORM_DATA:
                                        PRINTF("MQTT: Normal Data Received %s \n",
                                            mqtt_recv_data.data);
                                        break;

                                    case SUB_CMD:
                                        build_mqtt_pkt_sub(mqtt_recv_data.data,
                                            MBED_MQTT_PORT, &mqtt_send, &pkt);
                                        if (send_hdlc_mail(msg, HDLC_MSG_SND,
                                            &mqtt_thread_mailbox, (void*) &pkt))
                                        {
                                            PRINTF("mqtt_thread: sending pkt ");
                                            PRINTF("no %d \n", mqtt_thread_frame_no);
                                        } 
                                        else
                                        {
                                            PRINTF("mqtt_thread: failed to ");
                                            PRINTF("send pkt no\n");
                                        } 
                                        break;

                                    case PUB_CMD:
                                        //second byte is the length of the topic 
                                        pub_length = mqtt_recv_data.data[0]
                                            - '0';
                                        memcpy(topic_pub,
                                            (mqtt_recv_data.data + 1),
                                            pub_length);
                                        strcpy(data_pub,
                                            mqtt_recv_data.data + pub_length + 1);                             
                                        topic_pub[pub_length]='\0';
                                        PRINTF("The topic_pub %s\n",topic_pub);
                                        PRINTF("The data_pub %s\n", data_pub);                                 
                                        build_mqtt_pkt_pub(topic_pub, data_pub,
                                            MBED_MQTT_PORT, &mqtt_send, &pkt);
                                        if (send_hdlc_mail(msg, HDLC_MSG_SND,
                                            &mqtt_thread_mailbox, (void*) &pkt))
                                        {
                                            PRINTF("mqtt_thread: sending pkt ");
                                            PRINTF("no %d \n", mqtt_thread_frame_no); 
                                        }
                                        else
                                        {
                                            PRINTF("mqtt_thread: failed to ");
                                            PRINTF("send pkt no\n");
                                        } 
                                        break;
                                    case MOVE_CMD:
                                        //temporary case statement
                                        PRINTF("Move command received\n");
                                        mqtt_data_to_movement_t(&movement,
                                            &mqtt_recv_data);
                                        PRINTF("Move command type: %d\n",
                                                movement.move_type);
                                        switch(movement.move_type)
                                        {
                                            case INIT_IMU:
                                                init_minimu();
                                                break;
                                            case CALIBRATE:
                                                calibrate_compass();
                                                break;
                                            case ROTATE:
                                                PRINTF("Degrees: %d\n",
                                                    movement.degrees);
                                                PRINTF("Speed: %d\n",
                                                    movement.speed);
                                                rotate_degrees(movement.degrees,
                                                    movement.speed);
                                                break;
                                            case ROTATE_PART:
                                                rotate_parts(movement.degrees);
                                                break;
                                            case DRIVE_SPEED:
                                                PRINTF("Time: %d\n",
                                                    movement.time);
                                                PRINTF("Speed: %d\n",
                                                    movement.speed);
                                                drive_forward(movement.time,
                                                    movement.speed);
                                                break;
                                            case DRIVE_PID:
                                                drive_forward(movement.time);
                                                break;
                                            default:
                                                PRINTF("Move command error\n");
                                                break;
                                        }
                                        break;
                                    default:
                                        PRINTF("error. data type: %d\n", 
                                            mqtt_recv_data.data_type);
                                        break;
                                }
                                // Mbed send a pub message to the broker                        
                                break;

                            case MQTT_SUB_ACK:
                                PRINTF("mqtt_thread: SUB ACK message received\n");
                                break;
                            case MQTT_PUB_ACK:
                                PRINTF("mqtt_thread: PUB ACK message received\n");
                                break;
                            default:
                                mqtt_thread_mailbox.free(msg);
                                /* error */
                                break;

                        }
                        mqtt_thread_mailbox.free(msg);
                        hdlc_pkt_release(buf);     
                        fflush(stdout);
                        break;
                    default:
                        mqtt_thread_mailbox.free(msg);
                        /* error */
                        break;
                }
            }    
            if(exit) {
                exit = 0;
                break;
            }
        }

        mqtt_thread_frame_no++;
        Thread::wait(100);
    }
}


int main(void)
{
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    Thread mqtt_thr;
    mqtt_thr.start(_mqtt_thread);
    
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
