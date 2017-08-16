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
volatile bool                   mqtt_go = 0;
m3pi                            m3pi;
Mail<msg_t, HDLC_MAILBOX_SIZE>  mqtt_thread_mailbox;
// volatile bool  go_flag = 0;
/**
 * @brief      This is the MQTT thread on MBED
 */
Mail<msg_t, HDLC_MAILBOX_SIZE>  cont_thr_mailbox;

DigitalOut                      reset_riot(p26,1);
extern "C" void mbed_reset();

void reset_system(void)
{
    reset_riot = 0; 
    Thread::wait(1); 
    reset_riot = 1;
    Thread::wait(1); 
    reset_riot = 0; 
    Thread::wait(1); 
    reset_riot = 1;
    mbed_reset();
}

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
    
    uart_pkt_hdr_t  send_hdr = { 0, 0, 0};
    hdlc_buf_t      *buf;
    uart_pkt_hdr_t  recv_hdr;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = get_hdlc_mailbox();
    
    int             exit = 0;
    hdlc_entry_t    mqtt_thread = { NULL, MBED_MQTT_PORT, &mqtt_thread_mailbox };
    hdlc_register(&mqtt_thread);
    
    char            test_pub[] = "Hello world";
    char            topic_pub[16];
    char            data_pub[32];

    osEvent         evt;

    /**
     * Check if the MQTT connection is established by the openmote. If not,
     * DO NOT Proceed further before the follwing steps are complete. 
     * (1) The openmote sends a MQTT_GO msg once the mqtt connection is properly setup.
     * (2) The MBED replies by sending a MQTT_GO_ACK msg to the Openmote
     * 
     * After this sequece is complete, the mqtt is ready to go 
     */
    while(1)
    {
        evt = mqtt_thread_mailbox.get(60000);
        if (evt.status == osEventMail) 
        {
            msg = (msg_t*)evt.value.p;
            switch (msg->type)
            {
                case HDLC_PKT_RDY:
                    buf = (hdlc_buf_t *) msg->content.ptr;   
                    uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
                    if (recv_hdr.pkt_type == MQTT_GO){
                        PRINTF("mqtt_thread: the node is conected to the broker \n");
                        send_hdr.pkt_type = MQTT_GO_ACK;
                        send_hdr.dst_port = RIOT_MQTT_PORT;
                        send_hdr.src_port = MBED_MQTT_PORT;
                        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 
                        mqtt_thread_mailbox.free(msg);
                        hdlc_pkt_release(buf);
                        pkt.length = HDLC_MAX_PKT_SIZE;        
                        if (send_hdlc_mail(msg2, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt))
                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                        else
                            PRINTF("mqtt_thread: failed to send pkt no\n");
                    }
                    else{
                        /* ERROR */
                        PRINTF("mqtt_thread: wrong syntax\n");
                    }

                    break;
                case HDLC_RESP_SND_SUCC:
                    if (mqtt_thread_frame_no == 0){
                        mqtt_go = 1;
                        exit = 1;
                        PRINTF("mqtt_thread: sent GO_ACK!\n");
                    }
                    mqtt_thread_frame_no ++;
                    mqtt_thread_mailbox.free(msg);
                    break;

                case HDLC_RESP_RETRY_W_TIMEO:
                    Thread::wait(msg->content.value/1000);
                    PRINTF("mqtt_thread: retry frame_no %d \n", mqtt_thread_frame_no);
                    if (send_hdlc_mail(msg2, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt) < 0)
                    {
                        while (send_hdlc_retry_mail (msg2, &mqtt_thread_mailbox) < 0){
                            Thread::wait(10);
                        }
                    }
                    mqtt_thread_mailbox.free(msg);
                    break;

                default:
                    mqtt_thread_mailbox.free(msg);
                    break;
                   /* Error */                       
            } 
        }
        else{
            PRINTF("mqtt_thread: resetting the mbed\n");
            reset_system();
        }
        if(exit){
            exit = 0;
            break;
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
                        PRINTF("mqtt_thread: sent frame_no %d!\n", mqtt_thread_frame_no);
                        exit = 1;
                        mqtt_thread_mailbox.free(msg);
                        break;    
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg->content.value/1000);
                        PRINTF("mqtt_thread: retry frame_no %d \n", mqtt_thread_frame_no);
                        if (send_hdlc_mail(msg2, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt) < 0)
                        {
                            while (send_hdlc_retry_mail (msg2, &mqtt_thread_mailbox) < 0)
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
                                mqtt_recv = (mqtt_pkt_t *) uart_pkt_get_data(buf->data, buf->length);
                                PRINTF("The data received is %s \n", mqtt_recv->data);
                                PRINTF("The topic received is %s \n", mqtt_recv->topic); 
                                process_mqtt_pkt(mqtt_recv, &mqtt_recv_data);
                                switch (mqtt_recv_data.data_type){
                                    case NORM_DATA:
                                        PRINTF("MQTT: Normal Data Received %s \n", mqtt_recv_data.data);
                                        break;

                                    case SUB_CMD:
                                        build_mqtt_pkt_sub(mqtt_recv_data.data, MBED_MQTT_PORT, &mqtt_send, &pkt);
                                        if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt))
                                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                                        else
                                            PRINTF("mqtt_thread: failed to send pkt no\n"); 
                                        break;

                                    case PUB_CMD:
                                        //second byte is the length of the topic 
                                        pub_length = mqtt_recv_data.data[0] - '0';
                                        memcpy(topic_pub, (mqtt_recv_data.data + 1), pub_length);
                                        strcpy(data_pub, mqtt_recv_data.data + pub_length + 1);                             
                                        topic_pub[pub_length]='\0';
                                        PRINTF("The the topic_pub %s\n", topic_pub);
                                        PRINTF("The data_pub %s\n", data_pub);                                 
                                        build_mqtt_pkt_pub(topic_pub, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt);
                                        if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt))
                                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                                        else
                                            PRINTF("mqtt_thread: failed to send pkt no\n"); 
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


Mutex       data_mutex; 
float       sense_position_of_line;

void _cont_thread()
{
    hdlc_entry_t contr_thr = { NULL, CONT_THR_PORT, &cont_thr_mailbox };
    hdlc_register(&contr_thr);

    char            frame_no = 0;
    mqtt_pkt_t      mqtt_send;
    char            topic_pub[16] = "line\0";
    char            data_pub[32];
    char            send_data[HDLC_MAX_PKT_SIZE];
    int             status = 0;
    hdlc_pkt_t      pkt;
    pkt.data        = send_data;  
    pkt.length      = HDLC_MAX_PKT_SIZE;
    msg_t           *msg, *msg2;
    osEvent         evt;
    uart_pkt_hdr_t  send_hdr = { 0, 0, 0};
    int             exit = 0;

    while (1) 
    {
        // PRINTF("In mqtt_thread");
        // myled2 =! myled3;
        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 
        pkt.length = HDLC_MAX_PKT_SIZE;        

        while(1)
        {
            evt = cont_thr_mailbox.get();
            if (evt.status == osEventMail) 
            {
                msg = (msg_t*)evt.value.p;
                switch (msg->type)
                {
                    case HDLC_RESP_SND_SUCC:
                        PRINTF("_cont_thread: sent frame_no %d!\n", frame_no);
                        exit = 1;
                        cont_thr_mailbox.free(msg);
                        break;    
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg->content.value/1000);
                        PRINTF("_cont_thread: retry frame_no %d \n", frame_no);
                        if (send_hdlc_mail(msg2, HDLC_MSG_SND, &cont_thr_mailbox, (void*) &pkt) < 0)
                        {
                            while (send_hdlc_retry_mail (msg2, &cont_thr_mailbox) < 0)
                                Thread::wait(10);
                        }
                        cont_thr_mailbox.free(msg);
                        break;
                    case INTER_THREAD:
                        sprintf(data_pub, "%d%f",SENSOR_DATA, msg->content.line);                                                         
                        build_mqtt_pkt_pub(topic_pub, data_pub, CONT_THR_PORT, &mqtt_send, &pkt);
                        PRINTF("_cont_thread: sending update %s\n", mqtt_send.data);
                        if (send_hdlc_mail(msg, HDLC_MSG_SND, &cont_thr_mailbox, (void*) &pkt))
                            PRINTF("_cont_thread: sending pkt no %d \n", frame_no); 
                        else
                            PRINTF("_cont_thread: failed to send pkt no\n"); 

                        cont_thr_mailbox.free(msg);
                        break;
                    default:
                        cont_thr_mailbox.free(msg);
                        /* error */
                        break;
                }
            }    
            if(exit) {
                exit = 0;
                break;
            }
        }
        frame_no++;
    }
}

int main(void)
{
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    Thread mqtt_thr;
    mqtt_thr.start(_mqtt_thread);
    msg_t *msg;

    Thread contr_thr;
    contr_thr.start(_cont_thread);
    PRINTF("Starting the MBED\n");
        // Parameters that affect the performance
    float speed = 0.1;
    float correction = 0.1;   
    float threshold = 0.5;
 
    
    m3pi.locate(0,1);
    m3pi.printf("Line Flw");
 
    wait(2.0);
    
    m3pi.sensor_auto_calibrate();
    int countt = 0;
    int countt1 = 0;

    while(!mqtt_go)
    {
        // PRINTF("main_thr: waiting for go \n");
        Thread::wait(100);
    }


    while (1) 
    {

        // -1.0 is far left, 1.0 is far right, 0.0 in the middle
        float position_of_line = m3pi.line_position();

        // Line is more than the threshold to the right, slow the left motor
        if (position_of_line > threshold) {
            countt = 0;
            countt1 ++;

            m3pi.right_motor(speed);
            m3pi.left_motor(speed-correction);
            PRINTF("main_thr: case 1\n");
            if( countt1 == 1 ){
                msg = cont_thr_mailbox.alloc(); 
                msg->type = INTER_THREAD;
                msg->content.line = position_of_line;
                cont_thr_mailbox.put(msg);
            }
            
        }
 
        // Line is more than 50% to the left, slow the right motor
        else if (position_of_line < -threshold) {
            countt = 0;
            countt1 ++;

            m3pi.left_motor(speed);
            m3pi.right_motor(speed-correction);
            PRINTF("main_thr: case 2\n");

            if( countt1 == 1 ){
                msg = cont_thr_mailbox.alloc(); 
                msg->type = INTER_THREAD;
                msg->content.line = position_of_line;
                cont_thr_mailbox.put(msg);
            }
        }
 
        // Line is in the middle
        else {
            m3pi.forward(speed);
            countt ++;
            countt1 = 0;
            if( countt == 1 ){
                PRINTF("main_thr: case 3\n");

                msg = cont_thr_mailbox.alloc(); 
                msg->type = INTER_THREAD;
                msg->content.line = position_of_line;
                cont_thr_mailbox.put(msg);
            }
        }
        // Thread::wait(10);    
    }
    /* should be never reached */
    return 0;
}
