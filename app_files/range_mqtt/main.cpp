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
#include "data_conv.h"
#include "mqtt.h"

#define START_RANGE_MSG   "INIT_RANGE"

#define DEBUG   1
#define TEST_TOPIC   ("test/trial")
#define RANGE_TOPIC   ("range_info")

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
static volatile bool ranging = 0;

static char* pub_msg;
static Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;

Mail<msg_t, HDLC_MAILBOX_SIZE>  mqtt_thread_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE>  range_thr_mailbox;
 
#define NULL_PKT_TYPE   0xFF 
#define PKT_FROM_MAIN_THR   0

#define MBED_RANGE_PORT 5678
#define DATA_PER_PKT        ((HDLC_MAX_PKT_SIZE - UART_PKT_HDR_LEN - 1) / RANGE_DATA_LEN)
#define START_RANGE_THR 139       

typedef struct __attribute__((packed)) {
    uint8_t         last_pkt;      
    range_data_t    data[DATA_PER_PKT];                  
} range_hdr_t;

dist_angle_t get_range_data(){
    ranging = 1;
 
    int exit = 0;
    int pkt_size = sizeof(uart_pkt_hdr_t) + sizeof(range_params_t);
    char send_data[pkt_size];
    int tdoa_a;
    int tdoa_b;
    float dist_a;
    float dist_b;
    float dist;
    float angle = -361;
    float avg_dist = 0;
    float avg_angle = 0;

    
    msg_t *msg, *msg2;
    hdlc_buf_t *buf;
    hdlc_pkt_t pkt;
    
    uart_pkt_hdr_t recv_hdr;
    uart_pkt_hdr_t send_hdr = (uart_pkt_hdr_t){ MBED_RANGE_PORT, RIOT_MQTT_PORT, SOUND_RANGE_REQ };
    range_params_t params = (range_params_t){1, OMNI_SENSOR_MODE};
    
    /* misc */
    osEvent evt;

    range_data_t* time_diffs;
    range_hdr_t* range_hdr;

    dist_angle_t return_val;
    return_val.distance = -1;
    return_val.angle = 0;
    return_val.node_id = 0;

    
    int i = 0;
    int data_per_pkt;

    //send ranging request packet up hdlc   
    pkt.data = send_data;
    pkt.length = pkt_size;   
    uart_pkt_insert_hdr(pkt.data, pkt.length, &send_hdr); 
    uart_pkt_cpy_data(pkt.data, pkt.length, &params, sizeof(range_params_t));

    PRINTF("src_port: %d, dst_port: %d\n", send_hdr.src_port, send_hdr.dst_port);

    if (send_hdlc_mail(msg, HDLC_MSG_SND, &range_thr_mailbox, (void*) &pkt)){
        PRINTF("range_thread: sending range_req pkt\n"); 
    }
    else{
        PRINTF("range_thread: failed to send pkt no\n"); 
        return return_val;
    }
    //recieving data
    while(!exit)
    {
        PRINTF("range_thread: Waiting for response\n");
        evt = range_thr_mailbox.get();
        
        if(evt.status == osEventMail)
        {
            PRINTF("range_thread: Got a response\n");
            msg = (msg_t*)evt.value.p;

            switch (msg->type)
            {
                case HDLC_RESP_SND_SUCC:
                    PRINTF("range_thread: sent frame!\n");
                    range_thr_mailbox.free(msg);
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    Thread::wait(msg->content.value/1000);
                    PRINTF("range_thread: retrying\n");

                    /* Tries to allocate memory for msg2. */
                    msg2 = hdlc_mailbox_ptr->alloc();
                    if(msg2 == NULL) 
                    {
                        /* Blocking call until the memory has been allocated. */
                        // Thread::wait(50);
                        while(msg2 == NULL)
                        {
                            msg2 = range_thr_mailbox.alloc();  
                            Thread::wait(10);
                        }
                        msg2->type = HDLC_RESP_RETRY_W_TIMEO;
                        msg2->content.value = (uint32_t) RTRY_TIMEO_USEC;
                        msg2->sender_pid = osThreadGetId();
                        msg2->source_mailbox = &range_thr_mailbox;
                        range_thr_mailbox.put(msg2);
                    }
                    else
                    {
                        msg2->type = HDLC_MSG_SND;
                        msg2->content.ptr = &pkt;
                        msg2->sender_pid = osThreadGetId();
                        msg2->source_mailbox = &range_thr_mailbox;
                        hdlc_mailbox_ptr->put(msg2);
                    }
                    range_thr_mailbox.free(msg);
                    break;

                case HDLC_PKT_RDY:
                    /* Setting up buf, making it easier to access data from the msg. */ 
                    buf = (hdlc_buf_t *)msg->content.ptr;   
                    uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);

                    if(recv_hdr.pkt_type == SOUND_RANGE_DONE) 
                    {
                        PRINTF("range_thread: received range pkt\n");
                        range_hdr = (range_hdr_t *)uart_pkt_get_data(buf->data, buf->length);
                        time_diffs = (range_data_t *)range_hdr->data;
                        
                        data_per_pkt = (buf->length - sizeof(uart_pkt_hdr_t) - sizeof(uint8_t))/sizeof(range_data_t);
                        PRINTF("range_thread: There should be %d ranges in this pkt\n",data_per_pkt);

                        for(i = 0; i < data_per_pkt; i++){
                            PRINTF ("%d:\n", i);
                            tdoa_a = 0;
                            tdoa_b = 0;
                            /* Displaying results. */

                            if(time_diffs->status == RF_MISSED){
                                PRINTF("RF Ping missed\n");
                                time_diffs++;
                                continue;
                            }
                            else if(time_diffs->status == ULTRSND_MISSED){
                                PRINTF("Ultrsnd Ping missed\n");
                                time_diffs++;
                                continue;
                            }
                            
                            
                            tdoa_a = time_diffs->tdoa;
                            dist_a = tdoa_to_dist(tdoa_a);
                            PRINTF("TDoA = %lu\n", tdoa_a);

                            switch (params.ranging_mode)
                            {
                                case ONE_SENSOR_MODE:
                                    dist = dist_a;
                                    break;
                                case TWO_SENSOR_MODE:
                                    if(time_diffs->status > 2)
                                    {
                                        printf("Missed pin %d\n", MISSED_PIN_UNMASK - time_diffs-> status); 
                                    } 
                                    else
                                    {
                                        tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                                        dist_b = tdoa_to_dist(tdoa_b);
                                        PRINTF("OD = %lu\n", time_diffs-> orient_diff);
                                    }
                                    break;
                                case XOR_SENSOR_MODE:
                                    tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                                    dist_b = tdoa_to_dist(tdoa_b);
                                    PRINTF("OD = %lu\n", time_diffs-> orient_diff);
                                    break;
                                case OMNI_SENSOR_MODE:
                                    dist = dist_a;
                                    break;
                            }

                            //printf("\n******************************\n", dist);
                           if(tdoa_b != 0){
                                dist = calc_x(dist_a, dist_b);
                                if(time_diffs->status == 2){
                                    angle = od_to_angle(dist_b, dist_a);
                                }
                                else{
                                    angle = od_to_angle(dist_a, dist_b);
                                }
                                printf("Distance: %.2f\n", dist);
                                printf("Angle : %.2f\n", angle);
                            }
                            else{
                                printf("Distance: %.2f\n", dist);
                            }
                             printf("******************************\n");

                            return_val.distance = dist;
                            return_val.angle = angle;
                            return_val.node_id = 5;

                            time_diffs++;
                        }

                        if(range_hdr->last_pkt == 1){
                            printf("All data recieved\n");
                            exit = 1;
                        }
                    }
                    else
                    {
                        printf("main_thr: recieved non-range pkt\n");
                        exit = 1;
                    }
                    hdlc_pkt_release(buf);
                    range_thr_mailbox.free(msg);
                    break;
                default:
                    PRINTF("Reached default case\n");
                    /* error */
                    //LED3_ON;
                    range_thr_mailbox.free(msg);
                    break;
            }
        } 
        else{
            printf("Range: Didn't get mail: %02x\n",evt.status);
        }   
        if(exit) 
        {
            printf("Range: Exiting loop\n");
            exit = 0;
            break;
        }
    }
    
    ranging = 0;

    printf("Range: Returning value\n");
    printf("Range: %.2f, %.2f, %lu\n",return_val.distance,return_val.angle,return_val.node_id);
    return return_val;
}

void _range_thread(){

    char            topic_pub[16];
    char            data_pub[32];
    char *ptr;
    hdlc_pkt_t      pkt;
    
    pkt.data = data_pub;
    pkt.length = 32;

    mqtt_pkt_t      mqtt_send;
    osEvent evt;
    msg_t *msg, *msg2;

    dist_angle_t range_data;

    hdlc_entry_t range_thread = { NULL, MBED_RANGE_PORT, &range_thr_mailbox };
    hdlc_register(&range_thread);

    while(1)
    {

        PRINTF("range_thread: Waiting for start message\n");
        evt = range_thr_mailbox.get();
        if(evt.status == osEventMail)
        {
            PRINTF("range_thread: got mail\n");
            msg = (msg_t*)evt.value.p;
            if(msg->type == START_RANGE_THR){
                PRINTF("range_thread: got range init message, starting routine\n");
                range_data = get_range_data();
                PRINTF("range_thread: range_routine done. publishing data now\n");
                
                memcpy(topic_pub, RANGE_TOPIC, 10);
                snprintf(data_pub, 32, "%.2f,%d", range_data.distance, range_data.node_id);                          
                topic_pub[10]='\0';
                PRINTF("The the topic_pub %s\n", topic_pub);
                PRINTF("The data_pub %s\n", data_pub); 

                PRINTF("range_thread: 1- %s\n",data_pub);
                build_mqtt_pkt_pub(RANGE_TOPIC, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt);
                if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt)){
                    PRINTF("mqtt_thread: sending pkt of size\n"); 
                }
                else{

                    PRINTF("mqtt_thread: failed to send pkt no\n"); 
                }
            }
            else{
                PRINTF("range_thread: Recieved something other than start message\n");
                continue;
            }
        } 
        else{
            printf("range_thread: Didn't get mail: %02x\n",evt.status);
            continue;
        }  
    }
    
}

/**
 * @brief      This is the MQTT thread on MBED
 */
void _mqtt_thread()
{
    int             pub_length;
    char            mqtt_thread_frame_no = 0;
    msg_t           *msg, *msg2;
    char            send_data[32];
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
    
    int             exit = 0;
    hdlc_entry_t    mqtt_thread = { NULL, MBED_MQTT_PORT, &mqtt_thread_mailbox };
    hdlc_register(&mqtt_thread);
    
    char            test_pub[] = "Hello world";
    char            topic_pub[16];
    char            data_pub[32];

    range_data_t* time_diffs;
    range_hdr_t* range_hdr;

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
        uart_pkt_insert_hdr(pkt.data, 32, &send_hdr); 
        pkt.length = 32;        

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
                                        pub_msg = mqtt_recv_data.data;
                                        if(strcmp(pub_msg, START_RANGE_MSG) == 0){
                                            PRINTF("MQTT: pub_msg matches START_RANGE_MSG\n");
                                            if(!ranging){
                                                PRINTF("MQTT: telling thread to start ranging\n");
                                                msg2 = range_thr_mailbox.alloc();
                                                msg2->type = START_RANGE_THR;
                                                msg2->content.ptr = NULL;
                                                msg2->sender_pid = osThreadGetId();
                                                msg2->source_mailbox = &mqtt_thread_mailbox;
                                                range_thr_mailbox.put(msg2);
                                            }
                                        } else{
                                            PRINTF("MQTT: %s does not match %s\n", pub_msg, START_RANGE_MSG);
                                        }

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
                                        pub_length = mqtt_recv_data.data[0];
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


int main(void)
{
    hdlc_mailbox_ptr = hdlc_init(osPriorityRealtime);
   
    PRINTF("Starting mqtt thread\n");
    Thread mqtt_thr;
    mqtt_thr.start(_mqtt_thread);

    PRINTF("Starting range thread\n");
    Thread range_thr;
    range_thr.start(_range_thread);
    
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
