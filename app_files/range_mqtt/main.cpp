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
#include "mbed_movement.h"

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

#define MAX_NUM_ANCHORS 20     
#define DATA_STRING_SIZE  9

typedef struct __attribute__((packed)) {
    uint8_t         last_pkt;      
    range_data_t    data[DATA_PER_PKT];                  
} range_hdr_t;

typedef struct __attribute__((packed)) {
    int8_t         node_id;      
    uint16_t       tdoa;                  
} node_t;

static node_t nodes_reached[MAX_NUM_ANCHORS];
static uint8_t num_nodes_reached;

static uint8_t num_nodes_to_pub;

/**
 * @brief      Returns range data as a node_t
 *
 * @param[in]  data  The range data as a range_data_t struct
 *
 * @return     The data as a node_t.
 */
node_t get_node(range_data_t data){
    return {data.node_id, data.tdoa};
}

/**
 * @brief      Loads data into a buffer for publishing.
 *
 * @param      buff       The data buffer
 * @param[in]  buff_size  The buffer size
 * @param[in]  node       The node_t containing the range data
 *
 * @return     returns 0 on success, otherwise returns -1 on failure
 */
int load_data(char *buff, int buff_size, node_t node){
    if((num_nodes_to_pub + 1) * DATA_STRING_SIZE >= buff_size - 1){
        PRINTF("Buffer is full\n");
        return -1;
    }
    snprintf(buff + (num_nodes_to_pub * DATA_STRING_SIZE), buff_size - (num_nodes_to_pub * DATA_STRING_SIZE), "%05d,%02d;", node.tdoa, node.node_id);
    num_nodes_to_pub++;
    PRINTF("# of nodes = %d\n",num_nodes_to_pub);
    return 0;
}

/**
 * @brief      clears the data in the data buffer
 *
 * @param      buff       The data buffer
 * @param[in]  buff_size  The buffer size
 */
void clear_data(char *buff, int buff_size){
    memset(buff,0,buff_size);
    num_nodes_to_pub = 0;
}

/**
 * @brief      Gets the distance in feet and angle in degrees.
 *
 * @param      time_diffs    The range data
 * @param[in]  ranging_mode  The ranging mode
 *
 * @return     The distance and angle in a dist_angle_t struct.
 */
dist_angle_t get_dist_angle(range_data_t *time_diffs, uint8_t ranging_mode){
    int tdoa_a;
    int tdoa_b;
    float dist_a;
    float dist_b;
    float dist;
    float angle = -361;

    dist_angle_t return_val;
    return_val.distance = -1;
    return_val.angle = 0;
    return_val.node_id = 0;

    tdoa_a = time_diffs->tdoa;
    dist_a = get_dist(tdoa_a);

    switch (ranging_mode)
    {
        case ONE_SENSOR_MODE:
            dist = dist_a;
            break;
        case TWO_SENSOR_MODE:
            if(time_diffs->status > 2)
            {
                PRINTF("Missed pin %d\n", MISSED_PIN_UNMASK - time_diffs-> status); 
            } 
            else
            {
                tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                dist_b = get_dist(tdoa_b);
                PRINTF("OD = %lu\n", time_diffs-> orient_diff);
            }
            break;
        case XOR_SENSOR_MODE:
            tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
            dist_b = get_dist(tdoa_b);
            PRINTF("OD = %lu\n", time_diffs->orient_diff);
            break;
        case OMNI_SENSOR_MODE:
            dist = dist_a;
            break;
    }
    if(dist_b != 0){
        dist = get_mid_dist(dist_a, dist_b);
        if(time_diffs->status == 2){
            angle = get_angle(dist_b, dist_a);
        }
        else{
            angle = get_angle(dist_a, dist_b);
        }
        PRINTF("Distance: %.2f\n", dist);
        PRINTF("Angle : %.2f\n", angle);
    }
    else{
        PRINTF("Distance: %.2f\n", dist);
    }
     printf("******************************\n");

    return_val.distance = dist;
    return_val.angle = angle;
    return_val.node_id = time_diffs->node_id;

    printf("Range: Returning value\n");
    printf("Range: %.2f, %.2f, %d\n",return_val.distance,return_val.angle,return_val.node_id);
    return return_val;
}

/**
 * @brief      Gets the range data by communicating with the openmote over hdlc.
 *
 * @param[in]  params  The parameters for ranging
 *
 * @return     The range data.
 */
range_data_t get_range_data(range_params_t params){
    ranging = 1;
 
    int exit = 0;
    int pkt_size = sizeof(uart_pkt_hdr_t) + sizeof(range_params_t);
    char send_data[pkt_size];

    msg_t *msg, *msg2;
    hdlc_buf_t *buf;
    hdlc_pkt_t pkt;
    
    uart_pkt_hdr_t recv_hdr;
    uart_pkt_hdr_t send_hdr = (uart_pkt_hdr_t){ MBED_RANGE_PORT, RIOT_MQTT_PORT, SOUND_RANGE_REQ };
    
    /* misc */
    osEvent evt;

    range_data_t* time_diffs;
    range_hdr_t* range_hdr;
    
    int i = 0;
    int j = 0;
    int data_per_pkt;

    //send ranging request packet up hdlc   
    pkt.data = send_data;
    pkt.length = pkt_size;   
    uart_pkt_insert_hdr(pkt.data, pkt.length, &send_hdr); 
    uart_pkt_cpy_data(pkt.data, pkt.length, &params, sizeof(range_params_t));

    PRINTF("src_port: %d, dst_port: %d\n", send_hdr.src_port, send_hdr.dst_port);
    PRINTF("node_id: %d\n", params.node_id);

    if (send_hdlc_mail(msg, HDLC_MSG_SND, &range_thr_mailbox, (void*) &pkt)){
        PRINTF("range_thread: sending range_req pkt\n"); 
    }
    else{
        PRINTF("range_thread: failed to send pkt no\n"); 
        return {0,0,0,params.node_id};
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

                        if(data_per_pkt == 0){
                            ranging = 0;
                            hdlc_pkt_release(buf);
                            range_thr_mailbox.free(msg);
                            return (range_data_t){0,0,0,params.node_id};
                        }

                        for(i = 0; i < data_per_pkt; i++){
                            PRINTF ("%d:\n", i);
                            /* Displaying results. */
                            
                            PRINTF("TDoA = %lu\n", time_diffs->tdoa);

                            switch (params.ranging_mode)
                            {
                                case ONE_SENSOR_MODE:
                                    PRINTF("One Sensor Mode^\n");
                                    break;
                                case TWO_SENSOR_MODE:
                                    if(time_diffs->status > 2){
                                        printf("Missed pin %d\n", MISSED_PIN_UNMASK - time_diffs->status); 
                                    } 
                                    else{
                                        PRINTF("OD = %lu\n", time_diffs->orient_diff);
                                    }
                                    PRINTF("Two Sensor Mode^\n");
                                    break;
                                case XOR_SENSOR_MODE:
                                    PRINTF("OD = %lu\n", time_diffs->orient_diff);
                                    PRINTF("Xor Sensor Mode^\n");
                                    break;
                                case OMNI_SENSOR_MODE:
                                    PRINTF("Omni Sensor Mode^\n");
                                    break;
                            }

                            if(params.node_id == -1){
                                nodes_reached[j] = (node_t) {time_diffs->node_id, time_diffs->tdoa};
                                j++;
                                if(j >= MAX_NUM_ANCHORS){
                                    printf("Exceeded max number of anchors\n");
                                    return (range_data_t){0,0,0,params.node_id};
                                }
                                num_nodes_reached = j;
                            }
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

    return *(time_diffs-1);
}

/**
 * @brief      Discovers all nodes that can be ranged with. This is a wrapper function for get range data. The data is stored in a
 *             static array of node_t data
 *
 * @param[in]  ranging_mode  The ranging mode
 */
void range_all(uint8_t ranging_mode){
    get_range_data((range_params_t){-1, ranging_mode});
}

/**
 * @brief      Looks for a specific node to range with based on what's given in params. This is a wrapper function for get range data.
 *
 * @param[in]  params  The ranging parameters
 *
 * @return     the range data
 */
range_data_t range_node(range_params_t params){
    return get_range_data(params);
}

uint16_t lock_on_anchor(int8_t node_id){
    range_data_t raw_data; 
    dist_angle_t conv_data;
    float angle;

    if(!init_minimu()){
        PRINTF("Failed to init minimu");
        return;
    }

    calibrate_compass();

    while(angle > 5 && angle < -5){
        
        raw_data = range_node({node_id, TWO_SENSOR_MODE});
        if(raw_data.tdoa = 0){
            PRINTF("Locking failed: Anchor node unavailable\n");
            return 0;
        }
        conv_data = get_dist_angle(raw_data);
        angle = conv_data.angle;

        if(angle == -361){
            rotate_degrees(180,40);
        }
        else{
            if(angle < 75 && angle > -75){
                rotate_parts(angle);
            }
            else{
                rotate_degrees(angle,40);
            }
        }
    }
    return range_data.tdoa;
}

/**
 * @brief      This is the range thread that triggers the range routine for localization.
 *             To trigger localization, a msg of type START_RANGE_THR must be sent to the
 *             range_thread_mailbox containing the parameters for localization.
 */
void _range_thread(){

    char            data_pub[32];
    hdlc_pkt_t      pkt;

    int i = 0;
    
    pkt.data = data_pub;
    pkt.length = 32;

    mqtt_pkt_t      mqtt_send;
    osEvent evt;
    msg_t *msg;

    range_data_t range_data;
    range_params_t range_params;

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

                //***This is where the range routine will go*****
                
                PRINTF("range_thread: got range init message, starting routine\n");
                range_params = *(range_params_t*)(msg->content.ptr);

                if(range_params.node_id == -1){
                    range_all(range_params.ranging_mode);
                    printf("****************Discovery mode***************\n");
                    printf("Nodes reached:\n");
                    for(i=0; i<num_nodes_reached; i++){
                        printf("Node %d: %lu\n", nodes_reached[i].node_id, nodes_reached[i].tdoa);
                    }
                    printf("*********************************************\n");
                }
                else{
                    if(range_params.ranging_mode == TWO_SENSOR_MODE){
                        range_data = lock_on_anchor(range_params.node_id);
                    }
                    else{
                         range_data = range_node(range_params);
                    }

                    load_data(data_pub, 32, get_node(range_data));                         

                    build_mqtt_pkt_pub(RANGE_TOPIC, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt); 

                    //for some reason it will only publish if you include a print statement here
                    printf("tdoa = %lu\n",range_data.tdoa);
                    printf("node_id = %lu\n",range_data.node_id);
                    PRINTF("range_thread: range_routine done. publishing data now\n");

                    if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt)){
                        PRINTF("mqtt_thread: sending pkt of size\n"); 
                    }
                    else{

                        PRINTF("mqtt_thread: failed to send pkt no\n"); 
                    }
                    clear_data(data_pub, 32);
                }
                

                //*************************************************
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
    hdlc_pkt_t      pkt;
    range_params_t  params;

    pkt.data        = send_data;  
    pkt.length      = 32;

    mqtt_pkt_t      *mqtt_recv;
    mqtt_pkt_t      mqtt_send;
    mqtt_data_t     mqtt_recv_data;

    uart_pkt_hdr_t  send_hdr = { 0, 0, 0};
    hdlc_buf_t      *buf;
    uart_pkt_hdr_t  recv_hdr;
    
    int             exit = 0;
    hdlc_entry_t    mqtt_thread = { NULL, MBED_MQTT_PORT, &mqtt_thread_mailbox };
    hdlc_register(&mqtt_thread);
    
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
                                        if(strcmp(pub_msg + 2, START_RANGE_MSG) == 0){
                                            PRINTF("MQTT: pub_msg matches START_RANGE_MSG\n");
                                            if(!ranging){
                                                PRINTF("MQTT: telling thread to start ranging with node_id: %d\n",pub_msg[0] - '0');
                                                params.node_id = pub_msg[0] - '0';
                                                params.ranging_mode = pub_msg[1];
                                                msg2 = range_thr_mailbox.alloc();
                                                msg2->type = START_RANGE_THR;
                                                msg2->content.ptr = &params;
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
