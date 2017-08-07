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
#include "m3pi.h"
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
//to reset the mbed
extern "C" void mbed_reset();

#define DEBUG   1
#define TEST_TOPIC   ("init_info")

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

/* the only instance of pc -- debug statements in other files depend on it */
Serial                          pc(USBTX,USBRX,115200);
DigitalOut                      myled3(LED3); //to notify when a character was received on mbed
DigitalOut                      myled(LED1);
DigitalOut                      reset_riot(p20,1);

bool mqtt_go = 0;
volatile int turn = 0;
volatile int priochk = 0;
m3pi m3pi;

Mail<msg_t, HDLC_MAILBOX_SIZE>  mqtt_thread_mailbox;
Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

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
    
    uart_pkt_hdr_t  send_hdr = { 0, 0, 0};
    hdlc_buf_t      *buf;
    uart_pkt_hdr_t  recv_hdr;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = get_hdlc_mailbox();
    Mail<msg_t, HDLC_MAILBOX_SIZE> *mailbox_ptr;
    
    int             exit = 0;
    hdlc_entry_t    mqtt_thread = { NULL, MBED_MQTT_PORT, &mqtt_thread_mailbox };
    hdlc_register(&mqtt_thread);
    
    char            test_pub[] = "Hello world";
    char            topic_pub[16];
    char            data_pub[32];
    char            data_rssi_pub[32];
    int             len_clients=0;
    //length is set to 9 because of the null terminator 
    char            clients[2][9];       
    int             count=0;
    char            *node_ID; 
    char            node_new_ID[9];
    char            node_send_ID[9];      
                                
    osEvent         evt;


    /**
     * Check if the MQTT coneection is established by the openmote. If not,
     * DO NOT Proceed further. The openmote sends a MQTT_GO msg once the mqtt connection is properly setup.
     */
    while(1)
    {
        evt = mqtt_thread_mailbox.get(60000);
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
        PRINTF("resetting the mbed\n");
        mbed_reset();
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
                        
                        if (mqtt_thread_frame_no==30){
                            //resetting the mbed and riot after 30 iterations
                            printf("RESETTING the mbed\n");
                            reset_riot=0;                            
                            Thread::wait(400);
                            reset_riot=1;
                            Thread::wait(400);
                            reset_riot=0;
                            Thread::wait(400);
                            reset_riot=1;
                            mbed_reset();
                        }
                       
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
                    case INTER_THREAD:  
                        turn=1;
                        Thread::wait(200);
                        PRINTF("******************\n"); 
                        PRINTF("4\n");  
                        PRINTF("******************\n");                      
                        PRINTF("Pub message received\n");                        
                        PRINTF("The node_new_ID is %s\n", node_new_ID);
                        strcpy(data_pub,"2");
                        strcat(data_pub,node_new_ID);                       
                        //error in the strcat                        
                        PRINTF("The data to be pubbed is %s\n", data_pub);
                        PRINTF("The topic is %s\n",TEST_TOPIC);
                        build_mqtt_pkt_pub(TEST_TOPIC, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt);                        
                        if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt)){
                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                        }
                        else{
                            PRINTF("mqtt_thread: failed to send pkt no\n");
                        }                        
                        
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
                                        if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt)){
                                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                                        }
                                        else{
                                            PRINTF("mqtt_thread: failed to send pkt no\n"); 
                                        }
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
                                        if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt)){
                                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                                        }
                                        else{
                                            PRINTF("mqtt_thread: failed to send pkt no\n"); 
                                        }
                                        break;
                                    case LEN_CLIENTS_LIST:
                                        //receives the number of clients the server_script
                                        //is going to send to the node
                                        len_clients=mqtt_recv_data.data[0] - '0';
                                        PRINTF("The length of the clients list is %d \n",len_clients);
                                        break;
                                    case GET_CLIENTS:
                                        //handles the storing the clients and initiating the 
                                        //RSSI PING PONG
                                        if (len_clients!=0){                                            
                                            strcpy(clients[count], mqtt_recv_data.data);  
                                            PRINTF("The value of the client is %s\n", clients[count]);
                                            count++; 
                                            len_clients=len_clients-1;                                           
                                        }
                                        if (len_clients==0){
                                            for(int i=0;i<count;i++){
                                                PRINTF("The clients list %s\n",clients);                                                
                                            }
                                            if (strcmp(node_ID,PRIORITY_NODE)==0){
                                                PRINTF("STARTING THE RSSI PING PONG\n");
                                                priochk =1;
                                            }
                                            //send rssi_go message to the rssi thread
                                            msg2 = hdlc_mailbox_ptr->alloc();
                                            while (msg2 == NULL)
                                            {
                                                msg2 = mqtt_thread_mailbox.alloc();  
                                                Thread::wait(10);
                                            } 
                                            PRINTF("******************\n"); 
                                            PRINTF("2\n");  
                                            PRINTF("******************\n");

                                            msg2->type = INTER_THREAD;                                   
                                            msg2->sender_pid = osThreadGetId();
                                            msg2->source_mailbox = &mqtt_thread_mailbox;
                                            main_thr_mailbox.put(msg2);
                                            PRINTF("mqtt_thread: RSSI_GO message sent\n");
                                            mqtt_thread_mailbox.free(msg);
                                            count=0;
                                        }
                                        break;
                                    case RSSI_SEND: 
                                        turn =1; 
                                        priochk =1;                                      
                                        PRINTF("******************\n"); 
                                        PRINTF("6\n");  
                                        PRINTF("******************\n");
                                        //Tells the openmote to send an RSSI message over udp
                                        PRINTF("mqtt_thread: RSSI SEND message received\n");
                                        
                                        for (int c=0; c<2; c++){                                           
                                            if (strcmp(clients[c],node_ID)==0){
                                                PRINTF("the current node ID is %s\n", clients[c]);
                                            }
                                            else{
                                                PRINTF("the other clients are %s\n", clients[c]);   
                                                count =0;
                                                while(count < 8){                                                    
                                                    node_send_ID[count]=clients[c][count];
                                                    count++;
                                                }                                      
                                            }
                                        }
                                        node_send_ID[8]='\0';
                                        PRINTF("the node_send_ID is %s\n",node_send_ID);
                                        send_hdr.src_port = MBED_MQTT_PORT;
                                        send_hdr.dst_port = RSSI_RIOT_PORT;
                                        send_hdr.pkt_type = RSSI_SND;
                                        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 
                                        strcpy(pkt.data+UART_PKT_HDR_LEN, node_send_ID);

                                        if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt)){
                                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                                        }
                                        else{
                                            PRINTF("mqtt_thread: failed to send pkt no\n");
                                        }
                                        count=0;
                                        PRINTF("REACHED the end of it\n");                                                                               
                                        //send rssi_send to the rssi thread in RIOT
                                        break;
                                }                                                       
                                break;

                            case MQTT_SUB_ACK:
                                PRINTF("mqtt_thread: SUB ACK message received\n");
                                break;
                            case MQTT_PUB_ACK:
                                PRINTF("******************\n"); 
                                PRINTF("5\n");  
                                PRINTF("******************\n");
                                PRINTF("mqtt_thread: PUB ACK message received\n");
                                break;
                            case HWADDR_GET:
                                PRINTF("mqtt_thread: HWADDR received\n");
                                node_ID = (char *)uart_pkt_get_data(buf->data,buf->length);
                                memcpy(node_new_ID,node_ID,sizeof(node_new_ID));
                                node_new_ID[8]='\0';
                                
                                PRINTF("the new node ID is %s\n", node_new_ID);
                                PRINTF("mqtt_thread: %s\n",node_new_ID); 
                                PRINTF("******************\n"); 
                                PRINTF("1\n");  
                                PRINTF("******************\n");
                                break;  
                            case RSSI_PUB:                                 
                                PRINTF("******************\n"); 
                                PRINTF("8\n");  
                                PRINTF("******************\n");;
                                PRINTF("Pub message received\n");
                                strcpy(data_pub,"2");
                                strcat(data_pub,node_new_ID);                                                                           
                                PRINTF("The data to be pubbed is %s\n", data_pub);
                                PRINTF("The topic is %s\n",TEST_TOPIC);                               
                                build_mqtt_pkt_pub(TEST_TOPIC, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt);                        
                                if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt)){
                                    PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                                }
                                else{
                                    PRINTF("mqtt_thread: failed to send pkt no\n");
                                }
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
    msg_t *msg, *msg2;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t pkt;
    pkt.data = send_data;
    pkt.length = 0;
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

    Thread mqtt_thr;
    mqtt_thr.start(_mqtt_thread);
    
    int             exit = 0;

    osEvent         evt;
    myled = 1;

    while (1) 
    {
        // PRINTF("In mqtt_thread");
        myled =! myled;
        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 
        pkt.length = HDLC_MAX_PKT_SIZE;        

        while(1)
        {
            evt = main_thr_mailbox.get(3000);
            if (evt.status == osEventMail) 
            {
                msg = (msg_t*)evt.value.p;
                switch (msg->type)
                {               
                    
                    case INTER_THREAD:
                        //communicates with the mbed_mqtt thread
                        if (priochk==1){
                            //starting the rssi send messages                            
                            PRINTF("******************\n"); 
                            PRINTF("3\n");  
                            PRINTF("******************\n");
                            PRINTF("rssi_thread: RSSI_GO received\n");
                            Thread::wait(2000);
                            msg2 = hdlc_mailbox_ptr->alloc();
                            while (msg2 == NULL)
                            {
                                msg2 = main_thr_mailbox.alloc();  
                                Thread::wait(10);
                            }  
                            //delay so all the node receives all the addresses
                            Thread::wait(2000);                                          
                            msg2->type = INTER_THREAD;                                   
                            msg2->sender_pid = osThreadGetId();
                            msg2->source_mailbox = &main_thr_mailbox;
                            mqtt_thread_mailbox.put(msg2);
                            PRINTF("rssi_thread: RSSI_PUB message sent\n");
                            main_thr_mailbox.free(msg);                                                                                                                                   
                        }
                        break;
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
                        PRINTF("RSSI packet received\n");
                        buf = (hdlc_buf_t *)msg->content.ptr;   
                        uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
                        switch (recv_hdr.pkt_type)
                        {
                            case RSSI_DATA_PKT:                                
                                PRINTF("******************\n"); 
                                PRINTF("7\n");  
                                PRINTF("******************\n");
                                //handles the movement of the robot
                                rssi_value = (int8_t)(* ((char *)uart_pkt_get_data(buf->data, buf->length)));
                                rssi_value =rssi_value-73;
                                //displaying for now 
                                PRINTF("rssi_thread: RSSI is %d\n", rssi_value); 
                                //conditions to satisfy depending on the value of the RSSI
                                
                                if (rssi_value<(-38))
                                {
                                    PRINTF("The value is less than forty, move cautiously\n");
                                    m3pi.backward(speed);
                                    Thread::wait(delta_t);
                                    m3pi.stop();
                                    if (rssi_value<(-50)){
                                        PRINTF("The value is less than -50, move back\n");
                                    }
                                }
                                else
                                {
                                    PRINTF("greater than -40, move forward normally\n");
                                    m3pi.forward(speed);
                                    Thread::wait(delta_t);
                                    m3pi.stop();
                                }                                                      
                                                               
                                break;
                            default:
                                main_thr_mailbox.free(msg);
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

            if (evt.status == osEventTimeout){
                if (turn==1 && priochk==1)
                {
                    PRINTF("******************\n"); 
                    PRINTF("8\n");  
                    PRINTF("******************\n");                                       
                    msg2 = hdlc_mailbox_ptr->alloc();
                    while (msg2 == NULL)
                    {
                        PRINTF("error\n");
                        msg2 = main_thr_mailbox.alloc();  
                        Thread::wait(10);
                    }                                                                                  
                    msg2->type = INTER_THREAD;                                   
                    msg2->sender_pid = osThreadGetId();
                    msg2->source_mailbox = &main_thr_mailbox;
                    mqtt_thread_mailbox.put(msg2);
                    PRINTF("rssi_thread: RSSI_PUB message sent\n");                  
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
