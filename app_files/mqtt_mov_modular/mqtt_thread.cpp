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
#include "app-conf.h"

#include "mqtt_thread.h"

//to reset the mbed
extern "C" void mbed_reset();

extern Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

DigitalOut myled3(LED3); //to notify when a character was received on mbed


#define DEBUG   1
#define TEST_TOPIC   ("init_info")

#if (DEBUG) 
    #define PRINTF(...) pc.printf(__VA_ARGS__)
    extern Serial pc;
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

DigitalOut reset_riot(p26,1);


static unsigned char MQTT_STACK[DEFAULT_STACK_SIZE];

Mail<msg_t, HDLC_MAILBOX_SIZE>  mqtt_thread_mailbox;
Thread mqtt(osPriorityNormal, 
    (uint32_t) DEFAULT_STACK_SIZE, (unsigned char *)MQTT_STACK); 


volatile int mqtt_state = MQTT_DISCON;
static char mqtt_m3pi_ID[9];


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
/**
 * @brief      This is the MQTT thread on MBED
 */
void _mqtt_thread()
{
    int pub_length;
    char mqtt_thread_frame_no = 0;
    msg_t *msg, *msg2;
    float sensor_data;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    
    hdlc_pkt_t pkt = {send_data, HDLC_MAX_PKT_SIZE}; 

    mqtt_pkt_t  mqtt_send;
    mqtt_data_t mqtt_recv_data;
     
    uart_pkt_hdr_t send_hdr = { 0, 0, 0};
    hdlc_buf_t     *buf;
    uart_pkt_hdr_t recv_hdr;

    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr;
    hdlc_mailbox_ptr = get_hdlc_mailbox();
    
    int exit = 0;
    hdlc_entry_t mqtt_thread = { NULL, MBED_MQTT_PORT, &mqtt_thread_mailbox };
    hdlc_register(&mqtt_thread);
    
    char topic_pub[16];
    char data_pub[32];

    osEvent evt;

    char *rmt_ctrl_command;
    msg_t *msg_rmt_ctrl;

    /**
     * Check if the MQTT connection is established by the openmote. If not,
     * DO NOT Proceed further before the follwing steps are complete. 
     * (1) The openmote sends a MQTT_GO msg once the mqtt connection is properly setup.
     * (2) The MBED replies by sending a MQTT_GO_ACK msg to the Openmote
     * (3) The Openmote sends the HWADDR to the mbed
     * (4) The MBED replies by sending a HW_ADDR_ACK
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
                        mqtt_state = MQTT_RECV_MQTT_GO;
                        PRINTF("mqtt_thread: the node is conected to the broker \n");
                        send_hdr.pkt_type = MQTT_GO_ACK;
                        send_hdr.dst_port = RIOT_MQTT_PORT;
                        send_hdr.src_port = MBED_MQTT_PORT;
                        uart_pkt_insert_hdr(pkt.data, UART_PKT_HDR_LEN, &send_hdr); 
                        mqtt_thread_mailbox.free(msg);
                        hdlc_pkt_release(buf);
                        pkt.length = UART_PKT_HDR_LEN;        

                        if (send_hdlc_mail(msg2, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt))
                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                        else
                            PRINTF("mqtt_thread: failed to send pkt no\n");
                    }
                    else if (recv_hdr.pkt_type == HWADDR_GET){
                        mqtt_state = MQTT_RECV_HW_ADDR;
                        memcpy(mqtt_m3pi_ID, (char *)uart_pkt_get_data(buf->data, buf->length), EMCUTE_ID_LEN);  
                        mqtt_m3pi_ID[8]='\0';                     
                        PRINTF("mqtt_thread: HWADDR received; own node ID is %s\n", mqtt_m3pi_ID);
                        
                        send_hdr.pkt_type = HWADDR_ACK;
                        send_hdr.dst_port = RIOT_MQTT_PORT;
                        send_hdr.src_port = MBED_MQTT_PORT;
                        uart_pkt_insert_hdr(pkt.data, UART_PKT_HDR_LEN, &send_hdr); 
                        mqtt_thread_mailbox.free(msg);
                        hdlc_pkt_release(buf);
                        pkt.length = UART_PKT_HDR_LEN;        

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
                        PRINTF("mqtt_thread: sent GO_ACK!\n");
                    }
                    else{
                        exit = 1;
                        PRINTF("mqtt_thread: sent HW_ACK!\n");
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
    mqtt_state = MQTT_MBED_INIT_DONE;

    PRINTF("mqtt_thread: All Initialization Done\n");

    msg_rmt_ctrl = main_thr_mailbox.alloc();

    /**
     * The follwing is the main portion of the mqtt thread. make your changes here.
     */

    // Thread::wait(2000);
    // reset_system();
    while (1) 
    {
        // PRINTF("In mqtt_thread");
        myled3 =! myled3;
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
                                
                                process_mqtt_pkt((mqtt_pkt_t *) uart_pkt_get_data(buf->data, buf->length), &mqtt_recv_data);
                                // PRINTF("The data received is %s \n", mqtt_recv_data->data);
                                PRINTF("The topic received is %s \n", mqtt_recv_data.topic); 
                                switch (mqtt_recv_data.data_type){
                                    case NORM_DATA:
                                        PRINTF("MQTT: Normal Data Received %s \n", mqtt_recv_data.data);
                                        break;

                                    case SUB_CMD:
                                        build_mqtt_pkt_sub(mqtt_recv_data.data, MBED_MQTT_PORT, &mqtt_send, &pkt);
                                        if (send_hdlc_mail(msg2, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt))
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
                                        PRINTF("The topic to pub to %s\n", topic_pub);
                                        PRINTF("The data to pub %s\n", data_pub);                                 
                                        
                                        build_mqtt_pkt_pub(topic_pub, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt);
                                        if (send_hdlc_mail(msg2, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt))
                                            PRINTF("mqtt_thread: sending pkt no %d \n", mqtt_thread_frame_no); 
                                        else
                                            PRINTF("mqtt_thread: failed to send pkt no\n"); 
                                        break;

                                    case RMT_CTRL:
                                        PRINTF("mqtt_thread: the command received is %s\n", mqtt_recv_data.data);
                                        rmt_ctrl_command = (char *)mqtt_recv_data.data;
                                        msg_rmt_ctrl->type = INTER_THREAD;
                                        msg_rmt_ctrl->content.ptr = rmt_ctrl_command;
                                        msg_rmt_ctrl->sender_pid = osThreadGetId();
                                        main_thr_mailbox.put(msg_rmt_ctrl);                                       
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


Mail<msg_t, HDLC_MAILBOX_SIZE> *mqtt_init(osPriority priority) 
{
    mqtt.set_priority(priority);
    mqtt.start(_mqtt_thread);
    // PRINTF("mqtt: thread  id %d\n",mqtt.gettid());
    return &mqtt_thread_mailbox;
}


Mail<msg_t, HDLC_MAILBOX_SIZE> *get_mqtt_mailbox()
{
    return &mqtt_thread_mailbox;
}


Mutex data_mtx;

int get_mqtt_state (void){
    data_mtx.lock();
    int state = mqtt_state;
    data_mtx.unlock();
    return state;
}

void set_mqtt_state (int state){
    data_mtx.lock();
    mqtt_state = state;
    data_mtx.unlock();
}

void get_node_id (char *ret)
{
    strcpy(ret, mqtt_m3pi_ID); 
}
