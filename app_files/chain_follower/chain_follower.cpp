/**
 * helper functions chain follower example;
 */

#include <inttypes.h>
#include <string.h>
#include "hdlc.h"
#include <rtos.h>
#include <mbed.h>
#include <rtos.h> 
#include "controller.h"
#include "m3pi.h"
#include "main-conf.h"

#define DEBUG   1

#if (DEBUG) 
    #define PRINTF(...) pc.printf(__VA_ARGS__)
    extern Serial pc;
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

#define INVALID_MSG 0

Thread network_helper_thr(osPriorityNormal, (uint32_t) DEFAULT_STACK_SIZE/2);
Mail<msg_t, HDLC_MAILBOX_SIZE>  network_helper_mailbox;

#define NETWORK_HELPER_PORT 7000
#define NET_SLAVE_PORT 5001
#define FORWARD_TO_MBED_MAIN_PORT  8000

void send_msg(char *ipv6_addr_str, uint16_t port, type 
    Mail<msg_t, HDLC_MAILBOX_SIZE> *src_mailbox, uint16_t src_hdlc_port)
{
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr = get_hdlc_mailbox();
    hdlc_pkt_t hdlc_pkt;
    char data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt.data = data;
    uart_pkt_hdr_t uart_hdr = { src_hdlc_port, NET_SLAVE_PORT, NET_SEND_UDP};
    uart_pkt_insert_hdr(hdlc_pkt.data, sizeof(hdlc_pkt.data), &uart_hdr); 
    //insert ipv6 addr string along with terminating null char
    uart_pkt_cpy_data(hdlc_pkt.data, sizeof(hdlc_pkt.data), ipv6_addr_str, strlen(ipv6_addr_str) + 1)

    msg_t *msg;
    while (send_hdlc_mail(msg, HDLC_MSG_SND, src_mailbox, (void*) &hdlc_pkt) < 0)
    {
        Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
    }
}

void network_helper()
{
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mailbox_ptr = get_hdlc_mailbox();
    hdlc_pkt_t hdlc_pkt;
    char data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt.data = data;
    uart_pkt_hdr_t uart_hdr = { NETWORK_HELPER_PORT, NET_SLAVE_PORT, NET_SEND_UDP };
    uart_pkt_insert_hdr(hdlc_pkt.data, sizeof(hdlc_pkt.data), &uart_hdr); 

    msg_t *msg;

    uint32_t timeout = 0;
    int net_msg = INVALID_MSG; 

    while (true)
    {
        osEvent evt = network_helper_mailbox.get(timeout); 

        if(evt.status == osEventTimeout) {
            //send range or stop beacons 
            if (net_msg == RANGE_ME) {
                //insert ipv6 addr string along with terminating null char
                uart_pkt_cpy_data(hdlc_pkt.data, ipv6_addr_str, strlen(ipv6_addr_str) + 1)

                //insert destination UDP port number (uin16_t)
                 
                //insert payload of ONE byte representing the message type
                

                //send packet
                while (send_hdlc_mail(msg, HDLC_MSG_SND, network_helper_mailbox, (void*) &hdlc_pkt) < 0)
                {
                    Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                }
            } else if(net_msg = STOP_BEACONS) {
                //insert ipv6 addr string along with terminating null char
                uart_pkt_cpy_data(hdlc_pkt.data, sizeof(hdlc_pkt.data), ipv6_addr_str, strlen(ipv6_addr_str) + 1)

                //insert destination UDP port number (uin16_t)
                
                //insert payload of ONE byte representing the message type
                

                //send packet
                while (send_hdlc_mail(msg, HDLC_MSG_SND, network_helper_mailbox, (void*) &hdlc_pkt) < 0)
                {
                    Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                }
            }
        }

        if (evt.status != osEventMail) 
            continue; 

        msg = (msg_t*)evt.value.p;
        switch (msg->type)
        {
            case HDLC_RESP_SND_SUCC:
                network_helper_mailbox.free(msg);
                break;
            case HDLC_RESP_RETRY_W_TIMEO:
                Thread::wait(msg->content.value/1000);
                network_helper_mailbox.free(msg);

                while (send_hdlc_mail(msg, HDLC_MSG_SND, network_helper_mailbox, (void*) &hdlc_pkt) < 0)
                {
                    Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                }
                break;
            case HDLC_PKT_RDY:
                //should not receive packets in helper thread
                network_helper_mailbox.free(msg);
                break;
            case START_RANGE_ME:
                net_msg = RANGE_ME;
                timeout = 1000;
                network_helper_mailbox.free(msg);

                //insert ipv6 addr string along with terminating null char
                uart_pkt_cpy_data(hdlc_pkt.data, sizeof(hdlc_pkt.data), ipv6_addr_str, strlen(ipv6_addr_str) + 1);

                //insert destination UDP port number (uin16_t)
                
                
                //insert payload of ONE byte representing the message type
                

                //send packet
                while (send_hdlc_mail(msg, HDLC_MSG_SND, network_helper_mailbox, (void*) &hdlc_pkt) < 0)
                {
                    Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                }
                break;
            case STOP_RANGE_ME:
                net_msg = INVALID_MSG; 
                timeout = 0;
                network_helper_mailbox.free(msg);
                break;
            case START_STOP_BEACONS:
                net_msg = STOP_BEACONS;
                timeout = 1000;
                network_helper_mailbox.free(msg);
                //insert ipv6 addr string along with terminating null char
                memcpy(hdlc_pkt.data, ipv6_addr, strlen(ipv6_addr) + 1);
                uart_pkt_cpy_data(hdlc_pkt.data, sizeof(hdlc_pkt.data), ipv6_addr_str, strlen(ipv6_addr_str) + 1)

                //insert destination UDP port number (uin16_t)

                //insert payload of ONE byte representing the message type
                

                //send packet
                while (send_hdlc_mail(msg, HDLC_MSG_SND, network_helper_mailbox, (void*) &hdlc_pkt) < 0)
                {
                    Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                }
                break;
            case STOP_STOP_BEACONS: 
                net_msg = INVALID_MSG;
                timeout = 0;
                network_helper_mailbox.free(msg);
                break;
            default:
                network_helper_mailbox.free(msg); //error!
                break;
        } // switch
    } //while

    //this should never be reached
}

void network_helper_init(osPriority priority) 
{
    network_helper_thr.set_priority(priority);
    network_helper_thr.start(network_helper);
}

int start_movement (int net_msg_type)
{
    net_msg_t *message = network_message_pool.alloc();
    if (message != NULL)
    {
        message->type = net_msg_type;
        network_helper_queue.put(message);
        return(0);
    }
    return(-1);
}

void start_sending_stop_beacons_msgs()
{


}
void stop_sending_stop_beacons_msgs()
{

}
void start_sending_range_me_msgs()
{

}
void stop_sending_range_me_msgs()
{

}