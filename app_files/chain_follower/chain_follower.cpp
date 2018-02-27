/**
 * helper functions chain follower example 
 */

#include <inttypes.h>
#include <string.h>
#include "hdlc.h"
#include <rtos.h>
#include <mbed.h>
#include <rtos.h> 
#include "m3pi.h"
#include "main-conf.h"

#include "chain_follower.h"

#define DEBUG   1

#if (DEBUG) 
    #define PRINTF(...) pc.printf(__VA_ARGS__)
    extern Serial pc;
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

#define INVALID_MSG             -1
#define START_RANGE_ME_MSGS     0
#define STOP_RANGE_ME_MSGS      1
#define START_STOP_BEACONS_MSGS 2
#define STOP_STOP_BEACONS_MSGS  3

Thread network_helper_thr(osPriorityNormal, (uint32_t) DEFAULT_STACK_SIZE/2);
Mail<msg_t, HDLC_MAILBOX_SIZE>  network_helper_mailbox;

static void _tdoa_beacons(bool on, uint8_t node_id, Mail<msg_t, HDLC_MAILBOX_SIZE> *src_mailbox, 
    uint16_t src_hdlc_port, hdlc_pkt_t *hdlc_pkt)
{
    uart_pkt_hdr_t uart_hdr;
    uart_hdr.src_port = src_hdlc_port;
    uart_hdr.dst_port = RANGE_BEACONER_PORT;
    if (on) {
        uart_hdr.pkt_type = RANGE_BEACON_START;
    } else {
        uart_hdr.pkt_type = RANGE_BEACON_STOP;
    }

    //hardcoded max packet size
    uart_pkt_insert_hdr(hdlc_pkt->data, HDLC_MAX_PKT_SIZE, &uart_hdr);

    if (on) {
        hdlc_pkt->length = uart_pkt_cpy_data(hdlc_pkt->data, HDLC_MAX_PKT_SIZE, &node_id, sizeof(uint8_t));
    } else {
        hdlc_pkt->length = UART_PKT_HDR_LEN; //header only pkt
    }

    msg_t *msg;
    while (send_hdlc_mail(msg, HDLC_MSG_SND, src_mailbox, (void*) hdlc_pkt) < 0)
    {
        Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
    }
}

void tdoa_beacons_on(uint8_t node_id, Mail<msg_t, HDLC_MAILBOX_SIZE> *src_mailbox, 
    uint16_t src_hdlc_port, hdlc_pkt_t *hdlc_pkt) 
{
    _tdoa_beacons(1, node_id, src_mailbox, src_hdlc_port, hdlc_pkt);
}

void tdoa_beacons_off(Mail<msg_t, HDLC_MAILBOX_SIZE> *src_mailbox, 
    uint16_t src_hdlc_port, hdlc_pkt_t *hdlc_pkt) 
{
    _tdoa_beacons(0, 0, src_mailbox, src_hdlc_port, hdlc_pkt);
}

void net_send_udp(const char *ipv6_addr_str, uint16_t port, uint8_t net_msg_type,
    Mail<msg_t, HDLC_MAILBOX_SIZE> *src_mailbox, uint16_t src_hdlc_port, 
    hdlc_pkt_t *hdlc_pkt)
{   
    PRINTF("net_send_udp() a net_msg_type of %d\n", net_msg_type);
    uart_pkt_hdr_t uart_hdr;
    uart_hdr.src_port = src_hdlc_port;
    uart_hdr.dst_port = NET_SLAVE_PORT;
    uart_hdr.pkt_type = NET_SEND_UDP;
    //hardcoded max packet size
    uart_pkt_insert_hdr(hdlc_pkt->data, HDLC_MAX_PKT_SIZE, &uart_hdr); 
    //insert ipv6 addr string along with terminating null char
    memcpy(hdlc_pkt->data + UART_PKT_HDR_LEN, ipv6_addr_str, strlen(ipv6_addr_str) + 1);

    //insert destination UDP port number (uin16_t)
    hdlc_pkt->data[strlen(ipv6_addr_str) + 1] = port & 0xFF; //lower byte
    hdlc_pkt->data[strlen(ipv6_addr_str) + 2] = port >> 8; //upper byte
     
    //insert payload of ONE byte representing the message net_msg_type
    hdlc_pkt->data[strlen(ipv6_addr_str) + 3] = net_msg_type;

    //hdlc packet length is sum of all parts
    hdlc_pkt->length = UART_PKT_HDR_LEN + strlen(ipv6_addr_str) + 4;

    if (hdlc_pkt->length > HDLC_MAX_PKT_SIZE) {
        PRINTF("net_send_udp(): packet too large for hdlc!\n");
    }

    msg_t *msg;
    while (send_hdlc_mail(msg, HDLC_MSG_SND, src_mailbox, (void*) hdlc_pkt) < 0)
    {
        Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
    }
    PRINTF("net_send_udp: Success\n");

}

static void _network_helper()
{
    //registering for HDLC, but we should not receive packets here
    hdlc_entry_t network_helper = { NULL, NETWORK_HELPER_PORT, &network_helper_mailbox };
    hdlc_register(&network_helper);

    hdlc_pkt_t hdlc_pkt;
    char data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt.data = data;

    msg_t *msg;

    uint32_t timeout = 0;
    int net_msg_type = INVALID_MSG; 

    while (true)
    {
        osEvent evt = network_helper_mailbox.get(timeout); 

        if(evt.status == osEventTimeout) {
            //send range or stop beacons 
            if (net_msg_type == RANGE_ME) {
                net_send_udp(FOLLOWING_ROBOT_IPV6_ADDR, FORWARD_TO_MBED_MAIN_PORT,
                    RANGE_ME, &network_helper_mailbox, NETWORK_HELPER_PORT,
                    &hdlc_pkt);
                PRINTF("Sending RANGE_ME\n");
            } else if(net_msg_type == STOP_BEACONS) {
                net_send_udp(LEADING_ROBOT_IPV6_ADDR, FORWARD_TO_MBED_MAIN_PORT,
                    STOP_BEACONS, &network_helper_mailbox, NETWORK_HELPER_PORT,
                    &hdlc_pkt);
                PRINTF("Sending STOP_BEACONS\n");

            }
            continue; //get next mail
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

                while (send_hdlc_mail(msg, HDLC_MSG_SND, &network_helper_mailbox, (void*) &hdlc_pkt) < 0)
                {
                    Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                }
                break;
            case HDLC_PKT_RDY:
                //should not receive packets in helper thread
                network_helper_mailbox.free(msg);
                break;
            case START_RANGE_ME_MSGS:
                //send first message and start timeout to retry ever 1 second
                net_send_udp(FOLLOWING_ROBOT_IPV6_ADDR, FORWARD_TO_MBED_MAIN_PORT,
                    RANGE_ME, &network_helper_mailbox, NETWORK_HELPER_PORT,
                    &hdlc_pkt);
                PRINTF("Sending RANGE_ME\n");
                net_msg_type = RANGE_ME;
                timeout = 1000;
                network_helper_mailbox.free(msg);
                break;
            case STOP_RANGE_ME_MSGS:
                net_msg_type = INVALID_MSG; 
                timeout = 0;
                PRINTF("STOP RANGE_ME\n");

                network_helper_mailbox.free(msg);
                break;
            case START_STOP_BEACONS_MSGS:
                //send first message and start timeout to retry ever 1 second
                net_send_udp(LEADING_ROBOT_IPV6_ADDR, FORWARD_TO_MBED_MAIN_PORT,
                    STOP_BEACONS, &network_helper_mailbox, NETWORK_HELPER_PORT,
                    &hdlc_pkt);
                net_msg_type = STOP_BEACONS;
                timeout = 1000;
                network_helper_mailbox.free(msg);
                PRINTF("Start STOP_BEACONS_MSGS\n");

                break;
            case STOP_STOP_BEACONS_MSGS: 
                net_msg_type = INVALID_MSG;
                timeout = 0;
                PRINTF("Start STOP_STOP_BEACONS_MSGS\n");
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
    network_helper_thr.start(_network_helper);
}

void start_sending_stop_beacons_msgs()
{
    msg_t *msg = network_helper_mailbox.alloc();
    msg->type = START_STOP_BEACONS_MSGS;
    msg->content.ptr = NULL; //don't care
    msg->sender_pid = osThreadGetId();
    msg->source_mailbox = NULL; //don't care
    network_helper_mailbox.put(msg);
}
void stop_sending_stop_beacons_msgs()
{
    msg_t *msg = network_helper_mailbox.alloc();
    msg->type = STOP_STOP_BEACONS_MSGS;
    msg->content.ptr = NULL; //don't care
    msg->sender_pid = osThreadGetId();
    msg->source_mailbox = NULL; //don't care
    network_helper_mailbox.put(msg);
}
void start_sending_range_me_msgs()
{
    msg_t *msg = network_helper_mailbox.alloc();
    msg->type = START_RANGE_ME_MSGS;
    msg->content.ptr = NULL; //don't care
    msg->sender_pid = osThreadGetId();
    msg->source_mailbox = NULL; //don't care
    network_helper_mailbox.put(msg);
}
void stop_sending_range_me_msgs()
{
    msg_t *msg = network_helper_mailbox.alloc();
    msg->type = STOP_RANGE_ME_MSGS;
    msg->content.ptr = NULL; //don't care
    msg->sender_pid = osThreadGetId();
    msg->source_mailbox = NULL; //don't care
    network_helper_mailbox.put(msg);
}