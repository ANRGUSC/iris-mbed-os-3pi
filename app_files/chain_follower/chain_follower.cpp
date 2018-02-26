/**
 * helper functions chain follower example;
 */


#include <inttypes.h>
#include <string.h>
#include "hdlc.h"
#include "rtos.h"


#define NET_SLAVE_PORT 5001

void network_msg_handler()
{

    switch(state) {
        case stop_beacon:
            //ack any stop messages
            //wait for a "go range me" signal
            //when "go range me" received, request range and change state
        case ranging:
            //ack any "go range me messages"
            //wait for ranging thread to tell you ranging is done (repeat req is failed)
            //when ranging is done, send movement request to thread and change state
        case send_stops:
            //keep sending stop signals
            //when stop ack is received, turn on beaconing and change state
        case range_me_ack:
            //repeat "go range me" messages
            //if a "go range me" ack is received, go to range_me_no_ack state
            //if a stop beacon msg received, go to stop_beacon state
        case range_me_no_ack:
            //if a stop beacon msg received, send stop-beacon-ack and change to stop-beacon state
    }
}

void send_msg(char *ipv6_addr_str, uint16_t port, 
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

    msg_t *msg, *msg2;
    while (send_hdlc_mail(msg, HDLC_MSG_SND, src_mailbox, (void*) &hdlc_pkt) < 0)
    {
        Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
    }

    while(1)
    {
        evt = src_mailbox->get();

        if (evt.status == osEventMail) 
        {
            msg = (msg_t*)evt.value.p;

            switch (msg->type)
            {
                case HDLC_RESP_SND_SUCC:
                    src_mailbox->free(msg);
                    /* done! */
                    return;
                case HDLC_RESP_RETRY_W_TIMEO:
                    Thread::wait(msg->content.value/1000);
                    src_mailbox->free(msg);

                    while (send_hdlc_mail(msg, HDLC_MSG_SND, src_mailbox, (void*) &hdlc_pkt) < 0)
                    {
                        Thread::wait(HDLC_RTRY_TIMEO_USEC * 1000);
                    }
                    break;
                default:
                    src_mailbox->free(msg); //error!
                    break;
            } // switch
        } // if
    } // while
}