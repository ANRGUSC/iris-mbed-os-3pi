/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
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
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Full duplex hdlc implementation.
 *
 * This implementation leverages yahdlc, an open source library. The current 
 * implementation is stop & wait.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @}
 */



#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include "mbed.h"
#include "platform/CircularBuffer.h"

#include "hdlc.h"

#include "rtos.h"

#define DEBUG 1

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) */

static int hdlc_mail_count=0;
DigitalOut led2(LED2);


Serial pc(USBTX,USBRX,115200);

#define UART_BUFSIZE            (512U)
#define UART_NUM        3


static osThreadId hdlc_dispatcher_pid, sender_pid, hdlc_thread_pid;

Thread        *hdlcthread; //

Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mail_box;
Mail<msg_t, HDLC_MAILBOX_SIZE> *dispacher_hdlc_mail_box;


Timer global_time;

CircularBuffer<char, UART_BUFSIZE> *ctx;
Serial *hdlc_pc; //used to connect to pc for debugging through a virtual COM port

void write_hdlc(uint8_t *,int);

// static uart_ctx_t ctx;

static char hdlc_recv_data[HDLC_MAX_PKT_SIZE];
static char hdlc_send_frame[2 * (HDLC_MAX_PKT_SIZE + 2 + 2 + 2)];
static char hdlc_ack_frame[2 + 2 + 2 + 2];

static hdlc_buf_t recv_buf; // recv_buf.data = hdlc_recv_data; the initialization is done in the hdlc init function
static hdlc_buf_t send_buf;// = { .data = hdlc_send_frame };
static hdlc_buf_t ack_buf;//  = { .data = hdlc_ack_frame };

/* uart access control lock */
static uint32_t uart_lock = 0;


static void rx_cb(void)//(void *arg, uint8_t data)
{
    // uart_t dev = (uart_t)arg;
    // 
    unsigned char data;

    while (hdlc_pc->readable()) {
        data=hdlc_pc->getc();     // Get an character from the Serial
        ctx->push(data);     // Put to the ring/circular buffer

        if (data == YAHDLC_FLAG_SEQUENCE) {
            
            // wakeup hdlc thread 
            msg_t *msg = hdlc_mail_box->alloc();
            if(msg==NULL)
            {
                  PRINTF("hdlc: rx_cb no more space available on mailbox\n");
                  return;
            }
            hdlc_mail_count++;
            msg->sender_pid=osThreadGetId();
            msg->type = HDLC_MSG_RECV;
            msg->source_mailbox=hdlc_mail_box;
            // msg.content.value = (uint32_t)dev;
            hdlc_mail_box->put(msg); 
        }
    }

}

static void _hdlc_receive(unsigned int *recv_seq_no, unsigned int *send_seq_no)
{
    msg_t *msg, *ack_msg;
    int ret;
    int retval;
    char c;

    while(1) {
        retval = ctx->pop(c);

        if (retval == 0) {
            break;
        }

        
        // c = (char) retval;
        recv_buf.mtx.lock();

        ret = yahdlc_get_data(&recv_buf.control, &c, 1, recv_buf.data, &recv_buf.length);
        recv_buf.mtx.unlock();

        if(ret == -EIO) {
            puts("FCS ERROR!");
            PRINTF("hdlc: DATA FRAME SIZE %d\n", recv_buf.length);
            recv_buf.control.frame = (yahdlc_frame_t)0;
            recv_buf.control.seq_no = 0;
        }


        if (recv_buf.length > 0 && ret != -EIO && 
            (recv_buf.control.seq_no == *recv_seq_no % 8 ||
            recv_buf.control.seq_no == (*recv_seq_no - 1) % 8)) {
            /* valid data frame received */
            PRINTF("hdlc: received data frame w/ seq_no: %d\n", recv_buf.control.seq_no);



            /* always send ack */
            ack_msg = hdlc_mail_box->alloc();
            if(ack_msg==NULL)
            {
              PRINTF("hdlc: ACK no more space available on mailbox\n");
              return;
            }
            hdlc_mail_count++;
            ack_msg->sender_pid=osThreadGetId();
            ack_msg->type = HDLC_MSG_SND_ACK;
            ack_msg->content.value = recv_buf.control.seq_no;
            ack_msg->source_mailbox = hdlc_mail_box;
            hdlc_mail_box->put(ack_msg); 
            
            // PRINTF("mailbox: _hdlc_receive hdlc_mail_count %d\n",hdlc_mail_count);
            // msg_send_to_self(&ack_msg); /* send ack */

            /* pass on packet to dispatcher */
            if (recv_buf.control.seq_no == *recv_seq_no % 8) {
                /* lock pkt until dispatcher makes a copy and unlocks */
                recv_buf.mtx.lock();
                recv_buf.length = recv_buf.length; // ???????
                PRINTF("hdlc: got and expected seq_no %d\n", *recv_seq_no);
                msg = dispacher_hdlc_mail_box->alloc();
                msg->sender_pid = osThreadGetId();
                msg->type = HDLC_PKT_RDY;
                msg->content.ptr = &recv_buf;
                msg->source_mailbox = hdlc_mail_box;
                dispacher_hdlc_mail_box->put(msg); 
                hdlc_mail_count++;
                // msg_send(&msg, hdlc_dispatcher_pid);

                (*recv_seq_no)++;
            }

            recv_buf.control.frame = (yahdlc_frame_t)0;
            recv_buf.control.seq_no =  0;

        } else if (recv_buf.length == 0 && ret != -EIO &&
                    (recv_buf.control.frame == YAHDLC_FRAME_ACK ||
                     recv_buf.control.frame == YAHDLC_FRAME_NACK)) {
            PRINTF("hdlc: received ACK/NACK w/ seq_no: %d\n", recv_buf.control.seq_no);

            if(recv_buf.control.seq_no == *send_seq_no % 8) {
                (*send_seq_no)++;
                uart_lock = 0;
                msg=dispacher_hdlc_mail_box->alloc();
                msg->sender_pid=osThreadGetId();
                msg->type = HDLC_RESP_SND_SUCC;
                msg->content.value = (uint32_t) 0;
                msg->source_mailbox = hdlc_mail_box;
                dispacher_hdlc_mail_box->put(msg);
                hdlc_mail_count++; 
                PRINTF("hdlc: sender_pid is %d\n", sender_pid);
            }
                                
            recv_buf.control.frame = (yahdlc_frame_t)0;
            recv_buf.control.seq_no = 0;
        }
    }
}

static void hdlc(void const *arg)
{
    // uart_t dev = (uart_t)arg;
 
    uint32_t last_sent = 0;
    msg_t *msg, *reply, *msg2;
    unsigned int recv_seq_no = 0;
    unsigned int send_seq_no = 0;
    osEvent evt;

    while(1) {

        // PRINTF("hdlc: inside HDLC loop\n");
        // PRINTF("mailbox: hdlc hdlc_mail_count %d\n",hdlc_mail_count);
        
        led2=!led2;
        if(uart_lock) {
            int timeout = (int)RETRANSMIT_TIMEO_USEC - (int) global_time.read_us();
            if(timeout < 0) {
                PRINTF("hdlc: inside timeout negative\n");
                /* send message to self to resend msg */
                msg2=hdlc_mail_box->alloc();
                if(msg2==NULL) {
                    PRINTF("hdlc: no more space available on mailbox");
                    evt = hdlc_mail_box->get(50);
                    if(evt.status == osEventTimeout){
                        continue;
                    }
                }
                else
                {
                    hdlc_mail_count++;
                    msg2->sender_pid=osThreadGetId();
                    msg2->type = HDLC_MSG_RESEND;
                    msg2->source_mailbox = hdlc_mail_box;
                    hdlc_mail_box->put(msg2); 
                    evt = hdlc_mail_box->get();
                }

            } else {
                evt = hdlc_mail_box->get(timeout / 1000);
                if(evt.status == osEventTimeout){
                    continue;
                }
            }
        } else {
            PRINTF("hdlc: waiting for mail\n");
            evt = hdlc_mail_box->get();
        }
       
        if (evt.status == osEventMail) 
        {          
            msg = (msg_t*)evt.value.p;

            switch (msg->type) {
                case HDLC_MSG_RECV:
                    PRINTF("hdlc: receiving msg...\n");
                    _hdlc_receive(&recv_seq_no, &send_seq_no);
                    hdlc_mail_box->free(msg);
                    hdlc_mail_count--;
                    break;
                case HDLC_MSG_SND:
                    PRINTF("hdlc: request to send received from pid %d\n", msg->sender_pid);
                    if (uart_lock) {
                        /* ask thread to try again in x usec */
                        PRINTF("hdlc: uart locked, telling thr to retry\n");
                        reply=((Mail<msg_t, HDLC_MAILBOX_SIZE>*)msg->source_mailbox)->alloc();
                        reply->type = HDLC_RESP_RETRY_W_TIMEO;
                        reply->content.value = (uint32_t) RTRY_TIMEO_USEC;
                        reply->sender_pid=osThreadGetId();
                        ((Mail<msg_t, HDLC_MAILBOX_SIZE>*)msg->source_mailbox)->put(reply);
                    } else {
                        uart_lock = 1;
                        sender_pid = msg->sender_pid;
                        PRINTF("hdlc: sender_pid set to %d\n", sender_pid);
                        send_buf.control.frame = YAHDLC_FRAME_DATA;
                        send_buf.control.seq_no = send_seq_no % 8; 

                        hdlc_pkt_t *pkt = (hdlc_pkt_t*)msg->content.ptr;

                        yahdlc_frame_data(&(send_buf.control), pkt->data, 
                                pkt->length, send_buf.data, &send_buf.length);


                        write_hdlc((uint8_t *)send_buf.data, send_buf.length);
                        global_time.reset();
                    }  
                    hdlc_mail_box->free(msg); 
                    hdlc_mail_count--;
                    break;
                case HDLC_MSG_SND_ACK:
                    /* send ACK */
                    ack_buf.control.frame = YAHDLC_FRAME_ACK;
                    ack_buf.control.seq_no = msg->content.value;
                    yahdlc_frame_data(&(ack_buf.control), NULL, 0, ack_buf.data, &(ack_buf.length));    
                    PRINTF("hdlc: sending ack w/ seq no %d, len %d\n", ack_buf.control.seq_no,ack_buf.length);
                    write_hdlc((uint8_t *)ack_buf.data, ack_buf.length);
                    // hdlc_pc->write((uint8_t *)ack_buf.data, ack_buf.length,0,0);   
                    hdlc_mail_box->free(msg);
                    hdlc_mail_count--;
                    break;
                case HDLC_MSG_RESEND:
                    PRINTF("hdlc: Resending frame w/ seq no %d (on send_seq_no %d)\n", send_buf.control.seq_no, send_seq_no);
                    write_hdlc((uint8_t *)send_buf.data, send_buf.length);
                    // hdlc_pc->write((uint8_t *)send_buf.data, send_buf.length,0,0);
                    global_time.reset();
                    hdlc_mail_box->free(msg); 
                    hdlc_mail_count--;
                    break;
                case HDLC_MSG_REG_DISPATCHER:
                    PRINTF("hdlc: Registering dispatcher thread.\n");
                    hdlc_dispatcher_pid = msg->sender_pid;
                    dispacher_hdlc_mail_box=(Mail<msg_t, HDLC_MAILBOX_SIZE>*)msg->source_mailbox;
                    PRINTF("hdlc: hdlc_dispatcher_pid set to %d\n", hdlc_dispatcher_pid);
                    hdlc_mail_box->free(msg);
                    hdlc_mail_count--;
                    break;
                default:
                    PRINTF("INVALID HDLC MSG\n");
                    //LED3_ON;
                    hdlc_mail_box->free(msg); 
                    hdlc_mail_count--;
                    break;
            }
        }
    }

    // /* this should never be reached */
    // return;
}

int hdlc_pkt_release(hdlc_buf_t *buf) 
{
    if(buf->mtx.trylock())
    {
        PRINTF("hdlc: Packet not locked. Might be empty!\n");
        buf->mtx.unlock();
        return -1;
    }
    else
    {
        buf->control.frame = (yahdlc_frame_t)0;
        buf->control.seq_no = 0;
        buf->mtx.unlock();
        return 0;
    }

}

bool send_hdlc_pkt(msg_t *msg_req) 
{

    msg_t *msg;
    msg=hdlc_mail_box->alloc();
    if(msg==NULL) // No space available in the mailbox.
        return 0;

    hdlc_mail_count++;
    // PRINTF("mailbox: (send_hdlc_pkt) hdlc hdlc_mail_count %d\n",hdlc_mail_count);
    memcpy(msg,msg_req,sizeof(msg_t));
    hdlc_mail_box->put(msg);
    
    return 1;
}

void write_hdlc(uint8_t *ptr,int len)
{
    int count = 0;

    while ( count < len ) {
        if (hdlc_pc->writeable()) {
            hdlc_pc->putc(ptr[count]);   
            count++;
        }
    }
}

int hdlc_init(int stacksize, osPriority priority, const char *name, int dev, void **mail)
{
    led2=1;
    recv_buf.data = hdlc_recv_data;
    send_buf.data = hdlc_send_frame;
    ack_buf.data = hdlc_ack_frame;
    global_time.start();

    hdlc_mail_box = new Mail<msg_t, HDLC_MAILBOX_SIZE>;
    *mail = hdlc_mail_box;

    ctx = new CircularBuffer<char, UART_BUFSIZE>();
    hdlc_pc = new Serial(p28,p27, 115200); // need to make the pin tx/rx name into a function call
    hdlc_pc->attach(&rx_cb,Serial::RxIrq);
    
    hdlcthread = new Thread(hdlc);
    hdlcthread->set_priority(priority);
    
    hdlc_thread_pid = hdlcthread->gettid();
    PRINTF("hdlc: thread  id %d\n",hdlc_thread_pid);
    
    PRINTF("hdlc: hdlc thread creation done!!\n");

    return (int)hdlc_thread_pid;
}

