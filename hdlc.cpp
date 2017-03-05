/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
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
 *
 * @}
 */

//TODO?? Have to figure out how to automatically set DEVICE_SERIAL_ASYNCH=1 without changing the original Serial code
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include "mbed.h"
#include "platform/CircularBuffer.h"

#include "hdlc.h"

#include "rtos.h"
DigitalOut led2(LED2);
DigitalOut led3(LED3);

// #define ENABLE_DEBUG (0)
// #include "debug.h"
Serial pc(USBTX,USBRX,115200);

#define UART_BUFSIZE            (512U)
#define UART_NUM        3
// Mail<mail_t, 16> mail_box;

// static msg_t _hdlc_msg_queue[16];

static osThreadId hdlc_dispatcher_pid, sender_pid, hdlc_thread_pid;
Thread        *hdlcpointer;

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
    data=hdlc_pc->getc();     // Get an character from the Serial
    ctx->push(data);     // Put to the ring/circular buffer

    if(data == YAHDLC_FLAG_SEQUENCE)
    {
        
        // wakeup hdlc thread 
        msg_t *msg = hdlc_mail_box->alloc();
        msg->sender_pid=osThreadGetId();
        msg->type = HDLC_MSG_RECV;
        msg->source_mailbox=hdlc_mail_box;
        // msg.content.value = (uint32_t)dev;
        hdlc_mail_box->put(msg); 
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


        if (recv_buf.length > 0 && ret != -EIO && 
            (recv_buf.control.seq_no == *recv_seq_no % 8 ||
            recv_buf.control.seq_no == (*recv_seq_no - 1) % 8)) {
            /* valid data frame received */
            pc.printf("hdlc: received data frame w/ seq_no: %d\n", recv_buf.control.seq_no);

            /* always send ack */
            ack_msg = hdlc_mail_box->alloc();
            ack_msg->sender_pid=osThreadGetId();
            ack_msg->type = HDLC_MSG_SND_ACK;
            ack_msg->content.value = recv_buf.control.seq_no;
            hdlc_mail_box->put(ack_msg); 
            // msg_send_to_self(&ack_msg); /* send ack */

            /* pass on packet to dispatcher */
            if (recv_buf.control.seq_no == *recv_seq_no % 8) {
                /* lock pkt until dispatcher makes a copy and unlocks */
                recv_buf.mtx.lock();
                recv_buf.length = recv_buf.length; // ???????
                pc.printf("hdlc: got and expected seq_no %d\n", *recv_seq_no);
                msg=dispacher_hdlc_mail_box->alloc();
                msg->sender_pid=osThreadGetId();
                msg->type = HDLC_PKT_RDY;
                msg->content.ptr = &recv_buf;
                dispacher_hdlc_mail_box->put(msg); 
                // msg_send(&msg, hdlc_dispatcher_pid);

                (*recv_seq_no)++;
            }

            recv_buf.control.frame = (yahdlc_frame_t)0;
            recv_buf.control.seq_no =  0;

        } else if (recv_buf.length == 0 && 
                    (recv_buf.control.frame == YAHDLC_FRAME_ACK ||
                     recv_buf.control.frame == YAHDLC_FRAME_NACK)) {
            pc.printf("hdlc: received ACK/NACK w/ seq_no: %d\n", recv_buf.control.seq_no);

            if(recv_buf.control.seq_no == *send_seq_no % 8) {
                msg=dispacher_hdlc_mail_box->alloc();
                msg->sender_pid=osThreadGetId();
                msg->type = HDLC_RESP_SND_SUCC;
                msg->content.value = (uint32_t) 0;
                dispacher_hdlc_mail_box->put(msg); 

                pc.printf("hdlc: sender_pid is %d\n", sender_pid);
                // msg_send(&msg, hdlc_dispatcher_pid);
                (*send_seq_no)++;
                uart_lock = 0;
            }
                                
            recv_buf.control.frame = (yahdlc_frame_t)0;
            recv_buf.control.seq_no = 0;
        } 

    }
}

static void hdlc(void const *arg)
{
    // uart_t dev = (uart_t)arg;
    // msg_init_queue(_hdlc_msg_queue, 16);
    //     
    pc.printf("hdlc: Inside hdlc Thread\n");

    uint32_t last_sent = 0;
    msg_t *msg, *msg_org, *reply, *msg2;
    unsigned int recv_seq_no = 0;
    unsigned int send_seq_no = 0;
    osEvent evt;
    while(1) {

        pc.printf("hdlc: inside HDLC loop\n");
        Thread::wait(200);
        led2=!led2;
        if(uart_lock) {
            int timeout = (int) (last_sent + RETRANSMIT_TIMEO_USEC) - (int) global_time.read_us();
            if(timeout < 0) {
                pc.printf("hdlc: inside timeout negative\n");
                /* send message to self to resend msg */
                msg2=hdlc_mail_box->alloc();
                msg2->sender_pid=osThreadGetId();
                msg2->type = HDLC_MSG_RESEND;
                hdlc_mail_box->put(msg2); 
                evt = hdlc_mail_box->get();
                // msg_send_to_self(&msg2);
            } else {
                pc.printf("hdlc: inside timeout positive\n");

                Thread::wait_us((int32_t)timeout);
                continue;
                // timeout
                // if(0 > xtimer_msg_receive_timeout(&msg, timeout)) {
                    // continue;
                // }
            }
        } else {
            pc.printf("hdlc: waiting for mail\n");

            evt = hdlc_mail_box->get();
        }
        
        
        
        
        if (evt.status == osEventMail) 
        {          
            msg_org = (msg_t*)evt.value.p;
            msg=new msg_t(); 
            memcpy(msg, msg_org, sizeof(msg_t));
            hdlc_mail_box->free(msg_org);

            switch (msg->type) {
                case HDLC_MSG_RECV:
                    pc.printf("hdlc: receiving msg...\n");
                    _hdlc_receive(&recv_seq_no, &send_seq_no);
                    break;
                case HDLC_MSG_SND:
                    pc.printf("hdlc: request to send received from pid %d\n", msg->sender_pid);
                    // uart_lock=1;
                    if (uart_lock) {
                        /* ask thread to try again in x usec */
                        pc.printf("hdlc: uart locked, telling thr to retry\n");
                        reply=((Mail<msg_t, 16>*)msg->source_mailbox)->alloc();
                        reply->type = HDLC_RESP_RETRY_W_TIMEO;
                        reply->content.value = (uint32_t) RTRY_TIMEO_USEC;
                        reply->sender_pid=osThreadGetId();
                        ((Mail<msg_t, HDLC_MAILBOX_SIZE>*)msg->source_mailbox)->put(reply);
                        //msg_send(&reply, msg->sender_pid); need to figure this out
                    }else {
                        uart_lock = 1;
                        sender_pid = msg->sender_pid;
                        pc.printf("hdlc: sender_pid set to %d\n", sender_pid);
                        send_buf.control.frame = YAHDLC_FRAME_DATA;
                        send_buf.control.seq_no = send_seq_no % 8; 
                        
                        // equivalent to hdlc_pkt_t *pkt = msg.content.ptr;
                        hdlc_pkt_t *pkt = new hdlc_pkt_t();
                        hdlc_buf_t *pkt_buf=(hdlc_buf_t*)msg->content.ptr;
                        pkt->data= pkt_buf->data;
                        pkt->length= pkt_buf->length;

                        yahdlc_frame_data(&(send_buf.control), pkt->data, 
                                pkt->length, send_buf.data, &send_buf.length);
                        write_hdlc((uint8_t *)send_buf.data, send_buf.length);
                        // hdlc_pc->write((uint8_t *)send_buf.data, send_buf.length,NULL,NULL);

                        // uart_write(dev, (uint8_t *)send_buf.data, send_buf.length);
                        last_sent = global_time.read_us();
                    }   
                    break;
                case HDLC_MSG_SND_ACK:
                    /* send ACK */
                    ack_buf.control.frame = YAHDLC_FRAME_ACK;
                    ack_buf.control.seq_no = msg->content.value;
                    yahdlc_frame_data(&(ack_buf.control), NULL, 0, ack_buf.data, &(ack_buf.length));    
                    pc.printf("hdlc: sending ack w/ seq no %d\n", ack_buf.control.seq_no);
                    write_hdlc((uint8_t *)ack_buf.data, ack_buf.length);
                    // hdlc_pc->write((uint8_t *)ack_buf.data, ack_buf.length,0,0);   
                    break;
                case HDLC_MSG_RESEND:
                    pc.printf("hdlc: Resending frame w/ seq no %d (on send_seq_no %d)\n", send_buf.control.seq_no, send_seq_no);
                    write_hdlc((uint8_t *)send_buf.data, send_buf.length);
                    // hdlc_pc->write((uint8_t *)send_buf.data, send_buf.length,0,0);
                    last_sent = global_time.read_us();
                    break;
                case HDLC_MSG_REG_DISPATCHER:
                    pc.printf("hdlc: Registering dispatcher thread.\n");
                    hdlc_dispatcher_pid = msg->sender_pid;
                    dispacher_hdlc_mail_box=(Mail<msg_t, 16>*)msg->source_mailbox;
                    pc.printf("hdlc: hdlc_dispatcher_pid set to %d\n", hdlc_dispatcher_pid);
                    break;
                default:
                    pc.printf("INVALID HDLC MSG\n");
                    //LED3_ON;
                    break;
            }
        }
    }

    // /* this should never be reached */
    // return;
}

int hdlc_pkt_release(hdlc_buf_t *buf) 
{
    // if(buf->mtx.queue.next != NULL) { //how to do this??
    //     buf->control.frame = buf->control.seq_no = 0;
    //     mutex_unlock(&buf->mtx);
    //     return 0;
    // }
    
    buf->control.frame = (yahdlc_frame_t)0;
    buf->control.seq_no = 0;
    buf->mtx.unlock();
    return 0;
    
    pc.printf("hdlc: Packet not locked. Might be empty!\n");
    return -1;
}

int hdlc_init(int stacksize, osPriority priority, const char *name, int dev, void **mail)
// int hdlc_init(char *stack, int stacksize, osPriority priority, const char *name, int dev)
{
    led2=1;
    osThreadId res;
    recv_buf.data = hdlc_recv_data;
    send_buf.data = hdlc_send_frame;
    ack_buf.data = hdlc_ack_frame;
    global_time.start();
    /* check if uart device number is valid */
    // if(dev > UART_NUM - 1) {
    //     return -ENODEV;
    // }
    // 
    hdlc_mail_box= new Mail<msg_t, HDLC_MAILBOX_SIZE>;
    *mail=hdlc_mail_box;

    ctx= new CircularBuffer<char, UART_BUFSIZE>();
    hdlc_pc= new Serial(p28,p27,115200); // need to make the pin tx/rx name into a function call
    void (*fpointer)(void) = &rx_cb;
    hdlc_pc->attach(fpointer,Serial::RxIrq);
    
    hdlcpointer = new Thread(hdlc);
    hdlcpointer->set_priority(priority);
    
    res=hdlcpointer->gettid();
    pc.printf("hdlc: thread  id %d\n",res);

    // if (res <= 0) {
    //     return -EINVAL;
    // }

    hdlc_thread_pid = res;
    
    pc.printf("hdlc: hdlc thread creation done!!\n");

    return (int)res;
}
void write_hdlc(uint8_t *ptr,int len)
{
    pc.printf("hdlc: Inside write_hdlc\n");
    for(uint8_t i=1;i<len;i++)
        hdlc_pc->putc(*(ptr+i));   

}
