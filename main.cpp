#include "mbed.h"
#include "rtos.h"
#include "hdlc.h"


#include <stdlib.h>     /* srand, rand */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "yahdlc.h"
#include "fcs16.h"
DigitalOut myled(LED1);

// #ifndef UART_STDIO_DEV
// #define UART_STDIO_DEV          (UART_UNDEF)
// #endif

// #define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)

static msg_t _hdlc_dispatcher_msg_queue[16];
static osThreadId hdlc_pid;
Mail<msg_t, HDLC_MAILBOX_SIZE> dispacher_mailbox;

// static char hdlc_stack[THREAD_STACKSIZE_MAIN];

int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming packets */
    //msg_init_queue(_hdlc_dispatcher_msg_queue, 16);
    
    myled=1;
    void *mailbox_ptr;
    Mail<msg_t, HDLC_MAILBOX_SIZE> *hdlc_mail_box_ptr;
    pc.printf("In main");

    int hdlc_pidint=hdlc_init(NULL, (osPriority)NULL, "hdlc", 1,(void**)&hdlc_mail_box_ptr);//UART_DEV(1));
    // pc.printf("In main");

    hdlc_pid=(osThreadId)hdlc_pidint;
    // Thread::wait(1000);
    // printf("Available devices:               %i\n", UART_NUMOF);
    // printf("UART used for STDIO (the shell): UART_DEV(%i)\n\n", UART_STDIO_DEV);

    msg_t *msg_resp,*msg_req,*msg_req1,*msg_org;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t *pkt= new hdlc_pkt_t;
    pkt->data = send_data;
    pkt->length = 0;
    hdlc_buf_t *buf;
    pc.printf("In main");

    // random_init(xtimer_now());

    msg_req=hdlc_mail_box_ptr->alloc();
    msg_req->type = HDLC_MSG_REG_DISPATCHER;
    msg_req->content.value = (uint32_t) NULL;
    msg_req->sender_pid=osThreadGetId();
    msg_req->source_mailbox=&dispacher_mailbox;
    hdlc_mail_box_ptr->put(msg_req);
    // pc.printf("In main");

    pc.printf("dispatcher pid is %d \n", osThreadGetId());

    msg_req->type = 0;
    int exit = 0;
    osEvent evt;
    while(1)
    {
        myled=!myled;
        //Thread::wait(1000);
        pc.printf("inside main\n");
        pkt->data[0] = frame_no;

        for(int i = 1; i < HDLC_MAX_PKT_SIZE; i++) {
            pkt->data[i] = (char) ( rand() % 0x7E);
        }

        pkt->length = HDLC_MAX_PKT_SIZE;


        /* send pkt =*/
        msg_req1=hdlc_mail_box_ptr->alloc();
        msg_req1->type = HDLC_MSG_SND;
        msg_req1->content.ptr = pkt;
        msg_req1->sender_pid=osThreadGetId();
        msg_req1->source_mailbox=&dispacher_mailbox;
        
        printf("dispatcher: sending pkt no %d packet %d\n", frame_no,((hdlc_pkt_t*)msg_req1->content.ptr)->length);

        hdlc_mail_box_ptr->put(msg_req1);

        while(1)
        {
            myled=!myled;
            evt = dispacher_mailbox.get();
            msg_org = (msg_t*)evt.value.p;
            msg_resp=new msg_t(); 
            memcpy(msg_resp, msg_org, sizeof(msg_t));
            dispacher_mailbox.free(msg_org);

            // msg_receive(&msg_resp);
            if (evt.status == osEventMail) 
            {
                switch (msg_resp->type)
                {
                    case HDLC_RESP_SND_SUCC:
                        printf("dispatcher: sent frame_no %d!\n", frame_no);
                        exit = 1;
                        free(msg_resp);
                        break;
                    case HDLC_RESP_RETRY_W_TIMEO:
                        Thread::wait(msg_resp->content.value/1000);
                        pc.printf("dispatcher: retry frame_no %d \n", frame_no);
                        // exit = 1;
                        msg_req1=hdlc_mail_box_ptr->alloc();
                        msg_req1->type = HDLC_MSG_SND;
                        msg_req1->content.ptr = pkt;
                        msg_req1->sender_pid=osThreadGetId();
                        msg_req1->source_mailbox=&dispacher_mailbox;

                        hdlc_mail_box_ptr->put(msg_req1);
                        free(msg_resp);
                        break;
                    case HDLC_PKT_RDY:
                        buf = (hdlc_buf_t *)msg_resp->content.ptr;   
                        memcpy(recv_data, buf->data, buf->length);
                        printf("dispatcher: received pkt %d\n", recv_data[0]);
                        free(msg_resp);
                        // hdlc_pkt_release(buf);
                        // exit = 1;

                        break;
                    default:
                        free(msg_resp);
                        /* error */
                        //LED3_ON;
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

    /* should be never reached */
    return 0;
}
