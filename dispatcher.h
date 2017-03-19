#ifndef DISPATCHER_H_
#define DISPATCHER_H_
#include "yahdlc.h"
#include "rtos.h"
#include "mbed.h"
#include "hdlc.h"


// static bool dispacher_ready; // To guarantee that the dispacher is setup properly before you send message.
Mail<msg_t, HDLC_MAILBOX_SIZE> *dispacher_init();
Mail<msg_t, HDLC_MAILBOX_SIZE> *get_dispacther_mailbox();
int register_thread(Mail<msg_t, HDLC_MAILBOX_SIZE> *arg);

#endif /* DISPATCHER_H_ */