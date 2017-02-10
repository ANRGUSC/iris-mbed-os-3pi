// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details

#include "m3pi.h"
#include "mbed.h"
#include "rtos.h"
#include <string.h>
#include <stdlib.h>  
#include <math.h>
#define PESSIMISTIC 1
#define BASELINE 1
#define MOVEMENT 1
#define INTERVAL 10

#define LINEAR 1
typedef struct {
    float observed_data[3][1];   /* AD result of measured voltage */
} message_t;
MemoryPool<message_t, 16> mpool;
Queue<message_t, 16> queue;
// message_t *message;

Mutex data_mutex; 
Mutex control_mutex; 
static bool start_flag=1;
Thread        *RX_THREAD_POINTER;
Thread        *CONT_THREAD_POINTER;
Thread        *LQG_THREAD_POINTER;
Thread        *RECV_THREAD_POINTER;

m3pi m3pi;


DigitalOut myled(LED1); //to notify when a character was received on mbed
DigitalOut myled2(LED2); //to notify when a character was received on mbed
DigitalOut myled3(LED3); //to notify when a character was received on mbed
DigitalOut myled4(LED4); //to notify when a character was received on mbed

Serial xbee(p28, p27); //used to connected the mbed to the xbee over Serial UART comm.
Serial pc(USBTX, USBRX); 
int count=1;
bool antenna_rotation_dir_flag=0,safeflag=0;
bool flag_cont=0;
char current[32]; //holds the value of the character that was read
float cur_state[3][1]={{3.0},{0},{0}};
float next_state[3][1]={{0},{0},{0}};
float observed_data[3][1]={{0},{0},{0}};
// float data[MAX_NUM_SAMPLES];


float speed = 0.2;
float correction = 0.1;   
float threshold = 0.5;

/* This Function Handles the interrupt from the Xbee to
 get the rssi values. This callback function executes 
 every time a Serial intterupt is generated to get the
 command sent*/
void getdata()
{ 
        LPC_UART2->IER = 0;   // Disable Rx interrupt
        (*RX_THREAD_POINTER).signal_set(0x1); // Send signl to the reading thread 
                                            // rx_thread 
}

int delta_t=200;

/* This is the code for Xbee receiver data collection
 thread to get the rssi values.*/
void rx_thread(void const *argument){
    xbee.baud(115200);

    while (true) 
    {
        myled2=!myled2;
        // Signal flags that are reported as event are automatically cleared.
        Thread::signal_wait(0x1);
        if(xbee.readable()) //Check If there is somethig to read in the serial
        {
            xbee.gets(current,32);
            pc.printf("%s",current);
        }  
        if(current[0]=='w')
        {
            // pc.printf("Moving Forward\n");
            m3pi.forward(speed);
            Thread::wait(delta_t);
            m3pi.stop();
        }    
        else if (current[0]=='a')
        {
            // pc.printf("Moving Left\n");
            m3pi.left(speed);
            Thread::wait(delta_t);
            m3pi.stop();
            // pc.printf("%s",current);
        }   
        else if (current[0]=='s')
        {
            // pc.printf("Moving Backward\n");
            m3pi.backward(speed);
            Thread::wait(delta_t);
            m3pi.stop();
        }
        else if (current[0]=='d')
        {
         //   pc.printf("Moving Right\n");
            m3pi.right(speed);
            Thread::wait(delta_t);
            m3pi.stop();
        }
        else
        {
        //    pc.printf("No Movement\n");
        }
        /* Re-Enable the Receiver Interrupt */
        LPC_UART2->IER = 1;    
    }
}
 void eval_command(){ 
        LPC_UART0->IER = 0;
        // Disable Rx interrupt
        (*CONT_THREAD_POINTER).signal_set(0x8); // dereference of RX_THREAD_POINTER
} 
void pc_control_thread(void const *argument){
    
    while (true) {
        // Signal flags that are reported as event are automatically cleared.
        Thread::signal_wait(0x8);
        if(pc.readable()) //Check If there is somethig to read in the serial
            pc.printf("%c,",pc.getc());
        // flag_cont =1;
        LPC_UART0->IER = 1;               // Enable Rx interrupt
    }
}
 

int main() {
    // Thread rotating_platform(antenna_thread);
    // RECV_THREAD_POINTER=&rotating_platform;
    Thread t_rx(rx_thread);
    RX_THREAD_POINTER = &t_rx; 
    Thread pc_data(pc_control_thread);
    CONT_THREAD_POINTER = &pc_data;
    m3pi.locate(0,1);
    m3pi.printf("Rmt Control");
    m3pi.sensor_auto_calibrate();

   // 
    void (*fpointer)(void) = &eval_command;
    void (*fpointer1)(void) = &getdata;
    pc.attach(fpointer,Serial::RxIrq); 
    //attach a function to be done when a command is sent from the computer keyboard
    pc.baud(115200);
    xbee.baud(115200);
    xbee.attach(fpointer1,Serial::RxIrq);
    while (true) {
        myled = !myled;
        Thread::wait(300);
    }
}