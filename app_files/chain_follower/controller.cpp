#include <stdlib.h>  
#include <math.h>
#include <mbed.h>
#include <rtos.h> 
#include "controller.h"

#define DEBUG   1

#if (DEBUG) 
    #define PRINTF(...) pc.printf(__VA_ARGS__)
    extern Serial pc;
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

MemoryPool<cmessage_t, 16> c_pool;
Queue<cmessage_t, 16> c_queue;

static unsigned char CONT_THREAD_STACK[DEFAULT_STACK_SIZE];
Thread c_thread(osPriorityNormal, 
    (uint32_t) DEFAULT_STACK_SIZE, (unsigned char *)CONT_THREAD_STACK); 

/**
 * @brief      controller thread
 */
void _c_thread()
{
    while (true)
    {
        osEvent evt = c_queue.get();
        while( evt.status != osEventMessage )
        {
            Thread::wait(100);
            evt = c_queue.get();
        }
 
        cmessage_t *est = (cmessage_t*)evt.value.p;
        float dist_e = est->distance;
        float angle_e = est->angle;

        c_pool.free(est);
        PRINTF("Distance = %f, Angle = %f\n", dist_e, angle_e);

    }
}

void controller_init(osPriority priority) 
{
    c_thread.set_priority(priority);
    c_thread.start(_c_thread);
    // return &CONT_THREAD_STACK;
}


/**
 * @brief      Starts a movement.
 *
 * @param[in]  dist_e   The distance estimate
 * @param[in]  angle_e  The angle estimate
 *
 * @return     { status }
 */
int start_movement (float dist_e, float angle_e){
    cmessage_t *message = c_pool.alloc();
    if (message != NULL)
    {
        message->distance = dist_e; 
        message->angle = angle_e;          
        c_queue.put(message);
        return(0);
    }
    return(-1);
}

