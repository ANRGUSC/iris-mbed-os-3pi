#include <stdlib.h>  
#include <math.h>
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

MemoryPool<cmessage_t, 16> c_pool;
Queue<cmessage_t, 16> c_queue;
m3pi m3pi(p23, p9, p10);
 
// static unsigned char CONT_THREAD_STACK[DEFAULT_STACK_SIZE];
Thread c_thread;
// (osPriorityNormal, 
    // (uint32_t) DEFAULT_STACK_SIZE/2, (unsigned char *)CONT_THREAD_STACK); 

char speed = ROBOT_MAX_SPEED;
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
        int type = est->type;
        uint16_t dist_e = est->distance;
        uint16_t angle_e = est->angle;

        c_pool.free(est);
        PRINTF("Type = %d, Distance = %d, Angle = %d\n", type, dist_e, angle_e);
        
        uint16_t dist_to_travel, angle_to_rotate;

        if (type == NORMAL_MOV){
            PRINTF("Received a Normal Movement Request\n");

            // TODO? Controller decides how much to travel and how much to rotate

        }
        else if (type == SCANNING_MOV){
            PRINTF("Received a Scanning Movement Request\n");
            dist_to_travel = dist_e;
            angle_to_rotate = angle_e;
        }

        // TODO? Use slave m3pi functions for actual movements
        // m3pi.rotate_degrees_blocking(angle_to_rotate, 1, speed);
        // m3pi.move_straight_distance_blocking(speed, dist_to_travel);

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
int start_movement (int type, uint16_t dist_e, uint16_t angle_e)
{
    cmessage_t *message = c_pool.alloc();
    if (message != NULL)
    {
        message->type = type;
        message->distance = dist_e; 
        message->angle = angle_e;          
        c_queue.put(message);
        return(0);
    }
    return(-1);
}

