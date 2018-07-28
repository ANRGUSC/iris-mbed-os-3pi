#include "mbed.h"
#include "rtos.h"
#include "hdlc.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "yahdlc.h"
#include "fcs16.h"
#include "uart_pkt.h"
#include "main-conf.h"
#include "mqtt.h"
#include "app-conf.h"
#include "math.h"
#include "m3pi.h"

#include "movement_thread.h"


extern Mail<msg_t, HDLC_MAILBOX_SIZE>  main_thr_mailbox;

#define DEBUG   1
#define TEST_TOPIC   ("init_info")

#if (DEBUG) 
    #define PRINTF(...) pc.printf(__VA_ARGS__)
    extern Serial pc;
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

m3pi m3pi(p23, p9, p10);

static unsigned char MOVEMENT_STACK[DEFAULT_STACK_SIZE];

Mail<msg_t, HDLC_MAILBOX_SIZE>  movement_thread_mailbox;
Thread movement(osPriorityNormal, (uint32_t) DEFAULT_STACK_SIZE, (unsigned char *)MOVEMENT_STACK); 


volatile int movement_state = NO_INFO;

double toDegrees(double radians) {
    return radians * (180.0/ M_PI);
}

float* getCoordinatesInFloats(char *value_ptr) {
    char *xChar, *yChar;
    float lastX, lastY = -1;
    xChar = strtok((char *)value_ptr, ":");
    yChar = strtok(NULL, ":");
    lastX = atof(xChar);
    lastY = atof((const char*)yChar);
    float* ret = (float*)malloc(sizeof(float)*2);
    ret[0] = lastX;
    ret[1] = lastY;
    return ret;
}

double ticksToCartesian(double ticks) {
    return (ticks * (0.223402))/1000;
}

double cartesianToTicks(double cartesian) {
    return (cartesian * 1000)/(0.223402);
}

double getCartesianDistance(float* currentCoordinates, float* targetCoordinates) {
    return sqrt(pow(targetCoordinates[0] - currentCoordinates[0], 2) + pow(targetCoordinates[1] - currentCoordinates[1], 2));
}

double getOrientation(float* currentCoordinates, float* targetCoordinates, double orientationState) {
    PRINTF("toDegrees(atan2()) = %f\n", toDegrees(atan2(targetCoordinates[1] - currentCoordinates[1], targetCoordinates[0] - currentCoordinates[0])));
    PRINTF("orientationState = %f\n", orientationState);
    return ((-1)*(orientationState)) + toDegrees(atan2(targetCoordinates[1] - currentCoordinates[1], targetCoordinates[0] - currentCoordinates[0]));
}

double findPartOfCoordinate(double distanceMovedInCartesian, double distanceToTarget, double currentZ, double targetZ) {
    PRINTF("Movement: distanceMovedInCartesian = %f, distanceToTarget = %f, currentZ = %f, targetZ = %f\n", distanceMovedInCartesian, distanceToTarget, currentZ, targetZ);
    return ((1 - (distanceMovedInCartesian/distanceToTarget)) * currentZ) + ((distanceMovedInCartesian/distanceToTarget) * targetZ);
}

// Rotate degrees, fixing the 127 limit and handles negative degrees
void safelyRotateDegrees(double degreesToRotate, double* orientationState) {
    char direction = 1.0;
    char rotationSpeed = 30;
    *orientationState += degreesToRotate;
    if(degreesToRotate < 0) {
        direction = -1;
        degreesToRotate = (degreesToRotate * (-1));
    }

    double total = degreesToRotate; 
    while(total > 127) {
        PRINTF("Rotating: 127\n");
        m3pi.rotate_degrees((unsigned char)127, direction, rotationSpeed);
        total -= 127;
    }
    PRINTF("Rotating: %f\n", total);
    m3pi.rotate_degrees((unsigned char)total, direction, rotationSpeed);
    PRINTF("Movement: Rotating %f degrees with direction %f. Updating orientation to %f.\n", degreesToRotate, direction, orientationState);
}





/**
 * @brief      This is the Movement thread on MBED
 */
void _movement_thread()
{
    
    PRINTF("Movement: Welcome to the movementh thread.\n");
    msg_t *msg;
    osEvent evt;
    char *value_ptr;
    float* currentCoordinates = NULL;
    float* targetCoordinates = NULL;
    double orientationState = 0;
    char speedToMove = 50;
    double controlCoef = 0.5;
    

    while (1) 
    {
        PRINTF("Movement: In the movement loop.\n");

        while(1)
        {
            evt = movement_thread_mailbox.get(100);
            if (evt.status == osEventMail) 
            {
                msg = (msg_t*)evt.value.p;
                switch (msg->type)
                {
                    case INTER_THREAD:
                        PRINTF("INTER_THREAD\n"); 
                        break;
                    case CURRENT_LOCATION:
                        value_ptr = (char *)msg->content.ptr; 
                        value_ptr++;
                        
                        //Receive current location
                        currentCoordinates = getCoordinatesInFloats(value_ptr);
                        PRINTF("Movement MQTT Update: Current Coordinates: (%f, %f)\n", currentCoordinates[0], currentCoordinates[1]);
                        break;
                    case TARGET_LOCATION:
                        value_ptr = (char *)msg->content.ptr; 
                        value_ptr++;
                        
                        //Receive target location
                        targetCoordinates = getCoordinatesInFloats(value_ptr);
                        PRINTF("Movement MQTT Update: Target Coordinates: (%f, %f)\n", targetCoordinates[0], targetCoordinates[1]);
                        break;
                    default: 
                        PRINTF("Break on default\n"); 
                        break;
                }
            }    
            
            if(targetCoordinates != NULL && currentCoordinates != NULL) {
                PRINTF("Movement: Current coordinates: (%f, %f) --> (%f, %f) with a previous orientation of: %f.\n", currentCoordinates[0], currentCoordinates[1], targetCoordinates[0], targetCoordinates[1], orientationState);
                
                //Calculate the distance
                double distanceCurrentToTargetInCartesian = getCartesianDistance(currentCoordinates, targetCoordinates);
                double distanceCurrentToTargetInTicks = cartesianToTicks(distanceCurrentToTargetInCartesian);
                PRINTF("Movement: Distance to the target in cartesian = %f, Distance to the target in ticks = %f.\n", distanceCurrentToTargetInCartesian, distanceCurrentToTargetInTicks);

                //Calculate orientation and rotate if needed
                double degreesToRotate = getOrientation(currentCoordinates, targetCoordinates, orientationState);
                PRINTF("Movement: Degrees to rotate = %f.\n", degreesToRotate);
                safelyRotateDegrees(degreesToRotate, &orientationState);

                //Move 
                double controlledTicksWeMoved = controlCoef*distanceCurrentToTargetInTicks;
                m3pi.move_straight_distance(speedToMove, controlledTicksWeMoved);
                PRINTF("Movement: Ticks we moved = %f.\n", controlledTicksWeMoved);
                
                //Estimate distance moved
                double distanceLeftToGoInTicks = distanceCurrentToTargetInTicks - controlledTicksWeMoved;
                double distanceLeftToGoInCartesian = ticksToCartesian(distanceLeftToGoInTicks);
                double distanceMovedInCartesian = distanceCurrentToTargetInCartesian - distanceLeftToGoInCartesian;
                PRINTF("Movement: Distance moved = %f\n", distanceMovedInCartesian);

                //Estimate our new position
                PRINTF("Movement: Now we need to dead reckon our new position from (%f, %f) to target (%f, %f) after having moved %d with %d left to go.\n", currentCoordinates[0], currentCoordinates[1], targetCoordinates[0], targetCoordinates[1], distanceMovedInCartesian, distanceLeftToGoInCartesian);
                double estimatedX = findPartOfCoordinate(distanceMovedInCartesian, distanceCurrentToTargetInCartesian, currentCoordinates[0], targetCoordinates[0]);
                double estimatedY = findPartOfCoordinate(distanceMovedInCartesian, distanceCurrentToTargetInCartesian, currentCoordinates[1], targetCoordinates[1]);
                PRINTF("MOVEMENT: Estimated new position = (%f, %f) with a distance to go of %f\n", estimatedX, estimatedY, distanceLeftToGoInCartesian);
                currentCoordinates[0] = estimatedX;
                currentCoordinates[1] = estimatedY;
                
                //See if we need to stop
                PRINTF("Movement: Estimated new current coordinates = (%f, %f)\n", currentCoordinates[0], currentCoordinates[1]);
                if(distanceLeftToGoInTicks <= 10) {
                    PRINTF("Movement: We are less than ten ticks away, breaking the loop.\n");
                    currentCoordinates[0] = targetCoordinates[0];
                    currentCoordinates[1] = targetCoordinates[1];
                    targetCoordinates = NULL;
                }
                PRINTF("Movement: ------------------------------------------\n");
            }
        }

        Thread::wait(100);
    }
}


Mail<msg_t, HDLC_MAILBOX_SIZE> *movement_init(osPriority priority) 
{
    movement.set_priority(priority);
    movement.start(_movement_thread);
    return &movement_thread_mailbox;
}


Mail<msg_t, HDLC_MAILBOX_SIZE> *get_movement_mailbox()
{
    return &movement_thread_mailbox;
}


Mutex data_mtx_mov;

int get_movement_state (void){
    data_mtx_mov.lock();
    int state = movement_state;
    data_mtx_mov.unlock();
    return state;
}

void set_movement_state (int state){
    data_mtx_mov.lock();
    movement_state = state;
    data_mtx_mov.unlock();
}