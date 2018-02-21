#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

typedef struct {
    float distance;   /* AD result of measured voltage */
    float angle;
} cmessage_t;

void controller_init(osPriority priority);
int start_movement (float dist_e, float angle_e);


#endif