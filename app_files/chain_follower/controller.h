#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

enum {
    SCANNING_MOV,
    NORMAL_MOV
};

typedef struct {
    int type; // Whether actual movement or just scanning movement
    uint16_t distance;   /* AD result of measured voltage */
    uint16_t angle;
} cmessage_t;

void controller_init(osPriority priority);
int start_movement (int type, uint16_t dist_e, uint16_t angle_e);


#endif