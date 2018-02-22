#include "mbed.h"
#include "m3pi.h"
#include <stdint.h>

m3pi m3pi(p23, p9, p10);

/* the only instance of pc -- debug statements in other files depend on it */
// Serial pi_base(p9, p10, 115200);
Serial pc(USBTX, USBRX, 115200);

DigitalOut led1(LED3);

typedef struct
{
    int dState;
    int iState;
    int iMax, iMin;
    int pGainNum, pGainDen; //proportional gain numerator and denom (for integer controller)
    int iGainNum, iGainDen; //integral gain
    int dGainNum, dGainDen; //derivative gain
} PID_t;

int UpdatePID(PID_t *pid, int error, int position) 
{
    // Do nothing if the denominator of any constant is zero.
    if(pid->pGainDen == 0 || pid->iGainDen == 0 || pid->dGainDen == 0)
    {
        return 0;
    }   

    int pTerm, dTerm, iTerm;

    /* calculate the proportional term */   
    pTerm = (error * pid->pGainNum) / pid->pGainDen; 

    /* calculate the integral state with appropriate limiting */
    pid->iState += error;

    if (pid->iState > pid->iMax) 
        pid->iState = pid->iMax;
    else if (pid->iState < pid->iMin) 
        pid->iState = pid->iMin;

    iTerm = pid->iState * pid->iGainNum / pid->iGainDen; // calculate the integral term
    pc.printf("iTerm = %d\n", iTerm);

    dTerm = (pid->dState - position) * pid->dGainNum / pid->dGainDen ;
    pid->dState = position;

    return pTerm + dTerm + iTerm;
}

void DriveStraightDistance(PID_t *pid, char speed, int16_t dist_in_encoder_ticks)
{
    char max_change = speed + 10;
    m3pi.forward(speed);
    char right_speed = speed;

    while(1) 
    {
        int16_t m1Count = m3pi.m1_encoder_count();
        int16_t m2Count = m3pi.m2_encoder_count();

        pc.printf("left: %d, right: %d\n", m1Count, m2Count);

        if (m1Count > dist_in_encoder_ticks)
            break;


        int speed_diff = UpdatePID(pid, m1Count - m2Count, 0);
        pc.printf("speed diff: %d\n", speed_diff);

        right_speed += speed_diff;
        if(right_speed > max_change)
            right_speed = max_change;
        if(right_speed < 0)
            right_speed = 0;

        m3pi.right_motor(right_speed);

        pc.printf("right_speed: %d\n", right_speed);

        wait(0.1);
    }
}

int main() {
    pc.printf("Example of a PI-controller (full PID is todo)\n");

    int dist_in_encoder_ticks = 4 * 4478; //1 meter

    m3pi.forward(20,21);

    wait(20);

    PID_t DriveStraightPID;
    DriveStraightPID.iMax = 15;
    DriveStraightPID.iMin = -15;
    DriveStraightPID.pGainNum = 1;
    DriveStraightPID.pGainDen = 30;
    DriveStraightPID.iGainNum = 1;
    DriveStraightPID.iGainDen = 30;
    DriveStraightPID.dGainNum = 0;
    DriveStraightPID.dGainDen = 1;

    DriveStraightDistance(&DriveStraightPID, 50, dist_in_encoder_ticks);

    m3pi.stop();

    return 0;
}