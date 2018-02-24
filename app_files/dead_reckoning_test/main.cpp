#include "mbed.h"
#include "m3pi.h"
#include <stdint.h>
m3pi m3pi(p23, p9, p10);

/* the only instance of pc -- debug statements in other files depend on it */
// Serial pi_base(p9, p10, 115200);
Serial pc(USBTX, USBRX, 115200);

DigitalOut led1(LED3);

int main() {
    pc.printf("hello\n");
    led1 = !led1;
    m3pi.backward(20);
    wait(2.0);

    while(1)
    {

    }

    pc.printf("Pololu Magnetic Encoder Test\n");

    /**Note: a speed of 30 and under will most likely be the most accurate. A
     * speed of 50 may be accurate enough.
     */
   
    /* move in a 0.5m x 0.5m square */
    while(1)
    {
        /**
         * The first input is speed using the actual speed integers on the 3pi side.
         * The origina m3pi.h functions multiply this by 2.
         * 
         * 2 * 4476 is equal to 1 meter. The multiply by 2 is a fix that doesn't
         * make sense yet. To get 4476, you take 1000mm, which equals 1 meter, and divide
         * it by 0.223402mm, which should be how much the robot travels after one
         * encoder tick. 
         */
        m3pi.move_straight_distance_blocking(35, 4476);
        wait(0.5);
        /**First input is the angle in degrees. second input is direction (1 for 
         * positive and -1 for negative using the right hand rule). Third input is 
         * speed. Note, this speed is the actual speed on the 3pi side. The normal
         * functions multiply this by 2.
         */
        m3pi.rotate_degrees_blocking(90, 1, 35);
        wait(0.5);
        m3pi.move_straight_distance_blocking(35, 4476);
        wait(0.5);
        m3pi.rotate_degrees_blocking(90, 1, 35);
        wait(0.5);
        m3pi.move_straight_distance_blocking(35, 4476);
        wait(0.5);
        m3pi.rotate_degrees_blocking(90, 1, 35);
        wait(0.5);
        m3pi.move_straight_distance_blocking(35, 4476);
        wait(0.5);
        m3pi.rotate_degrees_blocking(90, 1, 35);
    } 

    return 0;
}