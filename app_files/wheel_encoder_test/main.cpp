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
    wait(1.0);

    pc.printf("Pololu Magnetic Encoder Test\n");

    m3pi.rotate_degrees(90, 1, 50);
    wait(3);
    m3pi.rotate_degrees(90, -1, 50);
    wait(3);
    
    /* 2 * 4476 is equal to 1 meter. The multiply by 2 is a fix that doesn't 
    make sense yet. To get 4476, you take 1000mm, which equals 1 meter, and divide
    it by 0.223402mm, which should be how much the robot travels after one
    encoder tick. */
    m3pi.move_straight_distance(50, 2 * 4476);

    while(1) {
        int16_t m1_count = m3pi.m1_encoder_count();
        int16_t m2_count = m3pi.m2_encoder_count();
        // char m1_error = m3pi.m1_encoder_error();
        // char m2_error = m3pi.m2_encoder_error();
        // char m1_error = 0;
        // char m2_error = 0;
        pc.printf("left: %d, right: %d \t\n", m1_count, m2_count);
        // pc.printf("left-err: %d, right-err: %d\n", m1_error, m2_error);
        led1 = !led1;
        wait(1.0);
    }

    return 0;
}