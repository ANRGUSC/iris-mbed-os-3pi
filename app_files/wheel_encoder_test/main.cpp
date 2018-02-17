#include "mbed.h"
#include "m3pi.h"
#include <stdint.h>

m3pi m3pi;

int main() {

    m3pi.printf("Pololu Magnetic Encoder Test\n");
    for(int i = 0; i < 5; i++) {
        wait (1);
        int16_t m1_count = m3pi.m1_encoder_count();
        int16_t m2_count = m3pi.m2_encoder_count();
        m3pi.printf("left: %d, right: %d\n", m1_count, m2_count);
    }

    m3pi.stop();       
}