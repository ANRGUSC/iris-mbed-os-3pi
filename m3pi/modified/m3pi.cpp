/* m3pi Library
 *
 * Copyright (c) 2007-2010 cstyles
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "mbed.h"
#include "m3pi.h"
#include <stdio.h>
#include <stdint.h>

#define DEBUG   0

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */
extern Serial pc;

m3pi::m3pi(PinName nrst, PinName tx, PinName rx) :  Stream("m3pi"), _nrst(nrst), _ser(tx, rx)  {
    _ser.baud(115200);
    reset();
}

m3pi::m3pi() :  Stream("m3pi"), _nrst(p23), _ser(p13, p14)  {
    _ser.baud(115200);
    reset();
}

void m3pi::reset () {
    _nrst = 0;
    wait (0.01);
    _nrst = 1;
    wait (0.1);
}

void m3pi::left_motor (char speed) {
    motor(0,speed);
}

void m3pi::right_motor (char speed) {
    motor(1,speed);
}

void m3pi::forward (char speed) {
    motor(1,speed);
    motor(0,speed);
}

void m3pi::forward (char speed, char scaling) {
    motor(1,speed+scaling);
    motor(0,speed);
}

void m3pi::backward (char speed) {
    motor(0,-1*speed);
    motor(1,-1*speed);
}

void m3pi::left (char speed) {
    motor(0,speed);
    motor(1,-1*speed);
}

void m3pi::right (char speed) {
    motor(0,-1*speed);
    motor(1,speed);
}

void m3pi::stop (void) {
    motor(0,0);
    motor(1,0);
}

void m3pi::motor (int motor, signed char speed) {
    char opcode = 0x0;
    if (speed > 0) {
        if (motor==1)
            opcode = M1_FORWARD;
        else
            opcode = M2_FORWARD;
    } else {
        if (motor==1)
            opcode = M1_BACKWARD;
        else
            opcode = M2_BACKWARD;
    }
    unsigned char arg = abs(speed);
    PRINTF("%d\n", speed);
    _ser.putc(opcode);
    _ser.putc(arg);
}

float m3pi::battery() {
    _ser.putc(SEND_BATTERY_MILLIVOLTS);
    char lowbyte = _ser.getc();
    char hibyte  = _ser.getc();
    float v = ((lowbyte + (hibyte << 8))/1000.0);
    return(v);
}

float m3pi::line_position() {
    int pos = 0;
    _ser.putc(SEND_LINE_POSITION);
    pos = _ser.getc();
    pos += _ser.getc() << 8;
    
    float fpos = ((float)pos - 2048.0)/2048.0;
    return(fpos);
}

char m3pi::sensor_auto_calibrate() {
    _ser.putc(AUTO_CALIBRATE);
    return(_ser.getc());
}


void m3pi::calibrate(void) {
    _ser.putc(PI_CALIBRATE);
}

void m3pi::reset_calibration() {
    _ser.putc(LINE_SENSORS_RESET_CALIBRATION);
}

void m3pi::PID_start(int max_speed, int a, int b, int c, int d) {
    _ser.putc(max_speed);
    _ser.putc(a);
    _ser.putc(b);
    _ser.putc(c);
    _ser.putc(d);
}

void m3pi::PID_stop() {
    _ser.putc(STOP_PID);
}

float m3pi::pot_voltage(void) {
    int volt = 0;
    _ser.putc(SEND_TRIMPOT);
    volt = _ser.getc();
    volt += _ser.getc() << 8;
    return(volt);
}


void m3pi::leds(int val) {

    BusOut _leds(p20,p19,p18,p17,p16,p15,p14,p13);
    _leds = val;
}


int m3pi::print (char* text, int length) {
    _ser.putc(length);       
    for (int i = 0 ; i < length ; i++) {
        _ser.putc(text[i]); 
    }
    return(0);
}

int m3pi::_putc (int c) {
    _ser.putc(0x1);       
    _ser.putc(c);         
    wait (0.001);
    return(c);
}

int m3pi::_getc (void) {
    char r = 0;
    return(r);
}

int m3pi::putc (int c) {
    return(_ser.putc(c));
}

int m3pi::getc (void) {
    return(_ser.getc());
}

int16_t m3pi::m1_encoder_count() {
    _ser.putc(SEND_M1_ENCODER_COUNT);
    char lowbyte = _ser.getc();
    char hibyte  = _ser.getc();
    int16_t left_cnt = lowbyte + (hibyte << 8);
    return(left_cnt);
}

int16_t m3pi::m2_encoder_count() {
    _ser.putc(SEND_M2_ENCODER_COUNT);
    char lowbyte = _ser.getc();
    char hibyte  = _ser.getc();
    int16_t right_cnt = lowbyte + (hibyte << 8);
    return(right_cnt);
}

char m3pi::m1_encoder_error() {
    _ser.putc(SEND_M1_ENCODER_ERROR);
    return(_ser.getc());
}

char m3pi::m2_encoder_error() {
    _ser.putc(SEND_M2_ENCODER_ERROR);
    return(_ser.getc());
}

void m3pi::rotate_degrees(unsigned char degrees, char direction, char speed) {
    _ser.putc(ROTATE_DEGREES);
    _ser.putc(degrees);
    _ser.putc(direction); 
    _ser.putc(speed);
}

void m3pi::rotate_degrees_blocking(unsigned char degrees, char direction, char speed) {
    PRINTF("Rotate degrees blocking:");
    _ser.putc(ROTATE_DEGREES_BLOCKING);
    _ser.putc(degrees);
    _ser.putc(direction); 
    _ser.putc(speed);
    PRINTF("encoder: %d\n", (int)_ser.getc());
}


void m3pi::move_straight_distance(char speed, uint16_t distance) {
    _ser.putc(DRIVE_STRAIGHT_DISTANCE);
    _ser.putc(speed);
    _ser.putc((char)(distance & 0xFF));
    _ser.putc((char)(distance >> 8));
}

void m3pi::move_straight_distance_blocking(char speed, uint16_t distance) {
    PRINTF("Moving straight blocking:");
    _ser.putc(DRIVE_STRAIGHT_DISTANCE_BLOCKING);
    _ser.putc(speed);
    _ser.putc((char)(distance & 0xFF));
    _ser.putc((char)(distance >> 8));
    PRINTF("encoder: %d\n",_ser.getc());
}


#ifdef MBED_RPC
const rpc_method *m3pi::get_rpc_methods() {
    static const rpc_method rpc_methods[] = {{ "forward", rpc_method_caller<m3pi, float, &m3pi::forward> },
        { "backward", rpc_method_caller<m3pi, float, &m3pi::backward> },
        { "left", rpc_method_caller<m3pi, float, &m3pi::left> },
        { "right", rpc_method_caller<m3pi, float, &m3pi::right> },
        { "stop", rpc_method_caller<m3pi, &m3pi::stop> },
        { "left_motor", rpc_method_caller<m3pi, float, &m3pi::left_motor> },
        { "right_motor", rpc_method_caller<m3pi, float, &m3pi::right_motor> },
        { "battery", rpc_method_caller<float, m3pi, &m3pi::battery> },
        { "line_position", rpc_method_caller<float, m3pi, &m3pi::line_position> },
        { "sensor_auto_calibrate", rpc_method_caller<char, m3pi, &m3pi::sensor_auto_calibrate> },


        RPC_METHOD_SUPER(Base)
    };
    return rpc_methods;
}
#endif
