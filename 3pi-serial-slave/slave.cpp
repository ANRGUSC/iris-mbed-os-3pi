/*
 * 3pi-serial-slave - An example serial slave program for the Pololu
 * 3pi Robot.  See the following pages for more information:
 *
 * http://www.pololu.com/docs/0J21
 * http://www.pololu.com/docs/0J20
 * http://www.poolu.com/
 * http://forum.pololu.com
 * 
 */
#include <stdint.h>
#include "Pololu3pi.h"

#define SEND_SIGNATURE 0x81
#define SEND_RAW_SENSOR_VALUES 0x86
#define SEND_TRIMPOT 0xB0
#define SEND_BATTERY_MILLIVOLTS 0xB1
#define PI_CALIBRATE 0xB4
#define LINE_SENSORS_RESET_CALIBRATION 0xB5
#define SEND_LINE_POSITION 0xB6
#define AUTO_CALIBRATE 0xBA
#define SET_PID 0xBB
#define STOP_PID 0xBC

/* Left Motor */
#define M1_FORWARD 0xC1
#define M1_BACKWARD 0xC2

/* Right Motor */
#define M2_FORWARD 0xC5
#define M2_BACKWARD 0xC6

/* Left Motor */
#define SEND_M1_ENCODER_COUNT 0xD1

/* Right Motor */
#define SEND_M2_ENCODER_COUNT 0xD2

#define SEND_M1_ENCODER_ERROR 0xD3
#define SEND_M2_ENCODER_ERROR 0xD4

#define MOVE_STRAIGHT_DISTANCE 0xE1

// PID constants
unsigned int pid_enabled = 0;
unsigned char max_speed = 255;
unsigned char p_num = 0;
unsigned char p_den = 0;
unsigned char d_num = 0;
unsigned char d_den = 0;
unsigned int last_proportional = 0;
unsigned int sensors[5];

PololuWheelEncoders encoders;

// This routine will be called repeatedly to keep the PID algorithm running
void pid_check()
{
    if(!pid_enabled)
        return;
    
    // Do nothing if the denominator of any constant is zero.
    if(p_den == 0 || d_den == 0)
    {
        set_motors(0,0);
        return;
    }   

    // Read the line position.
    unsigned int position = read_line(sensors, IR_EMITTERS_ON);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 2000;

    // Compute the derivative (change) of the position.
    int derivative = proportional - last_proportional;

    // Remember the last position.
    last_proportional = proportional;

    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the right.  If it is a negative number, the robot will
    // turn to the left, and the magnitude of the number determines
    // the sharpness of the turn.
    int power_difference = proportional*p_num/p_den + derivative*d_num/d_den;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    if(power_difference > max_speed)
        power_difference = max_speed;
    if(power_difference < -max_speed)
        power_difference = -max_speed;

    if(power_difference < 0)
        set_motors(max_speed+power_difference, max_speed);
    else
        set_motors(max_speed, max_speed-power_difference);
}

// A global ring buffer for data coming in.  This is used by the
// read_next_byte() and previous_byte() functions, below.
char buffer[100];

// A pointer to where we are reading from.
unsigned char read_index = 0;

// Waits for the next byte and returns it.  Runs pid_check() to keep following 
// the line.
char read_next_byte()
{
    while(serial_get_received_bytes() == read_index)
    {
    //     // pid_check takes some time; only run it if we don't have more bytes to process
    //     if(serial_get_received_bytes() == read_index)
    //         pid_check();
    }
    char ret = buffer[read_index];
    read_index ++;
    if(read_index >= 100)
        read_index = 0;
    return ret;
}

// Backs up by one byte in the ring buffer.
void previous_byte()
{
    read_index --;
    if(read_index == 255)
        read_index = 99;
}

// Returns true if and only if the byte is a command byte (>= 0x80).
char is_command(char byte)
{
    if (byte < 0)
        return 1;
    return 0;
}

// Returns true if and only if the byte is a data byte (< 0x80).
char is_data(char byte)
{
    if (byte < 0)
        return 0;
    return 1;
}

// If it's not a data byte, beeps, backs up one, and returns true.
char check_data_byte(char byte)
{
    if(is_data(byte))
        return 0;

    previous_byte();
    return 1;
}

/////////////////////////////////////////////////////////////////////
// COMMAND FUNCTIONS
//
// Each function in this section corresponds to a single serial
// command.  The functions are expected to do their own argument
// handling using read_next_byte() and check_data_byte().

// Sends the version of the slave code that is running.
// This function also shuts down the motors and disables PID, so it is
// useful as an initial command.
void send_signature()
{
    char str[] = "3pi1.1";
    serial_send_blocking(str, 6);
    set_motors(0,0);
    pid_enabled = 0;
}

// Reads the line sensors and sends their values.  This function can
// do either calibrated or uncalibrated readings.  When doing calibrated readings,
// it only performs a new reading if we are not in PID mode.  Otherwise, it sends
// the most recent result immediately.
void send_sensor_values(char calibrated)
{
    if(calibrated)
    {
        if(!pid_enabled)
            read_line_sensors_calibrated(sensors, IR_EMITTERS_ON);
    }
    else
        read_line_sensors(sensors, IR_EMITTERS_ON);
    serial_send_blocking((char *)sensors, 10);
}

// Sends the raw (uncalibrated) sensor values.
void send_raw_sensor_values()
{
    send_sensor_values(0);
}

// Sends the calibated sensor values.
void send_calibrated_sensor_values()
{
    send_sensor_values(1);
}

// Computes the position of a black line using the read_line()
// function, and sends the value.
// Returns the last value computed if PID is running.
void send_line_position()
{
    int message[1];
    unsigned int tmp_sensors[5];
    int line_position;

    if(pid_enabled)
        line_position = last_proportional+2000;
    else line_position = read_line(tmp_sensors, IR_EMITTERS_ON);

    message[0] = line_position;

    serial_send_blocking((char *)message, 2);
}

// Sends the trimpot value, 0-1023.
void send_trimpot()
{
    int message[1];
    message[0] = read_trimpot();
    serial_send_blocking((char *)message, 2);
}

// Sends the batter voltage in millivolts
void send_battery_millivolts()
{
    int message[1];
    message[0] = read_battery_millivolts();
    serial_send_blocking((char *)message, 2);
}

// Drives m1 (left motor) forward.
void m1_forward()
{
    char byte = read_next_byte();
    
    if(check_data_byte(byte))
        return;

    set_m1_speed(byte == 127 ? 255 : byte*2);
}

// Drives m2 (right motor) forward.
void m2_forward()
{
    char byte = read_next_byte();
    
    if(check_data_byte(byte))
        return;

    set_m2_speed(byte == 127 ? 255 : byte*2);
}

// Drives m1 (left motor) backward.
void m1_backward()
{
    char byte = read_next_byte();
    
    if(check_data_byte(byte))
        return;

    set_m1_speed(byte == 127 ? -255 : -byte*2);
}

// Drives m2 (right motor) backward.
void m2_backward()
{
    char byte = read_next_byte();
    
    if(check_data_byte(byte))
        return;

    set_m2_speed(byte == 127 ? -255 : -byte*2);
}

// Runs through an automatic calibration sequence
void auto_calibrate()
{
    char str[] = "c";
    time_reset();
    set_motors(60, -60);  
    while(get_ms() < 250)  
        calibrate_line_sensors(IR_EMITTERS_ON);  
    set_motors(-60, 60);  
    while(get_ms() < 750)  
        calibrate_line_sensors(IR_EMITTERS_ON);  
    set_motors(60, -60);  
    while(get_ms() < 1000)  
        calibrate_line_sensors(IR_EMITTERS_ON);  
    set_motors(0, 0); 
    
    serial_send_blocking(str, 1); 
}

// Turns on PID according to the supplied PID constants
void set_pid()
{
    unsigned char constants[5];
    unsigned char i;
    for(i=0;i<5;i++)
    {
        constants[i] = read_next_byte();
        if(check_data_byte(constants[i]))
            return;
    }

    // make the max speed 2x of the first one, so that it can reach 255
    max_speed = (constants[0] == 127 ? 255 : constants[0]*2);

    // set the other parameters directly
    p_num = constants[1];
    p_den = constants[2];
    d_num = constants[3];
    d_den = constants[4];

    // enable pid
    pid_enabled = 1;}


// Turns off PID
void stop_pid()
{
    set_motors(0,0);
    pid_enabled = 0;
}

// Sends current left wheel encoder count
void send_m1_encoder_count()
{
    int message[1];
    message[0] = encoders.getCountsM1();
    serial_send_blocking((char *)message, 2);
}

// Sends current right wheel encoder count
void send_m2_encoder_count()
{
    int message[1];
    message[0] = encoders.getCountsM2();
    serial_send_blocking((char *)message, 2);
}

void send_m1_encoder_error()
{
    char message = encoders.checkErrorM1();
    serial_send_blocking(&message, 1);  
}

// Sends current right wheel encoder count
void send_m2_encoder_error()
{
    char message = encoders.checkErrorM2();
    serial_send_blocking(&message, 1);
}

typedef struct
{
    double dState; // Last position input
    double iState; // Integrator state
    double iMax, iMin; // Maximum and minimum allowable integrator state
    double iGain, // integral gain
           pGain, // proportional gain
           dGain; // derivative gain
} PID_t;

double UpdatePID(PID_t *pid, double error, double position)
{
    double pTerm, dTerm, iTerm;
    pTerm = pid->pGain * error; // calculate the proportional term

    // calculate the integral state with appropriate limiting
    pid->iState += error;
    if (pid->iState > pid->iMax) pid->iState = pid->iMax;
    else if (pid->iState < pid->iMin) pid->iState = pid->iMin;

    iTerm = pid->iGain * pid->iState; // calculate the integral term

    dTerm = pid->dGain * (pid->dState - position);
    pid->dState = position;

    return pTerm + dTerm + iTerm;
}

unsigned char kp_num = 0;
unsigned char kp_den = 0;
unsigned char kd_num = 0;
unsigned char kd_den = 0;
unsigned int last_prop = 0;
unsigned int initial_speed = 0;

void drive_straight_pid_loop() 
{
    // Do nothing if the denominator of any constant is zero.
    if(kp_den == 0 || kd_den == 0)
    {
        set_motors(0,0);
        return;
    }   

    // Read diff between wheel encoders
    int proportional = encoders.getCountsAndResetM1() - encoders.getCountsAndResetM2();

    int speed_diff = proportional * kp_num / kp_den;


    // Compute the derivative (change) of the position.
    // int derivative = proportional - last_proportional;

    // Remember the last position.
    // last_proportional = proportional;

    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the right.  If it is a negative number, the robot will
    // turn to the left, and the magnitude of the number determines
    // the sharpness of the turn.
    // int power_difference = proportional*p_num/kp_den + derivative*d_num/kd_den;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    if(speed_diff > max_speed)
        speed_diff = max_speed;
    if(speed_diff < -max_speed)
        speed_diff = -max_speed;

    if(speed_diff < 0)
        set_motors(initial_speed + speed_diff, initial_speed);
    else
        set_motors(initial_speed, initial_speed - speed_diff);
}

void move_straight_distance()
{
    //Not yet implemented
}

/////////////////////////////////////////////////////////////////////

int main()
{
    pololu_3pi_init(2000);  

    // start receiving data at 115.2 kbaud
    set_digital_input(IO_D0, PULL_UP_ENABLED);
    serial_set_baud_rate(115200);
    serial_receive_ring(buffer, 100);

    /**PD2 PB0 PD4 PD7: note PD7 is connected to the green LED underneath. PD2
     * and PB0 will be for the left motor (m1) and PD4/PD7 will be for the right
     * motor (m2)
     */
    OrangutanDigital::setInput(IO_D2, PULL_UP_ENABLED);
    OrangutanDigital::setInput(IO_B0, PULL_UP_ENABLED);
    OrangutanDigital::setInput(IO_D4, PULL_UP_ENABLED);
    OrangutanDigital::setInput(IO_D7, PULL_UP_ENABLED);
    encoders.init(IO_B0, IO_D2, IO_D4, IO_D7);

    kp_num = 1;
    kp_den = 5;
    kd_num = 0;
    kd_den = 1;
    last_prop = 0;
    initial_speed = 50;

    while(1)
    {
        drive_straight_pid_loop();
    }

    while(1)
    {
        // wait for a command
        char command = read_next_byte();

        // The list of commands is below: add your own simply by
        // choosing a command byte and introducing another case
        // statement.

        switch(command)
        {
            case (char)0x00:
                // silent error - probable master resetting
                break;
            case (char)SEND_SIGNATURE:
                send_signature();
                break;
            case (char)SEND_RAW_SENSOR_VALUES:
                send_raw_sensor_values();
                break;
            case (char)0x87:
                send_calibrated_sensor_values();
                break;
            case (char)0xB0:
                send_trimpot();
                break;
            case (char)SEND_BATTERY_MILLIVOLTS:
                send_battery_millivolts();
                break;
            case (char)PI_CALIBRATE:
                calibrate_line_sensors(IR_EMITTERS_ON);
                send_calibrated_sensor_values();
                break;
            case (char)LINE_SENSORS_RESET_CALIBRATION:
                line_sensors_reset_calibration();
                break;
            case (char)SEND_LINE_POSITION:
                send_line_position();
                break;
            case (char)AUTO_CALIBRATE:
                auto_calibrate();
                break;
            case (char)SET_PID:
                set_pid();
                break;
            case (char)STOP_PID:
                stop_pid();
                break;
            case (char)M1_FORWARD:
                m1_forward();
                break;
            case (char)M1_BACKWARD:
                m1_backward();
                break;
            case (char)M2_FORWARD:
                m2_forward();
                break;
            case (char)M2_BACKWARD:
                m2_backward();
                break;
            case (char)SEND_M1_ENCODER_COUNT:
                send_m1_encoder_count();
                break;
            case (char)SEND_M2_ENCODER_COUNT:
                send_m2_encoder_count();
                break;
            case (char)SEND_M1_ENCODER_ERROR:
                send_m1_encoder_error();
                break;
            case (char)SEND_M2_ENCODER_ERROR:
                send_m2_encoder_error();
                break;
            case (char)MOVE_STRAIGHT_DISTANCE:
                move_straight_distance();
                break;
            // case (char)ROTATE_DEGREES:
            //     rotate_degrees();
            default:
                continue; // bad command
        }
    }
}

// Local Variables: **
// mode: C++ **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **