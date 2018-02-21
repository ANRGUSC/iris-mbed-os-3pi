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
#define ABS(x) ((x)>0?(x):-(x))


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

#define DRIVE_STRAIGHT 0xE1
#define DRIVE_STRAIGHT_DISTANCE 0xE2
#define ROTATE_DEGREES 0xE3
#define DRIVE_STRAIGHT_DISTANCE_BLOCKING 0xE4
#define ROTATE_DEGREES_BLOCKING 0xE5

// PID constants
unsigned int pid_enabled = 0;
unsigned char maxSpeed = 100;
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
    if(power_difference > maxSpeed)
        power_difference = maxSpeed;
    if(power_difference < -maxSpeed)
        power_difference = -maxSpeed;

    if(power_difference < 0)
        set_motors(maxSpeed+power_difference, maxSpeed);
    else
        set_motors(maxSpeed, maxSpeed-power_difference);
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
    maxSpeed = (constants[0] == 127 ? 255 : constants[0]*2);

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
} FloatPID;

double FloatUpdatePID(FloatPID *pid, double error, double position)
{
    double pTerm, dTerm, iTerm;
    pTerm = pid->pGain * error; // calculate the proportional term

    // calculate the integral state with appropriate limiting
    pid->iState += error;

    if (pid->iState > pid->iMax) 
        pid->iState = pid->iMax;
    else if (pid->iState < pid->iMin) 
        pid->iState = pid->iMin;

    iTerm = pid->iGain * pid->iState; // calculate the integral term

    dTerm = pid->dGain * (pid->dState - position);
    pid->dState = position;

    return pTerm + dTerm + iTerm;
}

typedef struct
{
    int dState;
    int iState;
    int iMax, iMin;
    int pGainNum, pGainDen; //proportional gain numerator and denom (for integer controller)
    int iGainNum, iGainDen; //integral gain
    int dGainNum, dGainDen; //derivative gain
} PID_t;

void InitializePID(PID_t *pid)
{
    char constants[10];

    for(unsigned int i = 0; i < sizeof(constants); i++)
    {
        constants[i] = read_next_byte();
        if(check_data_byte(constants[i]))
            return;
    }

    pid->dState = constants[0];
    pid->iState = constants[1];
    pid->iMax = constants[2];
    pid->iMin = constants[3];
    pid->pGainNum = constants[4];
    pid->pGainDen = constants[5];
    pid->iGainNum = constants[6];
    pid->iGainDen = constants[7];
    pid->dGainNum = constants[8];
    pid->dGainDen = constants[9];
}

int UpdatePID(PID_t *pid, int error, int position) 
{
    // Do nothing if the denominator of any constant is zero.
    if(pid->pGainDen == 0 || pid->iGainDen == 0 || pid->dGainDen == 0)
    {
        return 0;
    }   

    int pTerm, dTerm, iTerm;

    /* calculate the proportional term */   
    pTerm = error * pid->pGainNum / pid->pGainDen; 

    /* calculate the integral state with appropriate limiting */
    pid->iState += error;

    if (pid->iState > pid->iMax) 
        pid->iState = pid->iMax;
    else if (pid->iState < pid->iMin) 
        pid->iState = pid->iMin;

    iTerm = pid->iState * pid->iGainNum / pid->iGainDen; // calculate the integral term

    dTerm = (pid->dState - position) * pid->dGainNum / pid->dGainDen ;
    pid->dState = position;

    return pTerm + dTerm + iTerm;
}

void DriveStraight(PID_t *pid)
{
    //get initial speed from mbed
    int init_speed = 50; //read_next_byte();

    int right_speed = init_speed;

    int encoder_diff_error;

    while(1) 
    {
        encoder_diff_error = encoders.getCountsAndResetM1() - encoders.getCountsAndResetM2();
        int speed_diff = UpdatePID(pid, encoder_diff_error, 0); 

        // Compute the actual motor settings.  We never set either motor
        // to a negative value.
        if(speed_diff > maxSpeed)
            speed_diff = maxSpeed;
        if(speed_diff < -maxSpeed)
            speed_diff = -maxSpeed;

        if(speed_diff > 0) {
            right_speed = right_speed + speed_diff;
            set_motors(init_speed, right_speed);
        }
        else {
            right_speed = right_speed - speed_diff;
            set_motors(init_speed, right_speed);
        }
    }
}

void DriveStraightDistance(PID_t *pid)
{
    //get speed from mbed
    char init_speed = read_next_byte();
    //get distance from mbed in ***number of ticks***
    uint16_t distanceLowByte = read_next_byte();
    uint16_t distanceHighByte = read_next_byte();

    // char init_speed = 50;
    // uint16_t distanceLowByte = 4476 & 0xFF;
    // uint16_t distanceHighByte = 4476 >> 8;
    int m1Count, m2Count;

    int right_speed = init_speed;
    int encoder_diff_error;

    uint16_t distance = distanceLowByte + (distanceHighByte << 8);

    int m1CountOld = 0;
    int m2CountOld = 0;
    while(1) 
    {
        m1Count = encoders.getCountsM1();
        m2Count = encoders.getCountsM2();

        if (m1Count > distance)
            break;

        /* for some reason the controller is unstable when you don't reset the values */
        encoder_diff_error = (m1Count - m1CountOld) - (m2Count - m2CountOld); 

        m1CountOld = m1Count;
        m2CountOld = m2Count;
        int speed_diff = UpdatePID(pid, encoder_diff_error, 0); 

        // Compute the actual motor settings.  We never set either motor
        // to a negative value.
        if(speed_diff > maxSpeed)
            speed_diff = maxSpeed;
        if(speed_diff < -maxSpeed)
            speed_diff = -maxSpeed;

        if(speed_diff > 0) {
            right_speed = right_speed + speed_diff;
            set_motors(init_speed, right_speed + speed_diff);
        }
        else {
            right_speed = right_speed - speed_diff;
            set_motors(init_speed, right_speed);
        }
    }

    set_motors(0,0);
    encoders.getCountsAndResetM1();
    encoders.getCountsAndResetM2();
}

void RotateDegrees(PID_t *rotate_pid)
{
    unsigned char degrees = read_next_byte();
    char direction = read_next_byte();
    char init_speed = read_next_byte();
    char right_speed; 
    int encoder_diff;

    /* does not accept angles larger than 180 degrees */
    if (degrees > 180)
        return;

    if (direction > 0) {
        right_speed = init_speed;
        set_motors(-init_speed, right_speed);
    } else {
        right_speed = -init_speed;
        set_motors(init_speed, right_speed);
    }

    /**
     * [2 * b * pi / (180 * dist_per_rotary_tick_in_mm)] = 0.223402 mm
     * b = distance between wheels = 98 mm 
     */
    int target_diff = direction * (int)(15.3125 * (float)degrees);

    while(1)
    {
        /* right count minus left count */
        encoder_diff = encoders.getCountsM2() - encoders.getCountsM1();

        if (direction > 0)  {
            if(encoder_diff > target_diff) //target ticks
                break;
        } else {
            if(encoder_diff < target_diff) //target ticks
                break;
        }

        int speed_diff = UpdatePID(rotate_pid, -(encoders.getCountsM1()) - 
                  (encoders.getCountsM2()), 0);

        if(speed_diff > maxSpeed)
            speed_diff = maxSpeed;
        if(speed_diff < -maxSpeed)
            speed_diff = -maxSpeed;

        if (direction > 0) {
            if (speed_diff > 0)
                right_speed += speed_diff;
            else
                right_speed -= speed_diff;

            set_motors(-init_speed, right_speed);
        } else {
            if (speed_diff > 0)
                right_speed -= speed_diff;
            else
                right_speed += speed_diff;

            set_motors(init_speed, right_speed);
        }


    }

    set_motors(0, 0);
    encoders.getCountsAndResetM1();
    encoders.getCountsAndResetM2();
}

/////////////////////////////////////////////////////////////////////

int main()
{
    char message = 0;
    char message2 = 0;
    pololu_3pi_init(2000);  

    /* start receiving data at 115.2 kbaud */
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

    /* each encoder tick is 0.223402 mm in distance traveled. 32767 ticks is 
    about 7.320 meters so make sure to reset the counts accordingly. dist
    between the two wheels is approximately 98mm  */


    PID_t drive_straight_pid;
    PID_t rotate_pid;
    PID_t drive_straight_dist_pid;

    drive_straight_pid.pGainNum = 1;
    drive_straight_pid.pGainDen = 5;
    drive_straight_pid.iGainNum = 0;
    drive_straight_pid.iGainDen = 1;
    drive_straight_pid.dGainNum = 0;
    drive_straight_pid.dGainDen = 1;
    // DriveStraight(&drive_straight_pid);
    

    /* NOT WORKING YET */
    rotate_pid.pGainNum = 0;
    rotate_pid.pGainDen = 5;
    rotate_pid.iGainNum = 0;
    rotate_pid.iGainDen = 1;
    rotate_pid.dGainNum = 0;
    rotate_pid.dGainDen = 1;

    /* NOT YET WORKING */
    drive_straight_dist_pid.pGainNum = 0;
    drive_straight_dist_pid.pGainDen = 5;
    drive_straight_dist_pid.iGainNum = 0;
    drive_straight_dist_pid.iGainDen = 1;
    drive_straight_dist_pid.dGainNum = 0;
    drive_straight_dist_pid.dGainDen = 1;


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
            // case (char)INITIALIZE_DRIVE_PID:
            //     InitializePID(&drive_straight_pid);
            //     break;
            case (char)DRIVE_STRAIGHT:
                DriveStraight(&drive_straight_pid);
                break;
            case (char)ROTATE_DEGREES:
                RotateDegrees(&rotate_pid);
                break;
            case (char)DRIVE_STRAIGHT_DISTANCE:
                DriveStraightDistance(&drive_straight_dist_pid);
                break;
            case (char)DRIVE_STRAIGHT_DISTANCE_BLOCKING:
                if(encoders.getCountsM2() == 0){
                    message = 5;
                }
                else{
                    message = 6;
                }
                DriveStraightDistance(&drive_straight_dist_pid);
                serial_send_blocking(&message, 1);
                break;
            case (char)ROTATE_DEGREES_BLOCKING:
                if(encoders.getCountsM2() == 0){
                    message2 = 5;
                }
                else{
                    message2 = 6;
                }
                RotateDegrees(&drive_straight_dist_pid);
                serial_send_blocking(&message2, 1);
                break;
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