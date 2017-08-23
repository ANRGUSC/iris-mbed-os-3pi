#include "mbed_movement.h"

#define DEBUG   0

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

//Use pc definition from main file
extern Serial pc;

//Create default m3pi instance
m3pi robot;
//Create gyro/accelerometer sensor and compass
LSM6 sensor;
LIS3MDL compass;

bool init_minimu(void)
{
	//Initialize Gyro/Accelerometer and Compass so ready to take values
	if(!sensor.init())
		return false;
	PRINTF("Sensor init success\n");
	sensor.enableDefault();
	sensor.enableAxes(sensor.all_sensors, true, true, true);
	if(!compass.init())
		return false;
	PRINTF("Compass init success\n");
	compass.enableDefault();
	
	return true;
}

void calibrate_compass(void)
{
	Timer timer;
	uint8_t reg_value = 0;
	//Initialize max to smallest and min to largest possible value
	int16_t max[2] = {0x8000, 0x8000};
	int16_t min[2] = {0x7FFF, 0x7FFF};

	robot.right(40);
	timer.start();

	//Rotate for 2 seconds to find max and min magnetometer values
	while(timer.read_ms() < 2000)
	{
		reg_value = compass.readReg(compass.STATUS_REG) & DATA_MASK;
		if(!reg_value)
		{
			PRINTF("No data to read\n");
			continue;
		}
		if(reg_value == XDA | YDA)
			compass.read();
		else
		{
			PRINTF("Only one axis ready\n");
			continue;
		}
		if(compass.m.x > max[0])
			max[0] = compass.m.x;
		if(compass.m.x < min[0])
			min[0] = compass.m.x;
		if(compass.m.y > max[1])
			max[1] = compass.m.y;
		if(compass.m.y < min[1])
			min[1] = compass.m.y;

	}
	robot.stop();
	timer.stop();

	//Set correction to average compass x and y values
	//(since should be centered at 0)
	compass.correction.x = (max[0] + min[0]) / 2;
	compass.correction.y = (max[1] + min[1]) / 2;

	//Find radial distance between max and min values, approximate the
	//average radius,  and compare the two to find a scaling factor to
	//resize data to make it more circular
	float x_chord = (max[0] - min[0]) / 2.0;
	float y_chord = (max[1] - min[1]) / 2.0;
	float avg_rad = (x_chord + y_chord) / 2.0;

	compass.scale.x = avg_rad / x_chord;
	compass.scale.y = avg_rad / y_chord;
	PRINTF("Calibration complete\n");
}

void rotate_degrees(int16_t degrees, int8_t speed)
{
	bool right;
	bool passed_stop = false;
	char buf[6];
	uint8_t full_rotations, length;
	double partial_rotation;
	double stop_bearing, bearing, bearing_prev, bearing_0;
	double pred_bearing, distance_curr, distance_pred, dist_traveled;
	int time;
	int wait_time = 0;

	//Account for negative speed and degree values
	if(speed < 0)
	{
		speed = abs(speed);
		degrees *= -1;
	}
	if(speed < 15)	//15 is just above motor stall speed
		return;
	if(degrees > 0)
		right = true;
	else if(degrees < 0)
	{
		right = false;
		degrees = abs(degrees);
	}
	else
		return;

	full_rotations = 0;
	//Correction to degree value to avoid overshooting
	partial_rotation = degrees - 0.5 * speed;
	
	//If rotation to small, change speed and rotation to give approximate
	//rotation amount
	if(degrees < 30)
	{
		partial_rotation = 3.0;
		if(degrees < 13)
		{
			//Around 10 degree turn
			speed = 15;
		}
		else if(degrees < 18)
		{
			//Around 15 degree turn
			speed = 25;
		}
		else if(degrees < 23)
		{
			//Around 20 degree turn
			speed = 35;
		}
		else
		{
			//Around 25 degree turn
			speed = 50;
		}
	}
	//Small angle high speed correction
	else if(partial_rotation < 5.0)
		partial_rotation = 5.0;
	//Set number of full rotations and partial rotation angle
	while(partial_rotation >= 360)
	{
		full_rotations++;
		partial_rotation -= 360.0;
	}

	//starting value
	compass.read();
	bearing_prev = compass.data_handler();
	sensor.read();
	sensor.data_handler();


	if(right)
	{
		//Decrement by one to prevent thinking passed bearing on first compare
		bearing_0 = bearing_prev - 1.0;
		//Calculate heading to stop at, make sure 0-360, begin rotating m3pi
		if(bearing_0 < 0)
			bearing_0 += 360.0;
		stop_bearing = bearing_prev + partial_rotation;
		if(stop_bearing > 360)
			stop_bearing -= 360.0;
		robot.right(speed);
	}
	else
	{
		//Increment by one to prevent thinking passed bearing on first compare
		bearing_0 = bearing_prev + 1.0;
		//Calculate heading to stop at, make sure 0-360, begin rotating m3pi
		if(bearing_0 > 360)
			bearing_0 -= 360.0;
		stop_bearing = bearing_prev - partial_rotation;
		if(stop_bearing < 0)
			stop_bearing += 360.0;
		robot.left(speed);
	}
	//start timer because needed for speed calculation and gyro data
	sensor.timer.reset();
	sensor.timer.start();
	while(full_rotations > 0)
	{
		//Read data every time rotation loops
		PRINTF("Looped\n");
		compass.read();
		bearing = compass.data_handler();
		sensor.readGyro();
		sensor.data_handler();
		
		if(right)
		{
			//If robot crossed 360/0 barrier
			if(bearing_prev > 300 && bearing < 60)
			{
				if(bearing_0 > bearing_prev || bearing_0 < bearing)
					full_rotations--;
			}
			//if robot passed starting point
			else if(bearing_0 > bearing_prev && bearing_0 < bearing)
				full_rotations--;
		}
		else
		{
			//If robot crossed 360/0 barrier
			if(bearing_prev < 60 && bearing > 300)
			{
				if(bearing_0 < bearing_prev || bearing_0 > bearing)
					full_rotations--;
			}
			//If robot passed starting point
			else if(bearing_0 < bearing_prev && bearing_0 > bearing)
				full_rotations--;
		}
		//Update previous bearing value each time
		bearing_prev = bearing;
	}
	
	//Just used to tell which condition of loop triggered passed_stop=true
	int8_t error_stat = 0;

	while(!passed_stop)
	{
		//Read compass and Gyro
		//Gyro measurements not currently used, but readings taken in case
		//needed at some point
		//sensor.data_handler() MUST be used regardless as it resets the timer
		compass.read();
		bearing = compass.data_handler();
		sensor.readGyro();
		time = sensor.timer.read_us();
		sensor.data_handler();
		//Find predicted bearing at next loop using previous travel distance
		pred_bearing = 2.0 * bearing - bearing_prev;
		if(right)
			dist_traveled = bearing - bearing_prev;
		else
			dist_traveled = bearing_prev - bearing;
		if(dist_traveled < -0.5)
			dist_traveled += 360.0;
		//Sometimes on startup when hasn't started moving will get small
		//negative values so set value between -0.5 to 0 to 0
		else if(dist_traveled < 0)
			dist_traveled = 0.0;
		if(pred_bearing < -0.5)
			pred_bearing += 360.0;
		else if(pred_bearing < 0)
			pred_bearing = 0;
		else if(pred_bearing > 360)
			pred_bearing -= 360.0;

		if(right)
		{
			//Find distance to stop bearing from current and predicted bearing
			distance_curr = stop_bearing - bearing;
			distance_pred = pred_bearing - stop_bearing;
			PRINTF("Current distance: %.1f\n", distance_curr);
			PRINTF("Predicted distance: %.1f\n", distance_pred);
			//predicted value has passed stop if:
			//1. distance_pred > 0
			//2. the bearing is just less than 360, the predicted bearing is
			//	 just greater than 0, and the stop bearing is between them
			if(distance_pred > 0.1)
			{
				//If current distance > 0, then bearing has not yet passed stop
				if(distance_curr > 0.1)
				{
					PRINTF("3\n");
					error_stat = 3;
					wait_time = (int)(distance_curr / dist_traveled * time);
					passed_stop = true;
				}
				else if(bearing > 260 && stop_bearing < 100 && pred_bearing < 260)
				{
					PRINTF("4\n");
					error_stat = 4;
					distance_curr += 360.0;
					wait_time = (int)(distance_curr / dist_traveled * time);
					passed_stop = true;
				}
			}
			else if(bearing > 200 && pred_bearing < 160 && stop_bearing > bearing)
			{
				//If current distance > 0, then bearing has not yet passed stop
				if(distance_curr > 0.1)
				{
					PRINTF("5\n");
					error_stat = 5;
					wait_time = (int)(distance_curr / dist_traveled * time);
					passed_stop = true;
				}
			}
			//Check if prediction incorrect and bearing has already passed stop
			if(!passed_stop)
			{
				if(bearing > stop_bearing && bearing_prev < stop_bearing)
				{
					PRINTF("1\n");
					error_stat = 1;
					passed_stop = true;
				}
				else if(bearing_prev > 260 && bearing < 100)
				{
					if(bearing_prev < stop_bearing || bearing > stop_bearing)
					{	
						PRINTF("2\n");
						error_stat = 2;
						passed_stop = true;
					}
				}
			}
		}
		else
		{
			//Find distance to stop bearing from current and predicted bearing
			distance_curr = bearing - stop_bearing;
			distance_pred = stop_bearing - pred_bearing;
			//predicted value has passed stop if:
			//1. distance_pred > 0
			//2. the bearing is just greater than 0, the predicted bearing is
			//	 just less than 360, and the stop bearing is between them
			if(distance_pred > 0.1)
			{
				//If current distance > 0, then bearing has not yet passed stop
				if(distance_curr > 0.1)
				{
					wait_time = (int)(distance_curr / dist_traveled * time);
					passed_stop = true;
				}
				else if(bearing < 100 && stop_bearing > 260 && pred_bearing > 100)
				{
					distance_curr += 360.0;
					wait_time = (int)(distance_curr / dist_traveled * time);
					passed_stop = true;
				}
			}
			else if(bearing < 160 && pred_bearing > 200 && bearing > stop_bearing)
			{
				//If current distance > 0, then bearing has not yet passed stop
				if(distance_curr > 0.1)
				{
					wait_time = (int)(distance_curr / dist_traveled * time);
					passed_stop = true;
				}
			}
			//Check if prediction incorrect and bearing has already passed stop
			if(!passed_stop)
			{
				if(bearing < stop_bearing && bearing_prev > stop_bearing)
				{
					PRINTF("1\n");
					error_stat = 1;
					passed_stop = true;
				}
				else if(bearing_prev < 100 && bearing > 260)
				{
					if(bearing_prev > stop_bearing || bearing < stop_bearing)
					{	
						PRINTF("2\n");
						error_stat = 2;
						passed_stop = true;
					}
				}
			} 
		}
		//If wait time set, wait until timer reading passes wait time
		while(sensor.timer.read_us() < wait_time);
		bearing_prev = bearing;
	}

	//Stop movement and gather final data
	robot.stop();
	compass.read();
	bearing = compass.data_handler();
	sensor.readGyro();
	sensor.data_handler();
	sensor.timer.stop();

	//Print error stats and wait time to screen
	//Can print whatever want displayed for error or functionality purposes
	robot.cls();
	length = snprintf(buf, 6, "%d", error_stat);
	robot.locate(2, 0);
	robot.print(buf, length);
	
	length = snprintf(buf, 6, "%d", wait_time);
	robot.locate(2, 1);
	robot.print(buf, length);

	//Code to print mbed estimate of distance traveled
	
	/*double measured = bearing - bearing_0;
	if(right)
	{
		if(measured < 0)
			measured += 360.0;
		measured += degrees / 360 * 360;
	}
	else
	{
		if(measured > 0)
			measured -= 360.0;
		measured -= degrees / 360 * 360;
	}

	robot.cls();
	length = snprintf(buf, 6, "%.1f", measured);
	robot.locate(2, 0);
	robot.print(buf, length);
	
	measured += 0.5 * speed; 
	length = snprintf(buf, 6, "%.1f", measured);
	robot.locate(2, 1);
	robot.print(buf, length);*/
}

void rotate_parts(int16_t degrees)
{
	uint8_t count = 0;
	int8_t partial_deg[20];
	char buf[6];
	uint8_t length;

	//Change to less than one full rotation
	degrees %= 360;
	//Round degrees to nearest 5 or 10
	if(degrees % 5 > 2)
		degrees += 5 - degrees % 5;
	else if(degrees % 5 < -2)
		degrees += -5 - degrees % 5;
	else
		degrees -= degrees % 5;

	if(degrees < 5 && degrees > -5)
		return;
	else if(degrees < 10 && degrees > -10)
		degrees = 10;

	if(degrees > 0)
	{
		while((degrees >= 35) || (degrees == 25))
		{
			degrees -= 25;
			partial_deg[count] = 25;
			count++;
		}
		if(degrees >= 20)
		{
			degrees -= 20;
			partial_deg[count] = 20;
			count++;
		}
		if(degrees == 15)
		{
			partial_deg[count] = 15;
			count++;
		}
		else if(degrees == 10)
		{
			partial_deg[count] = 10;
			count++;
		}
	}
	else
	{
		while((degrees <= -35) || (degrees == -25))
		{
			degrees += 25;
			partial_deg[count] = -25;
			count++;
		}
		if(degrees <= -20)
		{
			degrees += 20;
			partial_deg[count] = -20;
			count++;
		}
		if(degrees == -15)
		{
			partial_deg[count] = -15;
			count++;
		}
		else if(degrees == -10)
		{
			partial_deg[count] = -10;
			count++;
		}
	}
	for(uint8_t i = 0; i < count; i++)
	{
		rotate_degrees(partial_deg[i], 40);
		Thread::wait(100);
	}
}

void drive_forward(uint16_t time, int8_t speed)
{
	Timer timer;
	int8_t scaling = 0;
	uint8_t length;
	char buf[6];
	float value, threshold;

	if(speed == 0)
		return;
	else if(speed < 0)
		speed = abs(speed);
	if(speed > 25)
		threshold = 15.0;
	else
		threshold = 10.0;

	timer.start();
	robot.forward(speed, scaling);
	while(timer.read_ms() <  time)
	{
		sensor.read();

		value = 0.00875 * sensor.g.z + 2.6;
		if(value > threshold)
		{
			scaling--;
			robot.forward(speed, scaling);
		}
		else if(value < -1.0 * threshold)
		{
			scaling++;
			robot.forward(speed, scaling);
		}
		robot.cls();
		length = snprintf(buf, 6, "%d", scaling);
		robot.locate(2, 0);
		robot.print(buf, length);
	
		length = snprintf(buf, 6, "%.2f", value);
		robot.locate(1, 1);
		robot.print(buf, length);
	}
	timer.stop();
	robot.stop();
}

void drive_forward(uint16_t time)
{
	float error, previous_error, pid_p, pid_d;
	char buf[6];
	float pid_i = 0.0;
	int8_t scaling = 0;
	uint8_t speed = 40;
	int8_t pidout, pidout_prev, length;
	unsigned int delT;
	double bearing, bearing_prev, difference;
	Timer run_timer, loop_timer;
	
	error = 0.0;
	pidout_prev = 0.0;
	compass.read();
	bearing_prev = compass.data_handler();

	run_timer.start();
	robot.forward(speed, scaling);
	loop_timer.start();
	while(run_timer.read_ms() < time)
	{
		previous_error = error;

		sensor.read();
		delT = loop_timer.read_us();
		loop_timer.reset();
		compass.read();
		bearing = compass.data_handler();

		difference = bearing - bearing_prev;
		if(bearing < 60 && bearing_prev > 300)
			difference += 360.0;
		else if(bearing > 300 && bearing_prev < 60)
			difference -= 360.0;

		error = 0.98 * (error + (G_FACTOR * sensor.g.z + DRIFT)
			* delT / 1000000) + 0.02 * (error + difference);

		pid_p = KP * error;

		pid_i = pid_i + KI * error;

		pid_d = KD * ((error - previous_error) / delT);

		pidout = (int8_t)(pid_p + pid_i + pid_d);

		if(pidout > 10 && pidout > pidout_prev)
		{
			scaling-=2;
			robot.forward(speed, scaling);
		}
		else if(pidout > 5 && pidout > pidout_prev)
		{
			scaling--;
			robot.forward(speed, scaling);
		}
		else if(pidout < -10 && pidout < pidout_prev)
		{
			scaling+=2;
			robot.forward(speed, scaling);
		}
		else if(pidout < -5 && pidout < pidout_prev)
		{
			scaling++;
			robot.forward(speed, scaling);
		}
		pidout_prev = pidout;
		bearing_prev = bearing;

		/*robot.cls();
		length = snprintf(buf, 6, "%.1f", difference);
		robot.locate(2, 0);
		robot.print(buf, length);

		length = snprintf(buf, 6, "%.1f", (G_FACTOR*sensor.g.z+DRIFT)*delT/1000000);
		robot.locate(2, 1);
		robot.print(buf, length);*/
	}
	robot.stop();
	run_timer.stop();
	loop_timer.stop();
}

node_info_t find_location(node_info_t &anchor1, node_info_t &anchor2,
	node_info_t &anchor3)
{
	node_info_t m3pi_node;
	float dx1_2, dy1_2, dx3_1, dy3_1, dx2_3, dy2_3;
	float dist1_2, dist3_1, dist2_3, a1_2, a2_3, a3_1;
	float bx1_2, by1_2, bx2_3, by2_3, bx3_1, by3_1;
	float h1_2, h2_3, h3_1, average_x, average_y;
	float rx1_2, ry1_2, rx2_3, ry2_3, rx3_1, ry3_1;
	float inter_x[6], inter_y[6], inter_dist[3], inter_dx[3], inter_dy[3];
	//weird order to use values from circle not used in measurements (see below)
	node_info_t node_array[3] = {anchor3, anchor1, anchor2};

	average_x = 0.0;
	average_y = 0.0;

	dx1_2 = anchor2.x - anchor1.x;
	dy1_2 = anchor2.y - anchor1.y;
	dx3_1 = anchor1.x - anchor3.x;
	dy3_1 = anchor1.y - anchor3.y;
	dx2_3 = anchor3.x - anchor2.x;
	dy2_3 = anchor3.y - anchor2.y;

	dist1_2 = sqrt((dx1_2 * dx1_2) + (dy1_2 * dy1_2));
	dist3_1 = sqrt((dx3_1 * dx3_1) + (dy3_1 * dy3_1));
	dist2_3 = sqrt((dx2_3 * dx2_3) + (dy2_3 * dy2_3));

	//Check if the m3pi has left the space between the nodes
	if((anchor1.radius > dist1_2 && anchor1.radius > dist3_1) ||
		(anchor2.radius > dist1_2 && anchor2.radius > dist2_3) ||
		(anchor3.radius > dist3_1 && anchor3.radius > dist2_3))
	{
		//return negative radius to signal error
		m3pi_node = {0.0, 0.0, -1.0};
		return m3pi_node;
	}

	//a is the distance from the center of the first point to where line
	//between circle centers and line connecting circle intersections meet
	//
	//call that intersection point b  (arbitrary)
	a1_2 = ((anchor1.radius * anchor1.radius - anchor2.radius * anchor2.radius
		+ dist1_2 * dist1_2) / (2.0 * dist1_2));
	a2_3 = ((anchor2.radius * anchor2.radius - anchor3.radius * anchor3.radius
		+ dist2_3 * dist2_3) / (2.0 * dist2_3));
	a3_1 = ((anchor3.radius * anchor3.radius - anchor1.radius * anchor1.radius
		+ dist3_1 * dist3_1) / (2.0 * dist3_1));

	//Find coordinates point b for each circle intersection
	bx1_2 = anchor1.x + (dx1_2 * a1_2 / dist1_2);
	by1_2 = anchor1.y + (dy1_2 * a1_2 / dist1_2);
	bx2_3 = anchor2.x + (dx2_3 * a2_3 / dist2_3);
	by2_3 = anchor2.y + (dy2_3 * a2_3 / dist2_3);
	bx3_1 = anchor3.x + (dx3_1 * a3_1 / dist3_1);
	by3_1 = anchor3.y + (dy3_1 * a3_1 / dist3_1);

	//Distance from point b to each of two intersection points
	//Use absolute value in case measurement is slightly off so you
	//don't get NaN
	h1_2 = sqrt(fabs(anchor1.radius * anchor1.radius - a1_2 * a1_2));
	h2_3 = sqrt(fabs(anchor2.radius * anchor2.radius - a2_3 * a2_3));
	h3_1 = sqrt(fabs(anchor3.radius * anchor3.radius - a3_1 * a3_1));

	//rx and ry are the x and y offsets of intersection points from pointb
	rx1_2 = -1.0 * dy1_2 * h1_2 / dist1_2;
	ry1_2 = dx1_2 * h1_2 / dist1_2;
	rx2_3 = -1.0 * dy2_3 * h2_3 / dist2_3;
	ry2_3 = dx2_3 * h2_3 / dist2_3;
	rx3_1 = -1.0 * dy3_1 * h3_1 / dist3_1;
	ry3_1 = dx3_1 * h3_1 / dist3_1;

	//Find half of the intersection points
	inter_x[0] = bx1_2 + rx1_2;
	inter_y[0] = by1_2 + ry1_2;
	inter_x[1] = bx2_3 + rx2_3;
	inter_y[1] = by2_3 + ry2_3;
	inter_x[2] = bx3_1 + rx3_1;
	inter_y[2] = by3_1 + ry3_1;
	//Second half on intersection points
	inter_x[3] = bx1_2 - rx1_2;
	inter_y[3] = by1_2 - ry1_2;
	inter_x[4] = bx2_3 - rx2_3;
	inter_y[4] = by2_3 - ry2_3;
	inter_x[5] = bx3_1 - rx3_1;
	inter_y[5] = by3_1 - ry3_1;

	//Find intersection of 3 circles and average them
	for(uint8_t i = 0; i < 3; i++)
	{

		//Find distance between intersection point and third circle center
		inter_dx[i] = inter_x[i] - node_array[i].x;
		inter_dy[i] = inter_y[i] - node_array[i].y;
		inter_dist[i] = sqrt(inter_dx[i] * inter_dx[i]
			+ inter_dy[i] * inter_dy[i]);
		PRINTF("inter_dist: %.3f\n", inter_dist[i]);

		//Check which intersection point also intersects with third circle
		if(fabs(inter_dist[i] - node_array[i].radius) < EPSILON)
		{
			average_x += inter_x[i];
			average_y += inter_y[i];
		}
		else
		{
			inter_dx[i] += inter_x[i+3] - inter_x[i];
			inter_dy[i] += inter_y[i+3] - inter_y[i];
			inter_dist[i] = sqrt(inter_dx[i] * inter_dx[i]
				+ inter_dy[i] * inter_dy[i]);

			if(fabs(inter_dist[i] - node_array[i].radius) >= EPSILON)
			{
				PRINTF("Error: Triangulation not possible\n");
				m3pi_node = {0.0, 0.0, -1.0};
				return m3pi_node;
			}
			average_x += inter_x[i+3];
			average_y += inter_y[i+3];
		}
	}
	average_x /= 3.0;
	average_y /= 3.0;
	m3pi_node = {average_x, average_y, 0.0};

	//Send location and focus on communicating
	return m3pi_node;
}