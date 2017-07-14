#ifndef DATA_CONV_H
#define DATA_CONV_H

#include <math.h>
#include <exception>



float tdoa_to_dist(int tdoa);
float od_to_angle(float a, float b);
float calc_x(float a, float b);

#endif