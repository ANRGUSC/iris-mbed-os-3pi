

#include "data_conv.h"

#define PI                  3.14159265
#define SEPARATION_DIST     2.75/12

float tdoa_to_dist(int tdoa){
    return ((float)tdoa -19628.977) / 885.274;
}

float od_to_angle(float a, float b){
    float ans;
    float x = calc_x(a,b);
    float ratio = (b*b - a*a)/(2*SEPARATION_DIST*x);
    
    if(ratio > 1 || ratio < -1){
        return -1;
    } else{
        return asin(ratio)*180/PI;
    }
    
}

float calc_x(float a, float b){
    return sqrt( a*a/2 + b*b/2 - SEPARATION_DIST*SEPARATION_DIST/4 );
}
