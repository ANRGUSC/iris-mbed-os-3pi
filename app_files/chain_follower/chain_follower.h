#ifndef CHAIN_FOLLOWER_H_
#define CHAIN_FOLLOWER_H_

//uncomment for the first robot (leading robot is actually last robot)
// #define LEADER_ROBOT
// #define LEADING_ROBOT_IPV6_ADDR     "ff02"
// #define FOLLOWING_ROBOT_IPV6_ADDR   "ff02"

//uncomment for the second robot
// #define SECOND_ROBOT
// #define LEADING_ROBOT_IPV6_ADDR     "ff02"
// #define FOLLOWING_ROBOT_IPV6_ADDR   "ff02"

//uncomment for the third robot
// #define END_ROBOT
// #define LEADING_ROBOT_IPV6_ADDR     "ff02"
// #define FOLLOWING_ROBOT_IPV6_ADDR   "ff02"

void start_sending_stop_beacons();
void stop_sending_stop_beacons();
void start_sending_range_me();
void stop_sending_range_me();

#endif /* CHAIN_FOLLOWER_H_ */
