#ifndef CHAIN_FOLLOWER_H_
#define CHAIN_FOLLOWER_H_

//uncomment for the first robot (leading robot is actually last robot)
// #define LEADER_ROBOT
// const char LEADING_ROBOT_IPV6_ADDR[] = "fe80::ff:fe00:fffe";
// const char FOLLOWING_ROBOT_IPV6_ADDR[] = "fe80::212:4b00:433:ed4e";

// //uncomment for the second robot
#define SECOND_ROBOT
const char LEADING_ROBOT_IPV6_ADDR[] = "fe80::212:4b00:613:66d";
const char FOLLOWING_ROBOT_IPV6_ADDR[] = "fe80::ff:fe00:fffe";

// //uncomment for the third robot
// #define END_ROBOT
// const char LEADING_ROBOT_IPV6_ADDR[] = "fe80::ff:fe00:df2f";
// const char FOLLOWING_ROBOT_IPV6_ADDR[] = "fe80::212:4b00:613:66d";

#define FORWARD_TO_MBED_MAIN_PORT   8000

//message types to send over UDP
#define INVALID_NET_MSG         -1
#define RANGE_ME                0
#define RANGE_ME_ACK            1
#define STOP_BEACONS            2
#define STOP_BEACONS_ACK        3

void tdoa_beacons_on(uint8_t node_id, Mail<msg_t, HDLC_MAILBOX_SIZE> *src_mailbox, 
    uint16_t src_hdlc_port, hdlc_pkt_t *hdlc_pkt);
void tdoa_beacons_off(Mail<msg_t, HDLC_MAILBOX_SIZE> *src_mailbox, 
    uint16_t src_hdlc_port, hdlc_pkt_t *hdlc_pkt);
void net_send_udp(const char *ipv6_addr_str, uint16_t port, uint8_t net_msg_type,
    Mail<msg_t, HDLC_MAILBOX_SIZE> *src_mailbox, uint16_t src_hdlc_port, 
    hdlc_pkt_t *hdlc_pkt);

void network_helper_init(osPriority priority);
void start_sending_stop_beacons_msgs();
void stop_sending_stop_beacons_msgs();
void start_sending_range_me_msgs();
void stop_sending_range_me_msgs();

#endif /* CHAIN_FOLLOWER_H_ */
