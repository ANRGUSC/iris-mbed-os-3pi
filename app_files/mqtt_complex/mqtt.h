#ifndef _MQTT_H_
#define _MQTT_H_


#define MBED_MQTT_PORT  200
#define RIOT_MQTT_PORT  170
#define MAIN_THR_PORT   165
#define NULL_PKT_TYPE   0xFF


#include "mbed.h"
#include "rtos.h"
#include "hdlc.h"
#include "uart_pkt.h"
#include "main-conf.h"   

typedef struct __attribute__((packed)){
    uint8_t data_type;
    char data[31];
} mqtt_data_t;


typedef enum {
    NORM_DATA             = 0,
    SUB_CMD               = 1,
    PUB_CMD               = 2
} type_mqtt_data_t;


void build_mqtt_pkt_pub(char topic[], char data[], uint16_t src_port,
                                    mqtt_pkt_t *mqtt_send_pkt, hdlc_pkt_t *pkt);
void build_mqtt_pkt_sub(char topic[], uint16_t src_port,
                                    mqtt_pkt_t *mqtt_send_pkt, hdlc_pkt_t *pkt);

void process_mqtt_pkt(mqtt_pkt_t *pkt, mqtt_data_t *data_pkt);

#endif