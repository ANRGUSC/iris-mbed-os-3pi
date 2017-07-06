#include "mqtt.h"
#define DEBUG   1

#if (DEBUG) 
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */
/**
 * @brief constructs a mqtt pub packet
 * @details this function generates a hdlc mqtt pub packet
 * 
 * @param topic             mqtt topic name
 * @param data              the data to be published    
 * @param src_port          hdlc source port
 * @param mqtt_send_pkt     pointer to the mqtt packet
 * @param pkt               pointer to the hdlc packet
 */

void build_mqtt_pkt_pub(char topic[], char data[], uint16_t src_port,
                                    mqtt_pkt_t *mqtt_send_pkt, hdlc_pkt_t *pkt)
{
    uart_pkt_hdr_t send_hdr = {0, 0, 0};
    send_hdr.pkt_type = MQTT_PUB;
    send_hdr.dst_port = RIOT_MQTT_PORT;
    send_hdr.src_port = src_port;
                                
    strcpy(mqtt_send_pkt->topic, topic);
    strcpy(mqtt_send_pkt->data, data);

    uart_pkt_cpy_data(pkt->data, HDLC_MAX_PKT_SIZE, mqtt_send_pkt, sizeof(mqtt_pkt_t));
    uart_pkt_insert_hdr(pkt->data, HDLC_MAX_PKT_SIZE, &send_hdr); 
}


/**
 * @brief constructs a mqtt sub packet
 * @details this function generates a hdlc mqtt sub packet
 * 
 * @param topic             mqtt topic name
 * @param src_port          hdlc source port
 * @param mqtt_send_pkt     pointer to the mqtt packet
 * @param pkt               pointer to the hdlc packet
 */
void build_mqtt_pkt_sub(char topic[], uint16_t src_port,
                                    mqtt_pkt_t *mqtt_send_pkt, hdlc_pkt_t *pkt)
{
    uart_pkt_hdr_t send_hdr = {0, 0, 0};
    send_hdr.pkt_type = MQTT_SUB;
    send_hdr.dst_port = RIOT_MQTT_PORT;
    send_hdr.src_port = src_port;
                                
    strcpy(mqtt_send_pkt->topic, topic);

    uart_pkt_cpy_data(pkt->data, HDLC_MAX_PKT_SIZE, mqtt_send_pkt, sizeof(mqtt_pkt_t));
    uart_pkt_insert_hdr(pkt->data, HDLC_MAX_PKT_SIZE, &send_hdr); 
}


void process_mqtt_pkt(mqtt_pkt_t *pkt, mqtt_data_t *data_pkt)
{ 
    data_pkt->data_type = pkt->data[0] - '0';
    strcpy(data_pkt->data, pkt->data + 1);

    // PRINTF("MQTT: Data type %d\n", data_pkt.data_type);

    switch (data_pkt->data_type){
        case NORM_DATA:
            PRINTF("MQTT: Normal Data Received %s \n", data_pkt->data);
            break;

        case SUB_CMD:
            PRINTF("MQTT: Sub command Received %s \n", data_pkt->data);
            break;

        case PUB_CMD:
            PRINTF("MQTT: Pub command Received %s \n", data_pkt->data);
            break;

        default:
            PRINTF("MQTT: Received data doesn't match with any type\n");

    }
}
