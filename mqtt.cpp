/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Pradipta Ghosh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal
 * with the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * - Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimers.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimers in the 
 *     documentation and/or other materials provided with the distribution.
 * - Neither the names of Autonomous Networks Research Group, nor University of 
 *     Southern California, nor the names of its contributors may be used to 
 *     endorse or promote products derived from this Software without specific 
 *     prior written permission.
 * - A citation to the Autonomous Networks Research Group must be included in 
 *     any publications benefiting from the use of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH 
 * THE SOFTWARE.
 */

/**
 * @file        mqtt.cpp
 * @brief       MQTT data processing related functions
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 */

#include "mqtt.h"
#define DEBUG   0

#if (DEBUG) 
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */
/**
 * @brief constructs a mqtt pub packet **Deprecated as of 11/21/17**
 * @details this function generates a hdlc mqtt pub packet
 * 
 * @param topic             mqtt topic name
 * @param data              the data to be published    
 * @param src_port          hdlc source port
 * @param mqtt_send_pkt     pointer to the mqtt packet
 * @param pkt               pointer to the hdlc packet
 */
void build_mqtt_pkt_pub(char* topic, char* data, uint16_t src_port,
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

    pkt->length = HDLC_MAX_PKT_SIZE;        
    
}

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
void build_mqtt_pkt_npub(char* topic, char* data, uint16_t src_port,
                                    mqtt_pkt_t *mqtt_send_pkt, size_t mqtt_data_len, hdlc_pkt_t *pkt)
{
    uart_pkt_hdr_t send_hdr = {0, 0, 0};
    send_hdr.pkt_type = MQTT_PUB;
    send_hdr.dst_port = RIOT_MQTT_PORT;
    send_hdr.src_port = src_port;
        
    strcpy(mqtt_send_pkt->topic, topic);
    strcpy(mqtt_send_pkt->data, data);

    pkt->length = uart_pkt_cpy_data(pkt->data, HDLC_MAX_PKT_SIZE, mqtt_send_pkt, MQTT_TOPIC_LEN + mqtt_data_len);
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
    pkt->length = HDLC_MAX_PKT_SIZE;        
    
}

/**
 * @brief      Process the received mqtt data
 *
 * @param      pkt       The received packet
 * @param      data_pkt  The perocessed data
 */
void process_mqtt_pkt(mqtt_pkt_t *pkt, mqtt_data_t *data_pkt)
{ 
    data_pkt->data_type = pkt->data[0] - '0';
    strcpy(data_pkt->data, pkt->data + 1);
    strcpy(data_pkt->topic, pkt->topic);

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
