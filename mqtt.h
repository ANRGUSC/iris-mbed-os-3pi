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
 * @file        mqtt.h
 * @brief       MQTT data processing related functions
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 */

 #ifndef _MQTT_H_
#define _MQTT_H_


#include "mbed.h"
#include "rtos.h"
#include "hdlc.h"
#include "uart_pkt.h"
#include "main-conf.h"   

#ifndef MBED_MQTT_PORT
    #define MBED_MQTT_PORT  200
#endif

#ifndef RIOT_MQTT_PORT
    #define RIOT_MQTT_PORT  170
#endif
/**
 * In our structure the mqtt data section is used to controlling one node's operation from another.
 * Towards this goal, we have subdivided the data section into two main subblocks
 *              ||================||===========================||
 *              || MQTT Data Type ||     MQTT Data value       ||
 *              ||================||===========================||
 * Now, if the data type is NORM_DATA, the data value is the actual DATA
 *                  ||===========||========================||
 *                  || NORM_DATA || Actual Data to publish ||
 *                  ||===========||========================||
 *                  
 * If, the data type is SUB_CMD, then the data value is the TOPIC to subscibe to.
 *                  ||===========||========================||
 *                  || SUB_CMD   || Topic to Subscribe to  ||
 *                  ||===========||========================||
 *                  
 * Lastly, if the data type is PUB_CMD, the data value is actually a bit complex.
 * In this formulation the dthe structure of the PUB_CMD is as follows.
 *        ||=========||==================|================|=================||
 *        || PUB_CMD || PUB Topic Length | PUB Topic name | Data to publish ||
 *        ||=========||==================|================|=================||
 *        
 * Here, the PUB Topic Length is represents the number of bytes used to repersent the TOPIC to use for publishing.
 * PUB Topic name represents tge TOPIC to puslish to.
 * Data to publish is actually the type of data to be published. For example say temperature_sensor data.
 * 
 */


/**
 * Structure used by the mqtt to communicate data/commands
 */
typedef struct __attribute__((packed)){
    uint8_t data_type;
    char topic[16];
    char data[32];
} mqtt_data_t;

/**
 * Types of MQTT data packets
 */
typedef enum {
    NORM_DATA             = 0,
    SUB_CMD               = 1,
    PUB_CMD               = 2, 
    LEN_CLIENTS_LIST	  = 3,
    GET_CLIENTS			  = 4,
    RSSI_SEND			  = 5,
    SENSOR_DATA           = 6,
    MOVE_CMD              = 7, 
    RMT_CTRL              = 8,
    INM_DATA              = 9,      	
} type_mqtt_data_t;

typedef enum {
    SERVER_ACK            = 0,
    SERVER_REQUEST        = 1,
    SERVER_SEND_RSSI      = 2,    
} type_server_msg_type;

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
void build_mqtt_pkt_pub(char topic[], char data[], uint16_t src_port, 
                                    mqtt_pkt_t *mqtt_send_pkt, hdlc_pkt_t *pkt);

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
void build_mqtt_pkt_npub(char topic[], char data[], uint16_t src_port, 
                                    mqtt_pkt_t *mqtt_send_pkt, size_t mqtt_data_len, hdlc_pkt_t *pkt);

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
                                    mqtt_pkt_t *mqtt_send_pkt, hdlc_pkt_t *pkt);
/**
 * @brief      Process the received mqtt data
 *
 * @param      pkt       The received packet
 * @param      data_pkt  The perocessed data
 */
void process_mqtt_pkt(mqtt_pkt_t *pkt, mqtt_data_t *data_pkt);

#endif
