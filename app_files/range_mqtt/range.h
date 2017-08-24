#ifndef RANGE_H
#define RANGE_H

#include "m3pi/modified/mbed_movement.h"
#include "data_conv.h"
#include "mqtt.h"
#include "rtos.h"
#include "hdlc.h"
#include "uart_pkt.h"

#define RANGE_TOPIC   ("range_info")

#define ONE_SENSOR_MODE       0x60 // 96
#define TWO_SENSOR_MODE       0x61 // 97
#define XOR_SENSOR_MODE       0x62 // 98
#define OMNI_SENSOR_MODE      0x63 // 99

#define RANGE_THR_START         0x63
#define RANGE_THR_COMPLETE      0x64

#define RANGE_DATA_LEN    6

#define RANGE_PORT 5678

#define START_RANGE_THR 139

#define DATA_PER_PKT        ((HDLC_MAX_PKT_SIZE - UART_PKT_HDR_LEN - 1) / RANGE_DATA_LEN)
#define MAX_NUM_ANCHORS 20     
#define DATA_STRING_SIZE  9

#define MISSED_PIN_UNMASK   13
#define RF_MISSED         20
#define ULTRSND_MISSED    21

#define MBED_RANGE_PORT 5678

/**
 * @brief Structure holding metrics measured by ultrasound ranging
 *
 * This structure is supposed to hold the Time Difference of Arrival
 * (TDoA), Orientation Differential (OD) between the TDoA of two sensors,
 * and any an pin flag to indicate which pin came first and if a pin had missed a ping.
 *
 * It can be extended
 */
typedef struct __attribute__((packed)){
    uint16_t tdoa; //time difference of arrival
    uint16_t orient_diff; //orientation differential
    uint8_t status; //pin flag to indicate which pin came first and if a pin had missed a ping
    int8_t node_id;
} range_data_t;

typedef struct __attribute__((packed)) {
    uint8_t         last_pkt;      
    range_data_t    data[DATA_PER_PKT];                  
} range_hdr_t;

/**
 * @brief Structure holding parameters for ultrasound ranging
 *
 * This structure is supposed to be used to send and interpret
 * range request packets between mbed and openmote
 *
 * It can be extended
 */
typedef struct __attribute__((packed)){
    int8_t node_id;
    uint8_t ranging_mode;
    // add more options in the future?
} range_params_t;

typedef struct __attribute__((packed)) {
    int8_t         node_id;      
    uint16_t       tdoa;                  
} node_t;

static uint8_t num_nodes_to_pub;

/**
 * @brief      Returns range data as a node_t
 *
 * @param[in]  data  The range data as a range_data_t struct
 *
 * @return     The data as a node_t.
 */
node_t get_node(range_data_t data);

/**
 * @brief      Loads data into a buffer for publishing.
 *
 * @param      buff       The data buffer
 * @param[in]  buff_size  The buffer size
 * @param[in]  node       The node_t containing the range data
 *
 * @return     returns 0 on success, otherwise returns -1 on failure
 */
int load_data(char *buff, int buff_size, node_t node);

/**
 * @brief      clears the data in the data buffer
 *
 * @param      buff       The data buffer
 * @param[in]  buff_size  The buffer size
 */
void clear_data(char *buff, int buff_size);

/**
 * @brief      Gets the distance in feet and angle in degrees.
 *
 * @param      time_diffs    The range data
 * @param[in]  ranging_mode  The ranging mode
 *
 * @return     The distance and angle in a dist_angle_t struct.
 */
dist_angle_t get_dist_angle(range_data_t *time_diffs, uint8_t ranging_mode);

/**
 * @brief      Gets the range data by communicating with the openmote over hdlc.
 *
 * @param[in]  params  The parameters for ranging
 *
 * @return     The range data.
 */
range_data_t get_range_data(range_params_t params);

/**
 * @brief      Discovers all nodes that can be ranged with. This is a wrapper function for get range data. The data is stored in a
 *             static array of node_t data
 *
 * @param[in]  ranging_mode  The ranging mode
 */
void range_all(uint8_t ranging_mode);

/**
 * @brief      Looks for a specific node to range with based on what's given in params. This is a wrapper function for get range data.
 *
 * @param[in]  params  The ranging parameters
 *
 * @return     the range data
 */
range_data_t range_node(range_params_t params);
range_data_t lock_on_anchor(int8_t node_id);

void init_range_thread();
void trigger_range_routine(range_params_t *params, msg_t *msg);
bool is_ranging();
node_t* get_nodes_reached();
uint8_t get_num_nodes_reached();

#endif