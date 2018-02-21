#ifndef RANGE_H
#define RANGE_H

#include "data_conv.h"
#include "mqtt.h"
#include "rtos.h"
#include "hdlc.h"
#include "uart_pkt.h"
#include "main-conf.h"

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
#define MAX_NUM_ANCHORS   10     
#define DATA_STRING_SIZE  9

#define MISSED_PIN_UNMASK   13
#define RF_MISSED           20
#define ULTRSND_MISSED      21

#define MBED_RANGE_PORT 5678

#define NODE_DATA_FLAG 0
#define NODE_DISC_FLAG 1

#define LOAD_DISC_NODE_LENG 3
#define ID_LENGTH           9

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

/**
 * @brief      Loads the emcute_id to the buffer in range so it can be identified when publishing data
 *
 * @param      buff  The buffer containing the emcute_id
 */
void range_load_id(hdlc_buf_t *buff);

/**
 * @brief      Returns range data as a node_t
 *
 * @param[in]  data  The range data as a range_data_t struct
 *
 * @return     The data as a node_t.
 */
node_t get_node(range_data_t data);

/**
 * @brief      Loads a node data into a given buffer. Wrapper function for load_data.
 *
 * @param      buff       The buffer
 * @param[in]  buff_size  The buffer size
 * @param[in]  node       The node
 *
 * @return     { 0 if successful, -1 if not }
 */
int load_node_data(char *buff, size_t buff_size, node_t node);

/**
 * @brief      Loads all discovered nodes into a given buffer. Wrapper function for load_data.
 *
 * @param      buff       The buffer
 * @param[in]  buff_size  The buffer size
 *
 * @return     { 0 if successful, -1 if not }
 */
int load_discovered_nodes(char *buff, size_t buff_size);

/**
 * @brief      Loads data into a buffer for publishing.
 *
 * @param      buff       The data buffer
 * @param[in]  buff_size  The buffer size
 * @param[in]  node       The node_t containing the range data
 *
 * @return     returns 0 on success, otherwise returns -1 on failure
 */
int load_data(char *buff, size_t buff_size, node_t node, int flag);

/**
 * @brief      clears the data in the data buffer
 *
 * @param      buff       The data buffer
 * @param[in]  buff_size  The buffer size
 */
void clear_data(char *buff, size_t buff_size);

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
void discover_nodes(uint8_t ranging_mode);

/**
 * @brief      Looks for a specific node to range with based on what's given in params. This is a wrapper function for get range data.
 *
 * @param[in]  params  The ranging parameters
 *
 * @return     the range data
 */
range_data_t range_node(range_params_t params);



/**
 * @brief      initializes the range_thread
 */
void init_range_thread();

/**
 * @brief      triggers the range routine defined in the range_thread 
 *
 * @param      params  The ranging parameters
 * @param      msg     msg pointer used for triggering
 */
void trigger_range_routine(range_params_t *params, msg_t *msg);


/**
 * @brief      triggers the range routine defined in the range_thread 
 *
 * @param      params  The ranging parameters
 * @param      msg     msg pointer used for triggering
 */
void trigger_range_routine_blocking(range_params_t *params, msg_t *msg);

/**
 * @brief      Determines if the thread is currently ranging.
 *
 * @return     True if ranging, False otherwise.
 */
bool is_ranging();

/**
 * @brief      Gets the nodes that were discovered.
 *
 * @return     The pointer to an array of node ids.
 */
node_t* get_nodes_discovered();

/**
 * @brief      Gets the number nodes discovered.
 *
 * @return     The number of nodes discovered.
 */
uint8_t get_num_nodes_discovered();

#endif