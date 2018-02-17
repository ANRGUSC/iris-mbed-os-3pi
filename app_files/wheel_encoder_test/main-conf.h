/* Include this file to create project-specific macros. Try to keep their use
in the application code, not inside libraries. */

/* keep this port under 255 because the application code is not ready to adapt
the new uint16_t port number */
#ifndef MAIN_CONF_H
#define MAIN_CONF_H

#define MBED_MQTT_PORT  200
#define RIOT_MQTT_PORT  170
#define MAIN_THR_PORT   165
#define NULL_PKT_TYPE   0xFF

/* RSSI dump thread port number */
#define RSSI_DUMP_PORT              9000

#define GET_SET_RANGING_THR_PORT    9100

/* for ARREST, set follower's short hwaddr here */
#define ARREST_FOLLOWER_SHORT_HWADDR    "ec:e1"
#define ARREST_LEADER_SHORT_HWADDR      "ed:81"

#define RSSI_LOCALIZATION_CHAN   26

#define ARREST_LEADER_SOUNDRF_IPV6_ADDR   "fe80::212:4b00:433:ed81"

#define ARREST_LEADER_SOUNDRF_PORT  9200

#define ARREST_LEADER_SOUNDRF_ID    170

#define ARREST_FOLLOWER_RANGE_THR_PORT  9300

#define DEFAULT_ULTRASOUND_THRESH   25

#define ONE_SENSOR_MODE       0x60 // 96
#define TWO_SENSOR_MODE       0x61 // 97
#define XOR_SENSOR_MODE       0x62 // 98
#define OMNI_SENSOR_MODE      0x63 // 99

#define RANGE_THR_START         0x63
#define RANGE_THR_COMPLETE      0x64

#define RANGE_DATA_LEN    6

#define MISSED_PIN_UNMASK   13
#define RF_MISSED         20
#define ULTRSND_MISSED    21

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

#endif
