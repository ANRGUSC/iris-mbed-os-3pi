/* Include this file to create project-specific macros. Try to keep their use
in the application code, not inside libraries. */

/* keep this port under 255 because the application code is not ready to adapt
the new uint16_t port number */
#define THREAD1_PORT    200
#define RIOT_PORT		170


#define MAIN_THR_PORT   165
#define MQTT_PKT_TYPE   0xFF 
#define SUB_ACK			0xFD
#define PUB_ACK			0xFC
#define NULL_PKT_TYPE 	0xFE

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

#define RANGE_REQ_FLAG      0x12
#define RANGE_RDY_FLAG      0x34
#define RANGE_GO_FLAG       0x56

#define DEFAULT_ULTRASOUND_THRESH   25