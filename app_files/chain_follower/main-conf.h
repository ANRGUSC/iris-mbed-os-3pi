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

#define NETWORK_HELPER_PORT     7000 
#define NET_SLAVE_PORT          5001
#define RANGE_SLAVE_PORT        5002
#define RANGE_BEACONER_PORT     5003

#define RSSI_LOCALIZATION_CHAN   26

#define DEFAULT_ULTRASOUND_THRESH   25

#define ID_LENGTH       			9

#define ROBOT_MAX_SPEED  50
#endif
