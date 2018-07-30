# Interior Node Movement Description

This code is a draft for coordinate movement inside a trilateration system.

There is an intial hadshaking sequence as follows:
* (1) The openmote sends a MQTT_GO msg once the mqtt connection is properly setup.
* (2) The MBED replies by sending a MQTT_GO_ACK msg to the Openmote
* (3) The Openmote sends the HWADDR to the mbed
* (4) The MBED replies by sending a HW_ADDR_ACK
After this sequece is complete, the mqtt is ready to go.

It has three threads:
* (1) The main thread which can be extended to suit different purposes.
* (2) The MQTT thread which communicates with the OpenMote and the main thread.
* (3) The movement thread which handles the simple P-controller for movement.

## A running border router is necessary 

[border router link](https://github.com/ANRGUSC/anrg-riot/tree/develop/examples/emcute_border_router_3pi)

## Set up the MQTT_SN broker

[updated mosquitto.rsmb](https://github.com/ANRGUSC/mosquitto.rsmb)

## Setup mqtt_modular on the Openmote
The example mbed_riot/mqtt_modular should be running on the riot side for its proper functioning.

## Run the control script (controlINM.py) or individual MQTT/ROMANO commands:
mosquitto_pub -h localhost -p 1886 -t 06130622 -m "1test"  //This will have node 06130622 subscribe to test
mosquitto_pub -h localhost -p 1886 -t test -m "9c0:0" //This will give the nodes subscribed to test the initial location of (0,0)
mosquitto_pub -h localhost -p 1886 -t test -m "9t1:1" //This will tell nodes subscribed to test to go to the target location of (1,1)

For any questions contact: strawn.kegan@gmail.com


