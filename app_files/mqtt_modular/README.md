# MQTT_MODULAR Description

This is a code for 3pi to 3pi communication via MQTT. 
But in general this code enables controlling one mbed from another. 
This code communicates with a mqtt thread on the Openmote side.

There is an intial hadshaking sequence as follows:
* (1) The openmote sends a MQTT_GO msg once the mqtt connection is properly setup.
* (2) The MBED replies by sending a MQTT_GO_ACK msg to the Openmote
* (3) The Openmote sends the HWADDR to the mbed
* (4) The MBED replies by sending a HW_ADDR_ACK
After this sequece is complete, the mqtt is ready to go 

## Set up the border router 

[border router link](https://github.com/ANRGUSC/anrg-riot/tree/develop/examples/emcute_border_router_3pi)

## Setup mqtt_modular on the Openmote
The example mbed_riot/mqtt_modular should be running on the riot side for its proper functioning.

## Set up the MQTT_SN broker

[updated mosquitto.rsmb](https://github.com/ANRGUSC/mosquitto.rsmb)

## Get the server_script

Run using python3 
Haven't tested python 2.7
[server_script](https://github.com/ANRGUSC/anrg-riot/tree/mqtt_bootstrap/examples/MQTT_HDLC/tools) 

After running the script and when both openmotes have successfully connected to the broker,
you will be prompted with a menu option

**NOTE: This script assumes you are connecting only 2 openmotes to the broker,
	edit the code to adjust this **

For any questions contact: pradiptg@usc.edu, dmdsouza@usc.edu
Responses might be delayed


