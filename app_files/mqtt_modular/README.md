# MQTT_COMPLEX Description

This is a code for 3pi to 3pi communication via MQTT. 
But in general this code enables controlling one mbed from another. 
This code communicates with a mqtt thread on the Openmote side.
When the Openmote send a command ``MQTT_GO``, MBED can talk directly to a MQTT broker through the openmote.
Before you send a mqtt msg check for the `mqtt_go` flag. 
If it is SET, then it is okay to use mqtt communication.

##Set up the border router 

[border router link](https://github.com/ANRGUSC/anrg-riot/tree/develop/examples/emcute_border_router_3pi)

##Setup MQTT_HDLC on the Openmote
The example MQTT_HDLC should be running on the riot side for its proper functioning.

##Set up the MQTT_SN broker

[updated mosquitto.rsmb](https://github.com/ANRGUSC/mosquitto.rsmb)

##Get the server_script

Run using python3 
Haven't tested python 2.7
[server_script](https://github.com/ANRGUSC/anrg-riot/tree/mqtt_bootstrap/examples/MQTT_HDLC/tools) 

After running the script and when both openmotes have successfully connected to the broker,
you will be prompted with a menu option

**NOTE: This script assumes you are connecting only 2 openmotes to the broker,
	edit the code to adjust this **

For any questions contact: pradiptg@usc.edu, dmdsouza@usc.edu
Responses might be delayed


