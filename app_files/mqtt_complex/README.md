# MQTT_TEST Description

This is a basic script for MQTT based HDLC. 
This code communicated with a mqtt thread on the Openmote side.
When the Openmote send a command ``MQTT_GO``, MBED can talk directly to a MQTT broker through the openmote.
Before you send a mqtt msg check for the `mqtt_go` flag. 
If it is SET, then it is okay to use mqtt communication.

##Set up the border router 

[border router link](https://github.com/ANRGUSC/anrg-riot/tree/develop/examples/emcute_border_router_3pi)

##Set up the MQTT_SN broker

[updated mosquitto.rsmb](https://github.com/ANRGUSC/mosquitto.rsmb)

##Get the server_script

Run using python3 
Haven't tested python 2.7
[server_script](https://github.com/ANRGUSC/anrg-riot/tree/mqtt_bootstrap/examples/MQTT_HDLC/tools) 

After running the script and when both openmotes have succesfully connected to the broker,
you will be prompted with a menu option

**NOTE: This script assumes you are connecting only 2 openmotes to the broker,
	edit the code to adjust this **

For any questions contact: dmdsouza@usc.edu
Responses might be delayed


