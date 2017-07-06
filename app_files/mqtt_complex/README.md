# MQTT_TEST Description

This is a basic script for MQTT based HDLC. 
This code communicated with a mqtt thread on the Openmote side.
When the Openmote send a command ``MQTT_GO``, MBED can talk directly to a MQTT broker through the openmote.
Before you send a mqtt msg check for the `mqtt_go` flag. 
If it is SET, then it is okay to use mqtt communication.
