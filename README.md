# ANRG m3pi-mbed-os

Requirements:

Install pip using python2 and script that can be found online:
`python2 get-pip.py`

Install mbed-cli:

`pip2 install mbed-cli`

Initializing mbed project after cloning this repo:

`cd m3pi-mbed-os`

`mbed deploy`

`mbed new .` #(not too sure about this line)





# Description

This is a basic script for MQTT based HDLC. 
This code communicated with a mqtt thread on the Openmote side.
When the Openmote send a command ``MQTT_GO``, MBED can talk directly to a MQTT broker through the openmote.
Before you send a mqtt msg check for the `mqtt_go` flag. 
If it is SET, then it is okay to use mqtt communication.

