# MQTT_RSSI Description

This is a code for controlling UDP messaging between 3pis via MQTT. 
But in general this code demonstrates how to use mqtt to implement different features for the Robotic Wireless Nodes.
In this example, we two m3pi robots sends UDP message to each other where the instruction to send the UDP message is communicated over MQTT.
Next, we briefly explain how the code works. 

There is an initial handshaking sequence between the mbed and the openmote in each 3pi as follows:
* (1) The openmote sends a MQTT_GO msg once the mqtt connection is properly setup.
* (2) The MBED replies by sending a MQTT_GO_ACK msg to the Openmote
* (3) The Openmote sends the HWADDR to the mbed
* (4) The MBED replies by sending a HW_ADDR_ACK
After this sequence is complete, the mqtt is ready to go 

Another key component of this setup is a server script connected to the mqtt broker.
Once both two 3pis are connected, the server scripts sends the lists of connected nodes to both the 3pis which stores these info.
This info consists of the 3pi specific topics (last eight character of their ipv6 address in this case) to control them directly.
* After receiving this info, one node starts the process by asking the server for a permission to send an UDP packet (`SERVER_SEND_RSSI`).
* Upon the request, the server grants the permission after some sequence check (`RSSI_SEND`).
* Upon receiving the `RSSI_SEND`, the node sends a UDP packet.
* When the other node receives the UDP packet, it sends a `SERVER_SEND_RSSI` request to the server for its turn to send the UDP packet.
* The server again replies with a `RSSI_SEND` to that node for its turn.
* This process continues.

In this example the UDP packets acts as the event triggers.
Thus to deal with missing UDP packets, there is a periodic request by each node.

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


