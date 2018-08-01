# MQTT_MOV_MODULAR Description


# How to set up a broker, a border router, an OpenMote, and a Mbed working together.      

Change directory into anrg-riot/examples     
emcute_border_router_3pi and emcute_client_3pi will be used

First make sure that the two previous example’s Makefiles have their `DEFAULT_CHANNEL` set to 26. 

     

### Clone the Really Small Mosquitto Broker (RSMB)     
Open a second terminal and change directories into the folder where you want to store your broker src files. 

Run git clone to download the RSMB     
`git clone https://github.com/eclipse/mosquitto.rsmb.git`

Change directories down into `mosquitto.rsmb/rsmb/src`     
     
Run `make`     
     
In this same directory make a file call `config.conf`   
We will be running the RSMB as capable of both MQTT and MQTT-SN. Port 8888 will be used for MQTT-SN and 1886 for MQTT. We are also enabling IPv6. 

The contents of `config.conf`:    
```
# add some debug output
trace_output protocol

# listen for MQTT-SN traffic on UDP port 8888
listener 8888 INADDR_ANY mqtts
  ipv6 true

# listen to MQTT connections on tcp port 1886
listener 1886 INADDR_ANY
  ipv6 true
```

### Run the RSMB with our config file    
`./broker_mqtts config.conf`       
      
The output should be similar to this:     
(See the onboarding/documentation document on the google drive)      


### Setup an OpenMote as the Emcute Border Router     
Back in the original terminal or in an additional one; change directories into emcute_border_router_3pi.        
     
Connect the OpenMote to a USB port and a J-link to flash the code over.    

You can make sure you have the correct USB selected by unplugging and re-plugging in the OpenMote on the OpenUSB or OpenBase to the USB port, then running `dmesg | grep tty` to see either ttyUSBx (Mote) or ttyACMx (Mbed).  For instance, if this is the only usb plugged in you will see ttyUSB0 or ttyACM0.

`make BOARD=openmote-cc2538 PROGRAMMER=jlink PORT=/dev/ttyUSB1 flash term`  

The Makefile automatically creates a tap interface with the legendary address `fd00:dead:beef::1`     
     
The OpenMote will automatically connect to the tap interface at fd00:dead:beef::1 unless changed to do otherwise. To test the code run `ping6 fd00:dead:beef::1` from the openmote terminal and you should receive a response. 

Flash the code in ../emcute_border_router to an openmote
The make script automatically creates a tap interface with address fd00:dead:beef::1
The openmote automatically connects to the tap interface at fd00:dead:beef::1, unless changed
To test the code, run ping6 fd00:dead:beef::1 from the openmote terminal and you should receive a response (3 packets transmitted, 3 received).      

### Set up an OpenMote client     
Switch the j-link over and find the correct number x for port selection (ttyUSBx).      
Switch directories to anrg-riot/examples/mbed_riot     
     
Switch the selected app in `anrg-riot/examples/mbed_riot/Makefile` to mqtt_modular     

    
      
Run:     
`make PORT=/dev/ttyUSBx flash term`     
         
### Flash the MBED     
If you have not yet done so, download a clone of:     
`https://github.com/ANRGUSC/m3pi-mbed-os.git`     
You can do so by running these commands to also set up the `mbed` environment:     
Using mbed-cli:

```
mbed import git@github.com:ANRGUSC/m3pi-mbed-os.git
cd m3pi-mbed-os.git
mbed toolchain GCC_ARM
mbed target LPC1768
```

Using git:

Initializing mbed project after cloning this repo:

```
git clone git@github.com:ANRGUSC/m3pi-mbed-os.git
cd m3pi-mbed-os
mbed deploy
mbed new .
mbed toolchain GCC_ARM
mbed target LPC1768
```
<!-- `cd m3pi-mbed-os`

`mbed deploy`

`mbed new .` #(not too sure about this line)

 -->

##### Install Pre-reqs (also for debugging):

1. Upgrade firmware of the mbed https://os.mbed.com/handbook/Firmware-LPC1768-LPC11U24
2. Install python packages: `pip install -r requirements.txt`
3. Install python, libusb and libncursus (i386 to be compatible with arm-none-eabi-gdb)
    
        sudo apt-get install python libusb-1.0-0-dev libncurses5:i386

4. It might be necessary to update your USB settings to get non-root access to DAP:

        sudo sh -c 'echo SUBSYSTEM==\"usb\", ATTR{idVendor}==\"0d28\", ATTR{idProduct}==\"0204\", MODE:=\"666\" > /etc/udev/rules.d/mbed.rules' 
        sudo /etc/init.d/udev restart   

#### Run Instructions:

By default there is no main.cpp and thus the `mbed compile` command will NOT work.
To run a app, use the provided script `load_app.py`.
You can directly compile and load a particular app by using `load_app.py` e.g.,
``` python load_app.py app_files/hdlc_test/ ```

Alternatively, to run a particular app manually:
- Go to the app folder under e.g., `app_files/hdlc_test/`

- Delete .mbedignore file in that folder

- Now you can use traditional `mbed compile -c` to compile the code

- After you are done, you can put back `.mbedignore` available from `app_files/ignore_file/`

#### Flash the code over using the load_app.py script     
`python load_app.py app_files/mqtt_modular/`    
(make sure to add the forward slash onto the end)     
     
### Testing     
`sudo apt install python3-pip`      
`pip3 install paho-mqtt python-etcd numpy`     
Run tools/server_script.py:     
`python3 server_script.py`     
      
You should be able to choose option 1 and see the client listed.     
You can open another terminal and run:
```
mosquitto_pub -h localhost -p 1886 -t <hw_address_here_as_topic>  -m “0helloworld”
mosquitto_pub -h localhost -p 1886 -t <hw_address_here_as_topic>  -m “2test”
mosquitto_pub -h localhost -p 1886 -t <hw_address_here_as_topic>  -m “1test”
mosquitto_pub -h localhost -p 1886 -t test  -m “0itworked”    
```

### Now to try moving the 3Pi, use the mqtt_mov_modular app     
#### Flash the code over using the load_app.py script     
`python load_app.py app_files/mqtt_mov_modular/`    
And use the same mosquitto_pub messaging, but send to one device “8w” to move forward, or a for left, or s for back, or d for right. 
#### You may have to adjust m3pi-mbed-os/mqtt.h so that around lines 100-114 the type_mqtt_data_t enum has RMT_CTRL, as this example was created by merging remote control and mqtt_modular. 
