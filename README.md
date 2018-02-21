# ANRG m3pi-mbed-os

Requirements (TODO: try python3 and update this readme):

Install pip using python2 and script that can be found online:
`python2 get-pip.py`

Install mbed-cli:

`pip2 install mbed-cli`


Clone:

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

# Install Pre-reqs (also for debugging):

1. Upgrade firmware of the mbed https://os.mbed.com/handbook/Firmware-LPC1768-LPC11U24
2. Install python packages: `pip install -r requirements.txt`
3. Install python, libusb and libncursus (i386 to be compatible with arm-none-eabi-gdb)
    
        sudo apt-get install python libusb-1.0-0-dev libncurses5:i386

4. It might be necessary to update your USB settings to get non-root access to DAP:

        sudo sh -c 'echo SUBSYSTEM==\"usb\", ATTR{idVendor}==\"0d28\", ATTR{idProduct}==\"0204\", MODE:=\"666\" > /etc/udev/rules.d/mbed.rules' 
        sudo /etc/init.d/udev restart   

# Run Instructions:

By default there is no main.cpp and thus the `mbed compile` command will NOT work.
To run a app, use the provided script `load_app.py`.
You can directly compile and load a particular app by using `load_app.py` e.g.,
``` python load_app.py app_files/hdlc_test/ ```

Alternatively, to run a particular app manually:
- Go to the app folder under e.g., `app_files/hdlc_test/`

- Delete .mbedignore file in that folder

- Now you can use traditional `mbed compile -c` to compile the code

- After you are done, you can put back `.mbedignore` available from `app_files/ignore_file/`

# Debugging:

https://docs.google.com/document/d/1zdjeOYkmbfHqdnZFqP798h9WB3x69WeN0IaHEcKrWus/edit#

