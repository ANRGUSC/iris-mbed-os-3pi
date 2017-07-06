# ANRG m3pi-mbed-os

Requirements:

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

# Run Instructions:

By default there is no main.cpp and thus the `mbed compile` command will NOT work.
To run a app, use the provided script `load_app.py`.
You can directly compile and load a particular app by using `load_app.py` e.g.,
``` python load_app.py app_files/hdld_test/ ``

Alternatively, to run a particular app manually:
- Go to the app folder under e.g., `app_files/hdlc_test/`

- Delete .mbedignore file in that folder

- Now you can use traditional `mbed compile -c` to compile the code

- After you are done, you can put back `.mbedignore` available from `app_files/ignore_file/`


