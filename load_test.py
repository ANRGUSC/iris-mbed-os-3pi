#!/usr/bin/env python3
import sys, os, shutil
from pyOCD.board import MbedBoard
import logging
import serial.tools.list_ports_linux as serial_tools
import glob

if len(sys.argv) < 2:
    print("Please specify the test file to insert or 'remove' to remove the test code.")
    print("Note that removing a test will not move the test file back into the "
          "'tests/' directory. Please manually save your changes if you have made edits.")
    print("Examples:")
    print("python3 load_test.py tests/hdlc_test.c")
    sys.exit()

shutil.copy(sys.argv[1], "./main.cpp")


def serial_ports(unique_id):
    ports = list(serial_tools.comports())
    for p in ports:
        if unique_id in p.hwid:
            return p.device

logging.basicConfig(level=logging.INFO)

os.system("mbed compile -c");
os.remove("main.cpp") 

board = MbedBoard.chooseBoard()

target = board.target
flash = board.flash
target.resume()
target.halt()

# print "pc: 0x%X" % target.readCoreRegister("pc")
#    pc: 0xA64

# target.step()
# print "pc: 0x%X" % target.readCoreRegister("pc")
#    pc: 0xA30

# target.step()
# print "pc: 0x%X" % target.readCoreRegister("pc")
#   pc: 0xA32
filename=glob.glob('./BUILD/LPC1768/GCC_ARM/*.bin')


flash.flashBinary(filename[0])
print("pc: 0x%X" % target.readCoreRegister("pc"))
#   pc: 0x10000000

target.reset()

# target.halt()
# print "pc: 0x%X" % target.readCoreRegister("pc")
# #   pc: 0xAAC
# 
cmd = "./pyterm -b 115200 -p %s" % serial_ports(board.unique_id)

os.system(cmd)

# board.uninit())