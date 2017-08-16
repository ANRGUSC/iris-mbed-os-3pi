#!/usr/bin/env python3
import sys, os, shutil
from pyOCD.board import MbedBoard
import logging
import serial.tools.list_ports_linux as serial_tools
import glob
import configparser
import argparse


# Parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("app_files", help="source file to compile")
parser.add_argument("-m", "--m3pi",
                        help = "Specifies which version ( 0 : default, 1: modified) of m3pi library to use, default is 0 (original)",
                        default = 0)

args = parser.parse_args()

# if len(sys.argv) < 2:
#     print("Please specify the test file to insert or 'remove' to remove the test code.")
#     print("Note that removing a test will not move the test file back into the "
#           "'tests/' directory. Please manually save your changes if you have made edits.")
#     print("Examples:")
#     print("python3 load_test.py app_files/hdlc_test/")
#     sys.exit()

app_path = args.app_files + '.mbedignore'
ignore_file = 'app_files/ignore_file/.mbedignore'

if args.m3pi == '0':
    m3pi_path = 'm3pi_library/original/.mbedignore'
elif args.m3pi == '1':
    m3pi_path = 'm3pi_library/modified/.mbedignore'
else:
    m3pi_path = 'm3pi_library/original/.mbedignore'

print m3pi_path


os.remove(app_path)
os.remove(m3pi_path)

def serial_ports(unique_id):
    ports = list(serial_tools.comports())
    for p in ports:
        if unique_id in p.hwid:
            return p.device


logging.basicConfig(level=logging.INFO)

os.system("mbed compile -c");

# os.remove("main.cpp") 
shutil.copy(ignore_file, app_path)
shutil.copy(ignore_file, m3pi_path)

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
