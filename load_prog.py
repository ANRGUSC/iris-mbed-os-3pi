from pyOCD.board import MbedBoard
import os, sys
import logging
import serial.tools.list_ports_linux as serial_tools
import glob

def serial_ports():
    ports = list(serial_tools.comports())
    for p in ports:
        if "MBED" in p.description:
            return p.device

logging.basicConfig(level=logging.INFO)

os.system("mbed compile");
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
cmd = "./pyterm -b 115200 -p %s" % serial_ports()
os.system(cmd)

# board.uninit()