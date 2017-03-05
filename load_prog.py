from pyOCD.board import MbedBoard
import os, sys
import logging
logging.basicConfig(level=logging.INFO)

os.system("mbed compile --profile mbed-os/tools/profiles/debug.json -c");
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

flash.flashBinary("./BUILD/LPC1768/GCC_ARM/pololu.bin")
print "pc: 0x%X" % target.readCoreRegister("pc")
#   pc: 0x10000000

target.reset()
# target.halt()
# print "pc: 0x%X" % target.readCoreRegister("pc")
# #   pc: 0xAAC

# board.uninit()