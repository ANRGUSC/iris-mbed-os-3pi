import sys, struct, serial
import readchar



if len(sys.argv) < 2:
   print "No device specified."
   print "Specify the serial port of the device you wish to connect to."
   print "Example:"
   print "   inquiryCommand.py Com12"
   print "or"
   print "   inquiryCommand.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# ser.isOpen()
   print 'Enter your commands below.\r\nInsert "x" to leave the application.'
   input=1
   while 1:
    # get keyboard input
      input = readchar.readchar()
      # Python 3 users
      # input = input(">> ")
      if input == 'x':
         ser.close()
         exit()
      else:
         ser.write(input + '\r\n')
