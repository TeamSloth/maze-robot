#!/usr/bin/env python
          
      
import time
import serial
          
      
ser = serial.Serial(

	port='/dev/ttyAMA0',
	baudrate = 57600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
	)
   #counter=0
          
      
while 1:
	ser.write("C10\r\n")
	time.sleep(1)
	ser.write("B12\r\n")
        time.sleep(1)
	#x = ser.readline()
	#print x
