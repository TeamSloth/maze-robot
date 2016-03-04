#!/usr/bin/env python
import time
import serial

counter=0

ser = serial.Serial(

	#port='/dev/ttyACM0',
        port='/dev/ttyAMA0',
	baudrate = 57600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
	)



while 1:
	#ser.write('Write counter: %d \n'%(counter))
	#cmdOut = byterray['rw 200']
	#ser.write(cmdOut)

	ser.write(" rw 200 > ".encode("utf-8"))
	x=ser.readline()
	print(x)
	time.sleep(.2)
	ser.write(" rw 0 > ".encode("utf-8"))
	time.sleep(.2)
	ser.write(" lw 200 > ".encode("utf-8"))
	time.sleep(.2)
	ser.write(" lw 0 > ".encode("utf-8"))
	time.sleep(.2)
	#counter += 1
	#print(counter)
