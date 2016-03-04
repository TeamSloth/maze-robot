# import the necessary packages
from __future__ import print_function
#from pivideostream import PiVideoStream
from colorpvstream import ColorSepPVStream
from fps import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2
import serial
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=50000,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())
 
	
# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from `picamera` module...")
vs = ColorSepPVStream().start()
time.sleep(2.0)
fps = FPS().start()

ser = serial.Serial(

	#port='/dev/ttyACM0',
        port='/dev/ttyAMA0',
	baudrate = 57600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
	)
 
# loop over some frames...this time using the threaded stream
while fps._numFrames < args["num_frames"]:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=400)
	print(vs.readxOffset())
	if (vs.readxOffset() > 30):
		ser.write(" rt > ".encode("utf-8"))
	elif ((vs.readxOffset() < 30) and (vs.readxOffset() > -30)):
		ser.write(" st > ".encode("utf-8"))
	else:
		ser.write(" lt > ".encode("utf-8"))


 
	# check to see if the frame should be displayed to our screen
	if args["display"] > 0:
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
 
	# update the FPS counter
	fps.update()
 
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
