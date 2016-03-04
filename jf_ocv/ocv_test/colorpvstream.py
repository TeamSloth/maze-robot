# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import numpy as np
import cv2

class ColorSepPVStream:
	def __init__(self, resolution=(320, 240), framerate=20):
		# initialize the camera and stream
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture,
			format="bgr", use_video_port=True)

		# initialize the frame and the variable used to indicate
		# if the thread should be stopped
		self.frame = None
		self.stopped = False
		self.xOffset = 0

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame

			#lower = np.array([17,15,100], dtype = "uint8")
			#upper = np.array([50,56,200], dtype = "uint8")
			#greenLower = (40, 100, 6)
			#greenUpper = (64, 255, 255)

			greenLower1 = (160, 120, 6)
			greenUpper1 = (180, 255, 255)

			greenLower2 = (160, 120, 6)
			greenUpper2 = (180, 255, 255)


			#pts = deque(maxlen=args["buffer"])

			image = f.array
			#rot_image = np.rot90(image,2)
			image = np.rot90(image,2)

			#blurred = cv2.GaussianBlur(rot_image, (11, 11), 0)
			hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


			mask1 = cv2.inRange(hsv, greenLower1, greenUpper1)
			mask2 = cv2.inRange(hsv, greenLower2, greenUpper2)
			mask = cv2.bitwise_or(mask1,mask2)
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)
			output = cv2.bitwise_and(image, image, mask = mask)

			# find contours in the mask and initialize the current
			# (x, y) center of the ball
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
				cv2.CHAIN_APPROX_SIMPLE)[-2]
			center = None
		 
			# only proceed if at least one contour was found
			if len(cnts) > 0:
				# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and
				# centroid
				c = max(cnts, key=cv2.contourArea)
				((x, y), radius) = cv2.minEnclosingCircle(c)
				M = cv2.moments(c)
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				#need to calculate only the X offset from image center
				self.xOffset = center[0] - 160 #320 pixel wide image
				#print(self.xOffset)
		 
				# only proceed if the radius meets a minimum size
				if radius > 10:
					# draw the circle and centroid on the frame,
					# then update the list of tracked points
					cv2.circle(output, (int(x), int(y)), int(radius),
						(0, 255, 255), 2)
					cv2.circle(output, center, 5, (0, 0, 255), -1)
		 
			# update the points queue
			#pts.appendleft(center)
			#print(center)

			self.frame = output
			self.rawCapture.truncate(0)

			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return

	def read(self):
		# return the frame most recently read
		return self.frame

	def readxOffset(self):
		# return the frame most recently read
		return self.xOffset
 
	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True
