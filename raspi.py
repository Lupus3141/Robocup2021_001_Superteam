#!/usr/bin/env python
# coding=utf-8

# To do:
# 
# Ecke und Ausgang RICHTIG finden und nicht premappen xD
# Nach dem Finden des Rescuekits richtig ausrichten und dann erst aufnehmen
# raspi kühler (aktiv)
# autostart von Linefollowerprogramm
# prüfen, ob auch wirklich eine Kugel aufgenommen wurde
# schnellere baudrate
# raspi übertakten
# Bei Lücke ein Stückchen in die richtige Richtung drehen (ein paar Werte, bevor weiß kam schauen, ob Linienpos rechts oder links war und dann ein Stück koriggieren)
# Dose umfahren und sich dabei nicht von anderen Linien irritieren lassen (neues ROI, ganz links am Kamerabild bzw. einfach alles rechts abschneiden)
# Silber erkennen verbessern
# Lebendes und totes Opfer unterscheiden

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import argparse
import time
import cv2
import serial
import random
import os
import math

CUT = (0, 320, 140, 192)

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout = 0.5) #establish serial connenction 


camera = PiCamera()
camera.resolution = (320, 192)
camera.rotation = 0
camera.framerate = 32
#rawCapture = PiRGBArray(camera, size=(320, 192))

while(not ser.is_open):
	print("Waiting for Serial...")
	time.sleep(0.1)
print("Opened:", ser.name, "aka Teensy 3.6") 


########## FUNCTIONS ##########
def mouseRGB(event, x, y, flags, param): #to adjust colour values eg for green dots
	if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
		colorsB = image_rgb[y, x, 0]
		colorsG = image_rgb[y, x, 1]
		colorsR = image_rgb[y, x, 2]
		colors = image_rgb[y, x]
		"""
		print("Red: ", colorsR)
		print("Green: ", colorsG)
		print("Blue: ", colorsB)
		print("BRG Format: ", colors)
		print("Coordinates of pixel: X: ", x,"Y: ", y)
		"""
		colour = np.uint8([[[colorsB, colorsG, colorsR]]])
		colour_hsv = cv2.cvtColor(colour, cv2.COLOR_BGR2HSV)
		print(colour_hsv)


def drive(motorLeft, motorRight, duration):
	send = str(motorLeft) + ':' + str(motorRight) + ':' + str(duration)
	print("Send:", send)
	ser.write(send.encode())
	duration = float(duration / 1000.0)
	time.sleep(0.1)
	while True: #waits for the teensy to execute the command
		readData = ser.readline().decode('ascii').rstrip()
		if readData == "1": 
			break

def turnRelative(deg):
	drive(0, 0, deg)

def armDown():
	sendAndWait("armDown")

def armUp():
	sendAndWait("armUp")

def sendAndWait(send): #sends command and waits for receiving the ok
	ser.write(send.encode())
	print("Send:", send)
	while True:
		readData = ser.readline().decode('ascii').rstrip()
		if readData == "1":
			break

def lineAdjust():
	angle = 180
	while abs(angle) > 2:
		print(angle)
		rawCapture = PiRGBArray(camera)
		camera.capture(rawCapture, format="bgr")
		image = rawCapture.array

		line = cv2.inRange(image, (0, 0, 0), (75, 75, 75))

		kernel = np.ones((4, 4), np.uint8)
		line = cv2.erode(line, kernel, iterations=3)
		line = cv2.erode(line, kernel, iterations=5)
		contours, hierarchy = cv2.findContours(line.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		if len(contours) > 0:
			rect = cv2.minAreaRect(contours[0])

			angle = rect[2]
			if(angle > 45):
				angle = angle - 90
			#print(angle)
			drive(130 + angle, 130 - angle, 0)

	rawCapture.truncate(0)
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		#print("Avg. FPS:", int(framesTotal / (time.time() - startTime - timeWaitet))) #sendet durchsch. Bilder pro Sekunde (FPS)
		camera.close()
		exit()


def rescueKit():
	rawCapture = PiRGBArray(camera)
	camera.capture(rawCapture, format="bgr")
	image = rawCapture.array
	image_rgb = image 

	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	image = cv2.GaussianBlur(image, ((9, 9)), 2, 2)

	rescuekit = cv2.inRange(image, (100, 150, 25), (145, 255, 255))

	kernel = np.ones((4, 4), np.uint8)
	rescuekit = cv2.erode(rescuekit, kernel, iterations=3)
	rescuekit = cv2.dilate(rescuekit, kernel, iterations=5)
	contours_rescuekit, hierarchy_rescuekit = cv2.findContours(rescuekit.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	if len(contours_rescuekit) > 0:	
		b = cv2.boundingRect(contours_rescuekit[0])
		x, y, w, h = b
		pos = x + w / 2 - 160

		#if(y < 120):
		#	drive(200, 200, 30)
		
		#print(pos)
		#if(abs(pos) > 4):
		#	turnRelative(pos / 4)

		#if y >= 120 and abs(pos) < 20:
		#	searching = False
		#	print("Nice")
		#	drive(-200, -200, 40)
		#	turnRelative(180)	
		#	drive(-200, -200, 70)
		#	armDown()
		#	armUp()
		#	return
		return (pos, y)
	else:
		return (300, 0)

	cv2.imshow("Rescuekit", rescuekit)
	cv2.imshow("Image", image_rgb)

	rawCapture.truncate(0)
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		print("Avg. FPS:", int(framesTotal / (time.time() - startTime - timeWaitet))) #sendet durchsch. Bilder pro Sekunde (FPS)
		camera.close()
		exit()

sendAndWait("setOrigin")

x = 0
y = 0
dirx = 1
diry = 0

hasRescueKit = False
goalTileX = 2
goalTileY = 1

#lineAdjust()

while True:
	#cv2.setMouseCallback("mouseRGB", mouseRGB)
	pos, cy = rescueKit()
	if pos != 300:
		#searchRescueKit()
		print("NICE")

		if(cy < 120):
			drive(255, 255, (130 - cy) * 1.5)
		if(abs(pos) > 10):
			turnRelative(pos / 4)

		if(cy > 115 and abs(pos) <= 10):
			drive(-200, -200, 50)
			turnRelative(190)
			drive(-200, -200, 100)
			armDown()
			drive(-150, 150, 50)
			drive(150, -150, 100)
			drive(-150, 150, 50)
			armUp()
			sendAndWait("turnToOrigin")

			turnRelative(math.atan2((goalTileY - y), (goalTileX - x)))
			h = math.sqrt(math.pow(goalTileX - x, 2) + math.pow(goalTileY - y, 2))
			drive(255, 255, h * 1000)
			sendAndWait("dropRescueKit")

	else:
		drive(255, 255, 1050)
		x = x + dirx
		y = y + diry
		
		print("X: " + str(x) + " Y: " + str(y))
		if(x != 3 and x != 0 and y != 1):
			sendAndWait("turnToOrigin")
		elif(y == 1):
			sendAndWait("turn180")

		if(x == 3 and y == 0):
			dirx = 0
			diry = 1

			drive(-200, -200, 100)
			sendAndWait("turn90")

			drive(255, 255, 50)
		elif(x == 3 and y == 1):
			dirx = -1
			diry = 0

			#turnRelative(90)
			sendAndWait("turn180")
			#drive(255, 255, 150)
		elif(x == 0 and y == 1):
			dirx = 0
			diry = 1

			#turnRelative(-90)
			sendAndWait("turn90")
			drive(-255, -255, 60)
		elif(x == 0 and y == 2):
			dirx = 1
			diry = 0

			#turnRelative(-90)
			sendAndWait("turnToOrigin")
			drive(255, 255, 125)
