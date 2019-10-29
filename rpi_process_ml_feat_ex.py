#!/usr/bin/env python
import time
import serial
import RPi.GPIO as GPIO
import struct
import threading
#client side
from Crypto.Cipher import AES
from Crypto import Random
#from Crypto.Util.Padding import pad
import hashlib
import base64
import socket
import sys
import binascii
import pandas as pd
import os
import numpy as np
import pickle
from joblib import load
from collections import deque

#initialise serial port
port = "/dev/ttyS0"
ser = serial.Serial(port, baudrate=115200)

# packet/device ids
nack = 0
ack = 1
message = 3
nodevice = 0
hand = 1
forearm = 2
back = 3

timestamp = 0
# serial comm variables
receiveBuffer = []
bufferLengthLimit = 15

#test handshake
HELLO = ('H').encode()
ACK = ('A').encode()
NACK = ('N').encode()
READY = ('R').encode()

#Initialise server
bs = 32; #base_size
key = "1234567890123456"
x = 0


# Initialize ML global variables
MODEL = load('rf24_v2rocket.joblib')
#window_size = 30
max_consecutive_agrees = 10
reverse_label_map = {
        0: "bunny",
        1: "cowboy",
        2: "handmotor",
        3: "rocket",
        4: "tapshoulders",
        5: "hunchback",
	99: "idle"
}

# End ML global variables


#if len(sys.argv) != 3 :
#    print('Invalid number of arguments')
#    print('python3 myclient.py [IP address] [Port]')
#    sys.exit()
# host = 'localhost'
# PORT_NUM = 7654
host = sys.argv[1]
PORT_NUM = int(sys.argv[2])

# Passes in a list of list (Size of list according to window size)
# Returns an appended list of feature_extracted values E.g [Mean1, Mean2, ...., Min1, Min2, ....., Max1, Max2, ...., SD1, ....]
def feature_extraction(window_rows):
    ## winows_rows are list of list

    feature_extracted_row = []
    feature_extracted_row.extend(window_rows.mean(0))
    feature_extracted_row.extend(window_rows.min(0))
    feature_extracted_row.extend(window_rows.max(0))
    feature_extracted_row.extend(window_rows.std(0))

    return feature_extracted_row

def encryptText(plainText, key):
    raw = pad(plainText)
    iv = Random.new().read(AES.block_size)
    cipher = AES.new(key.encode("utf8"),AES.MODE_CBC,iv)
    msg = iv + cipher.encrypt(raw.encode('utf8'))
    # msg = msg.strip()
    return base64.b64encode(msg)

def pad(var1):
    return var1 + (bs - len(var1)%bs)*chr(bs - len(var1)%bs)

def connectToServer(ip_addr, port_num):
	global sock
	print("Initialising the socket..")
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	print("Connecting to server...")
	try:
		print("Connect to server success")
		sock.connect((ip_addr, port_num))
	except:
        	print("Connection to Server failed.")
        	return False
	return True

def calculateChecksum(readingsarr, voltage, current, power, energy):
	checksum = 0

	for i in range(3, len(readingsarr)):
		readingInt = int(readingsarr[i])
		checksum = int(checksum) ^ readingInt

	# checksum = checksum ^ int(voltage)
	# checksum = checksum ^ int(current)
	# checksum = checksum ^ int(power)
	# checksum = checksum ^ int(energy)

	return int(checksum)

def handshake():
    print("InitiateHandshake")
    ser.write(HELLO)
    time.sleep(1)
    if ser.in_waiting > 0:
        reply = ser.read().decode()
        print(reply)
        if(reply == 'A'):
            ser.write(ACK)
            print('Handshake Complete')
            return True
        else:
            print('pending')
    return False

# def handshake():
# 	# send hello packet to Arduino
# 	x = ser.write(b'\x01\x02\x02')

# 	# receive ack packet from Arduino
# 	packetid = 0
# 	checksum = -1

# 	while 1:
# 		print("waiting for ack from duino")
# 		if ser.in_waiting:
# 			receivedString = ser.read_until().decode("utf-8")
# 			break
# 	receivedString = receivedString[:-1]
# 	packetid = int(receivedString[0])
# 	checksum = int(receivedString[1])

# 	if packetid == ack:
# 		if packetid != checksum:
# 			return 0
# 	elif packetid != ack:
# 		return 0

# 	print("ack received")

# 	# time1 = time.time()
# 	# time2 = time.time()

# 	# while (time2 - time1) < 1:
# 	# 	time2 = time.time()
# 	# send ack packet to Arduino
# 	ser.write(b'\x01\x01\x01')

# 	return 1

def storeInBuffer():
	while 1:
		if ser.in_waiting:
			receiveBuffer.append(ser.read_until().decode("utf-8"))

def formMessage(action, voltage, current, power, cumPower):
	message = "#" + action + "|" + str(format(voltage, '.2f')) + "|" + str(format(current, '.2f')) + "|" + str(format(power, '.2f')) + "|" + str(format(cumPower, '.2f')) + "|"
	return message.strip()

#send data to server
def sendToServer(action, voltage, current, power, cumPower):
    messageToSend = formMessage(action, voltage, current, power, cumPower)

    stringToSend = encryptText(messageToSend, key)
    sock.send(stringToSend) #need to encode as a string as it is what is expected on server side

def receiveSensorData():
	packetid = -1
	deviceid = -1
	deviceid2 = -1
	deviceid3 = -1
	checksum = -1
	global timestamp
	global csv_data
	csv_data = pd.DataFrame()
	timeBefore = time.time()
	numReceivedString = 0
	numCorrectData = 0

	# initialize ml variables
	ml_buffer = deque()
	current_prediction = None
	consecutive_agrees = 0
	window_size = 24
	predictionDelay = 0.5
	print("receive sensor data")

	while 1:
#		print("in receive sensor data loop\n")
		isCheckSumSuccess = False
		voltage = 0
		current = 0
		power = 0
		cumPower = 0
		if ser.in_waiting > 0:
			try:
				# receivedString = ser.read_until().decode("utf-8")
				receivedString = ser.readline().decode("utf-8")
			except Exception as e:
				print(e)
				print("error")
				continue
			#print(len(receiveBuffer))
			#receivedString = receiveBuffer.pop(0)
			numReceivedString = numReceivedString + 1
			receivedString = receivedString[:-1]
			receivedString = receivedString.lstrip('#')
			try:
				# parse ids
				ids = receivedString.split('(')[0]
				packetid = int(ids[0])
				deviceid = int(ids[1])
				deviceid2 = int(ids[2])
				deviceid3 = int(ids[3])

				if packetid == message:
					if  deviceid != nodevice and deviceid2 != nodevice and deviceid3 != nodevice:
						handsensor = receivedString.split('(')[1]
						forearmsensor = receivedString.split('(')[2]
						backsensor = receivedString.split('(')[3]
						voltage = float(receivedString.split('(')[4])
						current = float(receivedString.split('(')[5])
						power = float(receivedString.split('(')[6])
						cumPower = float(receivedString.split('(')[7])
						checksumArduino =int(receivedString.split('(')[8])
						# print("checksum arduino")
						# print(checksumArduino)
						# print(voltage)
						# print(current)
						# print(power)
						# print(energy)

						handsensorAccx = float(handsensor.split(';')[0])
						handsensorAccy = float(handsensor.split(';')[1])
						handsensorAccz = float(handsensor.split(';')[2])
						handsensorGyrox = float(handsensor.split(';')[3])
						handsensorGyroy = float(handsensor.split(';')[4])
						handsensorGyroz = float(handsensor.split(';')[5])

						forearmsensorAccx = float(forearmsensor.split(';')[0])
						forearmsensorAccy = float(forearmsensor.split(';')[1])
						forearmsensorAccz = float(forearmsensor.split(';')[2])
						forearmsensorGyrox = float(forearmsensor.split(';')[3])
						forearmsensorGyroy = float(forearmsensor.split(';')[4])
						forearmsensorGyroz = float(forearmsensor.split(';')[5])

						backsensorAccx = float(backsensor.split(';')[0])
						backsensorAccy = float(backsensor.split(';')[1])
						backsensorAccz = float(backsensor.split(';')[2])
						backsensorGyrox = float(backsensor.split(';')[3])
						backsensorGyroy = float(backsensor.split(';')[4])
						backsensorGyroz = float(backsensor.split(';')[5])

						readingsArr = []
						#timestamp = timestamp + 2000

						timestamp = time.time()
						readingsArr.append(0)
						readingsArr.append(timestamp)
						readingsArr.append(timestamp * (10 ** 6))
						readingsArr.append(handsensorAccx)
						readingsArr.append(handsensorAccy)
						readingsArr.append(handsensorAccz)
						readingsArr.append(handsensorGyrox)
						readingsArr.append(handsensorGyroy)
						readingsArr.append(handsensorGyroz)
						readingsArr.append(forearmsensorAccx)
						readingsArr.append(forearmsensorAccy)
						readingsArr.append(forearmsensorAccz)
						readingsArr.append(forearmsensorGyrox)
						readingsArr.append(forearmsensorGyroy)
						readingsArr.append(forearmsensorGyroz)
						readingsArr.append(backsensorAccx)
						readingsArr.append(backsensorAccy)
						readingsArr.append(backsensorAccz)
						readingsArr.append(backsensorGyrox)
						readingsArr.append(backsensorGyroy)
						readingsArr.append(backsensorGyroz)

						# collecting data for ML buffer
						ml_snapshot = [
							handsensorAccx, handsensorAccy, handsensorAccz,
							handsensorGyrox, handsensorGyroy, handsensorGyroz,
							forearmsensorAccx, forearmsensorAccy, forearmsensorAccz,
							forearmsensorGyrox, forearmsensorGyroy, forearmsensorGyroz,
							backsensorAccx, backsensorAccy, backsensorAccz,
							backsensorGyrox, backsensorGyroy, backsensorGyroz
						]
						ml_buffer.append(np.array(ml_snapshot))

						# checksumPi = calculateChecksum(readingsArr, voltage, current, power, energy)
						# print("checksum pi")
						# print(checksumPi)
						isCheckSumSuccess = True
						# if checksumPi != checksumArduino:
						# 	print("checksum wrong!")
						# 	isCheckSumSuccess = False

						ser.write(ACK)
						readingsarr = np.array(readingsArr)
						readingsarr = readingsarr.reshape(-1, len(readingsarr))
						csv_data = csv_data.append(pd.DataFrame(readingsarr))
						#print(readingsArr)
						# print(voltage)
						# print(current)
						# print(power)
						# print(energy)
						numCorrectData = numCorrectData + 1
						# else:
						# 	print("checksum wrong")
						# 	# send nack to arduino
						# 	ser.write(b'\x01\x00\x00')
						# 	#npdata = np.array(data)
				else:
					print("Received packet not message")
					continue

			except ValueError or IndexError:
				print("Error while parsing received string!")
				continue
		# else:
		# 	print("nothing in serial")

		#if (time.time() - timeBefore) > 40:
		#	print("40 seconds over!")
		#	print(numReceivedString)
		#	print(numCorrectData)
		#	csv_data.to_csv('sensor_data.csv', index = None, header=False)
		#	break

		# Start machine learning logic
		#print(len(ml_buffer))
		if len(ml_buffer) >= window_size:
			feature_vector = np.concatenate(ml_buffer).reshape(1, -1)
			prediction = MODEL.predict(feature_vector)[0]
			#print("quick prediction")
			#print(prediction)
			ml_buffer.clear()
			if current_prediction is None or prediction == current_prediction:
				consecutive_agrees += 1
				if consecutive_agrees == max_consecutive_agrees:
					action = reverse_label_map[prediction]
					if action != 'idle':
						print("Predicted move is {}".format(action))
						consecutive_agrees = 0
						if (time.time() - timeBefore) >= 55:
							sendToServer(action, voltage, current, power, cumPower)
						time.sleep(predictionDelay)
			else:
				consecutive_agrees = 0
			current_prediction = prediction

# # handshake with Arduino
isHandShakeSuccessful = handshake()

# if isHandShakeSuccessful == 1:
# 	print("handshake success")
# elif isHandShakeSuccessful == 0:
# 	print("handshake failed")


# if (isHandShakeSuccessful == 1) and connectToServer('localhost', 7654):
# 	# t1 = threading.Thread(target=storeInBuffer)
# 	# t2 = threading.Thread(target=receiveSensorData)

# 	# t1.start()
# 	# t2.start()

# 	# while 1:
# 	# 	#pass
# 	receiveSensorData()
if isHandShakeSuccessful and connectToServer(host, PORT_NUM):
	print("before receive sensor data")
	receiveSensorData()
	# timeBefore = time.time()
	# numStrings = 0
	# while 1:
	# 	if (time.time() - timeBefore) > 10:
	# 		print(numStrings)
	# 		break
	# 	if ser.in_waiting > 0:
	# 		try:
	# 			# receivedString = ser.read_until().decode("utf-8")
	# 			receivedString = ser.readline().decode("utf-8")
	# 			print(receivedString)
	# 			numStrings = numStrings + 1
	# 		except Exception as e:
	# 			print(e)
	# 			print("error")
	# 			continue
	# 	else:
	# 		print("nothing in serial")
