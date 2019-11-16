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
import itertools
from sklearn.metrics import mean_squared_error
from scipy.stats import skew

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

#handshake variables
HELLO = ('H').encode()
ACK = ('A').encode()
NACK = ('N').encode()
READY = ('R').encode()

#Initialise server
bs = 32; #base_size
key = "1234567890123456"
x = 0

# Initialize ML global variables
MODEL = load('rf_5_features.joblib')
MLP_MODEL = None
SCALER_MODEL = load('mlp_scaler.joblib')

reverse_label_map = {
		0: "bunny",
		1: "cowboy",
		2: "handmotor",
		3: "rocket",
		4: "tapshoulders",
		5: "hunchback",
		6: "jamesbond",
		7: "chicken",
		8: "movingsalute",
		9: "whip",
		10: "logout"
}

move_confidence = {
	"bunny": [0.7, 0.19],
	"cowboy": [0.7, 0.3],
	"handmotor": [0.7, 0.3],
	"rocket": [0.7, 0.3],
	"tapshoulders": [0.7, 0.3],
	"hunchback": [0.7, 0.3],
	"jamesbond": [0.7, 0.2],
	"chicken": [0.7, 0.3],
	"movingsalute": [0.7, 0.3],
	"whip": [0.7, 0.2],
	"logout": [0.7, 0.3]
}

host = sys.argv[1]
PORT_NUM = int(sys.argv[2])

def extract_poly_fit(channel_features):
	poly_coeff = np.polyfit(range(len(channel_features)), channel_features, 1)
	return poly_coeff.flatten()


def extract_skewness(channel_features):
	skewness = skew(channel_features, axis=0)
	return skewness


def extract_average_amplitude_change(channel_features):
	amplitude_changes = []
	for i in range(0, len(channel_features)-1):
		amplitude_changes.append(np.abs(channel_features[i+1]-channel_features[i]))
	return np.mean(amplitude_changes, axis=0)


def extract_average_moving_rms(channel_features):
	moving_rms = []
	for i in range(0, len(channel_features)-1):
		moving_rms.append(mean_squared_error(channel_features[i+1],
											 channel_features[i], multioutput='raw_values'))
	return np.mean(moving_rms, axis=0)


def feature_extraction(window_rows):
	feature_extracted_row = []
	feature_extracted_row.extend(window_rows[0])
	feature_extracted_row.extend(window_rows.mean(0))
	feature_extracted_row.extend(extract_average_moving_rms(window_rows))
	feature_extracted_row.extend(window_rows.min(0))
	feature_extracted_row.extend(window_rows.max(0))
	feature_extracted_row.extend(window_rows.std(0))

	return feature_extracted_row

def encryptText(plainText, key):
	raw = pad(plainText)
	iv = Random.new().read(AES.block_size)
	cipher = AES.new(key.encode("utf8"),AES.MODE_CBC,iv)
	msg = iv + cipher.encrypt(raw.encode('utf8'))
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

	for i in range(len(readingsarr)):
		readingInt = int(readingsarr[i])
		checksum = int(checksum) ^ readingInt

	checksum = checksum ^ int(voltage)
	checksum = checksum ^ int(current)
	checksum = checksum ^ int(power)
	checksum = checksum ^ int(energy)

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

def formMessage(action, voltage, current, power, cumPower):
	message = "#" + action + "|" + str(format(voltage, '.2f')) + "|" + str(format(current, '.2f')) + "|" + str(format(power, '.2f')) + "|" + str(format(cumPower, '.2f')) + "|"
	return message.strip()

#send data to server
def sendToServer(action, voltage, current, power, cumPower):
	messageToSend = formMessage(action, voltage, current, power, cumPower)

	stringToSend = encryptText(messageToSend, key)
	sock.send(stringToSend) #need to encode as a string as it is what is expected on server side


def predict_ml(trained_model, trained_mlp, scaler_model, input_buffer):
	input_feature_vector = np.concatenate(input_buffer)
	prediction_confidences = trained_model.predict_proba(input_feature_vector.reshape(1, -1))[0]
	prediction = np.argmax(prediction_confidences)
	confidence = prediction_confidences[prediction]
	predictions, confidences = [prediction], [confidence]
	if trained_mlp is not None:
		scaled_input_feature_vector = scaler_model.transform(input_feature_vector.reshape(1, -1))
		mlp_prediction_confidences = trained_mlp.predict_proba(scaled_input_feature_vector)[0]
		mlp_prediction = np.argmax(mlp_prediction_confidences)
		mlp_confidence = prediction_confidences[prediction]
		predictions.append(mlp_prediction)
		confidences.append(mlp_confidence)
	return predictions, confidences


def receiveSensorData():
	packetid = -1
	deviceid = -1
	deviceid2 = -1
	deviceid3 = -1
	checksum = -1
	global timestamp
	timeBefore = time.time()
	last_prediction_time = time.time()
	numReceivedString = 0

	# initialize ml variables
	ml_buffer = []
	input_buffer = []
	current_prediction = None
	consecutive_agrees = 0
	min_consecutive_agrees = 3
	current_prediction_high = None
	consecutive_agrees_high = 0
	min_consecutive_agrees_high = 2
	feature_window_size = 10
	prediction_window_size = 24
	predictionDelay = 0.5
	min_confidence = 0.65
	lower_min_confidence = 0.30

	pad_size = 5
	print("receive sensor data")

	while 1:
		isCheckSumSuccess = False
		voltage = 0
		current = 0
		power = 0
		cumPower = 0
		if ser.in_waiting > 0:
			try:
				receivedString = ser.readline().decode("utf-8")
			except Exception as e:
				print(e)
				print("error")
				continue
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

						checksumPi = calculateChecksum(readingsArr, voltage, current, power, energy)
						
						if checksumPi != checksumArduino:
							print("checksum wrong!")
							continue

						# collecting data for ML buffer
						if (time.time() - last_prediction_time) > predictionDelay:
							ml_snapshot = [
								handsensorAccx, handsensorAccy, handsensorAccz,
								handsensorGyrox, handsensorGyroy, handsensorGyroz,
								forearmsensorAccx, forearmsensorAccy, forearmsensorAccz,
								forearmsensorGyrox, forearmsensorGyroy, forearmsensorGyroz,
								backsensorAccx, backsensorAccy, backsensorAccz,
								backsensorGyrox, backsensorGyroy, backsensorGyroz
							]
							ml_buffer.append(ml_snapshot)

						

						ser.write(ACK)

				else:
					print("Received packet not message")
					continue

			except ValueError or IndexError:
				print("Error while parsing received string!")
				continue
		# else:
		# 	print("nothing in serial")

		

		# Start machine learning logic
		if len(ml_buffer) == feature_window_size:
			window_data = np.array(ml_buffer)
			feature_vector = np.array(feature_extraction(window_data))
			input_buffer.append(feature_vector)
			for _ in range(pad_size):
				if len(ml_buffer) > 0:
					ml_buffer = ml_buffer[1:]

		if len(input_buffer) == prediction_window_size:
			predictions, confidences = predict_ml(MODEL, MLP_MODEL, SCALER_MODEL, np.array(input_buffer))

			print("Confidence:{} Move:{}".format(np.min(confidences),reverse_label_map[predictions[0]]))
			
			if len(set(predictions)) == 1:  # prediction is taken
				action = reverse_label_map[predictions[0]]
				if np.min(confidences) > move_confidence[action][0]:
					if action == current_prediction_high:
						consecutive_agrees_high += 1
					else:
						consecutive_agrees_high = 1
					if consecutive_agrees_high >= min_consecutive_agrees_high:
						if action != 'idle':
							print("Predicted move from high threshold is {}".format(action))
							if (time.time() - timeBefore) >= 55:
								sendToServer(action, voltage, current, power, cumPower)
							# time.sleep(predictionDelay)
							last_prediction_time = time.time()
							consecutive_agrees_high = 0
							consecutive_agrees = 0
					current_prediction_high = action
				elif np.min(confidences) > move_confidence[action][1]:
					if action == current_prediction:
						consecutive_agrees += 1
					else:
						consecutive_agrees = 1
					if consecutive_agrees >= min_consecutive_agrees:
						if action != 'idle':
							print("Predicted move from low threshold is {}".format(action))
							if (time.time() - timeBefore) >= 55:
								sendToServer(action, voltage, current, power, cumPower)
							# time.sleep(predictionDelay)
							last_prediction_time = time.time()
							consecutive_agrees_high = 0
							consecutive_agrees = 0
					current_prediction = action
				# else:
				# 	consecutive_agrees = 0
				# current_prediction = action
			input_buffer = []

# # handshake with Arduino
isHandShakeSuccessful = handshake()

if isHandShakeSuccessful and connectToServer(host, PORT_NUM):
	print("before receive sensor data")
	receiveSensorData()