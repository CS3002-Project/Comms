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

# list that will stores as csv
csv_data = pd.DataFrame()

#initialise serial port
port = "/dev/ttyS0"
ser = serial.Serial(port, baudrate=9600)

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

#Initialise server
bs = 32; #base_size
key = "1234567890123456"
x = 0

if len(sys.argv) != 3 :
    print('Invalid number of arguments')
    print('python3 myclient.py [IP address] [Port]')
    sys.exit()
# host = 'localhost'
# PORT_NUM = 7654
host = sys.argv[1]
PORT_NUM = int(sys.argv[2])

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
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	print("Connecting to server...")
	try:
		sock.connect(ip_addr, port_num)
	except:
        debug("Connection to Server failed.")
        return False
    return True

def calculateChecksum(readingsarr, voltage, current, power, energy):
	checksum = 0
	for reading in readingsarr:
		readingInt = int(reading)
		checksum = checksum ^ readingInt

	checksum = checksum ^ (int)voltage
	checksum = checksum ^ (int)current
	checksum = checksum ^ (int)power
	checksum = checksum ^ (int)energy

	return checksum

def handshake():
	# send hello packet to Arduino
	x = ser.write(b'\x01\x02\x02')

	# receive ack packet from Arduino
	packetid = 0
	checksum = -1

	while 1:
		if ser.in_waiting:
			receivedString = ser.read_until().decode("utf-8")
			break
	receivedString = receivedString[:-1]
	packetid = int(receivedString[0])
	checksum = int(receivedString[1])

	if packetid == ack:
		if packetid != checksum:
			return 0
	elif packetid != ack:
		return 0

	print("ack received")

	# send ack packet to Arduino
	ser.write(b'\x01\x01\x01')

	return 1

def storeInBuffer():
	while 1:
		if ser.in_waiting:
			receiveBuffer.append(ser.read_until().decode("utf-8"))

def receiveSensorData():
	packetid = -1
	deviceid = -1
	deviceid2 = -1
	deviceid3 = -1
	checksum = -1

	while 1:
#		print("in receive sensor data loop\n")
		isCheckSumSuccess = False
		voltage = 0
		current = 0
		power = 0
		energy = 0
		if receiveBuffer:
			#receivedString = ser.read_until().decode("utf-8")
			print(len(receiveBuffer))
			receivedString = receiveBuffer.pop(0)


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
						energy = float(receivedString.split('(')[7])
						checksumArduino = float(int(receivedString.split('(')[8]))

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
						timestamp += 2000

						# timestamp = time.time()
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

						checksumPi = calculateChecksum(readingsArr, voltage, current, power, energy)
						
						if checksumPi == checksumArduino:
							print("checksum correct!")

							# send ack to arduino
							ser.write(b'\x01\x01\x01')
							ser.write(b'\x01\x00\x00')
							readingsarr = np.array(readingsarr)
							csv_data = csv_data.append(pd.DataFrame(readingsarr.flatten()))
							print(readingsArr)
							isCheckSumSuccess = True
						else:
							print("checksum wrong")
							# send nack to arduino
							npdata = np.array(data)
				else:
					print("Received packet not message")
					continue

			except ValueError or IndexError:
				print("Error while parsing received string!")
				continue
			export_csv = csv_data.to_csv('sensor_data.csv', index = None, header=False)

		if isCheckSumSuccess:
			# machine learning predict move from data

			# #send data to server
		 #    action = input("#'action'|'voltage'|'current'|'power'|'cumpower'|\n")#padding to ensure that the message is of fixed size
		 #    temp = str(action)
		 #    temp = temp.strip()
		 #    stringToSend = encryptText(temp, key)
		 #    print(str(len(stringToSend)))
		 #    print(stringToSend)
		 #    print(key)
		 #    sock.send(stringToSend) #need to encode as a string as it is what is expected on server side

# handshake with Arduino
isHandShakeSuccessful = handshake()

if isHandShakeSuccessful == 1:
	print("handshake success")
elif isHandShakeSuccessful == 0:
	print("handshake failed")

if (isHandShakeSuccessful == 1) and connectToServer(host, PORT_NUM):
	t1 = threading.Thread(target=storeInBuffer)
	t2 = threading.Thread(target=receiveSensorData)

	t1.start()
	t2.start()

	while 1:
		pass
	#receiveSensorData()
