   #!/usr/bin/env python
import time
import serial
import RPi.GPIO as GPIO
import struct
import threading

port = "/dev/ttyS0"
ser = serial.Serial(port, baudrate=9600)
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(40, GPIO.OUT)

nack = 0
ack = 1
message = 3

nodevice = 0
hand = 1
forearm = 2
back = 3

receiveBuffer = []
bufferLengthLimit = 15

def calculateChecksum(readingsarr):
	checksum = 0
	for reading in readingsarr:
		readingInt = int(reading)
		checksum = checksum ^ readingInt

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
						checksumArduino = int(receivedString.split('(')[4])

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

						checksumPi = calculateChecksum(readingsArr)
						if checksumPi == checksumArduino:
							print("checksum correct!")

							# send ack to arduino
							ser.write(b'\x01\x01\x01')
							print(readingsArr)
						else:
							print("checksum wrong")
							# send nack to arduino
							ser.write(b'\x01\x00\x00')
							print(readingsArr)
#						break

				else:
					print("Received packet not message")
					continue
			except ValueError or IndexError:
				print("Error while parsing received string!")
				continue


# handshake with Arduino
isHandShakeSuccessful = handshake()

if isHandShakeSuccessful == 1:
	print("handshake success")
elif isHandShakeSuccessful == 0:
	print("handshake failed")

t1 = threading.Thread(target=storeInBuffer)
t2 = threading.Thread(target=receiveSensorData)

t1.start()
t2.start()

while 1:
	pass
#receiveSensorData()
