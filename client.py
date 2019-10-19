#client side
from Crypto.Cipher import AES
from Crypto import Random
#from Crypto.Util.Padding import pad
import hashlib
import base64
import socket
import time
import sys
import binascii
import os

#import pandas as pd

#ip address
# host = 'localhost'
# PORT_NUM = 7654
host = sys.argv[1]
PORT_NUM = int(sys.argv[2])

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#need to include ip address of server to be connected
sock.connect((host, PORT_NUM))
print ('Connected, action?')
bs = 32; #base_size
key = "1234567890123456"
x = 0

def encryptText(plainText, key):
    raw = pad(plainText)
    iv = Random.new().read(AES.block_size)
    cipher = AES.new(key.encode("utf8"),AES.MODE_CBC,iv)
    msg = iv + cipher.encrypt(raw.encode('utf8'))
    # msg = msg.strip()
    return base64.b64encode(msg)

def pad(var1):
    return var1 + (bs - len(var1)%bs)*chr(bs - len(var1)%bs)

while (x<10):
#padding to ensure that the message is of fixed size
    action = input("#'action'|'voltage'|'current'|'power'|'cumpower'|\n")
    temp = str(action)
    temp = temp.strip()
    stringToSend = encryptText(temp, key)
    print(str(len(stringToSend)))
    print(stringToSend)
    print(key)
    sock.send(stringToSend) #need to encode as a string as it is what is expected on server side
    x = x+1

sock.close()
