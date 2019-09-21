#!/usr/bin/python3

import serial
from time import sleep

xbee = serial.Serial ("/dev/ttyS0", 9600)

while True:
	while xbee.inWaiting() < 1:
		sleep(0.01)
	c = xbee.read()
	if c == b"\x7e":
		buf = xbee.read(2)
		len = buf[0]*256 + buf[1]
		apid = int.from_bytes(xbee.read(1), "big")
		if apid == 0x81:
			addr = int.from_bytes(xbee.read(2), "big")
			str = int.from_bytes(xbee.read(1), "big")
			opt = int.from_bytes(xbee.read(1), "big")
			data = xbee.read(len-5).decode("UTF-8").rstrip()
			if addr == 0xAB02 and data[:2] == "2+":
				value = data[2]
				if value == '1':
					print("Door closed")
				else:
					print("Door open")
