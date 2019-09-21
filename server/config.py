#! /usr/bin/python3

import sys
import serial
import time

xbee = serial.Serial('/dev/ttyS0', 9600)

def enter_cmd_mode():
	print('Getting XBee into command mode ...')
	xbee.write(b'+++')
	count = 0
	while xbee.inWaiting() < 3:
		count += 1
		time.sleep(0.01)
		if count > 500:
			break
	recv = xbee.read(3)
	if xbee.inWaiting() > 0 or recv != b'OK\r':
		raise RuntimeError('Error getting XBee into command mode')
	print('XBee is now command mode')


def send_raw(cmd):
	xbee.write(cmd.encode('UTF-8'))

def recv_raw():
	count = 0
	while xbee.inWaiting() < 1:
		count += 1
		time.sleep(0.01)
		if count > 1000:
			print("Receive timeout")
			enter_cmd_mode()
			break
	len = xbee.inWaiting()
	time.sleep(0.01)
	while xbee.inWaiting() != len:
		len = xbee.inWaiting()
		time.sleep(0.01)
	return xbee.read(len).decode('UTF-8')

def main():
	enter_cmd_mode()
	while True:
		line = sys.stdin.readline().rstrip()
		send_raw("{}\r".format(line))
		reply = recv_raw().rstrip()
		print(reply)

if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		print()
		print('Good bye!')
