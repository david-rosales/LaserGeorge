#!/usr/bin/env python3

import socket

HOST = '18.30.108.130'  # The server's hostname or IP address
PORT = 65432        # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	s.connect((HOST, PORT))
	while True: # event loop
		a = input()
		s.sendall(b'Hello, world')
		data = s.recv(1024)
		print('Received', repr(data))
