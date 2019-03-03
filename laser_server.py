from laser_control import LaserControl
import socket

with open('./calibration.txt','r') as f:
	caliData = f.read()

parameters = caliData.split(" ")
paraDict = {}
for item in parameters:
	parameter_Value = item.split("=")
	paraDict[parameter_Value[0]] = parameter_Value[1]
print(paraDict)

myControl = LaserControl(theta_min = paraDict['theta_min'], theta_max = paraDict['theta_max'],
						 phi_min = paraDict['phi_min'], phi_max = paraDict['phi_max'],screen_distance = paraDict['screen_distance'])
laserState = "OFF"

HOST = '0.0.0.0'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    while True:
        s.listen(1)
        conn, addr = s.accept()
        with conn:
            print('Connected by', addr)
            while True:
                data = conn.recv(1024)
                if not data:
                    continue
                conn.sendall(data)
                print(data)
                (x,y) = data.decode().split(",")
                x = float(x)
                y = float(y)
                if x == -1 and y == -1:
                	if(laserState == "ON"):
                		laserState = "OFF"
                		myControl.turn_off()
                else:
                	if(laserState == "OFF"):
                		laserState = "ON"
                		myControl.turn_on()	
                	myControl.move2xy(x,y)
