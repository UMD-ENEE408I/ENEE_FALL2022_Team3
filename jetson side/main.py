import socket
import time
import struct
import cam_loc
import cv2
import numpy as np
import argparse
import threading


from LoadSong import LoadSong

meas_x = 10.0
meas_y = 10.0
meas_heading = 0.0
move_req = 0

HEADERSIZE = 10 
localIP     = "192.168.83.101"
localPort   = 3333
bufferSize  = 1024

coord_struct = struct.pack('fffi', meas_x, meas_y, meas_heading, move_req)

vid = cv2.VideoCapture(0) #cam feed into jetson, need to adjust for multiple streams

class latest_pic(threading.Thread):
    def __init__(self, camera):
        self.camera = camera
        self.frame = None
        super().__init__()
        # Start thread
        self.start()

    def run(self):
        while True:
            ret, self.frame = self.camera.read()
             
latest_pic = latest_pic(vid)                    
while latest_pic.frame is None:
	pass
	  
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")

# Loading Song
print("calling load song \n")
arrayOfSong, _lengthOfSong = LoadSong()
length = _lengthOfSong+1
count = 0
print("song loaded, orientation begin \n")

#update x,y, heading for first time


while meas_x == 10 or meas_y == 10:
    #image section
    frame = latest_pic.frame
    coords = cam_loc.read_tag(frame)
    if coords:
        meas_x = coords[0]
        meas_y = coords[1]
        meas_heading = coords[2]
        print(coords)
else:
    print("waiting for camera feed")
# After the loop release the cap object

#for testing only
#print("x = ")
#print(meas_x)
#print("y = ")
#print(meas_y)
#print("\n")
print("orientation complete \n")

# add here something to send mouse to wanted coords and heading




#loop for dance routine

# Hit enter when the song starts
input("Press Enter to continue...")
_time = time.time()
print("Starting.../n")
while(True):
    #udp section
    #time.sleep(1)
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    mouse_1 = [0,0]
    mouse_2 = [0,0]
    mouse_3 = [0,0]
    
    if mouse_1[1] == 0:
        mouse_1[0] = bytesAddressPair[0]
        mouse_1[1] = bytesAddressPair[1]
    elif mouse_1[1] == bytesAddressPair[1]:
        mouse_1[0] = bytesAddressPair[0]
        mouse_1[1] = bytesAddressPair[1]
    elif mouse_1[1] != bytesAddressPair[1] and mouse_2[1] == 0:
        mouse_2[0] = bytesAddressPair[0]
        mouse_2[1] = bytesAddressPair[1]
    elif mouse_2[1] == bytesAddressPair[1]:
        mouse_2[0] = bytesAddressPair[0]
        mouse_2[1] = bytesAddressPair[1]
    elif mouse_2[1] != bytesAddressPair[1] and mouse_3[1] == 0:
        mouse_3[0] = bytesAddressPair[0]
        mouse_3[1] = bytesAddressPair[1]
    elif mouse_3[1] == bytesAddressPair[1]:
        mouse_3[0] = bytesAddressPair[0]
        mouse_3[1] = bytesAddressPair[1]
    else:
        print("something went wrong with assigning clients")


    #image section
    frame = latest_pic.frame
    coords = cam_loc.read_tag(frame)
    if coords:
        meas_x = coords[0]
        meas_y = coords[1]
        meas_heading = coords[2]
        #print("Coords ")
        #print(coords)

    # sends dance move
    move_req = np.int_(arrayOfSong[1,count])
    move_req = move_req.tolist()
    #temporary
    if move_req == 2:
        move_req = 4
    
    # Updates Dance Move
    if(time.time()-_time > arrayOfSong[0,count]):
        count +=1
        

        if count>length-1:
            print("routine complete \n")
            continue
            #Can end the dance here
        


    if coords:
    	coord_struct = struct.pack('fffi', meas_x, meas_y, meas_heading, move_req)
    	print(struct.unpack('fffi', coord_struct))
    # Sending a reply to client
    if mouse_1[1] != 0 :
        UDPServerSocket.sendto(coord_struct, mouse_1[1])
    if mouse_2[1] != 0 :
        UDPServerSocket.sendto(coord_struct, mouse_2[1])
    if mouse_3[1] != 0 :
        UDPServerSocket.sendto(coord_struct, mouse_3[1])
      
    #just for testing, remove later
    clientMsg1 = "Message from Client:{}".format(mouse_1[0])
    clientIP1  = "Client IP Address:{}".format(mouse_1[1])
    clientMsg2 = "Message from Client:{}".format(mouse_2[0])
    clientIP2  = "Client IP Address:{}".format(mouse_2[1])
    clientMsg3 = "Message from Client:{}".format(mouse_3[0])
    clientIP3  = "Client IP Address:{}".format(mouse_3[1])
    print(struct.unpack('fffi', mouse_1[0]))
    #print(clientMsg1)
    print(clientIP1)
    #print(struct.unpack('ff', mouse_2[0]))
    #print(clientIP2)
    #print(struct.unpack('ff', mouse_3[0]))
    #print(clientIP3)
    #end testing section
    
# After the loop release the cap object
vid.release()
