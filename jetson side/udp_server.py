import socket
import time
import struct
import cam_loc.py

HEADERSIZE = 10 

localIP     = "192.168.105.101"

localPort   = 3333

bufferSize  = 1024

 

msgFromServer       = "Hello UDP Client"
heading_req = 0
v_req = 0
move_req = 1

coord_struct = struct.pack('ffi', heading_req, v_req, move_req)


# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

 

# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))

#need to create two threads here
 

print("UDP server up and listening")

 

# Listen for incoming datagrams

while(True):

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


    #just for testing, remove later
    clientMsg1 = "Message from Client:{}".format(mouse_1[0])
    clientIP1  = "Client IP Address:{}".format(mouse_1[1])
    clientMsg2 = "Message from Client:{}".format(mouse_2[0])
    clientIP2  = "Client IP Address:{}".format(mouse_2[1])
    clientMsg3 = "Message from Client:{}".format(mouse_3[0])
    clientIP3  = "Client IP Address:{}".format(mouse_3[1])
    print(struct.unpack('ff', mouse_1[0]))
    #print(clientMsg1)
    print(clientIP1)
    #print(struct.unpack('ff', mouse_2[0]))
    #print(clientIP2)
    #print(struct.unpack('ff', mouse_3[0]))
    #print(clientIP3)
    #end testing section
    
    
    move_req = int(input("enter a move"))
    coord_struct = struct.pack('ffi', heading_req, v_req, move_req)
    
   # Sending a reply to client
    if mouse_1[1] != 0 :
        UDPServerSocket.sendto(coord_struct, mouse_1[1])
    if mouse_2[1] != 0 :
        UDPServerSocket.sendto(coord_struct, mouse_2[1])
    if mouse_3[1] != 0 :
        UDPServerSocket.sendto(coord_struct, mouse_3[1])

    #image section
    vid = cv2.VideoCapture(0) #cam feed into jetson
    ret, frame = vid.read()
    image = frame
    cam_loc.read_tag(image)
    
