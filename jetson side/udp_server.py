import socket
import time
import struct


HEADERSIZE = 10 

localIP     = "192.168.26.101"

localPort   = 3333

bufferSize  = 1024

 

msgFromServer       = "Hello UDP Client"
heading_req = 14.56
v_req = 13.43
move_req = 1

coord_struct = struct.pack('ffi', heading_req, v_req, move_req)


# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

 

# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))

 

print("UDP server up and listening")

 

# Listen for incoming datagrams

while(True):

    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

    message = bytesAddressPair[0]

    address = bytesAddressPair[1]

    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    
    print(clientMsg)
    print(clientIP)
    print(struct.unpack('ffffiif', message))
   # Sending a reply to client

    UDPServerSocket.sendto(coord_struct, address)

    
