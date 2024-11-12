# import random
import serial          # import the module
import struct
import numpy as np
from time import sleep
ComPort = serial.Serial('COM12') # open COM12, please change the com port name as per your device.
ComPort.baudrate = 9600 # set Baud rate to 9600
ComPort.bytesize = 8    # Number of data bits = 8
ComPort.parity   = 'N'  # No parity
ComPort.stopbits = 1    # Number of Stop bits = 1  
arr = []
### uncomment the loop below for sending image to FPGA :)
for i in range(51200):
    ComPort.flushOutput()
    ComPort.flushInput()
    x = input()
    x = int(x,2)
    print(f"{i}: {x}")
    if x>127:
        x = x - 256
    ot= ComPort.write(struct.pack('b', x))      
    sleep(0.0009)
print("completed transmission")


#### uncomment the loop below for recieving the image from FPGA :)
for i in range(51200):
    ot= ComPort.read(size = 1)
    arr.append(ot)
    print(i,":  ",ot)
print("completed transmission")
arr1 = []
for i in arr:
    k = int.from_bytes(i, byteorder='little')
    arr1.append(k)

with open ("obtained_image.txt" ,'w') as f:
    for idx, pixel in enumerate(arr1):
        if idx == len(arr1) - 1:
            f.write(f"{int(pixel)}")
        else:
            f.write(f"{int(pixel)}\n")
f.close()

img = np.array(arr1).reshape((128,128))
plt.imshow(img, cmap = 'gray')
plt.show()