import serial
import matplotlib.pyplot as mp
import math
import numpy as np
mp.hold(True)
#TODO:
#write it to a file constantly
#make it so that it can be stopped with a keystroke
def doyourprocessinghere(line,times,angleX,angleY,angleZ,accelX,accelY,accelZ,Euler,X,Y,Z):
    print(line)
    temp = line.split()
    if (len(temp) != 7):
        return;
    for i in range(0,7):
        temp[i]=float(temp[i])
        
    temp[1]=temp[1]*math.pi/180
    temp[2]=temp[2]*math.pi/180
    temp[3]=temp[3]*math.pi/180
    times.append(temp[0])
    angleX.append(temp[1])
    angleY.append(temp[2])
    angleZ.append(temp[3])
    accelX.append(temp[4])
    accelY.append(temp[5])
    accelZ.append(temp[6])
    cx = math.cos(temp[1]/2)
    sx = math.sin(temp[1]/2)
    cy = math.cos(temp[2]/2)
    sy = math.sin(temp[2]/2)
    cz = math.cos(temp[3]/2)
    sz = math.sin(temp[3]/2)
    angle = 2*math.acos((cx*cy*cz-sx*sy*sz))
    x = cx*cy*sz+sx*sy*cz
    y =sx*cy*cz + cx*sy*sz;
    z =cx*sy*cz - sx*cy*sz;
    norm = math.sqrt(x**2+y**2+z**2)
    X.append(x/norm)
    Y.append(y/norm)
    Z.append(z/norm)
    Euler.append(angle)
    
    

ser = serial.Serial(
    port='/dev/cu.usbmodem4542331',\
    baudrate=9600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)
message = str("")
count=1
times=[]
angleX=[]
angleY=[]
angleZ=[]
accelX=[]
accelY=[]
accelZ=[]
Euler=[]
X=[]
Y=[]
Z=[]
line = ""
for i in range (1,1000000) :
    for character in ser.read():
        #print(chr(character) , end="")
        line = line+chr(character);
        if (character==10):
            doyourprocessinghere(line,times,angleX,angleY,angleZ,accelX,accelY,accelZ,Euler,X,Y,Z);
            line = ""
        #message += chr(line)
mp.figure(1)
mp.clf()
mp.plot(times,Euler,'k')
mp.plot(times,angleX,'r')
mp.plot(times,angleY,'r')
mp.plot(times,angleZ,'r')
mp.figure(2)
mp.plot(times,X,'r')
mp.plot(times,Y,'r')
mp.plot(times,Z,'r')
ser.close()
