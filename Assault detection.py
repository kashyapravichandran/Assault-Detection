'''
Created by Kashyap Ravichandran, Bharath Raj Nagoor Kani and Anand Subramanian

'''
import smbus           
from time import sleep         
import os
import time
import numpy as np 
import tensorflow as tf
import sys

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'


PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
Device_Address = 0x69

x=[]
maxs=100000
window_size=10
action=[]
pointer=0

def MPU_Init():
	#write to sample rate register
	global Device_Address
	bus.write_byte_data(Device_Address, SMPLRT_DIV,7)
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1,1)
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG,24)
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE,1)

def read_raw_data(addr):
	global Device_Address
		#Accelero and Gyro value are 16-bit
	high = bus.read_byte_data(Device_Address, addr)
	low = bus.read_byte_data(Device_Address, addr+1)
	#concatenate higher and lower value
	value = ((high << 8) | low)
	#to get signed value from mpu6050
	if(value > 32768):
			value = value - 65536
	return value

def push(y):
	global x,maxs
	x.append(y)
	if len(x)==maxs+1:
		x=x[1:]
		pointer=pointer-1

def window():
	global x,action,pointer
	action=x[pointer:(window_size+pointer),:]
	pointer=pointer+3
	
def model():
	## Attax finder
	global action
	window()
	#mean_walkX = np.mean(action[:,0][:270])
	#mean_walkY = np.mean(action[:,1][:270])
	#mean_walkZ = np.mean(action[:,2][:270])

	## Step1: Mean subtraction

	action=np.array(action)
        
	print(action.shape)
	action[:,0] = action[:,0] - 0.2036603
	action[:,1] = action[:,1] + 0.819364
	action[:,2] = action[:,2] + 0.596488

	## Step2: Window and accelerometer check

	#WINDOW_SIZE = 10
	#STRIDE = 3

		
	windowX = action[:,0]
	windowY = action[:,1]
	windowZ = action[:,2]
	gyroX = action[:,3]
	gyroY = action[:,4]
	gyroZ = action[:,5]
	
	maxWindowX = np.max(windowX)
	maxWindowY = np.max(windowY)
	maxWindowZ = np.max(windowZ)
	minWindowX = abs(np.min(windowX))
	minWindowY = abs(np.min(windowY))
	minWindowZ = abs(np.min(windowZ))
	
	maxGyroX = np.max(gyroX)
	maxGyroY = np.max(gyroY)
	maxGyroZ = np.max(gyroZ)
	minGyroX = abs(np.min(gyroX))
	minGyroY = abs(np.min(gyroY))
	minGyroZ = abs(np.min(gyroZ))
	
	if(maxWindowX > 2 or maxWindowY > 2 or maxWindowZ > 2 or minWindowX > 2 
				or minWindowY > 2 or minWindowZ > 2):
		
		if(maxGyroX > 40 or maxGyroY > 40 or maxGyroZ > 40 or minGyroX > 40 or minGyroY > 40 or minGyroZ > 40):
			return True

i=0;
total=np.load('/home/pi/Desktop/Data/sitfall.npy')
while True:
	'''
        acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ZOUT_H)
	
	#Read Gyroscope raw value
	gyro_x = read_raw_data(GYRO_XOUT_H)
	gyro_y = read_raw_data(GYRO_YOUT_H)
	gyro_z = read_raw_data(GYRO_ZOUT_H)
	
	#Full scale range +/- 250 degree/C as per sensitivity scale factor
	Ax = acc_x/16384.0
	Ay = acc_y/16384.0
	Az = acc_z/16384.0
	
	Gx = gyro_x/131.0
	Gy = gyro_y/131.0
	Gz = gyro_z/131.0
        '''
	
	push_list=total[i]
	#print(push_list.shape)
	push(push_list)
	condition=False
	print(i)	
	if len(x)>window_size:
		condition=model()

	if condition == True:
		print("Assault Detected. SMS scheduled...")
		#os.system('Python2 "/home/pi/Desktop/find_stations_hospitals.py"')
	i=i+1                      

		
