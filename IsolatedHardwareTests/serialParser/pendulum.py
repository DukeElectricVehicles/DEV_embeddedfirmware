#
# Gets data from the accelerometer/gyroscope instrument over serial, processes
# this data, and saves it to file in a Numpy array. This is similar to Caleb's
# code (see python_serial_test_caleb.py) but more robust and easier to
# maintain
#
# This code has only been tested on Windows; it does not run on Linux
#
# Daniel Winkelman (github.com/dwinkelman0)
#

import serial
import serial.tools.list_ports

import numpy as np

# Set up ports
ports = serial.tools.list_ports.comports()
for port in ports:
	print(port)

port = ports[0]
print("Selected port {:}".format(port.device))

# Array for storing data
class DataProcessor(object):
	def __init__(self, n_samples):
		self.data = np.zeros((n_samples, 11))
		self.loc = 0

	def SaveToFile(self, path):
		np.save(path, self.data)

	def Process(self, line):
		# Pre-process data
		components = line.split('\t')
		components = [float(i) for i in components]
		t, angle_x, angle_y, angle_z, accel_x, accel_y, accel_z = components
		angle_x, angle_y, angle_z = np.radians(angle_x), np.radians(angle_y), np.radians(angle_z)
		cx, cy, cz = np.cos(angle_x / 2), np.cos(angle_y / 2), np.cos(angle_z / 2)
		sx, sy, sz = np.sin(angle_x / 2), np.sin(angle_y / 2), np.sin(angle_z / 2)

		# Perform calculations
		angle = 2 * np.arccos((cx * cy * cz) - (sx * sy * sz))
		x = (cx * cy * sz) + (sx * sy * cz)
		y = (sx * cy * cz) + (cx * sy * sz)
		z = (cx * sy * cz) - (sx * cy * sz)
		norm = np.sqrt(x**2 + y**2 + z**2)
		X, Y, Z = x / norm, y / norm, z / norm

		# Add data to array
		# Change how data is placed into the file
		self.data[self.loc] = np.array([
			t, angle_x, angle_y, angle_z, accel_x, accel_y, accel_z, X, Y, Z, angle])
		self.loc += 1

		print(angle)
		

if __name__ == "__main__":
	# Read lines from device
	# Change this number for how much data to collect
	n_samples = 1000
	processor = DataProcessor(n_samples)
	with serial.Serial(port.device, baudrate=115200, timeout=5,
			parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS) as ser:
		for i in range(n_samples):
			line = ser.readline().decode('utf-8').replace('\n', '')
			processor.Process(line)

	# Change this line to save to a specific directory
	processor.SaveToFile("C:\\Users\\dwinkelman\\Documents\\pendulum_data.npy")

