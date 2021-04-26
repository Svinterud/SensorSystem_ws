#!/usr/bin/env python
#Read sensor-data from Arduino and publish as laserscan

import rclpy
import std_msgs.msg
from rclpy.node import Node


import serial
import math
import time
import numpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan, Range


#Initialize arduino communication
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)

class RangeToLaser(Node):
	def __init__(self):
		super().__init__("us_to_laser")
		nr_sensor = 9
		self.nr_sensor = nr_sensor
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.scan)
		self.pub_array = []
		self.msg = LaserScan()
		self.distance = [0]*nr_sensor
		for i in range(nr_sensor):
			sensor_num = i + 1
			topic_name = "/agv/sonar_to_laser%d"%sensor_num
			pub = self.create_publisher(LaserScan, topic_name, 5)
			self.pub_array.append(pub)
			
	


		#Laser message
		self.msg.header = std_msgs.msg.Header()
		self.msg.angle_min = math.radians(-15)
		self.msg.angle_max = math.radians(15)
		self.msg.angle_increment = math.radians(1)
		self.msg.time_increment = 0.0
		self.msg.range_min = 0.05
		self.msg.range_max = 4.0

	
	def scan(self):
		ranges = []
		self.msg.header.stamp = self.get_clock().now().to_msg()
		data = arduino.readline()
		if data:
			data_string = str(data)
			data_array = data_string.split(',')
		if data and len(data_array) == 14:
			us_1 = data_array[0]
			us_2 = data_array[1]
			us_3 = data_array[2]
			us_4 = data_array[3]
			us_5 = data_array[4]
			us_6 = data_array[5]
			us_7 = data_array[6]
			us_8 = data_array[7]
			us_9 = data_array[8]

			us_1 = float(us_1[2:])
			us_2 = float(us_2)
			us_3 = float(us_3)
			us_4 = float(us_4)
			us_5 = float(us_5)
			us_6 = float(us_2)
			us_7 = float(us_3)
			us_8 = float(us_4)
			us_9 = float(us_5)

			self.distance = [us_1,us_2,us_3,us_4,us_5, us_6, us_7, us_8, us_9]
			
			for i in range(self.nr_sensor):
				ranges = []
				ranges.append(self.distance[i]*0.01)
				ranges = ranges*20
				header_id = i + 1
				self.msg.header.frame_id = 'us_%d'%header_id
				self.msg.ranges = ranges
				self.pub_array[i].publish(self.msg)



class IrToLaser(Node):
	def __init__(self):
		super().__init__("ir_to_laser")
		nr_sensor = 5
		self.nr_sensor = nr_sensor
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.scan)
		self.pub_array = []
		self.msg = LaserScan()
		self.distance = [0]*nr_sensor
		for i in range(nr_sensor):
			sensor_num = i + 1
			topic_name = "/agv/ir_to_laser%d"%sensor_num
			pub = self.create_publisher(LaserScan, topic_name, 5)
			self.pub_array.append(pub)
			
	


		#Laser message
		self.msg.header = std_msgs.msg.Header()
		self.msg.angle_min = math.radians(-3)
		self.msg.angle_max = math.radians(3)
		self.msg.angle_increment = math.radians(1)
		self.msg.time_increment = 0.0
		self.msg.range_min = 1.0
		self.msg.range_max = 5.5

	
	def scan(self):
		ranges = []
		self.msg.header.stamp = self.get_clock().now().to_msg()
		data = arduino.readline()
		if data:
			data_string = str(data)
			data_array = data_string.split(',')
		if data and len(data_array) == 14:
			ir_1 = data_array[9]
			ir_2 = data_array[10]
			ir_3 = data_array[11]
			ir_4 = data_array[12]
			ir_5 = data_array[13]


			ir_1 = float(ir_1)
			ir_2 = float(ir_2)
			ir_3 = float(ir_3)
			ir_4 = float(ir_4)
			ir_5 = float(ir_5[:-5])


			self.distance = [ir_1,ir_2,ir_3,ir_4,ir_5]	
			for i in range(self.nr_sensor):
				ranges = []
				ranges.append(self.distance[i]*0.01)
				ranges = ranges*6
				header_id = i + 1
				self.msg.header.frame_id = 'ir_%d'%header_id
				self.msg.ranges = ranges
				self.pub_array[i].publish(self.msg)
	



		

def main(args=None):
	time.sleep(1)
	rclpy.init(args=args)
	us_to_laser = RangeToLaser()
	ir_to_laser = IrToLaser()
	rclpy.spin(us_to_laser)
	rclpy.spin(ir_to_laser)
	us_to_laser.destroy_node()
	ir_to_laser.destroy_node()
	rclpy.shutdown()
	

if __name__=="__main__":
	main()

