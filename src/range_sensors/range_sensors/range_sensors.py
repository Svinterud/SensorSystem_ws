#!/usr/bin/env python
#Read sensor-data from Arduino and publish
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range

import serial
import math
import time


"""
sensor_msgs/Range

uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range

"""

class Sonar():
	def __init__(self, distance=0.0, angle=0.0):
		self.distance = distance #Meters
		self.angle = angle #Radians

class IR():
	def __init__(self, distance=0.0, angle=0.0):
		self.distance = distance #Meters
		self.angle = angle #Radians


#Initialize arduino communication
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)



class SonarArray(Node):
	def __init__(self, 
		num_sensor = 5, #Number of active ultrasonic sensors
		range_min = 0.04, #Minimum valid range of sensor [m]
		range_max = 3.5 #Maximum valid range of sensor [m]
		):
		super().__init__("sonar_data")
		self.sonar_array = []
		self.pub_array = []
		self.distance = [0,0,0,0,0]
		self.num_sensor = num_sensor
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.scan)
		for i in range(num_sensor):
			sonar_num = i + 1
			sonar = Sonar()
			sonar.angle = math.radians(15)
			self.sonar_array.append(sonar)
			topic_name = "/agv/sonar_%d"%sonar_num
			topic_name = str(topic_name)
			pub = self.create_publisher(Range, topic_name, 5)
			self.pub_array.append(pub)
			sonar_num = 0
		
		#Default message
		message = Range()
		message.radiation_type = 0 
		message.min_range = range_min
		message.max_range = range_max
		self.msg = message		

	def scan(self):
		#Read data from arduino, expand to multiple sensors
		data = arduino.readline()
		if data:
			data_string = str(data)
			data_array = data_string.split(',')
		if data and len(data_array) == 10:
			us_1 = data_array[0]
			us_2 = data_array[1]
			us_3 = data_array[2]
			us_4 = data_array[3]
			us_5 = data_array[4]

			us_1 = float(us_1[2:])
			us_2 = float(us_2)
			us_3 = float(us_3)
			us_4 = float(us_4)
			us_5 = float(us_5)
			
			self.distance = [us_1, us_2, us_3, us_4, us_5]


		for i in range(self.num_sensor):
			self.sonar_array[i].distance = self.distance[i]*0.01
			header_id = i + 1
			self.msg.range = self.sonar_array[i].distance
			self.msg.range = 1.0
			self.msg.field_of_view = self.sonar_array[i].angle
			self.msg.header.frame_id = 'us_%d'%header_id
			self.pub_array[i].publish(self.msg)
			self.get_logger().info("US range[m]")


class IRArray(Node):
	def __init__(self, 
		num_sensor = 5, #Number of active infrared sensors
		range_min = 1.0, #Minimum valid range of sensor [m]
		range_max = 4.0 #Maximum valid range of sensor [m]
		):
		super().__init__("ir_data")
		self.ir_array = []
		self.pub_array = []
		self.distance = [0,0,0,0,0]
		self.num_sensor = num_sensor
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.scan)
		for i in range(num_sensor):
			ir_num = i + 1
			ir = IR()
			ir.angle = math.radians(5)
			self.ir_array.append(ir)
			topic_name = "/agv/ir_%d"%ir_num
			topic_name = str(topic_name)
			pub = self.create_publisher(Range, topic_name, 5)
			self.pub_array.append(pub)
			
		
		#Default message
		message = Range()
		message.radiation_type = 0 
		message.min_range = range_min
		message.max_range = range_max
		self.msg = message		

	def scan(self):
		#Read data from arduino, expand to multiple sensors
		data = arduino.readline()
		if data:
			data_string = str(data)
			data_array = data_string.split(',')
		if data and len(data_array) == 10:
			ir_1 = data_array[5]
			ir_2 = data_array[6]
			ir_3 = data_array[7]
			ir_4 = data_array[8]
			ir_5 = data_array[9]

			ir_1 = float(ir_1)
			ir_2 = float(ir_2)
			ir_3 = float(ir_3)
			ir_4 = float(ir_4)
			ir_5 = float(ir_5[:-5])
			
			self.distance = [ir_1, ir_2, ir_3, ir_4, ir_5]


		for i in range(self.num_sensor):
			self.ir_array[i].distance = self.distance[i]
			header_id = i + 1
			self.msg.range = self.ir_array[i].distance*0.01

			self.msg.range = 1.0
			self.msg.field_of_view = self.ir_array[i].angle
			self.msg.header.frame_id = 'ir_%d'%header_id
			self.pub_array[i].publish(self.msg)
			self.get_logger().info("Range[m] %s" %self.msg.range)

def main(args=None):
	time.sleep(1)
	rclpy.init(args=args)
	ir_data = IRArray()
	sonar_data = SonarArray(5, 0.3, 3.5)
	rclpy.spin(sonar_data)
	rclpy.spin(ir_data)
	sonar_data.destroy_node()
	ir_data.destroy_node()
	rclpy.shutdown()

if __name__=="__main__":
	main()

		

