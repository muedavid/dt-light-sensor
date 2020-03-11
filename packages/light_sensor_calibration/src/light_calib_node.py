#!/usr/bin/env python
"""
The orientation is how you write them
Plugins on raspberry pi: o o 1 o o
                         2 3 4 o 5

Plugins on tcs34725:     o 1 2 4 3 o 5

"""
import rospy
from duckietown_msgs.msg import LightSensor
import time
from Adafruit_GPIO import I2C
import RPi.GPIO as GPIO
import Adafruit_TCS34725
import smbus
import yaml
import os.path
from duckietown_utils import get_duckiefleet_root
import numpy as np
from future.builtins import input


class LightSensorCalibrator(object):

    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # GPIO setup
        # Choose BCM or BOARD numbering schemes
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18, GPIO.OUT)
        # turn off LED
        GPIO.output(18, GPIO.LOW)

        # Set integrationtime and gain
        self.tcs = Adafruit_TCS34725.TCS34725(
            integration_time=Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_700MS,
            gain=Adafruit_TCS34725.TCS34725_GAIN_1X)

        # #Set parameter
        # self.readParamFromFile()
        # #Set local gain using yaml
        # self.mult = self.setup_parameter("~mult",1)
        # self.offset = self.setup_parameter("~offset", 1)

        # ROS-Publications
        self.msg_light_sensor = LightSensor()
        self.sensor_pub = rospy.Publisher(
            '~sensor_data', LightSensor, queue_size=1)
        rate = rospy.Rate(10)
        self.lux1 = []
        self.lux2 = []
        input("Are you ready for the first light evaluation (ENTER)?")

        for count in range(23):
            if count > 3:
                self.lux1.append(self.get_lux())
            rate.sleep()

        val1 = int(input("How much was the light luminescence?"))
        input("Are you ready for the next light evaluation (ENTER)?")

        for count in range(23):
            if count > 3:
                self.lux2.append(self.get_lux())
            rate.sleep()
        val2 = int(input("How much was the light luminescence?"))

        std1 = np.std(self.lux1)
        med1 = np.median(self.lux1)
        std2 = np.std(self.lux2)
        med2 = np.median(self.lux2)
        # make sure that the standard deviation is not to big
        self.mult = (val2-val1)/(med2-med1)
        self.offset = val1 - self.mult * med1
        self.set_param()
        return

    def set_param(self):
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "mult": int(self.mult),
            "offset": int(self.offset)
        }
        
        file_name = self.getFilePath(self.veh_name)
        if file_name != 'False':
            with open(file_name, 'w') as outfile:
                outfile.write(yaml.dump(data, default_flow_style=False))

            rospy.loginfo("[%s] Saved to %s" % (self.node_name, file_name))

    def get_lux(self):
        # Read R, G, B, C color data from the sensor.
        r, g, b, c = self.tcs.get_raw_data()
        # Calulate color temp
        temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
        # Calculate lux and multiply it with gain
        lux = Adafruit_TCS34725.calculate_lux(r, g, b)
        real_lux = lux
        # Calculate lux out of RGB measurements.
        print("temp [k]= ", temp)
        print("r :", r)
        print("g :", g)
        print("b :", b)
        print("c :", c)
        print("lux = ", lux)
        print("real_lux: ", real_lux)
        # Publish to topic

        # TODO: add other things to header
        self.msg_light_sensor.header.stamp = rospy.Time.now()
        self.msg_light_sensor.header.frame_id = rospy.get_namespace()[
            1:-1]  # splicing to remove /

        self.msg_light_sensor.real_lux = real_lux
        self.msg_light_sensor.lux = lux
        self.msg_light_sensor.temp = temp
        self.sensor_pub.publish(self.msg_light_sensor)
        return lux

        # rate.sleep()

    def getFilePath(self, name):
        if os.path.isdir('/data/config/calibrations/light-sensor/'):
            #das ganze automatisieren
            print ("calibration is already done. delete File to callibrate again")
            return ('False')
        else:
            os.makedirs('/data/config/calibrations/light-sensor/')
            return ('/data/config/calibrations/light-sensor/' + name + ".yaml")

    # def readParamFromFile(self):
    # 	#Check file existance
    # 	fname = self.getFilePath(self.veh_name)
    # 	# Use default.yaml if file doesn't exsit
    # 	if not os.path.isfile(fname):
    # 		rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
    # 		fname = self.getFilePath("default")

    # 	with open(fname, 'r') as in_file:
    # 		try:
    # 			yaml_dict = yaml.load(in_file)
    # 		except yaml.YAMLError as exc:
    # 			rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
    # 			rospy.signal_shutdown()
    # 			return

  #       # Set parameters using value in yaml file
    # 	if yaml_dict is None:
  #       	# Empty yaml file
    # 		return
    # 	for param_name in ["gainr", "gain", "mult", "offset"]:
    # 		param_value = yaml_dict.get(param_name)
    # 		if param_name is not None:
    # 			rospy.set_param("~"+param_name, param_value)
    # 		else:
    # 			# Skip if not defined, use default value instead.
    # 			pass

    # def setup_parameter(self, param_name, default_value):
    # 	value = rospy.get_param(param_name, default_value)
  #   	# Write to parameter server for transparency
    # 	rospy.set_param(param_name, value)
    # 	rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
    # 	return value


if __name__ == '__main__':
    rospy.init_node('light_sensor_node', anonymous=False)
    light_calib_node = LightSensorCalibrator()
    # rospy.spin()
