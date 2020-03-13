#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import LightSensor
import time
# from Adafruit_GPIO import I2C
import RPi.GPIO as GPIO
import Adafruit_TCS34725
# import smbus
import yaml
import os.path
# from duckietown_utils import get_duckiefleet_root


class LightSensorNode(object):

    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.debug = False
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

        #define parameter
        self.mult = 0
        self.offset = 0
        self.default_mult=1
        self.default_offset=0
        #set parametervalue
        self.readParamFromFile()


        # ROS-Publications
        self.msg_light_sensor = LightSensor()
        self.sensor_pub = rospy.Publisher(
            '~sensor_data', LightSensor, queue_size=1)
        get_lux_timer = rospy.Timer(rospy.Duration(secs=1), self.get_lux)

    def get_lux(self, timerevent):
        # Read R, G, B, C color data from the sensor.
        r, g, b, c = self.tcs.get_raw_data()
        # Calulate color temp
        temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
        # Calculate lux and multiply it with gain
        lux = Adafruit_TCS34725.calculate_lux(r, g, b)
        real_lux = self.mult * lux + self.offset
        # Calculate lux out of RGB measurements.
        if self.debug:
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

        self.msg_light_sensor.r = r
        self.msg_light_sensor.g = g
        self.msg_light_sensor.b = b
        self.msg_light_sensor.c = c
        self.msg_light_sensor.real_lux = real_lux
        self.msg_light_sensor.lux = lux
        self.msg_light_sensor.temp = temp
        self.sensor_pub.publish(self.msg_light_sensor)

    def getFilePath(self, name):
        return ('/data/config/calibrations/light-sensor/' + name + ".yaml")

    def readParamFromFile(self):
        # Check file existance
        fname = self.getFilePath(self.veh_name)
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default values" % (self.node_name, fname))
            self.mult = self.default_mult
            self.offset = self.default_offset 
            
        else:
            with open(fname, 'r') as in_file:
                try:
                    yaml_dict = yaml.load(in_file)
                    self.mult = yaml_dict["mult"]
                    self.offset = yaml_dict["offset"]
                except yaml.YAMLError as exc:
                    rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" % (
                        self.node_name, fname, exc))
                    rospy.signal_shutdown("light sensor exiting")
                    return

        """
        rospy.set_param("~mult",self.mult)
        rospy.set_param("~offset",self.offset)
        """
        rospy.loginfo("[%s] %s = %s " % (self.node_name,"mult", self.mult))
        rospy.loginfo("[%s] %s = %s " % (self.node_name,"offset", self.offset))


if __name__ == '__main__':
    rospy.init_node('light_sensor_node', anonymous=False)
    light_sensor_node = LightSensorNode()
    rospy.spin()
