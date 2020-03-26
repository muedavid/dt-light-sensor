#!/usr/bin/env python
import rospy
import time
import RPi.GPIO as GPIO
import Adafruit_TCS34725
import yaml
import os.path
from duckietown_msgs.msg import LightSensor
from duckietown import DTROS


class LightSensorNode(DTROS):
    """Node handling the measurment of light.
        measures the light in the room and publish it.

        Publishers:
           ~sensor_data (:obj:`LightSensor`): Publishes measured intesitiy of red, green and blue, the calculated lux and the real_lux value that takes 
           callibration into account. It publishes also the temperature of the light. 
    """
    
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LightSensorNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~default_mult'] = None
        self.parameters['~default_offset']=None
        self.parameters['~frequency']=None
        self.updateParameters()

        self.veh_name = rospy.get_namespace().strip("/")
        
        # GPIO setup
        # Choose BCM or BOARD numbering schemes
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18, GPIO.OUT)
        GPIO.output(18, GPIO.LOW)

        # Set integrationtime and gain
        self.tcs = Adafruit_TCS34725.TCS34725(
            integration_time=Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_700MS,
            gain=Adafruit_TCS34725.TCS34725_GAIN_1X)

        #define file path
        self.filepath = '/data/config/calibrations/light-sensor/' + self.veh_name + ".yaml"

        #define parameter
        self.mult = 0
        self.offset = 0
        self.default_mult=self.parameters['~default_mult'] #default value = 1
        self.default_offset= self.parameters['~default_offset'] #default value = 0
        #set parametervalue
        self.readParamFromFile()
        
        #define time between evaluations in sec. 
        self.timesensor = 1/self.parameters['~frequency'] #value = 1: every second one measurment


        # ROS-Publications
        self.msg_light_sensor = LightSensor()
        self.sensor_pub = self.publisher('~sensor_data', LightSensor, queue_size=1)
        while not rospy.is_shutdown():
            self.get_lux()
            rospy.sleep(rospy.Duration.from_sec(self.timesensor))
        
    def get_lux(self):
        # Read R, G, B, C color data from the sensor.
        r, g, b, c = self.tcs.get_raw_data()
        # Calulate color temp
        temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
        # Calculate lux and multiply it with gain
        lux = Adafruit_TCS34725.calculate_lux(r, g, b)
        real_lux = self.mult * lux + self.offset

        # Publish to topic
        #header
        self.msg_light_sensor.header.stamp = rospy.Time.now()
        self.msg_light_sensor.header.frame_id = rospy.get_namespace()[1:-1] 

        self.msg_light_sensor.r = r
        self.msg_light_sensor.g = g
        self.msg_light_sensor.b = b
        self.msg_light_sensor.c = c
        self.msg_light_sensor.real_lux = real_lux
        self.msg_light_sensor.lux = lux
        self.msg_light_sensor.temp = temp
        self.sensor_pub.publish(self.msg_light_sensor)

    

    def readParamFromFile(self):
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(self.filepath):
            rospy.logwarn("[%s] %s does not exist. Using default values" % (self.node_name, self.filepath))
            self.mult = self.default_mult
            self.offset = self.default_offset
            
        else:
            with open(self.filepath, 'r') as in_file:
                yaml_dict = yaml.load(in_file)
                self.mult = yaml_dict["mult"]
                self.offset = yaml_dict["offset"]
                
        rospy.loginfo("[%s] from callibration %s = %s " % (self.node_name,"mult", self.mult))
        rospy.loginfo("[%s] from callibration %s = %s " % (self.node_name,"offset", self.offset))


if __name__ == '__main__':
    node = LightSensorNode(node_name='light_sensor_node')
    rospy.spin()
