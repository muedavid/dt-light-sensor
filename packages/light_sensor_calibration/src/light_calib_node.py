#!/usr/bin/env python
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

        med1 = np.median(self.lux1)
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
        if file_name != 'file_exist':
            with open(file_name, 'w') as file:
                file.write(yaml.dump(data, default_flow_style=False))
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
        return lux

    def getFilePath(self, name):
        #check if we have already done the callibration
        if os.path.isdir('/data/config/calibrations/light-sensor/'):
            print ("calibration is already done. If you want to save your new callibration delete the old file")
            return ('file_exist')
        else:
            os.makedirs('/data/config/calibrations/light-sensor/')
            return ('/data/config/calibrations/light-sensor/' + name + ".yaml")


if __name__ == '__main__':
    rospy.init_node('light_sensor_node', anonymous=False)
    light_calib_node = LightSensorCalibrator()
