#!/usr/bin/env python
import rospy
import time
#from Adafruit_GPIO import I2C
import RPi.GPIO as GPIO
import Adafruit_TCS34725
#import smbus
import yaml
import os.path
import shutil
import numpy as np
from duckietown import DTROS
#from duckietown_utils import get_duckiefleet_root
from future.builtins import input


class LightSensorCalibrator(DTROS):
    def __init__(self, node_name,disable_signals=False):
        # initialize the DTROS parent class
        super(LightSensorCalibrator, self).__init__(node_name=node_name)

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
        self.rate = rospy.Rate(10)
        
        #paths
        self.dirname = '/data/config/calibrations/light-sensor/'
        self.filename = self.dirname  + self.veh_name + ".yaml"
        
        # look if we have already done a callibration
        if os.path.isdir('/data/config/calibrations/light-sensor/'):
            decission = input("calibration is already done. Do you want to make a newcallibration ? [Y/N]")
            if (decission == "Y") or (decission =="y"):# versuche wie or wirklich aussehen muss. 
                shutil.rmtree (self.dirname)
            else:
                print("the old calibration is still valid and the node will shutdown because we already have a callibration file")
        
        #make the folder for saving the callibration        
        os.makedirs(self.dirname) # if decission was not y or Y we will get now an error and process shutdown
        
        #vaiables
        self.mult = 1
        self.offset = 0
        self.lux = 0
        
        #callibration
        self.callibrator ()
        
        #set the parameter
        self.set_param()

    def callibrator(self):
        lux1 = []
        lux2 = []
        
        input("Are you ready for the first light evaluation (ENTER)?")
        
        count = 0
        endtime = 0 #if it isn't possible to have in 40 measurments 20 that are good enough we sill shutdown the process
        while count < 23:
            if count > 2:
                self.get_lux()
                if count > 6:
                    average = np.average (lux1)
                    if np.abs (self.lux - average) > 15:
                        count -= 1 # we repeat the measurment
                        print("this measurment will be repeated")
                    else:
                        lux1.append(self.lux)
                else:
                    lux1.append(self.lux)
            self.rate.sleep()
            count += 1
            endtime += 1
            if endtime == 28:
                print("The have needed to much measurments to achieve a satisfying result of the callibration")
                count = 0
                endtime = 0
                lux1 = []
                input("are you ready to restart the evaluation")

        val1 = int(input("How much was the light luminescence?"))
        
        input("Are you ready for the next light evaluation (ENTER)?")

        
        count = 0
        endtime = 0 #if it isn't possible to have in 40 measurments 20 that are good enough we sill shutdown the process
        while count < 23:
            if count > 3:
                self.get_lux()
                if count > 6:
                    average = np.average (lux2)
                    if np.abs(self.lux - average) > 15:
                        count -= 1 # we repeat the measurment
                        print("this measurment will be repeated")
                    else:    
                        lux1.append(self.lux)
                else:    
                    lux2.append(self.lux)
            self.rate.sleep()
            count += 1
            endtime += 1
            if endtime == 28:
                print("The have needed to much measurments to achieve a satisfying result of the callibration")
                count = 0
                endtime = 0
                lus2 = []
                input("are you ready to restart the evaluation")

        val2 = int(input("How much was the light luminescence?"))

        med1 = np.median(lux1)
        med2 = np.median(lux2)
        # make sure that the standard deviation is not to big
        self.mult = (val2-val1)/(med2-med1)
        self.offset = val1 - self.mult * med1
        
    def set_param(self):
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "mult": int(self.mult),
            "offset": int(self.offset)
        }
        
        with open(self.filename, 'w') as file:
            file.write(yaml.dump(data, default_flow_style=False))
            rospy.loginfo("[%s] Saved to %s" % (self.node_name, self.filename))

    def get_lux(self):
        # Read R, G, B, C color data from the sensor.
        r, g, b, c = self.tcs.get_raw_data()
        # Calulate color temp
        temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
        # Calculate lux and multiply it with gain
        self.lux = Adafruit_TCS34725.calculate_lux(r, g, b)

        # Calculate lux out of RGB measurements.
        print("temp [k]= ", temp)
        print("r :", r)
        print("g :", g)
        print("b :", b)
        print("c :", c)
        print("lux = ", self.lux)

if __name__ == '__main__':
    node = LightSensorCalibrator(node_name='light_calib_node',disable_signals=False)