# README

This is an experimental image to read and use the light sensor that are used in the ETHZ Autolab.

To launch calibration procedure, use : 

`dts duckiebot demo --duckiebot_name WATCHTOWER_NAME --demo_name light_sensor_calibration --package_name light_sensor_calibration --image duckietown/dt-light-sensor:daffy-arm32v7 --debug` 

Then to launch the light sensor node, use :

`dts duckiebot demo --duckiebot_name WATCHTOWER_NAME --demo_name light_sensor --package_name light_sensor --image duckietown/dt-light-sensor:daffy-arm32v7` 

To plug in the RGB sensor correctly see the notes below:

We are always holding the hardware in this direction, in which we can read everything.

ping of the Wachtower: from the left
oo4ooo

123oo5

o       o   yellow  o o o

orange  red green   o o black

the ping of the sensor

o4132o5

o yellow orange green red o black

