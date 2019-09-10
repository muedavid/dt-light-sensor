# README

This is an experimental image to read and use the light sensor that are used in the ETHZ Autolab.

To launch, use : 

`docker -H <hostname.local> run --name dt-light-sensor --privileged -v /data:/data -dit --network=host duckietown/dt-light-sensor:daffy-arm32v7`