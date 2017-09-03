#!/usr/bin/env python

from CoordinateCalculator import CoordinateCalculator
import time

attitude_reader_port = "/dev/ttyUSB0"
anchors = []
anchors.append({"x": -.92, "y": 0, "z": 0, "serial_port": "/dev/ttyUSB1", "bias": -.5})
anchors.append({"x": 0, "y": .72, "z": 0, "serial_port": "/dev/ttyUSB2", "bias": -.0})
anchors.append({"x": .92, "y": 0, "z": 0, "serial_port": "/dev/ttyUSB3", "bias": -.0})
#anchors.append({"x": 0, "y": -.45, "z": .78, "serial_port": "/dev/ttyUSB4"})

coordinate_calculator = CoordinateCalculator(attitude_reader_port, anchors)
time.sleep(5)

while True:
    try:
        location = coordinate_calculator.get_location()
        print("Derived locations: ")
        print(str(location))
    except Exception as e:
        print("Unable to derive locations: " + str(e))
    time.sleep(.5)
