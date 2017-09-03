#!/usr/bin/env python

from __future__ import with_statement 
from SerialDistanceReader import SerialDistanceReader
from Trilateration import Trilateration
from pyMultiwii import MultiWii
from threading import Thread
from threading import Lock
import time
import random

"""
Continually sends updated values from the controller.  This function
is expected to be invoked as another thread in order to constantly
send values while separating the logic of calculating those values to the
main thread waiting for user input from the console.
"""
def bg_update_distances(serial_distance_readers, lock):
    while True:
        try:
            for serial_distance_reader in serial_distance_readers:
                serial_distance_reader.thread_set_latest_distance(lock)
                print("Retrieved distance: " + str(serial_distance_reader.get_latest_distance()) + " from " + 
                    serial_distance_reader.ser.port)
        except Exception as e:
            print("Exception during reading distance from " + serial_distance_reader.ser.port + ": " + str(e))

        #time.sleep(random.uniform(.1, .2))

""" TODO document this class """
class CoordinateCalculator :
    anchors = []
    thread_lock = Lock()

    """
    anchor_configs is a list of dictionaries that contains the following members:
        x
        y
        z
        serial_port
    """
    def __init__(self, attitude_reader_port, anchor_configs) :
        self.locations = []
        for anchor_config in anchor_configs :
            self.anchors.append(SerialDistanceReader(anchor_config["serial_port"], anchor_config["bias"]))
            self.locations.append([float(anchor_config["x"]), float(anchor_config["y"]), float(anchor_config["z"])])

            print ("Initializing anchor " + anchor_config["serial_port"] + 
                " with locations: " + str(self.locations[-1]))

        thread = Thread(
            target=bg_update_distances, args=(self.anchors, self.thread_lock))
        thread.daemon = True
        thread.start()
        time.sleep(5)

        self.trilateration = Trilateration(self.locations, 100)
        self.attitude_reader = MultiWii(attitude_reader_port)
        print ("Initialized multiwii on " + attitude_reader_port)

    def get_location(self) :
        anchor_distances = []
        with self.thread_lock:
            for i in range(len(self.anchors)):
                    anchor_distances.append(self.anchors[i].get_latest_distance())
        print("Anchor distances: " + str(anchor_distances))

        return self.trilateration.get_location_from_distances(anchor_distances)
