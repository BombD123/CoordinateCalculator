#!/usr/bin/env python

from __future__ import with_statement 
import serial, time, re
from datetime import datetime

class NoSerialDataException(Exception):
    pass

class SerialDistanceReader:
    max_seconds_stale_data = 1
    latest_retrieval_time = 0

    def __init__(self, serPort, calibration_bias):
        self.latest_distance = -1
        self.calibration_bias = calibration_bias

        self.ser = serial.Serial()
        self.ser.port = serPort
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False

        # Currently set a timeout of 2 seconds on a call to read from the serial
        # port, so that we can handle a failure to read in a more grand way than
        # a timeout.
        self.ser.timeout = 2

        self.range_parser = re.compile("Range: ([^ ]+) m")

        try:
            self.ser.open()
            self.ser.flushInput()
        except Exception, error:
            print "Error opening port " + self.ser.port + ": " + str(error)
            raise;

    """ TODO """
    def thread_set_latest_distance(self, lock):
        while True:
            #print("Retrieving line from " + self.ser.port)
            cur_line = self.ser.readline()
            #print("Retrieved line from " + self.ser.port + ": " + cur_line)
            #if len(cur_line) == 0:
            #    raise NoSerialDataException()
            match = self.range_parser.search(cur_line)
            if match != None and float(match.groups()[0]) >= 0:
                with lock:
                    self.latest_distance = float(match.groups()[0]) + self.calibration_bias
                    self.latest_retrieval_time = datetime.now()
                return

    def get_latest_distance(self):
        if (datetime.now() - self.latest_retrieval_time).seconds > self.max_seconds_stale_data:
            raise NoSerialDataException()
        return self.latest_distance

if __name__ == "__main__":
    import threading
    distance_reader1 = SerialDistanceReader("/dev/ttyUSB1", -.0)
    distance_reader2 = SerialDistanceReader("/dev/ttyUSB2", -.4)
    distance_reader3 = SerialDistanceReader("/dev/ttyUSB3", -.3)
    lock = threading.Lock()
    while (True):
        distance_reader1.thread_set_latest_distance(lock)
        print("Distance1: " + str(distance_reader1.get_latest_distance()))
        #distance_reader2.thread_set_latest_distance(lock)
        #print("Distance2: " + str(distance_reader2.get_latest_distance()))
        #distance_reader3.thread_set_latest_distance(lock)
        #print("Distance3: " + str(distance_reader3.get_latest_distance()))
