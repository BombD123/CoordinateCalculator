#!/usr/bin/env python

from Trilateration import Trilateration
import numpy
import math

def reconstruct_point(beacon_points, point):
    print("Point: " + str(point))
    distances = []
    for i in range(0, len(beacon_points)):
        distance = 0
        for i_coord in range(0, len(beacon_points[i])):
            distance += (beacon_points[i][i_coord] - point[i_coord])*(beacon_points[i][i_coord] - point[i_coord])
        distances.append(math.sqrt(distance))
    print("Real distances: " + str(distances))
    for i in range(len(distances)):
        distances[i] = distances[i] + numpy.random.normal(0, 1)

    reconstructed_point = trilat.get_location_from_distances(distances)
    print("Reconstructed:")
    print(numpy.around(reconstructed_point, decimals=10))

if __name__ == "__main__":
    beacon_points = [[25, 0, 0], [0, -25, 0], [-25, 0, 0], [0, 0, 10]]
    trilat = Trilateration(beacon_points, 100)

    reconstruct_point(beacon_points, [0, 0, 0])
    reconstruct_point(beacon_points, [25, 0, 0])
    reconstruct_point(beacon_points, [25, 0, 10])
    reconstruct_point(beacon_points, [100, 200, 1000])
