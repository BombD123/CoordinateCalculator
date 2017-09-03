#!/usr/bin/env python

import numpy
import math

class Trilateration:
    origin_point = numpy.array([])
    pinv_a = numpy.matrix([])
    d_squared = numpy.array([])

    def __init__(self, anchor_points, num_iterations):
        numpy.set_printoptions(precision=2)
        self.origin_point = numpy.matrix(anchor_points[0]).transpose()
        self.anchor_points = numpy.matrix(anchor_points)
        self.num_iterations = num_iterations

        a_mat = self.get_a_matrix(anchor_points)
        self.pinv_a = numpy.linalg.pinv(a_mat)
        self.d_squared = self.get_d_squared_elements_vector(a_mat)

    def get_a_matrix(self, anchor_points):
        # First we create our matrix that we will invert.  This is
        # done by finding the pairwise subtracted x, y, and z coordinates

        a_mat = numpy.matrix(numpy.zeros((len(anchor_points) - 1, len(anchor_points[0]))))

        for i_coord in range(0, len(anchor_points[0])):
            for i_point in range(1, len(anchor_points)):
                a_mat[i_point - 1, i_coord] = anchor_points[i_point][i_coord] - anchor_points[0][i_coord]

        return a_mat

    def get_d_squared_elements_vector(self, a_mat):
        d_squared_elems = numpy.zeros((a_mat.shape[0], 1))

        for i_elem in range(0, a_mat.shape[0]):
            for i_coord in range(0, a_mat.shape[1]):
                d_squared_elems[i_elem] += a_mat[i_elem, i_coord] * a_mat[i_elem, i_coord];
        
        return d_squared_elems

    """
    This method uses linear least square distance reduction to provide an initial guess
    for the optimal point.
    """
    def get_initial_location_from_distances(self, distances):
        b_vec = numpy.zeros((len(distances)-1, 1))

        print ("Distances: " + str(distances))
        origin_dist_sqr = distances[0]*distances[0]
        for i in range(1, len(distances)):
            b_vec[i - 1] = .5*(origin_dist_sqr - distances[i]*distances[i] + self.d_squared[i - 1])

        return self.pinv_a*b_vec + self.origin_point

    def get_location_from_distances(self, distances):
        # First check if any of the distances are zero.  If so, that's our point.
        for anchor_index in range(len(distances)):
            if distances[anchor_index] < .0001:
                print ("Point is sufficiently close to anchor " + str(anchor_index) + "; returning anchor point.")
                return self.anchor_points[anchor_index]

        current_location = self.get_initial_location_from_distances(distances)

        #print("Initial point estimate: " + str(current_location.T))
        for i in range(1, self.num_iterations):
            anchor_point_diff = self.get_anchor_point_differences(current_location, self.anchor_points)
            #print("Anchor point diffs: \n" + str(anchor_point_diff))
            anchor_distance_errors = self.get_anchor_distance_errors(
                anchor_point_diff, distances, anchor_point_diff)
            #print("Anchor point errors: \n" + str(anchor_distance_errors))

            #print ("Current error in point estimation: \n" + str(sum(anchor_distance_errors)))

            j_t_j_inv = self.calc_JTJ_inv(distances, anchor_distance_errors, anchor_point_diff)
            #print ("JTJ_INV: \n" + str(j_t_j_inv))
            j_t_f = self.calc_JTF(distances, anchor_distance_errors, anchor_point_diff)
            #print ("JTF: \n" + str(j_t_f))

            #print ("Shapes: " + str(type(j_t_j_inv)) + ", " + str(type(j_t_f)))
            #print ("Multiplying: \n" + str(numpy.dot(j_t_j_inv, j_t_f)))

            current_location = current_location - numpy.dot(j_t_j_inv, j_t_f)
            #print("Point estimate: \n" + str(current_location))
        return current_location

    def get_anchor_point_differences(self, current_location, anchor_points):
        differences = numpy.zeros((len(anchor_points), len(current_location)))
        for i in range(differences.shape[0]):
            for j in range(differences.shape[1]):
                differences[i, j] = current_location[j] - anchor_points[i, j]
        return differences

    def get_anchor_distance_errors(self, anchor_point_distance, anchor_distances, anchor_point_diff):
        distance_errors = numpy.zeros((len(anchor_distances), 1))
        for anchor_index in range(len(distance_errors)):
            real_dist_squared = 0
            for dimension_index in range(len(anchor_point_diff[anchor_index])):
                real_dist_squared = (real_dist_squared + 
                    anchor_point_diff[anchor_index,dimension_index] * 
                    anchor_point_diff[anchor_index,dimension_index])
            distance_errors[anchor_index] = math.sqrt(real_dist_squared) - anchor_distances[anchor_index]
        return distance_errors

    def calc_JTJ_inv(self, anchor_distances, anchor_distance_errors, anchor_point_diff):
        j_t_j = numpy.zeros((len(anchor_point_diff[0]), len(anchor_point_diff[0])))

        for i in range(j_t_j.shape[0]):
            for j in range(j_t_j.shape[1]):
                for anchor_index in range(len(anchor_distances)):
                    j_t_j[i, j] = (j_t_j[i, j] + 
                        (anchor_point_diff[anchor_index, i]*anchor_point_diff[anchor_index, j]) /
                        (anchor_distance_errors[anchor_index] + anchor_distances[anchor_index]))

        return numpy.linalg.pinv(j_t_j)

    def calc_JTF(self, anchor_distances, anchor_distance_errors, anchor_point_diff):
        j_t_f = numpy.zeros((len(anchor_point_diff[0]), 1))
        
        for i in range(len(j_t_f)):
            for anchor_index in range(len(anchor_distances)):
                j_t_f[i] = (j_t_f[i] + 
                    anchor_point_diff[anchor_index, i] * anchor_distance_errors[anchor_index] /
                    (anchor_distance_errors[anchor_index] + anchor_distances[anchor_index]))
        return j_t_f
