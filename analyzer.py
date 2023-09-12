#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import distance
import numpy as np

def calculate_energy_cost(p1, p2):
    dist_2d = distance.euclidean(p1[:2], p2[:2])
    dist_3d = distance.euclidean(p1, p2)
    z_diff = abs(p1[2] - p2[2])
    theta = np.arctan2(z_diff, dist_2d) # 角度の絶対値
    energy = (2 * 432.432 * theta + 100) * dist_3d # エネルギー消費量
    return energy

def calculate_angle(p1, p2, p3):
    v1 = np.array(p2[:2]) - np.array(p1[:2]) # Use only x,y components for vectors
    v2 = np.array(p3[:2]) - np.array(p2[:2]) # Use only x,y components for vectors
    angle_diff = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    return abs(angle_diff)

def calculate_angle_cost(angle):
    return angle * 551

def cloud_callback(msg):
    # Convert PointCloud2 to array of points
    raw_positions = [(x, y, z) for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]

    # Remove consecutive duplicates in positions
    positions = [raw_positions[i] for i in range(len(raw_positions)) if i == 0 or raw_positions[i] != raw_positions[i-1]]
    
    # Calculate path length
    path_length = sum(distance.euclidean(p1, p2) for p1, p2 in zip(positions, positions[1:]))
    
    # Calculate total height change
    total_height_change = sum(abs(p1[2] - p2[2]) for p1, p2 in zip(positions, positions[1:]))
    
    # Calculate total turn angle and energy cost
    total_turn_angle = 0
    total_energy = 0
    for p1, p2, p3 in zip(positions, positions[1:], positions[2:]):
        angle = calculate_angle(p1, p2, p3)
        total_turn_angle += angle
        total_energy += calculate_angle_cost(angle) + calculate_energy_cost(p1, p2)
    
    rospy.loginfo("Path length: {}".format(path_length))
    rospy.loginfo("Total height change: {}".format(total_height_change))
    rospy.loginfo("Total turn angle: {}".format(total_turn_angle))
    rospy.loginfo("Total energy: {}".format(total_energy))
    rospy.loginfo("-------------------------------------------------")

def main():
    rospy.init_node('path_analyzer')
    rospy.Subscriber('/tour5/pcd_pointcloud', PointCloud2, cloud_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
