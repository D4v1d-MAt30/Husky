#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez, Arturo Gil
@Time: February 2024
# """

import numpy as np
from robots.accelerometer import Accelerometer
from robots.objects import CoppeliaObject
from robots.ouster import Ouster
from robots.simulation import Simulation
from robots.husky import HuskyRobot
from scipy.spatial.transform import Rotation as R

# Calculates and returns the robot's orientation angle in degrees.
def get_angle(robot_center):
    T = robot_center.get_transform()
    theta_rad = np.arctan2(T[1,0],T[0,0])
    theta_deg = np.degrees(theta_rad)
    # Rotation = T[0:3,0:3]
    # rot = R.from_matrix(Rotation)
    # angles = rot.as_euler('zyx', degrees=True)
    return theta_deg

#Calculates the difference between current and final positions along the specified axis.
def distance_difference(robot_center, final_position, axis):
    T = robot_center.get_transform()
    if axis not in (0, 1):
        raise ValueError("axis must be 0 (x) or 1 (y)")
    current_position = T[axis, 3]
    distance = (final_position - current_position)
    return distance

#Returns the target orientation angle based on movement axis and direction.
def path_angle(initial_position, final_position, axis, direction):
    positive_angles = [0, 90]
    negative_angles = [180, -90]
    if axis not in (0, 1):
        raise ValueError("axis must be 0 (x) or 1 (y)")

    if initial_position < final_position[axis]:
        moving_positive = 1
    else: moving_positive = -1

    if direction == 'forward':
        moving_direction = 1
    else: moving_direction = -1

    if moving_positive * moving_direction == 1:
        angle = positive_angles [axis]
    else: angle = negative_angles [axis]
    return angle

#Returns sign correction for angular offset adjustment.
def w2_sign(angle, direction):
    valid_angles = {0: 1, -90: 1, 180: -1, 90: -1}
    if angle not in valid_angles:
        raise ValueError("Angle must be one of 0, 90, -90, 180")
    sign = valid_angles[angle]
    if direction == 'backward':
        sign = -sign
    return sign

#Returns the difference between a2 - a1 in degrees, between -180 and 180
def angle_difference(a1, a2):
    diff = (a2 - a1 + 180) % 360 - 180
    return diff

#Computes the angular correction needed for maintaining correct heading.
def rotation_correction(robot_center, current_angle, lateral_axis, path_center, target_angle, direction):
    angular_error = angle_difference(current_angle, target_angle)
    if abs(angular_error) < 2:
        w1 = 0
    elif abs(angular_error) > 15:
        w1= np.clip(angular_error / 5, -1, 1)
    else:
        w1 = np.clip(angular_error / 10, -0.5, 0.5)
    lateral_error = distance_difference(robot_center, path_center, lateral_axis)
    if abs(lateral_error) < 0.01:
        w2 = 0
    else:
        w2 = np.clip(lateral_error, -1,1)
        sign = w2_sign(target_angle, direction)
        w2 = w2 * sign
    w = w1 + w2
    return w

def straight_movement(robot_center, robot, simulation, final_position, movement_axis, direction ='forward'):
    print('ROBOT STRAIGHT')
    T = robot_center.get_transform()
    initial_position = T[movement_axis, 3]
    target_angle = path_angle(initial_position, final_position, movement_axis, direction)
    lateral_axis = 1 - movement_axis
    while True:
        current_angle = get_angle(robot_center)
        #yaw, pitch, roll = angles
        #print("Euler angles:", angles)
        distance_error = distance_difference(robot_center, final_position[movement_axis], movement_axis)
        v = np.clip(abs(distance_error) / 3, 0.5, 1.5)
        w = rotation_correction(robot_center, current_angle, lateral_axis, final_position[lateral_axis], target_angle, direction)
        threshold = 0.25
        if initial_position >= final_position[movement_axis]:
            if distance_error >= -threshold:
                robot.move(v=0, w=0)
                break
        else:
            if distance_error <= threshold:
                robot.move(v=0, w=0)
                break
        if direction == 'backward':
            v = -v
        robot.move(v=v, w=w)
        T = robot_center.get_transform()
        print(T)
        print('Angular velocity (w):', w)
        print('Linear velocity (v):', v)
        simulation.wait()

def turning_movement(robot_center, robot, simulation, turn_angle, direction = 'forward'):
    print('ROBOT TURNING')
    initial_angle = get_angle(robot_center)
    #yaw_start, pitch_start, roll_start = angles_start
    while True:
        current_angle = get_angle(robot_center)
        #yaw_current, pitch_current, roll_current = angles_current
        remaining_angle = angle_difference(current_angle, initial_angle + turn_angle)
        if abs(remaining_angle) < 5:
            robot.move(v=0, w=0)
            break
        if abs(remaining_angle) > 15:
            w = np.clip(remaining_angle / 5.0, -1, 1)
        else:
            w = np.clip(remaining_angle / 10.0, -0.5, 0.5)
        if direction == 'forward':
            v = 0.25
        else:
            v = -0.25
        robot.move(v=v, w=w)
        T = robot_center.get_transform()
        print(T)
        print(current_angle)
        simulation.wait()

def simulate():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # Simulate a LiDAR
    lidar = Ouster(simulation=simulation)
    lidar.start(name='/OS1')
    # A dummy object ath the robot center
    robot_center = CoppeliaObject(simulation=simulation)
    robot_center.start(name='/HuskyCenter')
    # an accelerometer
    accel = Accelerometer(simulation=simulation)
    accel.start(name='/Accelerometer')

    # MOVEMENTSDA
    print('MOVING ROBOT')
    straight_movement(robot_center, robot, simulation, final_position = (0, -8.5), movement_axis = 1)
    turning_movement(robot_center, robot, simulation, turn_angle = 90)
    straight_movement(robot_center, robot, simulation, final_position = (19.5, -9), movement_axis = 0)#Punto1
    turning_movement(robot_center, robot, simulation, turn_angle = 90)
    straight_movement(robot_center, robot, simulation, final_position = (20, 10), movement_axis = 1)
    turning_movement(robot_center, robot, simulation, turn_angle = 90)
    straight_movement(robot_center, robot, simulation, final_position = (15.5, 10.5), movement_axis = 0)
    turning_movement(robot_center, robot, simulation, turn_angle=-90)
    straight_movement(robot_center, robot, simulation, final_position = (15, 21), movement_axis = 1)
    turning_movement(robot_center, robot, simulation, turn_angle=-90)
    straight_movement(robot_center, robot, simulation, final_position = (16, 21.5), movement_axis = 0)#Punto2
    straight_movement(robot_center, robot, simulation, final_position = (15.5, 21.5), movement_axis = 0, direction='backward')
    turning_movement(robot_center, robot, simulation, turn_angle=90, direction='backward')
    straight_movement(robot_center, robot, simulation, final_position = (15, 22), movement_axis = 1)
    turning_movement(robot_center, robot, simulation, turn_angle=90)
    straight_movement(robot_center, robot, simulation, final_position = (-8, 22.5), movement_axis = 0)
    turning_movement(robot_center, robot, simulation, turn_angle=90)
    turning_movement(robot_center, robot, simulation, turn_angle=-90)
    straight_movement(robot_center, robot, simulation, final_position = (-17.5, 21.5), movement_axis = 0)
    turning_movement(robot_center, robot, simulation, turn_angle=90)
    straight_movement(robot_center, robot, simulation, final_position = (-18, 19), movement_axis = 1)#Punto3
    turning_movement(robot_center, robot, simulation, turn_angle=90)
    straight_movement(robot_center, robot, simulation, final_position = (-12, 18.5), movement_axis = 0)
    turning_movement(robot_center, robot, simulation, turn_angle=-90)
    straight_movement(robot_center, robot, simulation, final_position = (-11.5, -9.5), movement_axis = 1)
    turning_movement(robot_center, robot, simulation, turn_angle=-90)
    straight_movement(robot_center, robot, simulation, final_position = (-13.5, -10), movement_axis = 0)#Punto4
    straight_movement(robot_center, robot, simulation, final_position = (-12, -10), movement_axis = 0, direction='backward')
    turning_movement(robot_center, robot, simulation, turn_angle=-90, direction='backward')
    straight_movement(robot_center, robot, simulation, final_position = (-11.5, 0), movement_axis = 1)
    turning_movement(robot_center, robot, simulation, turn_angle=-90)
    straight_movement(robot_center, robot, simulation, final_position = (0, 0.5), movement_axis = 0)#Punto final

    #  TORQUES
    # now, obtain the mean torques or torques for each wheel
    # during 50 simulat steps.
    # for i in range(2):
    #     tau = robot.get_mean_wheel_torques()
    #     axyz = accel.get_accel_data()
    #     T = robot_center.get_transform()
    #     print(T)
    #     print(tau)
    #     print(axyz)
    #     robot.wait()
    # # robot.forward()
    # # robot.wait(130)
    # # robot.left()
    # # robot.wait(130)
    # # robot.right()
    # # robot.wait(100)
    # # # get lidar data
    # for i in range(1):
    # lidar.save_pointcloud('lidar/simulated_pointcloud6.pcd')
    # lidar.draw_pointcloud()
    simulation.stop()


if __name__ == "__main__":
    simulate()