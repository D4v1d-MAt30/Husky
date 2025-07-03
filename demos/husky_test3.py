#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez, Arturo Gil
@Time: February 2024
# """

import numpy as np
from pynput import keyboard
from robots.accelerometer import Accelerometer
from robots.objects import CoppeliaObject
from robots.ouster import Ouster
from robots.simulation import Simulation
from robots.husky import HuskyRobot
from scipy.spatial.transform import Rotation as R

v = 0
w = 0

v_increment = 0.25  # m/s
w_increment = 0.25  # rad/s

global simulation
global robot
global robot_center

simulation = None
robot = None
robot_center = None

def obtener_angulo(robot_center):
    T = robot_center.get_transform()
    theta = 180*(np.arctan2(T[1, 0], T[0, 0]))/np.pi        #Valido mientras no suba pendientes
    print("theta:", theta)
    Rotation = T[0:3,0:3]
    rot = R.from_matrix(Rotation)
    angles = rot.as_euler('zyx', degrees=True)
    yaw, pitch, roll = angles
    #If the robot is moving on a plane without slope then we only need yaw
    # if abs(roll) < 0.1 and abs(pitch) < 0.1:
    #     angles = yaw
    return angles

def on_press(key):
    global v, w
    try:
        print('Key pressed: {0} '.format(key.char))
        # for the rest of actions,  decode action from actions dictionary
        if key.char == 'w':
            v += v_increment
            print(f" Forward - v = {v:.2f}, w = {w:.2f}")
        elif key.char == 's':
            v -= v_increment
            print(f" Backward - v = {v:.2f}, w = {w:.2f}")
        if key.char == 'a':
            w += w_increment
            print(f" Turn left - v = {v:.2f}, w = {w:.2f}")
        elif key.char == 'd':
            w -= w_increment
            print(f" Turn right - v = {v:.2f}, w = {w:.2f}")
        elif key.char == 'x':
            w = 0
            v = 0
            print(f" Stop - v = {v:.2f}, w = {w:.2f}")
    except (AttributeError, KeyError):
        print('special key pressed: {0}'.format(key))
    robot.move(v=v, w=w)
    simulation.wait()
    angles = obtener_angulo(robot_center)
    print(angles[0], angles[1], angles[2])
    return True

def on_release(key):
    print('Key released: {0}'.format(key))
    if key == keyboard.Key.esc:
        print('Exiting')
        exit()
        return False

if __name__ == "__main__":
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

    print('Use: w for forward, s for backward, a for turning left, a for turning right and x to stop')
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    simulation.stop()
