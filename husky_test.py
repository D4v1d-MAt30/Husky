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

def obtener_angulo(robot_center):
    T = robot_center.get_transform()
    theta_rad = np.atan2(T[1,0],T[0,0])
    theta_deg = np.degrees(theta_rad)
    # Rotation = T[0:3,0:3]
    # rot = R.from_matrix(Rotation)
    # angles = rot.as_euler('zyx', degrees=True)
    return theta_deg

def diferencia_dist(robot_center, pos_final, valorxy):
    T = robot_center.get_transform()
    if valorxy not in (0, 1):
        raise ValueError("valorxy debe ser 0 (x) o 1 (y)")
    pos_actual = T[valorxy, 3]
    distancia = (pos_final - pos_actual)
    return distancia

def diferencia_angulo(a1, a2):
    #Devuelve la diferencia de a2 - a1 en grados, entre -180 y 180
    diff = (a2 - a1 + 180) % 360 - 180
    return diff

def movimiento_recto(robot_center, robot, simulation, angulo, valor_max, valor_xy, sentido ='positive'):
    print('ROBOT STRAIGHT')
    T = robot_center.get_transform()
    pos_inicial = T[valor_xy, 3]
    while True:                                                             #Falta ajustar para que quede dentro de las lineas
        angle_current = obtener_angulo(robot_center)
        #yaw, pitch, roll = angles
        #print("Euler angles:", angles)
        error_dist = diferencia_dist(robot_center, valor_max, valor_xy)
        v = np.clip(abs(error_dist) / 4, 0.2, 1.2)
        error_angular = diferencia_angulo(angulo, angle_current)
        w = np.clip(-error_angular / 10, -0.5, 0.5)
        if pos_inicial >= valor_max:
            if error_dist >= 0:
                robot.move(v=0, w=0)
                break
        else:
            if error_dist <= 0:
                robot.move(v=0, w=0)
                break
        if sentido == 'negative':
            v = -v
        robot.move(v=v, w=w)
        T = robot_center.get_transform()
        print(T)
        print('valor de w:')
        print(w)
        simulation.wait()

def movimiento_curva(robot_center, robot, simulation, giro, sentido = 'positive'):
    print('ROBOT TURNING')
    angle_start = obtener_angulo(robot_center)
    #yaw_start, pitch_start, roll_start = angles_start
    while True:
        angle_current = obtener_angulo(robot_center)
        #yaw_current, pitch_current, roll_current = angles_current
        error_angular = diferencia_angulo(angle_start, angle_current)
        w = np.clip((giro - error_angular) / 10.0, -0.5, 0.5)
        if abs(error_angular) > abs(giro):
            robot.move(v=0, w=0)
            break
        if sentido == 'positive':
            v = 0.1
        elif sentido == 'negative':
            v = -0.1
        robot.move(v=v, w=w)
        T = robot_center.get_transform()
        print(T)
        print(angle_current)
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
    movimiento_recto(robot_center, robot, simulation, angulo = -90, valor_max = -8.5, valor_xy = 1)
    movimiento_curva(robot_center, robot, simulation, giro = 90)
    movimiento_recto(robot_center, robot, simulation, angulo = 0, valor_max = 19.5, valor_xy = 0)#Punto1
    movimiento_curva(robot_center, robot, simulation, giro = 90)
    movimiento_recto(robot_center, robot, simulation, angulo = 90, valor_max = 10, valor_xy = 1)
    movimiento_curva(robot_center, robot, simulation, giro = 90)
    movimiento_recto(robot_center, robot, simulation, angulo=180, valor_max=15.5, valor_xy=0)
    movimiento_curva(robot_center, robot, simulation, giro=-90)
    movimiento_recto(robot_center, robot, simulation, angulo = 90, valor_max = 21, valor_xy = 1)
    movimiento_curva(robot_center, robot, simulation, giro=-90)
    movimiento_recto(robot_center, robot, simulation, angulo = 0, valor_max = 16, valor_xy = 0)#Punto2
    movimiento_recto(robot_center, robot, simulation, angulo = 0, valor_max = 15.5, valor_xy = 0, sentido='negative')
    movimiento_curva(robot_center, robot, simulation, giro=90, sentido='negative')
    movimiento_recto(robot_center, robot, simulation, angulo = 90, valor_max = 22, valor_xy = 1)
    movimiento_curva(robot_center, robot, simulation, giro=90)
    movimiento_recto(robot_center, robot, simulation, angulo=180, valor_max=-8, valor_xy=0)
    movimiento_curva(robot_center, robot, simulation, giro=90)
    movimiento_curva(robot_center, robot, simulation, giro=-90)
    movimiento_recto(robot_center, robot, simulation, angulo=180, valor_max=-17.5, valor_xy=0)
    movimiento_curva(robot_center, robot, simulation, giro=90)
    movimiento_recto(robot_center, robot, simulation, angulo=-90, valor_max=19, valor_xy=1)#Punto3
    movimiento_curva(robot_center, robot, simulation, giro=90)
    movimiento_recto(robot_center, robot, simulation, angulo=0, valor_max=-12, valor_xy=0)
    movimiento_curva(robot_center, robot, simulation, giro=-90)
    movimiento_recto(robot_center, robot, simulation, angulo=-90, valor_max=-9.5, valor_xy=1)
    movimiento_curva(robot_center, robot, simulation, giro=-90)
    movimiento_recto(robot_center, robot, simulation, angulo=180, valor_max=-13.5, valor_xy=0)#Punto4
    movimiento_recto(robot_center, robot, simulation, angulo=180, valor_max=-12, valor_xy=0, sentido='negative')
    movimiento_curva(robot_center, robot, simulation, giro=-90, sentido='negative')
    movimiento_recto(robot_center, robot, simulation, angulo=90, valor_max=0, valor_xy=1)
    movimiento_curva(robot_center, robot, simulation, giro=-90)
    movimiento_recto(robot_center, robot, simulation, angulo=0, valor_max=0, valor_xy=0)#Punto final

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
    #     data = lidar.get_laser_data()
    #     lidar.save_pointcloud('lidar/simulated_pointcloud7.pcd')
    #     lidar.down_sample()
    #     lidar.estimate_normals()
    #     lidar.draw_pointcloud()
    simulation.stop()


if __name__ == "__main__":
    simulate()