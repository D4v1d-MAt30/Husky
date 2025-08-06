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
from Movement import PathPlanner

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

    # # Define a sequence of movements for the robot to follow
    print('MOVING ROBOT')
    movements = [
        {"final_position": (0, -8.5), "movement_axis": 1},
        {"final_position": (19.5, -9), "movement_axis": 0},                             #Point 1
        {"final_position": (20, 10), "movement_axis": 1},
        {"final_position": (15.5, 10.5), "movement_axis": 0},
        {"final_position": (15, 20), "movement_axis": 1},
        {"final_position": (16.5, 20.5), "movement_axis": 0},                             #Point 2
        {"final_position": (17, 22.5), "movement_axis": 1},
        {"final_position": (-8, 23), "movement_axis": 0},
        {"final_position": (-8.5, 22), "movement_axis": 1},
        {"final_position": (-17.5, 21.5), "movement_axis": 0},
        {"final_position": (-18, 19), "movement_axis": 1},                              #Point 3
        {"final_position": (-12, 18.5), "movement_axis": 0},
        {"final_position": (-11.5, 4.5), "movement_axis": 1},
        {"final_position": (-13, 4), "movement_axis": 0},
        {"final_position": (-13.5, -9.5), "movement_axis": 1},                          #Point 4
        {"final_position": (-12, -10), "movement_axis": 0},
        {"final_position": (-11.5, 0), "movement_axis": 1},
        {"final_position": (0, 0.5), "movement_axis": 0},                               #Final point
    ]

    edges_before = np.empty((0, 2))

    for i in range(len(movements)):
        current = movements[i]

        # Set the previous movement parameters (or default if first movement)
        if i == 0:
            previous = {"final_position": (0, 0), "movement_axis": current["movement_axis"]}
        else:
            previous = movements[i - 1]

        if i == len(movements) - 1:
            last = True
        else:
            last = False

        # Create a PathPlanner instance for the current movement segment
        movement_planner = PathPlanner(
        robot_center,
        robot,
        simulation,
        lidar,
        final_position=current["final_position"],
        movement_axis=current["movement_axis"],
        previous_axis=previous["movement_axis"],
        previous_position=previous["final_position"],
        last= last,
        direction=current.get("direction", "forward")
    )

        # Execute the movement and update edges_before accordingly
        edges_before = movement_planner.move_to_target(edges_before)

    simulation.stop()


if __name__ == "__main__":
    simulate()