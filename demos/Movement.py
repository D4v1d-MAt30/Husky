import numpy as np
from scipy.spatial.transform import Rotation as R

# Calculates and returns the robot's inclination angle in degrees.
def get_inclination(robot_center):
    T = robot_center.get_transform()
    Rotation = T[0:3, 0:3]
    rot = R.from_matrix(Rotation)
    angles = rot.as_euler('zyx', degrees=True)
    pitch = angles[1]
    return pitch

# Calculates and returns the robot's orientation angle in degrees.
def get_angle(robot_center):
    T = robot_center.get_transform()
    theta_rad = np.arctan2(T[1,0],T[0,0])
    theta_deg = np.degrees(theta_rad)
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
def path_angle(axis, direction, axis_sign):
    positive_angles = [0, 90]
    negative_angles = [180, -90]
    if axis not in (0, 1):
        raise ValueError("axis must be 0 (x) or 1 (y)")

    if direction == 'forward':
        moving_direction = 1
    else: moving_direction = -1

    if axis_sign * moving_direction == 1:
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

#Calculates the angular correction needed for maintaining correct heading.
def rotation_correction(current_angle, lateral_error, target_angle, direction):
    angular_error = angle_difference(current_angle, target_angle)
    if abs(angular_error) < 0.5:
        w1 = 0
    elif abs(angular_error) > 15:
        w1 = np.clip(angular_error / 5, -1, 1)
    else:
        w1 = np.clip(angular_error / 10, -0.5, 0.5)
    if abs(lateral_error) < 0.01:
        w2 = 0
    else:
        w2 = np.clip(lateral_error, -1,1)
        sign = w2_sign(target_angle, direction)
        w2 = w2 * sign
    print('Valor w1:', w1, 'Valor w2:', w2)
    w = w1 + w2
    return w

# Returns angular and linear velocity needed to follow the path.
def velocities(robot_center, current_angle, target_angle, direction, lateral_axis, final_position, distance_error):
    angular_error = angle_difference(current_angle, target_angle)
    lateral_error = distance_difference(robot_center, final_position[lateral_axis], lateral_axis)
    if abs(angular_error) > 15 and abs(lateral_error) < 1 :
        v = 0.25
        w = np.clip(angular_error / 5, -1, 1)
    else:
        v = np.clip(abs(distance_error) / 3, 0.5, 1)
        w = rotation_correction(current_angle, lateral_error, target_angle, direction)
    if direction == 'backward':
        v = -v
    pitch = get_inclination(robot_center)
    if pitch > 15:
        v = 2 * v
    return w, v

# Generates a list of intermediate points for the robot path based on movement direction and axis.
def path_points(movement_axis, final_position, previous_axis, previous_position, axis_sign):
    if previous_axis == movement_axis:
        point1 = None
    else:
        point1 = previous_position
    initial_position = previous_position[movement_axis]
    points_movement = np.round(np.arange(initial_position, final_position[movement_axis], axis_sign), 3).tolist()
    points = []
    if point1:
        points.append(list(point1))
    for value in points_movement:
        if movement_axis == 0:
            points.append([value, final_position[1]])
        else:
            points.append([final_position[0], value])
    return points

# Returns 1 if the movement along the specified axis is positive, -1 if it's negative.
def sign_axis(final_position, previous_position, movement_axis):
    if final_position[movement_axis] > previous_position[movement_axis]:
        axis_sign = 1
    else:
        axis_sign = -1
    return axis_sign

# Performs straight-line movement of the robot with heading and velocity corrections.
def straight_movement(robot_center, robot, simulation, final_position, movement_axis, previous_axis, previous_position, direction ='forward'):
    print('ROBOT STRAIGHT')
    axis_sign = sign_axis(final_position, previous_position, movement_axis)
    target_angle = path_angle(movement_axis, direction, axis_sign)
    print('target_angle', target_angle)
    lateral_axis = 1 - movement_axis
    points = path_points(movement_axis, final_position, previous_axis, previous_position, axis_sign)
    while True:
        current_angle = get_angle(robot_center)
        distance_error = distance_difference(robot_center, final_position[movement_axis], movement_axis)
        w, v = velocities(robot_center, current_angle, target_angle, direction, lateral_axis, final_position, distance_error)
        threshold = 0.25
        if distance_error * axis_sign <= threshold:
            robot.move(v=0, w=0)
            break
        robot.move(v=v, w=w)
        T = robot_center.get_transform()
        print(T)
        print('Angular velocity (w):', w)
        print('Linear velocity (v):', v)
        simulation.wait()