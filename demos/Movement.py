import numpy as np
from edge_rrt_planner import RRT, EdgeDetector

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
    return w, v

# Generates a list of intermediate points for the robot path based on movement direction and axis.
def waypoints(movement_axis, final_position, previous_axis, previous_position, axis_sign):
    if previous_axis == movement_axis:
        point1 = None
    else:
        point1 = previous_position
    initial_position = previous_position[movement_axis]
    points_movement = np.round(np.arange(initial_position, final_position[movement_axis], axis_sign), 3).tolist()
    if points_movement[-1] != final_position[movement_axis]:
        points_movement.append(round(final_position[movement_axis], 3))
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

def path_points(lidar, start_point, goal_point, current_point):
    # Extract edge and free points
    near_detector = EdgeDetector(lidar, voxel=0.12, small_radius=0.2, large_radius=0.3, min_distance=0, max_distance=5, threshold=0.05, margin=0.25)
    near_edges, near_free = near_detector.get_edges()
    far_detector = EdgeDetector(lidar, voxel=0.25, small_radius=0.4, large_radius=1, min_distance=5, max_distance=10, threshold=0.15, margin=0.25)
    far_edges, far_free = far_detector.get_edges()
    #Add free points that are very close to the robot but aren't seen by lidar
    min_z = min(near_free[:, 2])
    anillos = [np.array([[0, 0, min_z]])]
    radius = np.arange(0.2, 2.5, 0.2)
    for r in radius:
        num_points = int(np.ceil(2 * np.pi * r / 0.2))
        thetas = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        xs = r * np.cos(thetas)
        ys = r * np.sin(thetas)
        zs = np.full_like(xs, min_z)
        ring_points = np.column_stack((xs, ys, zs))
        anillos.append(ring_points)
    inside_points = np.vstack(anillos)

    # Combine all edge and free points, far_free not needed because it's included inside near_free
    all_edges = np.concatenate((near_edges, far_edges), axis=0)
    all_free = np.concatenate((inside_points, near_free), axis=0)

    # # Visualize with near and far edges all together
    # all_points = np.concatenate((all_free, all_edges), axis=0)
    # colors_edges = np.tile([1.0, 0.0, 0.0], (all_edges.shape[0], 1))
    # colors_free = np.tile([0.5, 0.5, 0.5], (all_free.shape[0], 1))
    # lidar.from_points(all_points)
    # colors = np.concatenate((colors_free, colors_edges), axis=0)
    # lidar.choose_colors(colors)
    # lidar.draw_pointcloud()
    # Edges that over the robot are eliminated, since we will use a 2D dimension map for RRT and may cause problems
    # all_edges = all_edges[all_edges[:, 2] <= 0.5]
    free = all_free[:, :2] + current_point
    edges = all_edges[:, :2] + current_point
    rrt_instance = RRT(start_point, goal_point, free, edges)
    path_nodes = rrt_instance.run()
    return path_nodes


def straight_movement(path, robot_center, points, direction):
    current_point = path[0]
    target_point = path[1]
    delta_x = target_point[0] - current_point[0]
    delta_y = target_point[1]- current_point[1]
    target_angle = np.degrees(np.arctan2(delta_y, delta_x))
    target_angle = (target_angle + 180) % 360 - 180
    current_angle = get_angle(robot_center)
    angular_error = angle_difference(current_angle, target_angle)
    v = 1
    if np.array_equal(target_point, points[-1]):
        v = 0.25
    if np.array_equal(target_point, points[-2]):
        v = 0.5
    if abs(angular_error) < 1:
        w = 0
    elif abs(angular_error) > 15:
        if abs(angular_error) > 30:
            w = np.clip(angular_error / 5, -1.5, 1.5)
        w = np.clip(angular_error / 5, -1, 1)
        v = min(0.5, v)
    else:
        w = np.clip(angular_error / 10, -0.5, 0.5)
    if direction == 'backward':
        v = -v
    return v, w

def rrt_movement(path, robot_center, i, direction):
    current_point = path[i]
    target_point = path[i+1]
    delta_x = target_point[0] - current_point[0]
    delta_y = target_point[1] - current_point[1]
    target_angle = np.degrees(np.arctan2(delta_y, delta_x))
    target_angle = (target_angle + 180) % 360 - 180
    current_angle = get_angle(robot_center)
    angular_error = angle_difference(current_angle, target_angle)
    v = 0.25
    if abs(angular_error) < 1:
        w = 0
    elif abs(angular_error) > 15:
        if abs(angular_error) > 30:
            w = np.clip(angular_error / 5, -1.5, 1.5)
        w = np.clip(angular_error / 5, -1, 1)
    else:
        w = np.clip(angular_error / 10, -0.5, 0.5)
    if direction == 'backward':
        v = -v
    return v, w

# Performs straight-line movement of the robot with heading and velocity corrections.
def movement(robot_center, robot, simulation, lidar, final_position, movement_axis, previous_axis, previous_position, direction ='forward'):
    print('ROBOT STRAIGHT')
    axis_sign = sign_axis(final_position, previous_position, movement_axis)
    points = np.array(waypoints(movement_axis, final_position, previous_axis, previous_position, axis_sign))
    # target_angle = path_angle(movement_axis, direction, axis_sign)
    # print('target_angle', target_angle)
    # lateral_axis = 1 - movement_axis
    i = 1
    threshold = 0.25
    next_point = 0
    while True:
        if next_point > 0:
            i = i - next_point
            next_point = 0

        T = robot_center.get_transform()
        current_position = T[0:2, 3]
        distance = (points[i, movement_axis] - current_position[movement_axis])
        if distance * axis_sign <= threshold:
            i += 1
            if i >= len(points):
                break



        lidar.get_laser_data()
        path = path_points(lidar, current_position, points[i], current_position)
        while(path is None and i < len(points) - 1 ):
            print('Buscar camino')
            i += 1
            T = robot_center.get_transform()
            current_position = T[0:2, 3]
            lidar.get_laser_data()
            path = path_points(lidar, current_position, points[i], current_position)
            next_point += 1

        if i == len(points) or path is None:
            print('No hay camino')
            break

        if len(path) == 2:
            for j in range(3):
                T = robot_center.get_transform()
                current_position = T[0:2, 3]
                short_path = [current_position, path[1]]
                distance_error = distance_difference(robot_center, final_position[movement_axis], movement_axis)
                if distance_error * axis_sign <= threshold:
                    robot.move(v=0, w=0)
                    break
                v, w = straight_movement(short_path, robot_center, points, direction)
                robot.move(v, w)
                simulation.wait()
        elif len(path) > 2:
            print('Muchos puntos')
            for j in range(10):
                if j + 1 >= len(path):
                    break
                distance_error = distance_difference(robot_center, final_position[movement_axis], movement_axis)
                if distance_error * axis_sign <= threshold:
                    robot.move(v=0, w=0)
                    break
                v, w = rrt_movement(path, robot_center, j, direction)
                robot.move(v, w)
                simulation.wait()

# # Performs straight-line movement of the robot with heading and velocity corrections.
# def straight_movement(robot_center, robot, simulation, final_position, movement_axis, previous_axis, previous_position, direction ='forward'):
#     print('ROBOT STRAIGHT')
#     axis_sign = sign_axis(final_position, previous_position, movement_axis)
#     target_angle = path_angle(movement_axis, direction, axis_sign)
#     print('target_angle', target_angle)
#     lateral_axis = 1 - movement_axis
#     points = waypoints(movement_axis, final_position, previous_axis, previous_position, axis_sign)
#     while True:
#         current_angle = get_angle(robot_center)
#         distance_error = distance_difference(robot_center, final_position[movement_axis], movement_axis)
#         w, v = velocities(robot_center, current_angle, target_angle, direction, lateral_axis, final_position, distance_error)
#         threshold = 0.25
#         if distance_error * axis_sign <= threshold:
#             robot.move(v=0, w=0)
#             break
#         robot.move(v=v, w=w)
#         T = robot_center.get_transform()
#         print(T)
#         print('Angular velocity (w):', w)
#         print('Linear velocity (v):', v)
#         simulation.wait()