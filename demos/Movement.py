import numpy as np
from edge_rrt_planner import RRT, EdgeDetector
from scipy.spatial.transform import Rotation as R

class PathPlanner:
    def __init__(self, robot_center, robot, simulation, lidar, final_position,
                 movement_axis, previous_axis, previous_position, last, direction = 'forward'):
        """
                Initializes the PathPlanner object.
                Args:
                    robot_center: Reference to the robot's center transform.
                    robot: Robot movement controller.
                    simulation: Simulation object.
                    lidar: LIDAR sensor object.
                    final_position: Final [x, y] goal position.
                    movement_axis: Axis of main movement (0 for x, 1 for y).
                    previous_axis: Previous movement axis.
                    previous_position: Previous position of the robot.
                    last: Flag to trigger path drawing.
                    direction: 'forward' or 'backward' motion.
                """
        self.robot_center = robot_center
        self.robot = robot
        self.simulation = simulation
        self.lidar = lidar
        self.final_position = final_position
        self.movement_axis = movement_axis
        self.previous_axis = previous_axis
        self.previous_position = previous_position
        self.direction = direction
        self.edges_before =  np.empty((0, 2))
        self.should_draw_path = last
        self.error_recta_sum = 0.0
        self.error_recta_count = 0
        self.real_trajectory = []
        self.planned_trajectories = []

    # Returns the current yaw angle of the robot in degrees.
    def get_angle(self):
        T = self.robot_center.get_transform()
        theta_rad = np.arctan2(T[1,0],T[0,0])
        theta_deg = np.degrees(theta_rad)
        return theta_deg

    # Returns the current (x, y) position of the robot as a 2D point.
    def get_current_point(self):
        T = self.robot_center.get_transform()
        return T[0:2, 3]

    # Checks if robot inclination exceeds a threshold (5 degrees).
    def get_inclination(self):
        T = self.robot_center.get_transform()
        rot = R.from_matrix(T[0:3, 0:3])
        angles = rot.as_euler('zyx', degrees=True)
        return abs(angles[1]) > 5 or (angles[2] > 5)

    #Returns the difference between current and target position along the movement axis.
    def distance_difference(self):
        if self.movement_axis not in (0, 1):
            raise ValueError("axis must be 0 (x) or 1 (y)")
        T = self.robot_center.get_transform()
        return self.final_position[self.movement_axis] - T[self.movement_axis, 3]

    #Returns smallest signed angle difference between a2 and a1, between -180 and 180
    def angle_difference(self, a1, a2):
        return (a2 - a1 + 180) % 360 - 180

    # Compute angular error from current to target point.
    def compute_target_angle_error(self, current_point, target_point):
        delta = target_point - current_point
        target_angle = np.degrees(np.arctan2(delta[1], delta[0]))
        target_angle = (target_angle + 180) % 360 - 180
        current_angle = self.get_angle()
        return self.angle_difference(current_angle, target_angle)

    # Returns the sign of movement along the specified axis:
    # +1 if moving in positive direction, -1 if moving in negative direction.
    def sign_axis(self):
        if self.final_position[self.movement_axis] > self.previous_position[self.movement_axis]:
            return 1
        else:
            return -1

    # Checks if the robot has reached (or passed) the target along the movement axis within a given threshold.
    def has_reached_target(self, axis_sign, threshold):
        distance_error = self.distance_difference()
        return distance_error * axis_sign <= threshold

    # Checks if the robot has reached (or passed) the waypoint along the movement axis within a given threshold.
    def has_reached_waypoint(self, current_position, target_point, axis_sign, threshold):
        distance = target_point[self.movement_axis] - current_position[self.movement_axis]
        return distance * axis_sign <= threshold

    # Generates a list of intermediate 2D waypoints along the movement axis toward final_position.
    # Returns: List of [x, y] points.
    def waypoints(self, axis_sign):
        if self.previous_axis == self.movement_axis:
            point1 = None
        else:
            point1 = self.previous_position

        initial = self.previous_position[self.movement_axis]
        target = self.final_position[self.movement_axis]
        points_movement = np.round(np.arange(initial, target, axis_sign), 3).tolist()

        if points_movement[-1] != target:
            points_movement.append(round(target, 3))

        points = []
        if point1:
            points.append(list(point1))

        for value in points_movement:
            if self.movement_axis == 0:
                points.append([value, self.final_position[1]])
            else:
                points.append([self.final_position[0], value])
        return points

    # Generates a collision-free path using RRT. Returns: path_nodes: list of 2D path points,
    # near_edges: detected edge points, start_collision: bool flag indicating collision at start
    def path_points(self, goal_point, current_point):
        # Extract edge and free points
        near_detector = EdgeDetector(self.lidar, voxel=0.12, small_radius=0.2, large_radius=0.3, min_distance=0, max_distance=4, threshold=0.045, margin=0.25)
        near_edges, near_free = near_detector.get_edges()
        far_detector = EdgeDetector(self.lidar, voxel=0.25, small_radius=0.4, large_radius=1, min_distance=4, max_distance=10, threshold=0.15, margin=0.25)
        far_edges, far_free = far_detector.get_edges()

        # Add synthetic free points (rings of points) around the robot at ground level
        min_z = min(near_free[:, 2])
        rings = [np.array([[0, 0, min_z]])]
        for r in np.arange(0.2, 2.5, 0.2):
            num_points = int(np.ceil(2 * np.pi * r / 0.2))
            thetas = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
            xs = r * np.cos(thetas)
            ys = r * np.sin(thetas)
            zs = np.full_like(xs, min_z)
            ring_points = np.column_stack((xs, ys, zs))
            rings.append(ring_points)
        inside_points = np.vstack(rings)

        # Combine all edge and free points; far_free is excluded since it's already in near_free
        all_edges = np.concatenate((near_edges, far_edges), axis=0)
        all_free = np.concatenate((inside_points, near_free), axis=0)

        # Optional visualization of the combined point cloud (commented out)
        # all_points = np.concatenate((all_free, all_edges), axis=0)
        # colors_edges = np.tile([1.0, 0.0, 0.0], (all_edges.shape[0], 1))
        # colors_free = np.tile([0.5, 0.5, 0.5], (all_free.shape[0], 1))
        # self.lidar.from_points(all_points)
        # colors = np.concatenate((colors_free, colors_edges), axis=0)
        # self.lidar.choose_colors(colors)
        # self.lidar.draw_pointcloud()

        # Filter out edges that are above the robot sensor (keep only those below 0.2m)
        all_edges = all_edges[all_edges[:, 2] <= 0.2]
        near_edges = near_edges[near_edges[:, 2] <= 0.2]

        # Shift free and edge points relative to the robot's current position
        free = all_free[:, :2] + current_point
        current_edges = all_edges[:, :2] + current_point

        # Combine current and previously known edges
        edges = np.concatenate((current_edges, self.edges_before), axis=0)

        # Run RRT (Rapidly-exploring Random Tree) path planning
        rrt_instance = RRT(current_point, goal_point, free, edges)
        path_nodes, start_collision = rrt_instance.run()

        # Draw the RRT path if the goal corresponds to the final gymkhana point ([0, 0.5]) and drawing is enabled
        if np.array_equal(goal_point, np.array([0, 0.5])) and self.should_draw_path :
            rrt_instance.draw(path_nodes)

        return path_nodes, near_edges[:, :2] + current_point, start_collision

    # Calculates angular velocity and adjusts linear speed based on angular error and movement direction.
    def calculate_turning_velocity(self, angular_error, v):
        abs_error = abs(angular_error)

        if abs_error < 1:
            w = 0
        elif abs_error > 15:
            if abs_error > 30:
                w = np.clip(angular_error / 5, -1.5, 1.5)
            else:
                w = np.clip(angular_error / 5, -1, 1)
            v = min(0.5, v)
        else:
            w = np.clip(angular_error / 10, -0.5, 0.5)

        # Reverse direction if moving backwards
        if self.direction == 'backward':
            v = -v

        return v, w

    # Calculates linear and angular velocity to follow a straight path.
    # Slows down near the end or during collisions/inclination.
    def straight_movement(self, path, points, start_collision):
        current_point = path[0]
        target_point = path[1]
        angular_error = self.compute_target_angle_error(current_point, target_point)

        # Velocity profile
        v = 1
        if np.array_equal(target_point, points[-1]):
            v = 0.25
        if np.array_equal(target_point, points[-2]):
            v = 0.5

        v, w = self.calculate_turning_velocity(angular_error, v)

        if self.get_inclination() or start_collision:
            v = 0.25

        return v, w

    # Calculates velocity and angular velocity to follow RRT path segment.
    def rrt_movement(self, path, i):
        current_point = self.get_current_point()
        target_point = path[i+1]
        angular_error = self.compute_target_angle_error(current_point, target_point)

        v = 0.25

        v, w = self.calculate_turning_velocity(angular_error, v)

        return v, w

    def point_to_path_distance(self, path):
        """ Calcula la distancia mínima entre un punto real del robot y una trayectoria planificada (polilínea formada por los nodos del path). point : [x, y] posición real del robot path : Nx2 array de nodos devueltos por edge_rrt_planner """

        point = np.array(self.get_current_point())
        min_dist = np.inf
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            ab = b - a
            ap = point - a  # Proyección del punto sobre el segmento
            if np.allclose(ab, 0):
                dist = np.linalg.norm(ap)
            else:
                t = np.dot(ap, ab) / np.dot(ab, ab)
                t = np.clip(t, 0.0, 1.0)
                closest = a + t * ab
                dist = np.linalg.norm(point - closest)
            min_dist = min(min_dist, dist)
        return min_dist


    # Executes movement logic for the robot to reach final_position.
    # Uses straight-line or RRT-based navigation depending on environment.
    def move_to_target(self, edges_before):
        """
           Main control loop that moves the robot toward final_position using either straight-line
           or RRT-based navigation, depending on the local environment.

           This method:
           - Generates intermediate waypoints between the previous and final positions.
           - For each waypoint, attempts to compute a collision-free path using RRT.
           - If RRT fails for the current waypoint, it tries the next one.
           - Moves the robot step by step toward the waypoint, using either straight-line
             or RRT-based control, while checking for collisions and progress.

           Args:
               edges_before: Array of previously detected obstacle edges to include in planning.

           Returns:
               Updated edge points after navigation (combined previous and new edges).
           """
        print('New final position')

        axis_sign = self.sign_axis()        # Determine direction of movement along the axis
        points = np.array(self.waypoints(axis_sign))
        self.edges_before = edges_before
        waypoint_index = 1
        threshold = 0.25    # Distance threshold to consider a waypoint as reached
        skipped_waypoints = 0

        while True:
            # If skipped waypoints previously, backtrack
            if skipped_waypoints > 0:
                waypoint_index -= skipped_waypoints
                skipped_waypoints = 0

            current_position = self.get_current_point()

            # Check if current waypoint is reached along the movement axis
            if self.has_reached_waypoint(current_position, points[waypoint_index], axis_sign, threshold):
                waypoint_index += 1
                if waypoint_index >= len(points):
                    return self.edges_before, self.error_recta_sum, self.error_recta_count, np.array(self.planned_trajectories), np.array(self.real_trajectory)    # All waypoints reached

            # Update LIDAR and try to find a path to the next waypoint. If path is not found, skip to the next waypoint
            self.lidar.get_laser_data()
            path, near_edges, start_collision = self.path_points(points[waypoint_index], current_position)
            while path is None and waypoint_index < len(points) - 1 :
                waypoint_index += 1
                current_position = self.get_current_point()
                self.lidar.get_laser_data()
                path, near_edges, start_collision = self.path_points(points[waypoint_index], current_position)
                skipped_waypoints += 1

            self.edges_before = np.concatenate((self.edges_before, near_edges), axis=0)

            if waypoint_index == len(points) or path is None:
                return self.edges_before, self.error_recta_sum, self.error_recta_count, np.array(self.planned_trajectories), np.array(self.real_trajectory)

            current_path_reference = path.copy()

            if len(path) == 2:
                # Straight-line segment (2 nodes)
                for j in range(3):           # Repeat up to 3 times (max 0.3m movement)
                    current_position = self.get_current_point()

                    if self.has_reached_waypoint(current_position, points[waypoint_index], axis_sign, threshold):
                        break

                    current_path = [current_position, path[1]]

                    # Final stop condition if close to goal
                    if self.has_reached_target(axis_sign, threshold):
                        self.robot.move(v=0, w=0)
                        return self.edges_before, self.error_recta_sum, self.error_recta_count, np.array(self.planned_trajectories), np.array(self.real_trajectory)

                    error = self.point_to_path_distance(current_path_reference)

                    self.error_recta_sum += error
                    self.error_recta_count += 1

                    self.real_trajectory.append(self.get_current_point().copy())
                    self.planned_trajectories.append(path[1].copy())

                    # Compute velocities and move
                    v, w = self.straight_movement(current_path, points, start_collision)
                    self.robot.move(v, w)
                    self.simulation.wait()

            elif len(path) > 2:
                # Complex path generated via RRT
                for j in range(10):         # Repeat up to 10 times (~0.25m movement)
                    if j + 1 >= len(path):
                        break

                    # Final stop condition if close to goal
                    if self.has_reached_target(axis_sign, threshold):
                        self.robot.move(v=0, w=0)
                        return self.edges_before, self.error_recta_sum, self.error_recta_count, np.array(self.planned_trajectories), np.array(self.real_trajectory)


                    self.real_trajectory.append(self.get_current_point().copy())
                    self.planned_trajectories.append(path[j + 1].copy())

                    # Compute velocities and move
                    v, w = self.rrt_movement(path, j)
                    self.robot.move(v, w)
                    self.simulation.wait()
