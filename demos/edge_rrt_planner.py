import numpy as np
import random
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------
# Class EdgeDetector
# ---------------------------------------------------------------------
# Responsible for detecting edges and potential collisions in a 3D point cloud
# using the Difference of Normals (DoN) method.
# It also filters out points considered free space, which can be used
# for path planning with the RRT algorithm.
class EdgeDetector:
    def __init__(self, lidar, voxel, small_radius, large_radius, min_distance, max_distance, threshold, margin):
        self.lidar = lidar
        self.voxel = voxel
        self.small_radius = small_radius
        self.large_radius = large_radius
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.threshold = threshold
        self.margin = margin

    # Compute the Difference of Normals(DoN) between two normal estimations
    # with different radius to detect surface variation (edges).
    def difference_of_normals(self):
        self.lidar.estimate_normals(radius=self.small_radius)
        normals_small = np.asarray(self.lidar.pointcloud.normals).copy()

        self.lidar.estimate_normals(radius=self.large_radius)
        normals_large = np.asarray(self.lidar.pointcloud.normals)

        return (normals_large - normals_small) / 2

    # Filter points based on the magnitude of the DoN vector and distance constraints.
    def filter_edge_indices(self, don_abs, data):
        distances = np.linalg.norm(data, axis=1)
        mask = (don_abs > self.threshold) & (distances < self.max_distance) & (distances > self.min_distance)
        return np.where(mask)[0]

    # Filter points likely to be in collision based on a bounding box
    # around the vehicle and vertical clearance constraints.
    def filter_collision_indices(self, data):
        if self.min_distance > 2:
            return np.array([], dtype=int)

        # Vehicle dimensions and sensor height
        size_x, size_y, size_z = 0.67, 0.99, 1.25
        clearance = 0.13
        lidar_height = 1.1325

        x, y, z = np.abs(data[:, 0]), np.abs(data[:, 1]), data[:, 2]
        mask = ((x - self.margin < size_x / 2) &
                (y - self.margin < size_y / 2) &
                (clearance < (lidar_height + z)) &
                ((lidar_height + z) < size_z))
        return np.where(mask)[0]

    # Main method to extract edge points and free (non-edge) points.
    def get_edges(self):
        self.lidar.down_sample(voxel=self.voxel)
        data = np.asarray(self.lidar.pointcloud.points)

        # Compute difference of normals and their magnitude
        diff_norm = self.difference_of_normals()
        don_abs = np.linalg.norm(diff_norm, axis=1)

        # Filter edge and collision indices (robot footprint with safety margin)
        edge_indices = self.filter_edge_indices(don_abs, data)
        collision_indices = self.filter_collision_indices(data)

        # # Colorize: gray for all, red for edges, green for collisions
        # colors = np.full((data.shape[0], 3), 0.5)
        # colors[edge_indices] = [1.0, 0.0, 0.0]         # red
        # colors[collision_indices] = [0.0, 1.0, 0.0]  # green
        # self.lidar.choose_colors(colors)
        # self.lidar.draw_pointcloud()

        # Combine both sets of indices
        total_indices = np.concatenate((edge_indices, collision_indices))
        edge_points = data[total_indices]

        # Extract points not part of edges or collisions
        mask = np.ones(len(data), dtype=bool)
        mask[total_indices] = False
        free_points = data[mask]

        return edge_points, free_points


# ---------------------------------------------------------------------
# Class Node
# ---------------------------------------------------------------------
# Represents a single node in the RRT tree. Stores the node’s coordinates
# and the index of its parent node, which is used to reconstruct the path
# once the goal is reached.
class Node:
    def __init__(self, x, y, parent_index):
        self.x = x  # x-coordinate of the node
        self.y = y  # y-coordinate of the node
        self.parent_index = parent_index  # index of the parent node in the tree

# ---------------------------------------------------------------------
# Class RRT (Rapidly-Exploring Random Tree)
# ---------------------------------------------------------------------
# Implements the RRT path planning algorithm.
# Starting from a given point, it builds a tree by exploring the free space
# while avoiding obstacles. A KDTree is used for fast nearest-neighbor queries.
# The class also handles goal adjustment if the target is in collision,
# and provides visualization of the final path and exploration tree.
class RRT:
    def __init__(self, start_point, goal_point, free_points, obstacle_points, incremental_distance=0.2,
                 max_iterations=500, threshold=0.75, goal_threshold=0.2):
        # Initialize start and goal
        self.start = np.array(start_point, dtype=np.float32)
        self.goal = np.array(goal_point, dtype=np.float32)

        # Environment points
        self.free_points = free_points
        self.obstacle_points = obstacle_points

        # Parameters
        self.incremental_distance = incremental_distance
        self.max_iterations = max_iterations
        self.threshold = threshold
        self.goal_threshold = goal_threshold
        self.x_bounds = (np.min(self.free_points[:, 0]), np.max(self.free_points[:, 0]))
        self.y_bounds = (np.min(self.free_points[:, 1]), np.max(self.free_points[:, 1]))

        # Spatial index for fast nearest neighbor search
        self.free_points_kdtree = KDTree(self.free_points)
        self.num_objects = len(obstacle_points)
        self.use_kdtree = self.num_objects > 50  # Use KDTree only if there are many obstacles for faster collision checking

        self.node_list: list[Node] = []  # Stores the nodes in the RRT tree
        self.start_check_collision = False  # Flag to lower collision threshold during start position adjustment

    # Generate a random configuration in the space
    def rand_conf(self, near_rand):
        if near_rand:
            # Bias sampling near the goal
            return np.array([
                np.random.uniform(self.goal[0] - self.goal_threshold, self.goal[0] + self.goal_threshold),
                np.random.uniform(self.goal[1] - self.goal_threshold, self.goal[1] + self.goal_threshold)
            ], dtype=np.float32)
        else:
            # Uniform sampling within the space bounds
            return np.array([random.uniform(*self.x_bounds), random.uniform(*self.y_bounds)], dtype=np.float32)

    # Find the nearest existing node to the random sample
    def nearest_vertex(self, q_rand, kd_tree):
        if kd_tree is None:
            points = np.array([[n.x, n.y] for n in self.node_list], dtype=np.float32)
            index = np.argmin(np.linalg.norm(points - q_rand, axis=1))
        else:
            _, index = kd_tree.query(q_rand)
        return self.node_list[index], index

    # Generate a new configuration by moving from q_near toward q_rand
    def new_conf(self, q_near, q_rand):
        direction = q_rand - q_near
        if np.linalg.norm(direction) < 1e-6:
            return q_near  # Avoid zero movement
        direction /= np.linalg.norm(direction)
        q_new = q_near + direction * self.incremental_distance
        _, index = self.free_points_kdtree.query(q_new)
        return self.free_points[index]

    # Check if a point is within a free space threshold
    def check_free_points(self, point):
        distance, _ = self.free_points_kdtree.query(point)
        return np.all(distance <= self.goal_threshold)

    # Check if the given point or array of points is too close to any obstacles
    def check_obstacle(self, point, obstacles):
        if self.num_objects == 0:
            return False
        if self.start_check_collision:
            threshold = self.threshold/3    # Use a tighter threshold when adjusting start position
        else:
            threshold = self.threshold
        if self.use_kdtree:
            distances, _ = obstacles.query(point, distance_upper_bound=threshold)
        else:
            if point.shape == (2,):
                distances = np.linalg.norm(self.obstacle_points - point, axis=1)
            else:
                diff = point[:, np.newaxis, :] - self.obstacle_points[np.newaxis, :, :]
                distances = np.linalg.norm(diff, axis=2)
        return np.any(distances < threshold)

    # Returns True if a collision is detected along the straight-line path between q_start and q_finish, otherwise False
    def check_path(self, q_finish, q_start, obstacles):
        if self.num_objects == 0:
            return False  # No obstacles to check
        # Discretize the path between the nodes
        x = np.linspace(q_start[0], q_finish[0], 10)
        y = np.linspace(q_start[1], q_finish[1], 10)
        path_points = np.column_stack((x, y)).astype(np.float32)
        if self.check_obstacle(path_points, obstacles):
            return True  # Collision detected
        return not self.check_free_points(path_points)

    # Reconstruct the path from goal to start
    def reconstruct_path(self):
        nodes_path = []
        current = self.node_list[-1]
        while current is not None:
            nodes_path.append([current.x, current.y])
            if current.parent_index is None:
                break
            current = self.node_list[current.parent_index]
        return np.array(nodes_path[::-1])

    # Find candidates that are near a point and aren't close to an obstacle
    def find_candidates(self, point, obstacles):
        # Find nearby free points within radius 2 of the original goal
        close_indices = self.free_points_kdtree.query_ball_point(point, r=2)
        if len(close_indices) == 0:
            return None  # No nearby free points found

        candidates = self.free_points[close_indices]

        # Filter out candidates that are in collision with obstacles
        mask_valid = [not self.check_obstacle(p, obstacles) for p in candidates]
        return candidates[mask_valid]

    # Adjusts the goal if the original goal is not in free space
    def new_goal(self, obstacles):
        candidates = self.find_candidates(self.goal, obstacles)

        if candidates is None or len(candidates) == 0:
            print("[New Goal] No nearby candidate points found.")
            return None     # No valid candidates

        # Return the candidate closest to the original goal
        closest_index = np.argmin(np.linalg.norm(candidates - self.goal, axis=1))
        return candidates[closest_index]

    # Adjusts the goal if the start position it is in collision.
    # Returns a valid nearby point if a collision-free path can be established.
    def start_collision(self, obstacles):
        candidates = self.find_candidates(self.start, obstacles)

        if candidates is None or len(candidates) == 0:
            print("[Start Collision] No nearby candidate points found.")
            return None  # No valid candidates found near the start

        self.start_check_collision = True   # Lower collision threshold during checking

        # Filter candidates where a collision-free path from the start exists
        valid_candidates = []
        for i in range(len(candidates)):
            if not self.check_path(candidates[i], self.start, obstacles):
                valid_candidates.append(candidates[i])

        self.start_check_collision = False

        if len(valid_candidates) == 0:
            print("[Start Collision] All nearby candidates result in collision.")
            return None  # No valid path to any candidate

        valid_candidates = np.array(valid_candidates)

        # Select candidate closest to the original goal
        closest_index = np.argmin(np.linalg.norm(valid_candidates - self.goal, axis=1))
        return valid_candidates[closest_index]

    # Builds the RRT tree starting from the start position toward the goal.
    # Returns the planned path (as a list of points) and a boolean indicating if start adjustment was used
    def build_rrt(self):
        # Create KDTree for obstacles if there are many
        obstacles_kdtree = None
        if self.use_kdtree:
            obstacles_kdtree = KDTree(self.obstacle_points)

        # Abort if goal is in collision
        if self.check_obstacle(self.goal, obstacles_kdtree):
            print('Goal is in collision')
            return None, False

        # Check if start is in collision or very close;
        if self.check_obstacle(self.start, obstacles_kdtree):
            new_goal = self.start_collision(obstacles_kdtree)
            if new_goal is None:
                return None, False   # No reachable replacement for the goal
            return np.array([[self.start[0], self.start[1]],[new_goal[0], new_goal[1]]]), True

        # Check if goal is in free space; adjust if not
        if not self.check_free_points(self.goal):
            new_goal = self.new_goal(obstacles_kdtree)
            if new_goal is None:
                return None, False   # No reachable replacement for the goal
            else:
                self.goal = new_goal

        # Initialize tree with the start node
        start_node = Node(self.start[0], self.start[1], None)
        self.node_list = [start_node]

        # Check if a direct path from start to goal is collision-free
        collision = self.check_path(self.goal, self.start, obstacles_kdtree)
        if not collision:
            # Direct connection to goal is possible
            goal_node = Node(self.goal[0], self.goal[1], 0)
            self.node_list.append(goal_node)
            return self.reconstruct_path(), False

        near_rand = False
        kd_tree = None
        points = []
        # Main RRT loop
        for i in range(self.max_iterations):
            q_rand = self.rand_conf(near_rand)

            # Use KDTree to speed up nearest neighbor search if many nodes exist
            if len(self.node_list) > 200:
                if len(points) != len(self.node_list):  # Rebuilding KDTree only when node_list changes
                    points = np.array([[n.x, n.y] for n in self.node_list])
                    kd_tree = KDTree(points)
            else:
                kd_tree = None

            # Find the nearest existing node to the random sample
            q_near, index = self.nearest_vertex(q_rand, kd_tree)
            q_near_pos = np.array([q_near.x, q_near.y], dtype=np.float32)

            # If close to goal, stop and reconstruct path
            if np.linalg.norm(q_near_pos - self.goal) <= self.goal_threshold:
                goal_node = Node(self.goal[0], self.goal[1], index)
                self.node_list.append(goal_node)
                return self.reconstruct_path(), False

            # Extend toward q_rand to create q_new
            q_new = self.new_conf(q_near_pos, q_rand)
            if np.linalg.norm(q_new - q_near_pos) < 1e-3:
                continue  # Skip nodes too close

            # Check for collision at the new point
            if self.check_obstacle(q_new, obstacles_kdtree):
                continue

            # Add new node to the tree
            new_node = Node(q_new[0], q_new[1], index)
            self.node_list.append(new_node)

            # Activate goal biasing when close
            if np.linalg.norm(q_new - self.goal) < 1.5 * self.threshold:
                near_rand = True

        return None, False  # Path not found

    # Visualize the RRT tree, obstacles, start and goal
    def draw(self, path_nodes):
        plt.figure(figsize=(10, 10))
        plt.title("RRT Path")
        plt.xlabel("X")
        plt.ylabel("Y")

        # Draw tree edges
        for node in self.node_list:
            if node.parent_index is not None:
                parent = self.node_list[node.parent_index]
                plt.plot([node.x, parent.x], [node.y, parent.y], 'g-', linewidth=0.5)
                plt.plot(node.x, node.y, 'go', markersize=2)

        # Draw obstacles
        if self.obstacle_points.size > 0:
            obs_x, obs_y = self.obstacle_points[:, 0], self.obstacle_points[:, 1]
            plt.plot(obs_x, obs_y, 'ks', markersize=4, label='Obstacles')

        # Draw start and goal
        plt.plot(self.start[0], self.start[1], 'bo', markersize=8, label='Start')
        plt.plot(self.goal[0], self.goal[1], 'ro', markersize=8, label='Goal')

        # Draw final path
        if path_nodes is not None:
            plt.plot(path_nodes[:, 0], path_nodes[:, 1], 'r-', linewidth=2, label='Found Path')

        plt.legend(loc='upper right')
        plt.axis('equal')
        plt.grid(True)
        plt.show()

    # Run the RRT algorithm and display result
    def run(self):
        path_nodes, start_collision = self.build_rrt()
        if path_nodes is None:
            print("Couldn't find path")
        # else:
        #     for i, node in enumerate(path_nodes):
        #         print(f"Iteration {i}: x = {node[0]:.3f}, y = {node[1]:.3f}")
        # self.draw(path_nodes)
        return path_nodes, start_collision
