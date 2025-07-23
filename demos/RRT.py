import numpy as np
import random
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y, parent_index):
        self.x = x
        self.y = y
        self.parent_index = parent_index

#Generates a random configuration point within given x and y limits.
def rand_conf(x_size, y_size):
        q_rand = np.array([round(random.uniform(*x_size), 3), round(random.uniform(*y_size), 3)], dtype=np.float32)
        return q_rand

#Finds the nearest node in the tree to the random point qrand. Uses KDTree for efficient nearest neighbor search if needed
def nearest_vertex(node_list, q_rand, kd_tree):
    if kd_tree is None:
        points = np.array([[n.x, n.y] for n in node_list], dtype=np.float32)
        distances = np.linalg.norm(points - q_rand, axis=1)
        index = np.argmin(distances)
        return node_list[index], index
    else:
        _, index = kd_tree.query(q_rand)
        return node_list[index], index

#Generates a new node configuration by moving from q_near to q_rand by a step_size.
# Then the new configuration is the nearest free point.
def new_conf(q_near, q_rand, incremental_distance, free_points_kdtree, free_points):
    q_near_pos = np.array([q_near.x, q_near.y], dtype=np.float32)
    direction = q_rand - q_near_pos
    #If q_rand is the same point as q_near it returns q_near as q_new
    if q_rand == q_near:
        return np.array([q_near.x, q_near.y])
    direction_unitary = direction/np.linalg.norm(direction)
    q_new = q_near_pos + direction_unitary * incremental_distance
    _, index = free_points_kdtree.query(q_new)
    return free_points[index]

#Checks if the path between q_old and q_new collides with any obstacles, or they are very close to obstacles.
def check_collision(q_new, q_old, obstacle_points, threshold, use_kdtree, goal_point, goal_threshold):
    x = np.linspace(q_old.x, q_new[0], 10)
    y = np.linspace(q_old.y, q_new[1], 10)
    path_points = np.column_stack((x, y)).astype(np.float32)

    goal = np.array(goal_point, dtype=np.float32)
    distances_to_goal = np.linalg.norm(path_points - goal, axis=1)
    if np.any(distances_to_goal < goal_threshold):
        return "goal"

    if use_kdtree:
        distances,_ = obstacle_points.query(path_points, distance_upper_bound=threshold)
    else:
        diff = path_points[:, np.newaxis, :] - obstacle_points[np.newaxis, :, :]
        distances = np.linalg.norm(diff, axis=2)

    if np.any(distances < threshold):
        return True
    else :
        return False

#Reconstructs the path from the last node back to the start node.
def path(node_list):
    nodes_path = []
    current_node = node_list[-1]
    while current_node is not None:
        nodes_path.append(current_node)
        if current_node.parent_index is None:
            break
        current_node = node_list[current_node.parent_index]
    return nodes_path[::-1]

#Builds the RRT tree and searches for a path from start to goal.
def rrt(start_point, goal_point, incremental_distance, max_iterations, free_points, obstacle_points, threshold, goal_threshold):
    x_size = (np.min(free_points[:, 0]), np.max(free_points[:, 0]))
    y_size = (np.min(free_points[:, 1]), np.max(free_points[:, 1]))

    free_points_kdtree = KDTree(free_points)
    #We only use kdtree when there are more than 50 obstacles
    num_obstacles = len(obstacle_points)
    use_kdtree = num_obstacles > 50

    obstacles_kdtree = None
    if use_kdtree:
        obstacles_kdtree = KDTree(obstacle_points)

    node_list = [Node(start_point[0], start_point[1], None)]

    for i in range (max_iterations):
        q_rand = rand_conf(x_size, y_size)

        # We only use kdtree when there are more than 200 nodes
        if len(node_list) > 200:
            points = np.array([[node.x, node.y] for node in node_list], dtype=np.float32)
            kd_tree = KDTree(points)
        else:
            kd_tree = None

        q_near, q_near_index = nearest_vertex(node_list, q_rand, kd_tree)
        q_new = new_conf(q_near, q_rand, incremental_distance, free_points_kdtree, free_points)

        if num_obstacles == 0:
            collision = False
        else:
            if use_kdtree:
                collision = check_collision(q_new, q_near, obstacles_kdtree, threshold, use_kdtree, goal_point, goal_threshold)
            else:
                collision = check_collision(q_new, q_near, obstacle_points, threshold, use_kdtree, goal_point, goal_threshold)

        if collision == "goal":
            new_node = Node(goal_point[0], goal_point[1], q_near_index)
            node_list.append(new_node)
            path_nodes = path(node_list)
            return node_list, path_nodes

        if collision == False:
            new_node = Node(q_new[0], q_new[1], q_near_index)
            node_list.append(new_node)

            # Check if goal is reached
            if np.linalg.norm(q_new - np.array(goal_point)) < goal_threshold:
                path_nodes = path(node_list)
                return node_list, path_nodes

    return node_list, []

#Visualizes the RRT tree, obstacles, start/goal points, and the found path.
def draw_rrt(node_list, start_point, goal_point, path_nodes, obstacle_points):
    plt.figure(figsize=(10, 10))
    plt.title("RRT Path")
    plt.xlabel("X")
    plt.ylabel("Y")

    # Draw tree edges and nodes
    for node in node_list:
        if node.parent_index is not None:
            parent = node_list[node.parent_index]
            plt.plot([node.x, parent.x], [node.y, parent.y], 'g-', linewidth=0.5)
            plt.plot(node.x, node.y, 'go', markersize=2)

    # Draw obstacles
    if obstacle_points.size > 0:
        obs_x, obs_y = obstacle_points[:, 0], obstacle_points[:, 1]
        plt.plot(obs_x, obs_y, 'ks', markersize=4, label='Obstacles')

    # Draw start and goal points
    plt.plot(start_point[0], start_point[1], 'bo', markersize=8, label='Start')
    plt.plot(goal_point[0], goal_point[1], 'ro', markersize=8, label='Goal')

    # Draw the found path
    path_x = [node.x for node in path_nodes]
    path_y = [node.y for node in path_nodes]
    plt.plot(path_x, path_y, 'r-', linewidth=2, label='Found Path')

    plt.legend(loc = 'upper right')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Run the RRT algorithm and print the path nodes found. Then visualize the tree and path.
def rrt_algorithm(start_point, goal_point, incremental_distance, max_iterations, free_points, obstacle_points, threshold, goal_threshold):
    node_list, path_nodes = rrt(start_point, goal_point, incremental_distance, max_iterations, free_points, obstacle_points, threshold, goal_threshold)
    if not path_nodes :
        print("Couldn't find path")
    for i, node in enumerate(path_nodes):
        print(f"Iteration {i}: x = {node.x}, y = {node.y}")
    draw_rrt(node_list, start_point, goal_point, path_nodes, obstacle_points)
    return path_nodes
