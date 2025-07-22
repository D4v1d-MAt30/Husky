import numpy as np
import random

class Node:
    def __init__(self, x, y, parent_index):
        self.x = x
        self.y = y
        self.parent_index = parent_index

def rand_conf(x_size, y_size):
        min_x, max_x = x_size
        min_y, max_y = y_size
        qrand = np.array([round(random.uniform(min_x, max_x), 3), round(random.uniform(min_y, max_y), 3)])
        return qrand

def nearest_vertex(node_list, qrand):
    shortest_distance = float('inf')
    qnear = ()
    qnear_index = None
    for i, node in enumerate(node_list):
        distance = np.linalg.norm(np.array(qrand) - np.array([node.x, node.y]))
        if distance < shortest_distance:
            shortest_distance = distance
            qnear = node
            qnear_index = i
    return qnear, qnear_index

def new_conf(qnear, qrand, incremental_distance, free_points):
    qnear_pos = np.array([qnear.x, qnear.y])
    direction = qrand - qnear_pos
    direction_unitary = direction/np.linalg.norm(direction)
    qnew = qnear_pos + direction_unitary * incremental_distance
    shortest_distance = float('inf')
    qnew_aprox = ()
    for points in free_points:
        distance = np.linalg.norm(qnew - points)
        if distance < shortest_distance:
            shortest_distance = distance
            qnew_aprox = points
    return qnew_aprox

def check_collision(qnew, qold, obstacles_points, threshold):
    x2, y2 = qnew[0], qnew[1]
    x1, y1 = qold.x, qold.y
    x = np.linspace(x1, x2, 100)
    y = np.linspace(y1, y2, 100)
    path_points = np.column_stack((x, y))
    diff = path_points[:, np.newaxis, :] - obstacles_points[np.newaxis, :, :]
    distances = np.linalg.norm(diff, axis=2)
    if np.any(distances < threshold):
        return True
    else :
        return False

def RRT(start_point, finish_point, incremental_distance, K, free_points, obstacles_points, threshold):
    x_size = (np.min(free_points[:, 0]), np.max(free_points[:, 0]))
    y_size = (np.min(free_points[:, 1]), np.max(free_points[:, 1]))
    node_list = []
    node_list.append(Node(start_point[0], start_point[1], None))
    for i in range (K):
        qrand = rand_conf(x_size, y_size)
        qnear, qnear_index = nearest_vertex(node_list, qrand)
        qnew = new_conf(qnear, qrand, incremental_distance, free_points)
        collision = check_collision(qnew, qnear, obstacles_points, threshold)
        if collision == False:
            new_node = Node(qnew[0], qnew[1], qnear_index)
            node_list.append(new_node)
        #     print(f"Iter {i}: Added node at {qnew}, parent=({qnear.x}, {qnear.y})")
        # else:
        #     print("Node wasn't added due to a collision")
        if np.linalg.norm(qnew - np.array(finish_point)) < 1:
            path_nodes = path(node_list)
            return node_list, path_nodes

def path(node_list):
    nodes_path = []
    number_node = len(node_list) - 1
    current_node = node_list[number_node]
    while current_node is not None:
        nodes_path.append(current_node)
        if current_node.parent_index is None:
            break
        current_node = node_list[current_node.parent_index]
    return nodes_path[::-1]

import matplotlib.pyplot as plt
def draw_rrt(node_list, start_point, finish_point, path_nodes, obstacles_points):
    plt.figure(figsize=(10, 10))
    plt.title("RRT Path")
    plt.xlabel("X")
    plt.ylabel("Y")

    # Draw tree conections
    for node in node_list:
        if node.parent_index is not None:
            parent = node_list[node.parent_index]
            plt.plot([node.x, parent.x], [node.y, parent.y], 'g-', linewidth=0.5)

    # Draw nodes
    for node in node_list:
        plt.plot(node.x, node.y, 'go', markersize=2)

    # Draw obstacles
    if obstacles_points.size > 0:
        obs_x = [p[0] for p in obstacles_points]
        obs_y = [p[1] for p in obstacles_points]
        plt.plot(obs_x, obs_y, 'ks', markersize=4, label='Obstáculos')

    # Draw start point and finish point
    plt.plot(start_point[0], start_point[1], 'bo', markersize=8, label='Inicio')
    plt.plot(finish_point[0], finish_point[1], 'ro', markersize=8, label='Fin')

    # Draw path
    path_x = [node.x for node in path_nodes]
    path_y = [node.y for node in path_nodes]
    plt.plot(path_x, path_y, 'r-', linewidth=2, label='Ruta encontrada')

    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def RRT_algorithm(start_point, finish_point, incremental_distance, K, free_points, obstacles_points, threshold):
    node_list, path_nodes = RRT(start_point, finish_point, incremental_distance, K, free_points, obstacles_points, threshold)
    for i in range(len(path_nodes)):
        print('Número iteración', i)
        print('Valor x:', path_nodes[i].x, 'Valor y:', path_nodes[i].y)
    draw_rrt(node_list, start_point, finish_point, path_nodes, obstacles_points)
    return path_nodes
