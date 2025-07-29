import numpy as np
from RRT import RRT
from robots.ouster import Ouster

def difference_of_normals(small_radius, large_radius):
    lidar.estimate_normals(radius=small_radius)
    # Copy to preserve the normals for small radius
    normals_small = np.asarray(lidar.pointcloud.normals).copy()
    lidar.estimate_normals(radius=large_radius)
    normals_large = np.asarray(lidar.pointcloud.normals)
    # Compute the Difference of Normals (DoN)
    diff_norm = (normals_large - normals_small) / 2
    return diff_norm

def filter_edge_indices(don_abs, data, threshold, max_distance, min_distance):
    distances = np.linalg.norm(data, axis=1)
    mask = (don_abs > threshold) & (distances < max_distance) & (distances > min_distance)
    edge_indices = np.where(mask)[0]
    return edge_indices

def filter_collision_indices(min_distance, margin, data):
    if min_distance > 2:
        return np.array([], dtype=int)
    size_x, size_y, size_z = 0.67, 0.99, 1.25
    clearance = 0.13
    x, y, z = np.abs(data[:, 0]), np.abs(data[:, 1]), data[:, 2]
    lidar_height = 1.1325
    mask = ((x - margin < size_x / 2) & (y - margin < size_y / 2) &
            (clearance < (lidar_height + z)) & ((lidar_height + z) < size_z))
    collision_indices = np.where(mask)[0]
    return collision_indices

def get_edges(lidar, voxel, small_radius, large_radius, min_distance, max_distance, threshold, margin):
    lidar.down_sample(voxel=voxel)
    data = np.asarray(lidar.pointcloud.points)

    diff_norm=difference_of_normals(small_radius, large_radius)
    don_abs = np.linalg.norm(diff_norm, axis=1)

    # Determine edge points using the threshold
    edge_indices = filter_edge_indices(don_abs, data, threshold, max_distance, min_distance)

    # Determine collision points (robot footprint with safety margin)
    collision_indices = filter_collision_indices(min_distance, margin, data)

    # Colorize: gray for all, red for edges, green for collisions
    # colors = np.full((data.shape[0], 3), 0.5)
    # colors[edge_indices] = [1.0, 0.0, 0.0]         # rojo
    # colors[collision_indices] = [0.0, 1.0, 0.0]  # verde
    # lidar.choose_colors(colors)
    # lidar.draw_pointcloud()

    # Combine edge and collision indices without removing duplicates
    total_indices = np.concatenate((edge_indices, collision_indices))
    edge_points = data[total_indices]

    mask = np.ones(len(data), dtype=bool)
    mask[total_indices] = False
    free_points = data[mask]
    return edge_points, free_points

# Create a lidar instance without simulation
lidar = Ouster(simulation=None)

# Load the PCD file
lidar.from_file("lidar/simulated_pointcloud12.pcd")

# Extract edge and free points
near_edges, near_free = get_edges(lidar, voxel=0.12, small_radius=0.2, large_radius=0.3, min_distance=0, max_distance=5, threshold=0.05, margin=0.5)
far_edges, far_free = get_edges(lidar, voxel=0.25, small_radius=0.4, large_radius=1, min_distance=5, max_distance=10, threshold=0.15, margin=0.5)

#Add free points that are very close to the robot but aren't seen by lidar
inside_points = [(0, 0, min(near_free[:, 2]))]
radius = np.arange(0.2, 2.5, 0.2)
for r in radius:
    perimeter = 2 * np.pi * r
    num_points = int(np.ceil(perimeter / 0.2))
    for i in range(num_points):
        theta = 2 * np.pi * i / num_points
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        inside_points.append((x, y, min(near_free[:, 2])))
np.array(inside_points)

# Combine all edge and free points
all_edges = np.concatenate((near_edges, far_edges), axis=0)
all_free = np.concatenate((inside_points, near_free, far_free), axis=0)

all_points = np.concatenate((all_edges, all_free), axis=0)
colors_edges = np.tile([1.0, 0.0, 0.0], (all_edges.shape[0], 1))
colors_free = np.tile([0.5, 0.5, 0.5], (all_free.shape[0], 1))
lidar.from_points(all_points)
colors = np.concatenate((colors_edges, colors_free), axis=0)
lidar.choose_colors(colors)
lidar.draw_pointcloud()

points = np.array([[0, 0], [2, 0], [1, 3], [-5, 1], [6, 3], [6, -3], [6, 9], [6, 0]])

for i in range(len(points)-1):
    rrt_instance = RRT(points[i], points[i+1], all_free[:, :2], all_edges[:, :2])
    path_nodes = rrt_instance.run()
    if path_nodes == [] and i < len(points)-2:
        rrt_instance = RRT(points[i], points[i + 2], all_free[:, :2], all_edges[:, :2])
        path_nodes = rrt_instance.run()