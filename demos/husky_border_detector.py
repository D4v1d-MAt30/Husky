import numpy as np
from robots.ouster import Ouster

def difference_of_normals(small_radius, large_radius):
    lidar.estimate_normals(radius=small_radius)
    # Copy to preserve the normals for small radius
    normals_small = np.asarray(lidar.pointcloud.normals).copy()
    lidar.estimate_normals(radius=large_radius)
    normals_large = np.asarray(lidar.pointcloud.normals).copy()
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
    colors = np.full((data.shape[0], 3), 0.5)
    colors[edge_indices] = [1.0, 0.0, 0.0]         # rojo
    colors[collision_indices] = [0.0, 1.0, 0.0]  # verde
    lidar.choose_colors(colors)
    lidar.draw_pointcloud()

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
lidar.from_file("lidar/simulated_pointcloud1.pcd")

# Extract edge and free points
near_edges, near_free = get_edges(lidar, voxel=0.1, small_radius=0.2, large_radius=0.3, min_distance=0, max_distance=5, threshold=0.05, margin=0.5)
far_edges, far_free = get_edges(lidar, voxel=0.25, small_radius=0.4, large_radius=1, min_distance=5, max_distance=10, threshold=0.15, margin=0.5)

# Combine all edge and free points
all_edges = np.concatenate((near_edges, far_edges), axis=0)
all_free = np.concatenate((near_free, far_free), axis=0)

all_points = np.concatenate((all_edges, all_free), axis=0)
colors_edges = np.tile([1.0, 0.0, 0.0], (all_edges.shape[0], 1))
colors_free = np.tile([0.5, 0.5, 0.5], (all_free.shape[0], 1))
lidar.from_points(all_points)
colors = np.concatenate((colors_edges, colors_free), axis=0)
lidar.choose_colors(colors)
lidar.draw_pointcloud()
#Save point cloud with the edges in red
#lidar.save_pointcloud('lidar/simulated_pointcloud7.pcd')
