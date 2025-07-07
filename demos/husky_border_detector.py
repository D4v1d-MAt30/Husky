#Prueba usar pra leer las normales de los objetos y detectarlos sin simulacion
import numpy as np
from robots.ouster import Ouster

def get_edges(lidar, voxel, small_radius, large_radius, min_distance, max_distance, threshold, margin):
    lidar.down_sample(voxel=voxel)
    data = np.asarray(lidar.pointcloud.points)

    lidar.estimate_normals(radius=small_radius)
    # Copy to preserve the normals for small radius
    normals_small = np.asarray(lidar.pointcloud.normals).copy()    # copy es necesario para que no sobreescriba
                                                                    # y se borre el valor para este radio
    lidar.estimate_normals(radius=large_radius)
    normals_large = np.asarray(lidar.pointcloud.normals).copy()

    # Compute the Difference of Normals (DoN)
    diff_norm = (normals_large - normals_small) / 2
    don_abs = np.linalg.norm(diff_norm, axis=1)

    # Determine edge points using the threshold
    # borders_indices = np.where(DoN_abs > threshold)[0]
    edge_indices = []
    for i in range(len(don_abs)):
        distance = np.linalg.norm(data[i, :])
        if don_abs[i] > threshold and distance < max_distance and distance > min_distance:
            edge_indices.append(i)

    # Determine collision points (robot footprint with safety margin)
    collision_indices = []
    if min_distance < 2 :
        size_x, size_y, size_z = 0.7, 1, 1.25
        wheel_radius = 0.35463
        min_z = min(data[:, 2])
        for i in range(data.shape[0]):
            x, y, z = data[i]
            if (abs(x) - margin < size_x / 2 and abs(y) - margin < size_y / 2 and
                (min_z + wheel_radius) < z < (min_z + size_z)):
                collision_indices.append(i)

    # Colorize: gray for all, red for edges, green for collisions
    colors = np.tile([0.5, 0.5, 0.5], (data.shape[0], 1))
    colors[edge_indices] = [1.0, 0.0, 0.0]         # rojo
    colors[collision_indices] = [0.0, 1.0, 0.0]  # verde
    lidar.choose_colors(colors)
    lidar.draw_pointcloud()
    # Combine edge and collision indices without removing duplicates
    if (not collision_indices) and (not edge_indices):
        total_indices = []
    else:
        total_indices = (edge_indices + collision_indices)
    edge_points = data[total_indices]
    all_indices = np.arange(len(data))
    free_indices = np.setdiff1d(all_indices, total_indices)
    #free_indices = [i for i in range(len(data)) if i not in total_indices]
    free_points = data[free_indices]
    return edge_points, free_points

# Create a lidar instance without simulation
lidar = Ouster(simulation=None)

# Load the PCD file
lidar.from_file("lidar/simulated_pointcloud3.pcd")

# Extract edge and free points
near_edges, near_free = get_edges(lidar, voxel=0.12, small_radius=0.25, large_radius=0.45, min_distance=0, max_distance=5, threshold=0.05, margin=2.9)
far_edges, far_free = get_edges(lidar, voxel=0.25, small_radius=0.4, large_radius=1, min_distance=4.5, max_distance=10, threshold=0.15, margin=0.5)

# Combine all edge and free points
all_borders = np.vstack((near_edges, far_edges))
all_free = np.vstack((near_free, far_free))

#Guardar la nube de puntos marcando los bordes
#lidar.save_pointcloud('lidar/simulated_pointcloud7.pcd')
