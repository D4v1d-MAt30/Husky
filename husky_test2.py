#Prueba usar pra leer las normales de los objetos y detectarlos sin simulacion
import numpy as np
from robots.ouster import Ouster

def obtener_bordes(lidar, voxel, radio_peque, radio_grande, dist_min, dist_max, umbral, margen):
    lidar.down_sample(voxel=voxel)
    data = np.asarray(lidar.pointcloud.points)

    lidar.estimate_normals(radius=radio_peque)
    # Para poder acceder a los datos
    normales_peque = np.asarray(lidar.pointcloud.normals).copy()    # copy es necesario para que no sobreescriba
                                                                    # y se borre el valor para este radio
    lidar.estimate_normals(radius=radio_grande)
    normales_grande = np.asarray(lidar.pointcloud.normals).copy()

    # Se calcula la diferencia y su norma
    diff_norm = (normales_grande - normales_peque) / 2
    don_abs = np.linalg.norm(diff_norm, axis=1)

    # A partir de un umbral se calcula donde están los bordes
    umbral = umbral
    # bordes = np.where(DoN_abs > umbral)[0]
    bordes = []
    for i in range(len(don_abs)):
        if don_abs[i] > umbral and np.linalg.norm(data[i, :]) < dist_max and np.linalg.norm(data[i, :]) > dist_min:
            bordes.append(i)

    # Se obtiene los puntos donde se chocaria el robot y se le añade un margen en los ejes x e y para que no se pegue demasiado a los obstaculos

    puntos_choque = []
    if dist_min < 2 :
        tamano_x = 0.7
        tamano_y = 1
        tamano_z = 1.25
        wheel = 0.35463
        minimo_z = min(data[:, 2])
        for i in range(data.shape[0]):
            x, y, z = data[i]
            if (abs(x) - margen < tamano_x / 2 and abs(y) - margen < tamano_y / 2 and
                (minimo_z + wheel) < z < (minimo_z + tamano_z)):
                puntos_choque.append(i)

    # Se pinta de gris todos los puntos y luego de rojo solo los de bordes y se agrega a lidar
    colors = np.tile([0.5, 0.5, 0.5], (data.shape[0], 1))
    colors[bordes] = [1.0, 0.0, 0.0]         # rojo
    colors[puntos_choque] = [0.0, 1.0, 0.0]  # verde
    lidar.elegir_colores(colors)
    lidar.draw_pointcloud()
    #Coge los indices de bordes y choque y quita duplicados
    indices_totales = np.vstack((bordes + puntos_choque))
    puntos_bordes = data[indices_totales]
    todos_puntos = np.arange(len(data))
    indices_libres = np.setdiff1d(todos_puntos, indices_totales)
    #indices_libres = [i for i in range(len(data)) if i not in indices_totales]
    puntos_libres = data[indices_libres]
    return puntos_bordes, puntos_libres

# Crear una instancia sin simulación
lidar = Ouster(simulation=None)

# Cargar el archivo PCD
lidar.from_file("lidar/simulated_pointcloud6.pcd")

#Se calcula los puntos donde hay bordes
bordes_cerca, libres_cerca = obtener_bordes(lidar, voxel=0.1, radio_peque=0.15, radio_grande=0.35, dist_min=0, dist_max=5, umbral=0.045, margen=0.5)
bordes_lejos, libres_lejos = obtener_bordes(lidar, voxel=0.25, radio_peque=0.4, radio_grande=1, dist_min=4.5, dist_max=8, umbral=0.15, margen=0.5)

#Se cogen todos los puntos donde hay bordes
bordes_totales = np.vstack((bordes_cerca, bordes_lejos))
libres_totales = np.vstack((libres_cerca, libres_lejos))

#Guardar la nube de puntos marcando los bordes
#lidar.save_pointcloud('lidar/simulated_pointcloud7.pcd')
