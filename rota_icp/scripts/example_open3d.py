import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

# 1. Generar la nube de puntos origen (Source) en forma de "L" (2D)
n_points = 200
# Línea horizontal: de (0,0) a (2,0)
linea_h = np.column_stack((np.linspace(0, 2, n_points), np.zeros(n_points)))
# Línea vertical: de (0,0) a (0,2)
linea_v = np.column_stack((np.zeros(n_points), np.linspace(0, 2, n_points)))

# Combinar puntos y añadir Z = 0 para Open3D
pts_2d = np.vstack((linea_h, linea_v))
pts_3d_source = np.hstack((pts_2d, np.zeros((pts_2d.shape[0], 1))))

source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(pts_3d_source)

# 2. Definir una transformación rígida conocida (Traslación + Rotación en 2D)
t_real_2d = np.array([0.5, -0.3])
angulo_rad = np.radians(15.0)  # Rotación opcional de 15 grados

# Matriz de transformación 4x4
T_conocida = np.eye(4)
T_conocida[0, 0] = np.cos(angulo_rad)
T_conocida[0, 1] = -np.sin(angulo_rad)
T_conocida[1, 0] = np.sin(angulo_rad)
T_conocida[1, 1] = np.cos(angulo_rad)
T_conocida[0:2, 3] = t_real_2d

# 3. Crear la nube objetivo (Target) aplicando T_conocida a Source + ruido ligero
target = o3d.geometry.PointCloud(source)
target.transform(T_conocida)

# Agregar un ruido gaussiano mínimo para simular sensores reales
ruido = np.random.normal(0, 0.005, size=np.asarray(target.points).shape)
target.points = o3d.utility.Vector3dVector(np.asarray(target.points) + ruido)

# 4. Ejecutar ICP (Point-to-Point)
umbral_distancia = 0.5  # Distancia máxima de correspondencia
T_inicial = np.eye(4)   # Estimación inicial identidad

reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, umbral_distancia, T_inicial,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
)

# 5. Extraer y comparar resultados
T_estimada = reg_p2p.transformation

print("Resultados ICP")
print("T_conocida:")
print(T_conocida)
print("T_estimada:")
print(T_estimada)

# Graficar
source.paint_uniform_color([1, 0, 0])      # Rojo: Source original
target.paint_uniform_color([0, 1, 0])      # Verde: Target
source_alineada = o3d.geometry.PointCloud(source)
source_alineada.transform(T_estimada)
source_alineada.paint_uniform_color([0, 0, 1]) # Azul: Source alineada con ICP

pts_target = np.asarray(target.points)[:, :2]
pts_alineada = np.asarray(source_alineada.points)[:, :2]
pts_source = np.asarray(source.points)[:, :2]

plt.figure(figsize=(7, 7))
plt.scatter(pts_source[:, 0], pts_source[:, 1], c='red', s=5, label='Source Original')
plt.scatter(pts_target[:, 0], pts_target[:, 1], c='green', s=5, label='Target')
plt.scatter(pts_alineada[:, 0], pts_alineada[:, 1], c='blue', s=5, label='Source Alineada (ICP)')

plt.title("ICP 2D: Alineación de Esquina")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()