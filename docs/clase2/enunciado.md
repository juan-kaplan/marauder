# Robótica Móvil: Clase 2

## Objetivos
  - Transofrmaciones SE(2)
  - Localización por odometria de encoders
  - Mapeo del entorno con LiDAR
  - Ros Launch
  - Ros Bag
  - Experimentos en simulación y robot real


## Odometria de encoders

Utilizar la información de los encoders (robot real) o joints (simulación) para calcular la pose del robot en el plano.
Publicar la pose con un mensaje del tipo `nav_msgs/Odometry`

---

### Calculo de odometría

* $r$: Radio de las ruedas.
* $L$: Distancia entre las ruedas.
* $N$: Número de pulsos del encoder por revolución completa.

#### 1. Desplazamiento Lineal de Cada Rueda

En un intervalo de tiempo $\Delta t$ el encoder cuenta $\Delta N$ pulsos, el desplazamiento de cada rueda ($D_i$) es:

$$D_{izq} = 2 \pi r \cdot \frac{\Delta N_{izq}}{N}$$

$$D_{der} = 2 \pi r \cdot \frac{\Delta N_{der}}{N}$$

#### 2. Movimiento del Centro del Robot

El desplazamiento lineal ($\Delta s$) y la variación de orientación ($\Delta \theta$) del centro del robot se calcula como:

* Desplazamiento lineal ($\Delta s$):

$$\Delta s = \frac{D_{der} + D_{izq}}{2}$$


* Cambio de orientación ($\Delta \theta$):

$$\Delta \theta = \frac{D_{der} - D_{izq}}{L}$$

#### 3. Actualización de la Pose

Conociendo el estado anterior $(x_k, y_k, \theta_k)$, la nueva pose en el plano se calcula sumando el avance proyectado en cada eje:

$$\theta_{k+1} = \theta_k + \Delta \theta$$

$$x_{k+1} = x_k + \Delta s \cdot \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)$$

$$y_{k+1} = y_k + \Delta s \cdot \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)$$

*(Nota: Usar $\theta_k + \frac{\Delta \theta}{2}$ mejora la precisión al aproximar el arco de curvatura).*

---

#### 4. Velocidades del Robot

Para calcular las velocidades lineal ($v$) y angular ($\omega$) instantáneas dividiendo por el tiempo de muestreo $\Delta t$:

$$v = \frac{\Delta s}{\Delta t}$$

$$\omega = \frac{\Delta \theta}{\Delta t}$$


## Mapeo

Utilizando la información de localización de la pose, publicada utilizando un mensaje `nav_msgs/Odometry`, generar un mapa de ocupación y publicarlo utilizando el tipo de mensaje `nav_msgs/OccupancyGrid`.

El mapa de ocupación se debe generar a partir de la información de la pose del robot y del registro del entorno mediante el sensor LiDAR. Utilizar el mensaje `sensor_msgs/LaserScan`.

### Calculo de nube de puntos

#### 1. Captura de puntos en el Marco del sensor (Sensor)

El LiDAR entrega la informacion de forma polar, para cada punto un ángulo de medición ($\alpha$) y una distancia ($d$). Para convertir esos datos al sistema cartesiano $(x_{lidar}, y_{lidar})$ centrado en el LiDAR:

$$x_{lidar} = d \cdot \cos(\alpha)$$

$$y_{lidar} = d \cdot \sin(\alpha)$$

#### 2. Transformación al Marco del Robot (Local)

Si el LiDAR no está justo en el centro geométrico del robot, sino desplazado en $(x_{S}, y_{S})$ y rotado un ángulo $\theta_{S}$, podemos transformar la nube de puntos utilizando la matriz de transformación:

$$\begin{bmatrix} X_{robot} \\ Y_{robot} \\ 1 \end{bmatrix} = \begin{bmatrix} \cos\theta_S & -\sin\theta_S & x_S \\ \sin\theta_S & \cos\theta_S & y_S \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_{lidar} \\ y_{lidar} \\ 1 \end{bmatrix}$$

#### 3. Transformación al Marco del Mapa (Global)

De igual forma para transformar los puntos al marco de referencia global se aplica la matriz de transformación de la pose del robot. Utilizando la pose actual del robot calculada por la odometría $(x_R, y_R, \theta_R)$:

$$\begin{bmatrix} X_{global} \\ Y_{global} \\ 1 \end{bmatrix} = \begin{bmatrix} \cos\theta_R & -\sin\theta_R & x_R \\ \sin\theta_R & \cos\theta_R & y_R \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_{robot} \\ y_{robot} \\ 1 \end{bmatrix}$$

### Calculo de Grilla de ocupación

Para mapear puntos del mundo continuo a celdas discretas dentro de una grilla de ocupación se debe considerar la resolución y las dimensiones de la grilla

* $(x, y)$: Coordenadas métricas continuas del punto en el mapa (en metros).
* $res$: Resolución del mapa (metros por celda, ej. $0.05\text{ m/celda}$).
* $W, H$: Ancho (*width*) y alto (*height*) de la grilla en número de celdas.

#### 1. Conversión: Coordenadas del Mundo a Grilla 2D

La celda $(gx, gy)$ correspondiente a un punto $(x, y)$ se obtiene mediante:

$$gx = \left\lfloor \frac{x}{res} \right\rfloor$$

$$gy = \left\lfloor \frac{y}{res} \right\rfloor$$

*Donde $\lfloor \cdot \rfloor$ denota la función suelo (truncado a entero).*

#### 2. Conversión a Índice 1D (Arreglo Unidimensional)

Si la grilla se almacena como un vector plano (como el estándar de ROS en `nav_msgs/OccupancyGrid` ordenado por filas):

$$index = gy \cdot W + gx$$

## Referencias

- [nav_msgs](https://docs.ros.org/en/jazzy/p/nav_msgs/)
- [nav_msgs/Odometry](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html)
- [nav_msgs/OccupancyGrid](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html)
- [sensor_msgs](https://docs.ros.org/en/jazzy/p/sensor_msgs/)
- [sensor_msgs/LaserScan](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html)
