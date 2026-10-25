# Localización (capa 1)

Implementación de la persona A según [`arquitectura.md`](arquitectura.md): un
solo nodo, `rota_localization/localization_node.py`, con los modos `encoders`,
`lidar` y `mixed`, más las funciones compartidas de `rota_common`
(`se2`, `msgs`, `scan`, `icp`).

## Qué publica

- `/rel_odom` (`Odometry`, `reference → robot_frame`)
- `/localization/odom` (`Odometry`, `odom → robot_frame`)
- `/localization/reference` (`PoseStamped` en `odom`, una por referencia nueva)
- TF `odom → reference → robot_frame`

Parámetros: `mode`, `tau_d`, `tau_theta`, `robot_frame`,
`wheel_source` (`joint_states` o `ticks`), `left_wheel_joint`,
`right_wheel_joint`, `wheel_radius`, `meters_per_tick`, `wheel_separation`,
`min_scan_range`, `max_scan_range`, `icp_max_distance`, `icp_max_iterations`,
`icp_min_fitness`. Los valores por defecto son los del TurtleBot3 burger
(`meters_per_tick` es el medido en el TB4 y solo cuenta con
`wheel_source: ticks`). El perfil `tb4` (`rota_bringup/config/tb4.yaml`) cambia
`robot_frame`, `wheel_source`, `meters_per_tick`, `wheel_separation` y
`max_scan_range`.

## Cómo correrlo

Con el workspace compilado (`python -m colcon build --symlink-install`) y el
venv activo:

```bash
ros2 launch rota_bringup sim_launch.py                   # la simulación
ros2 launch rota_bringup teach_launch.py mode:=encoders   # o lidar, o mixed
```

`sim_launch.py` levanta `turtlebot3_world` en Gazebo, con el robot en
`(-2, -0.5)`, y `teach_launch.py` el nodo con `teach_node`. Para mover el robot, `teleop_twist_keyboard` o
`ros2 topic pub /cmd_vel`. La odometría de Gazebo (`/odom`) queda como
referencia de la pose real.

## Modos

- **encoders:** integra las ruedas en cada `/joint_states` o `/wheel_ticks`.
- **lidar:** ICP del scan contra el scan de la referencia. La estimación
  inicial usa la última velocidad (velocidad constante).
- **mixed:** integra las ruedas en cada `/joint_states` o `/wheel_ticks` y en cada scan corre
  ICP contra la referencia, usando la pose integrada como estimación inicial.
  La pose de las ruedas es la de "ahora", no la del instante del scan (unos
  30 ms de diferencia). Las referencias se crean solo en un scan, así que si
  el LiDAR se corta no se crean más.

Una referencia nueva se crea cuando `reference → robot` supera `tau_d` o
`tau_theta`, así que la TF `odom → robot_frame` no salta.

## Para tener en cuenta

### Persona B: Teach and Repeat

- **Suscribirse a `/localization/reference` con `RELIABLE` + `TRANSIENT_LOCAL`
  y depth 100.** La referencia 0 sale con el primer `/joint_states` o `/wheel_ticks`
  (encoders) o con el primer scan (lidar y mixto). Una suscripción volátil que arranque
  unos milisegundos tarde la pierde.
- **El stamp de cada referencia es el del mensaje que la creó.** En lidar y
  mixto es un scan, así que el buffer del Teach lo encuentra por igualdad. En
  encoders es un `/joint_states` o un `/wheel_ticks` y hay que buscar el scan más cercano.
- **Usar `rota_common.icp.align`, no `open3d` directo.** `icp.py` fija
  `OMP_NUM_THREADS=1` antes de importar Open3D. Sin eso, un ICP de ~0.5 ms
  pasa a cientos de ms cuando corre junto al localizador.
- **`align` devuelve `(pose, fitness, ok)`.** Si no puede correr, devuelve la
  estimación inicial con `ok=False`. En un pasillo largo sin rasgos, ICP no ve
  el avance (no hay manejo de ese caso): la estimación inicial tiene que ser
  buena.
- **La TF `odom → robot_frame` llega a distinto ritmo según el modo.**
  Encoders y mixto: con cada `/joint_states` (30 Hz en Gazebo) o `/wheel_ticks`
  (unos 31 Hz en el TB4). Lidar: con cada scan (5 Hz en el TB3, 8 Hz en el
  TB4), y si el LiDAR se corta, deja de publicarse. El Repeat tiene que frenar
  en ese caso.

### Persona C: control e infraestructura

- **`OMP_NUM_THREADS=1` en los launches**, por si algún nodo importa `open3d`
  por su cuenta (`sim_launch.py` ya lo hace).
- **Apagar la TF `odom → base_footprint` de Gazebo**, o `base_footprint` queda
  con dos padres. `sim_launch.py` lo hace con una copia del modelo del
  TurtleBot3 con `publish_odom_tf` en false. La odometría de Gazebo sigue en
  `/odom`. En Jazzy es otro camino. En el TB4 no hace falta: la TF del Create 3
  va por `/tb4_1/tf` y la nuestra por `/tf`.
- **Gazebo y RViz no abren desde la terminal de VS Code (snap)**: mueren con
  `GLib-GIO-ERROR`. Desde una terminal normal funcionan.
- **Al cortar un launch de Gazebo, matar el grupo de procesos entero**
  (`setsid` + `kill -- -PGID`). Si no, quedan `gzserver` y nodos vivos
  publicando en los mismos tópicos.
- **`robot_frame`:** `base_footprint` en el TB3 y `base_link` en el TB4. Lo fija
  el perfil del robot, igual que la geometría.

## Pendiente

- TB4, medido en los bags contra la odometría del Create 3 (que no es ground
  truth): 4.37e-4 m por tick y separación de 0.2306 m. Falta probar en el
  robot: dar una vuelta a mano a una rueda y leer el contador, una recta
  medida, y que los relojes de la laptop y del robot estén sincronizados (el
  Repeat compara la hora de la laptop con los stamps de la TF).
- El LiDAR real tarda 0.1 a 0.2 s por vuelta y en los giros el scan sale
  deformado. Gazebo Classic no lo simula.
