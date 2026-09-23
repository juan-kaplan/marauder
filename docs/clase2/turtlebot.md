# TurtleBot

Para modificiar el funcionamiento de la simulación es recomable trabaajar sobre un paquete independiente a los netivos de ROS. Incluir este paquete en el workspace y seguir los pasos a continuación para utilizar un paquete de simulación propio.

## Simulación TurtleBot3

- REF: https://docs.robotis.com/docs/systems/turtlebot3/overview/
- REF: https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation

## clonar o instalar:

```
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

```
colcon build --symlink-install
```

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

```
TURTLEBOT3_MODEL=burger ros2 launch turtlebot3_gazebo empty_world.launch.py
```
