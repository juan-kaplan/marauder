
# Sanity check

## Verificar los fuentes utilizados

```bash
$ echo $AMENT_PREFIX_PATH
/opt/ros/jazzy
```

## Verificar que el `~/.bashrc` no importe otros workspaces

Deberian ver solo esto
```bash
...
source /opt/ros/jazzy/setup.bash
...
```

## Verificar que el `~/.bashrc` no defina otras variables de entorno

Buscar variables de entorno predefinidas en el archivo y relacionadas con el proyecto.

```bash
$ env

ROS_DISTRO=jazzy
ROS_VERSION=2
ROS_PYTHON_VERSION=3

GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share
GZ_CONFIG_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz

AMENT_PREFIX_PATH=/opt/ros/jazzy

CMAKE_PREFIX_PATH=/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

LD_LIBRARY_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib

PATH=/home/seb/.pyenv/shims:/home/seb/.pyenv/bin:/home/seb/.config/Code/User/globalStorage/github.copilot-chat/debugCommand:/home/seb/.config/Code/User/globalStorage/github.copilot-chat/copilotCli:/home/seb/.local/bin:/home/seb/.pyenv/bin:/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/seb/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin:/home/seb/.local/bin:/home/seb/.local/bin:/home/seb/.local/bin:/home/seb/.vscode/extensions/ms-python.debugpy-2026.6.0-linux-x64/bundled/scripts/noConfigScripts:/home/seb/.local/bin
```

## Turtlebot3

### Instalar

sudo apt install ros-jazzy-turtlebot3

### Check si estamos seleccionando bien el paquete

```
$ ros2 pkg prefix turtlebot3_gazebo
/opt/ros/jazzy
```

### Ejecutar el ejemplo

TURTLEBOT3_MODEL=burger ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

TURTLEBOT3_MODEL=burger ros2 run turtlebot3_teleop teleop_keyboard

