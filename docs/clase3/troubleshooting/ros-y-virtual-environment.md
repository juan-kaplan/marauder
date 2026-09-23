# Utilizando ROS y Virtual Environment

## Crear el Virtual Environment

```bash
python3 -m venv <venv_name> --system-site-packages
```

## Instalar la dependencia

```bash
<venv_name>/bin/activate
python3 -m pip install <dependencie>
python3 -m colcon build --packages-select <package_name>
```

## Compilar el nodo

```bash
<venv_name>/bin/activate
python3 -m colcon build
```

## Ejecutar

```bash
<venv_name>/bin/activate
source install/setup.bash
ros2 run <package_name> <executable_name>
```