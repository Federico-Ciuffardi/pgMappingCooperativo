# Exploración multirobot basada en grillas de ocupación probabilística y diagramas de Voronoi

## Como usar (Ubuntu 20.04)

1. Instalar ROS noetic y crear un workspace de catkin:
* Seguir tutorial de http://wiki.ros.org/noetic/Installation/Ubuntu instalando
  la version completa (`ros-noetic-desktop-full`).
* Seguir tutorial de
  http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
  salteando el primer paso (ROS ya instalado).

2. Instalar las dependencias:
```bash
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-teb-local-planner
sudo apt install ros-noetic-map-msgs
```

3. Clonar en workspace de catkin: 

```bash
git clone https://gitlab.fing.edu.uy/federico.ciuffardi/pioneer_p3dx_model ~/catkin_ws/src/pioneer_p3dx_model

git clone https://gitlab.fing.edu.uy/federico.ciuffardi/pgmappingcooperativo.git ~/catkin_ws/src/pgmappingcooperativo
```

4. Ejecutar una simulación de prueba:
```bash
~/catkin_ws/src/pgmappingcooperativo/scripts/pgmch test office <robot_number> <cell_size>
```
Donde:
* `robot_number` es la cantidad de robots a utilizar (entre 1 y 5)
* `cell_size` es el tamaño de las celdas de las grillas de ocupación (recomendado entre 0.25 y 1)

## Documentación

Por más información:
* Ejecutar `~/catkin_ws/src/pgmappingcooperativo/scripts/pgmch --help`
* Leer el contenido de `~/catkin_ws/src/pgmappingcooperativo/launch/multirobot.launch`
* Leer el [informe](https://gitlab.fing.edu.uy/federico.ciuffardi/pgmappingcooperativo/-/blob/master/informe_pg_federico_ciuffardi.pdf)
