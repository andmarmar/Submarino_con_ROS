# Simulación y control de un robot submarino 

Tod el trabajo realizado ha sido mediante la ayuda de dos principales fuentes de trabajo

> El repositorio   [uuv_simulator](https://github.com/uuvsimulator/uuv_simulator)

> La  [página de documentación](https://uuvsimulator.github.io/packages/uuv_simulator/intro/) 

# Propósito del trabajo
Los cambios realizados sobre el repositorio principal han sido creados para la realización de un proyecto para la asignatura de Control y Programación de Robots en el 4º año del Grado de Ingenería Electrónica, Robótica y Mecatrónica en la Universidad de Sevilla.

En este repositorio solo se tratan los cambios realizados por y para la asignatura, si necesitan más información para usar el repositorio, acudan a él. Esto es de ámbito didáctico y específico para un proyecto de la universidad.

# Entorno
Todo el trabajo está hecho en Ubuntu 18.04 usando ROS en su versión melodic. Además para la representación de los modelos se ha usado tanto Gazebo, como de Rviz.

> **Mundos de Gazebo**
El repositorio trae diferentes mapas para poder usar el robot en diferentes situaciones y comprobar su funcionamiento. Además de estos, en este repositorio se ha creado uno más
para poder probar de forma efectiva nuestro trabajo bajo las condiciones pedidas.

> **Controlador**
A parte de los controladores ya realizados, se ha creado basado en un modelo completamente ideal, un controlador por par computado.

# Uso del repositorio
Para hacer de este repositorio algo más accesible para todas las personas, se han creado archivos .launch con tutoriales para que con un único ejecutable se puedan comprobar el funcionamiento de las partes mas simples del repositorio original. 

> **Instalación y preparación**
En primer lugar se debe disponer de un ordenador que tenga el sistema operativo de Ubuntu 18.04, o en su defecto una partición de su disco duro, ya que realizar todo el trabajo usando una máquina virtual puede ser bastante tedioso si no se dispone de un ordenador muy potente. Una vez instalado en su ordenador el sistema operativo basta con crear una carpeta y clonar el repositorio.
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone "url"
cd ..
catkin_make
```

Al arrancar cada terminal, se debera de introducir
```
cd catkin_ws
source devel/setuo.bash
```
> **Mando de XBOX y Teclado**
``` 1º terminal (cargar un mundo)
roslaunch uuv_gazebo_worlds mangalia.launch

``` 2º terminal (cargar el modelo rexrov)
roslaunch uuv_descriptions upload_rexrov.launch

``` 3º terminal (Control por teclado)
roslaunch uuv_control_cascaded_pid key_board_velocity.launch uuv_name:=rexrov model_name:=rexrov

``` 3º terminal (Control por joystick)
roslaunch uuv_control_cascaded_pid joy_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0

```
> **Controlador**
```
roslaunch uuv_tutorial_dp_controller start_tutorial_dp_controller_demo.launch

```
> **Creación de una corriente**
``` Añadir una corriente manualmente
rosservice call /hydrodynamics/set_current_velocity "velocity: 1.0"
rosservice call /hydrodynamics/set_current_horz_angle "angle: 0.8"
rosservice call /hydrodynamics/set_current_vert_angle "angle: 0.5"

``` Añadir una corriente o otras perturbaciones a traves de disturbances.yaml
roslaunch uuv_tutorial_disturbances tutorial_disturbance_bueno.launch
```

> **Creación plume**

``` 1º terminal (servidor de velocidad)
roslaunch uuv_plume_simulator start_current_velocity_server.launch

``` 2º terminal (servidor de plume)
roslaunch uuv_plume_simulator start_plume_server.launch current_velocity_topic:=/current_velocity_server/current_velocity update_rate:=5

``` 3º terminal (ejecución de plume turbulento)
roslaunch uuv_plume_simulator start_demo_turbulent_plume.launch

```
# Referencias usadas


@article{Simulator,
   author = {Daniel Cook, Andres Vardy, Ron Lewis},
   journal = {ResearchGate},
   title = {A survey of AUV and robot simulators for multi-vehicle operations},
   url = {https://www.researchgate.net/publication/301403241_A_survey_of_AUV_and_robot_simulators_for_multi-vehicle_operations},
   year = {2014},
}
@article{Girona,
   author = {Eva Rodriguez},
   journal = {Agencia Sinc},
   title = {Robots e inteligencia artificial protegen y vigilan la vida submarina},
   url = {https://www.agenciasinc.es/Reportajes/Robots-e-inteligencia-artificial-protegen-y-vigilan-la-vida-submarina},
   year = {2024},
}
@misc{ROS2,
   author = {Liquid},
   title = {Plankton},
   url = {https://github.com/Liquid-ai/Plankton},
}
@article{Kinematic,
   author = {Reza N.Jazar},
   journal = {Springer},
   title = {Theory of Applied Robotics Kinematics, Dynamics, and Control, Second Edition},
   url = {http://www.coep.ufrj.br/~ramon/COE-841/robotics/book%202010%20-%20Theory%20of%20Applied%20Robotics%20Kinematics,%20Dynamics,%20and%20Control%20-%20Jazar.pdf},
}
```



# Crédito a los creadores del repositorio
If you are using this simulator for your publication, please cite:

```
@inproceedings{Manhaes_2016,
	doi = {10.1109/oceans.2016.7761080},
	url = {https://doi.org/10.1109%2Foceans.2016.7761080},
	year = 2016,
	month = {sep},
	publisher = {{IEEE}},
	author = {Musa Morena Marcusso Manh{\~{a}}es and Sebastian A. Scherer and Martin Voss and Luiz Ricardo Douat and Thomas Rauschenbach},
	title = {{UUV} Simulator: A Gazebo-based package for underwater intervention and multi-robot simulation},
	booktitle = {{OCEANS} 2016 {MTS}/{IEEE} Monterey}
}
```
# License

UUV Simulator is open-sourced under the Apache-2.0 license. See the
[LICENSE](https://github.com/uuvsimulator/uuv_simulator/blob/master/LICENSE) file for details.

For a list of other open source components included in UUV Simulator, see the
file [3rd-party-licenses.txt](https://github.com/uuvsimulator/uuv_simulator/blob/master/3rd-party-licenses.txt).
