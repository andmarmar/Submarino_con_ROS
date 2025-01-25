# Simulación y control de un robot submarino 

Tod el trabajo realizado ha sido mediante la ayuda de dos principales fuentes de trabajo

> El repositorio   [uuv_simulator](https://github.com/uuvsimulator/uuv_simulator)

> La  [página de documentación](https://uuvsimulator.github.io/packages/uuv_simulator/intro/) 

# Propósito del trabajo
Los cambios realizados sobre el repositorio principal han sido creados para la realización de un proyecto para la asignatura de Control y Programación de Robots en el 4º año del Grado de Ingenería Electrónica, Robótica y Mecatrónica.

En este repositorio solo se tratan los cambios realizados por y para la asignatura, si necesitan la información principal para usar el repositorio, acudan a él. Esto es de ámbito didáctico y específico para un proyecto de universidad.



# Entorno
Todo el trabajo está hecho en Ubuntu 18.04 usando ROS en su versión melodic. Además para la representación de los modelos se ha usado tanto Gazebo, como de Rviz.

> **Mundos de Gazebo**
El repositorio trae diferentes mapas para poder usar el robot en diferentes situaciones y comprobar su funcionamiento. Además de estos, en este repositorio se ha creado uno más
para poder probar de forma efectiva nuestro trabajo bajo las condiciones pedidas.

> **Controlador**
A parte de los controladores ya realizados, se ha creado basado en un modelo completamente ideal, un controlador por par computado.



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
