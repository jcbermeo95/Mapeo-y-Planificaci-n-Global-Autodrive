# Mapeo y Planificación Global de Trayectorias en AutoDRIVE

Proyecto del segundo parcial parte I.

# Introducciòn

El objetivo es integrar un sistema completo de navegación robótica. El proceso se divide en:
1. **Mapeo (SLAM):** Generación de un mapa 2D del entorno de AutoDrive utilizando `SLAM Toolbox`.
2. **Planificación Global:** Implementación del algoritmo D* para calcular la ruta óptima desde un punto A hasta un punto B.
3. **Suavizado:** Aplicación de B-Spline para garantizar la viabilidad cinemática de la trayectoria.

## Requisitos y Dependencias
Para ejecutar este proyecto, asegúrate de tener instalado:
* **Ubuntu 22.04** + **ROS 2 Humble**
* **AutoDRIVE Simulator**
* **SLAM Toolbox**
* **Python 3**

Para la instalaciòn de Ubuntu 22.04 y ROS 2 Humble, se recomienda seguir los pasos establecidos en el siguiente enlace https://github.com/widegonz/Ubuntu-Installation

Para la instalaciòn de AutoDRIVE Simulator, se recomienda seguir los pasos establecidos en https://github.com/nabihandres/AUTODRIVE/blob/main/Tutorial%201%3A%20AutoDrive%20Installation%20and%20Setup.md

# Mapeo de la pista en AutoDRIVE

Para generar el mapeo (SLAM) en la pista del AutoDRIVE, se recomienda seguir los pasos establecidos en https://github.com/nabihandres/AUTODRIVE/blob/main/Tutorial%203%3A%20Simultaneous%20Localization%20and%20Maping.md

La evidencia del mapeo generado en este proyecto se presentan en el video del siguiente enlace: https://www.youtube.com/watch?v=K4ekre-al0M. El resultado generado despuès del mapeo son un archivo pgm (mostrados en la figura 1) y un archivo yaml. En la imagen se ha modificado para colocar una barra horizontal negra para que el algoritmo de planificaciòn global pueda ser computado. 

<p align="center">
  <img width="122" height="315" alt="F1tenth_map1" src="https://github.com/user-attachments/assets/27e8f7a4-1263-4af9-8bd0-964ba1ad8383" /><br>
  <em>Figura 1. Mapa generado usando SLAM Toolbox</em>
</p>

El archivo yaml generado presenta las siguientes caracterìsticas
### Map Configuration Parameters (`.yaml`)

### Parámetros de Configuración del Mapa (`.yaml`)

Este archivo define los metadatos utilizados para interpretar el mapa de cuadrícula de ocupación 2D en ROS 2.

| Parámetro | Valor | Descripción |
| :--- | :--- | :--- |
| **`image`** | `F1tenth_map.pgm` | Ruta al archivo de imagen que contiene los datos del mapa (generalmente un archivo PGM / Portable Gray Map). |
| **`mode`** | `trinary` | Determina cómo se interpretan los valores de los píxeles: se clasifican en libre, ocupado o desconocido. |
| **`resolution`** | `0.05` | La resolución espacial del mapa, medida en **metros por píxel**. (0.05 equivale a 5 cm por píxel). |
| **`origin`** | `[-4.18, -8.79, 0]` | Las coordenadas del mundo real `[x, y, yaw]` (metros, radianes) correspondientes a la esquina inferior izquierda del mapa. |
| **`negate`** | `0` | Invierte la interpretación de los colores blanco y negro. `0` = estándar (blanco es libre, negro es ocupado). |
| **`occupied_thresh`** | `0.65` | Umbral de probabilidad (de 0 a 1). Los valores superiores a este se consideran completamente ocupados (obstáculos). |
| **`free_thresh`** | `0.25` | Umbral de probabilidad (de 0 a 1). Los valores inferiores a este se consideran espacio totalmente libre y navegable. |

# Generaciòn de la trayectoria de planificaciòn global y suavizado de curva.

Primero se requiere descargar el repositorio desde el terminal:
```bash
git clone [https://github.com/jcbermeo95/Mapeo-y-Planificaci-n-Global-Autodrive.git](https://github.com/jcbermeo95/Mapeo-y-Planificaci-n-Global-Autodrive.git)
```
Ahora crear una carpeta dentro del repositorio descargado , ubicado en Home, con el nombre de python_motion_planning y agregar las carpetas de common, controller, path_planner y traj_optimizer, además del archivo __init__.py. Crear ademàs otra carpeta en el repositorio con el nombre de Mapas-F1Tenth. Previo a mover los archivos pgm y yaml creados en el paso del mapeo se debe modificar la pista generada agregando un obstáculo rectangular para indicar separar la región del punto inicial y final de la trayectoria generada por el algoritmo de planificación global, en este caso D*. Esta imagen tiene que convertirse a formato png y en el archivo yaml se tiene que modificar el nombre del camino con su respectiva extensión png en el parámetro image. 
