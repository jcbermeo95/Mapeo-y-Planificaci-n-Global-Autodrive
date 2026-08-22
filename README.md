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

This file defines the metadata used to interpret the 2D occupancy grid map in ROS 2.

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **`image`** | `F1tenth_map.pgm` | Path to the image file containing the map data (usually a Portable Gray Map). |
| **`mode`** | `trinary` | Determines pixel classification: pixels are interpreted as either free, occupied, or unknown. |
| **`resolution`** | `0.05` | The spatial resolution of the map, measured in **meters per pixel**. (0.05 = 5 cm per pixel). |
| **`origin`** | `[-4.18, -8.79, 0]` | The real-world coordinates `[x, y, yaw]` (meters, radians) of the map's bottom-left pixel. |
| **`negate`** | `0` | Inverts color interpretation. `0` = standard (white is free, black is occupied). |
| **`occupied_thresh`** | `0.65` | Threshold (0 to 1). Values above this are treated as completely occupied (obstacles). |
| **`free_thresh`** | `0.25` | Threshold (0 to 1). Values below this are treated as completely free (navigable space). |

# Generaciòn de la trayectoria de planificaciòn global y suavizado de curva.

Primero se requiere descargar el repositorio desde el terminal:
```bash
git clone [https://github.com/jcbermeo95/Mapeo-y-Planificaci-n-Global-Autodrive.git](https://github.com/jcbermeo95/Mapeo-y-Planificaci-n-Global-Autodrive.git)
