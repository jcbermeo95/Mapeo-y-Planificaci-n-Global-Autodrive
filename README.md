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
Se recomienda cambiar el nombre del repositorio a Repositorio. Ahora crear una carpeta dentro del repositorio descargado , ubicado en Home, con el nombre de python_motion_planning y agregar las carpetas de common, controller, path_planner y traj_optimizer, además del archivo __init__.py. Crear ademàs otra carpeta en el repositorio con el nombre de Mapas-F1Tenth. Previo a mover los archivos pgm y yaml creados en el paso del mapeo se debe modificar la pista generada agregando un obstáculo rectangular para indicar separar la región del punto inicial y final de la trayectoria generada por el algoritmo de planificación global, en este caso D*. Esta imagen tiene que convertirse a formato png y en el archivo yaml se tiene que modificar el nombre del camino con su respectiva extensión png en el parámetro image. 

Para generar las trayectorias de planificación global se tiene que ir al terminal realizar los siguientes comandos:
```bash
cd Repositorio
python3 f1tenth_map.py
```

El resultado generado por este algoritmo de planificación global D* se evidencia en el siguiente enlace: https://youtu.be/RnizpRwMj-s. 

<img width="284" height="633" alt="Screenshot from 2026-08-21 23-02-52" src="https://github.com/user-attachments/assets/f45551c4-ada2-4c6e-b799-cedb32502b14" />
<img width="284" height="633" alt="Screenshot from 2026-08-21 23-03-02" src="https://github.com/user-attachments/assets/cf9f7787-59fa-4e8c-8a89-7e62dfcbe08b" />
<img width="284" height="633" alt="Screenshot from 2026-08-21 23-03-08" src="https://github.com/user-attachments/assets/fc7e8797-737b-41e1-9e67-a927275dff2f" />

La primera imagen representa a la trayectoria generada por el algoritmo D* sin suavizado de curva, la segunda imagen representa a la trayectoria con suavizado usando B-Spline y la tercera imagen es una comparación de las dos curvas.

### Explicación del Algoritmo y Parámetros

Este script implementa un pipeline completo de **planificación de movimiento para vehículos autónomos (F1Tenth)**, abarcando desde el procesamiento del mapa hasta la generación de trayectorias suaves y su exportación.

---

### Fases del Algoritmo

1. **Carga y Procesamiento del Mapa (`load_map` y `grid_from_map`):**
   * Lee la configuración YAML y la imagen PGM del mapa.
   * Aplica un factor de reducción (*downsampling*) y binariza la imagen para distinguir espacios libres de obstáculos.
   * Dilata los obstáculos para añadir un margen de seguridad (evita que el vehículo colisione con las paredes).
   * Convierte la matriz resultante en una cuadrícula (`Grid`) estructurada para los algoritmos de búsqueda.

2. **Transformación de Coordenadas:**
   * Traduce las coordenadas del mundo real en metros (`x_start`, `y_start`, etc.) a índices de píxeles en la cuadrícula del mapa mediante `world_to_map()`.

3. **Planificación de Ruta Global (D\*):**
   * Utiliza el algoritmo **D\*** (*Dynamic A\****) para encontrar la ruta óptima basada en grafos desde el punto inicial (*start*) hasta la meta (*goal*). Es ideal para entornos estáticos o con replanificación dinámica.

4. **Suavizado de Trayectoria (B-Splines):**
   * La ruta inicial de D\* está compuesta por celdas discretas (en forma de cuadrícula escalonada). El generador **B-Spline** (`BSpline`) interpola y suaviza la curva para hacerla viable y continua para la dinámica de un vehículo de carreras.

5. **Visualización y Exportación:**
   * Genera tres gráficos independientes: la ruta D\* sin suavizar, la curva B-Spline suavizada, y una comparativa superpuesta de ambas.
   * Guarda los puntos de la ruta resultante en archivos CSV (`d_path.csv` y `bspline.csv`).

---

### Parámetros Principales a Modificar

Puedes personalizar el comportamiento del script modificando las siguientes variables en el bloque principal (`if __name__ == "__main__":`):

| Parámetro | Dónde se encuentra | Descripción |
| :--- | :--- | :--- |
| **`yaml_path`** | Línea principal | Ruta al archivo `.yaml` del mapa que deseas cargar (ej. `F1tenth_map1.yaml`). |
| **`downsample_factor`** | Línea principal | Factor de reducción de escala del mapa. Valores más altos aceleran el cálculo pero reducen la precisión geométrica; valores bajos aumentan la resolución y el tiempo de procesamiento. |
| **`x_start, y_start`** | Línea principal | Coordenadas en el mundo real (metros) para la posición inicial del vehículo. |
| **`x_goal, y_goal`** | Línea principal | Coordenadas en el mundo real (metros) para el punto de destino u objetivo. |
| **`step` (BSpline)** | `BSpline(step=0.5, k=4)` | Distancia o paso de interpolación para los puntos de la curva suavizada. |
| **`k` (BSpline)** | `BSpline(step=0.5, k=4)` | Grado de la curva B-Spline (por defecto `4` para curvas cúbicas suaves). |

Se generarán dos archivos csv para la curva sin suavizar y con suavizado, encontrados en Repositorio. 
