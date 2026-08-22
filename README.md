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
Para generar el mapeo (SLAM) en la pista del AutoDRIVE, se recomienda seguir los pasos establecidos en https://github.com/nabihandres/AUTODRIVE/blob/main/Tutorial%203%3A%20Simultaneous%20Localization%20and%20Maping.md
