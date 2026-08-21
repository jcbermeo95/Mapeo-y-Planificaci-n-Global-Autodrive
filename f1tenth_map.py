import os
import cv2
import csv
import yaml
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pathlib import Path
from python_motion_planning.common import *
from python_motion_planning.path_planner import *
from python_motion_planning.traj_optimizer import *


def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path)  # asegurar Path
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)


    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    map_img = cv2.flip(map_img, 0)
    
    resolution = map_config['resolution']
    origin = map_config['origin']

    # Binarizar: 1 = ocupado, 0 = libre
    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1

    # Engrosar obstáculos según el factor
    if downsample_factor > 12:
        map_bin = cv2.dilate(map_bin, np.ones((5, 5), np.uint8), iterations=2)
    elif downsample_factor >= 4:
        map_bin = cv2.dilate(map_bin, np.ones((3, 3), np.uint8), iterations=1)
    # para 1-3 no se dilata

    # Downsampling con interpolación adecuada
    map_bin = map_bin.astype(np.float32)
    h, w = map_bin.shape
    new_h, new_w = h // downsample_factor, w // downsample_factor
    map_bin = cv2.resize(map_bin, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Re-binarizar según nivel
    if downsample_factor > 12:
        map_bin = (map_bin > 0.10).astype(np.uint8)
    elif downsample_factor >= 4:
        map_bin = (map_bin > 0.25).astype(np.uint8)
    else:
        map_bin = (map_bin >= 0.5).astype(np.uint8)

    # Ajustar resolución
    resolution *= downsample_factor

    return map_bin, resolution, origin


def grid_from_map(map_bin):
    h, w = map_bin.shape
    env = Grid(bounds=[[0, w],[0, h]])
    env.fill_boundary_with_obstacles()
    for x in range(w):
      for y in range(h):
        if map_bin[y,x]==1:
          env[x,y] = TYPES.OBSTACLE
    env.inflate_obstacles(radius=1)
    return env


def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)


def map_to_world(x_map, y_map, resolution, origin, image_height):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)


def save_path_as_csv(path, filename, resolution, origin, image_height):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x_map, y_map in reversed(path):  # invertir para ir de start a goal
            x, y = map_to_world(x_map, y_map, resolution, origin, image_height)
            writer.writerow([x, y])


if __name__ == "__main__":
    HERE = Path(__file__).resolve().parent
    yaml_path = HERE / "Mapas-F1Tenth" / "F1tenth_map1.yaml"
    downsample_factor = 5  # Ajusta este valor según lo que necesites

    x_start, y_start= 0.45, 0.67
    x_goal, y_goal = 0.36, 2.00

    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)
    map_ = grid_from_map(map_bin)

    start = world_to_map(x_start, y_start, resolution, origin)
    goal = world_to_map(x_goal, y_goal, resolution, origin)
    map_[start] = TYPES.START
    map_[goal] = TYPES.GOAL
    print(f"Start (map): {start}, Goal (map): {goal}")
    
    planner = DStar(map_=map_, start=start, goal=goal)
    path, path_info = planner.plan()
    path_world = map_.path_map_to_world(path)
    
    generator = BSpline(step=0.5, k=4)
    smooth_path, curve_info = generator.generate(path_world)
    
    #Primer grafico de camino sin suavizar
    vis1 = Visualizer2D()
    vis1.plot_grid_map(map_)
    vis1.ax.set_xlim(map_.bounds[0])
    vis1.ax.set_ylim(map_.bounds[1])
    vis1.fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
    vis1.plot_path(path, style="-", color="C2", linewidth=2)
    vis1.show()
    vis1.close()
    
    #Camino con Bspline suavizado
    vis2 = Visualizer2D()
    vis2.plot_grid_map(map_)
    vis2.ax.set_xlim(map_.bounds[0])
    vis2.ax.set_ylim(map_.bounds[1])
    vis2.fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
    vis2.plot_path(smooth_path, style="-", color="C1", linewidth=2)
    vis2.show()
    vis2.close()
    
    #Comparacion de las dos curvass
    vis3 = Visualizer2D()
    vis3.plot_grid_map(map_)
    vis3.ax.set_xlim(map_.bounds[0])
    vis3.ax.set_ylim(map_.bounds[1])
    vis3.fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
    vis3.plot_path(smooth_path, style="-", color="C1", map_frame=False, linewidth=4)
    vis3.plot_path(path, style="-", color="C2", linewidth=2)
    vis3.show()
    vis3.close()
    
    save_path_as_csv(path, "d_path_real.csv", resolution, origin, map_bin.shape[0])
