#!/usr/bin/env python3
import os
import heapq
import numpy as np
from enum import Enum
from functools import partial


import rospy
from mapping.simple_grid_map import GridMap3D
from mapping.map_visualizer import VoxelVisualizer
from planners.planer_base import PathPlannerWrapper
from planners.a_star_planner import AStarPlanner
from planners.path_visualization import PathVisualizer


def timer_callback(visualizer, event):
    visualizer.send_update()

def draw_path(path_vis, path, event):
    path_vis.draw_path(path)  

if __name__ == "__main__":
    rospy.init_node('path_planning_test')
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Читаем карту из файла
    map = GridMap3D()
    map.read_from_file(script_dir + "/map_example.map")
    
    visualizer = VoxelVisualizer(size=1)
    # очищаем текущий массив карты
    visualizer.clear()
    for x, y, z, value in map:
        if np.isclose(value, 1.0):
            visualizer.add_marker(x, y, z) 
    
    planner = PathPlannerWrapper(AStarPlanner())
    
    # Устанавливаем целевое положение и текущее положение
    current_pose = (-4, -4, 3)
    goal_pose = (1, 10, 4)
    
    path = planner.find_path(current_pose, goal_pose, map)
    
    path_vis = PathVisualizer()
    
    rospy.Timer(rospy.Duration(1.0), partial(timer_callback, visualizer))
    rospy.Timer(rospy.Duration(1.0), partial(draw_path, path_vis, path))
    
    # Spin to keep the node running
    rospy.spin()
    
