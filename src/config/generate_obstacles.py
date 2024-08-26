#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 29 11:57:42 2020

@author: Alexandre Bonnefond
@author: Théotime Balaguer
"""
import time
from pathlib import Path
import numpy as np 
import matplotlib.pyplot as plt
from os import path, mkdir, chmod
import yaml
import sys


def gen_obstacles(distrib = "uniform", num_obst = 30, radius_obst = 500, std = 100):
    
    if distrib == "uniform":
        coordX = np.random.uniform(ArenaCenterX - radius, ArenaCenterX + radius, num_obst)
        coordY = np.random.uniform(ArenaCenterY - radius, ArenaCenterY + radius, num_obst)
        
     
    elif distrib == "normal":
        coordX = np.random.normal(ArenaCenterX, radius/2, num_obst)
        coordY = np.random.normal(ArenaCenterY, radius/2, num_obst)
    
    else:
        sys.exit("Distribution not supported. Supported: [uniform, normal]")
        
    coords = np.vstack((coordX, coordY)).T
    size_obst = np.random.normal(radius_obst, std, num_obst)
    try:
        if check_intersection(coords, size_obst) == True:
            coords, size_obst = gen_obstacles(distrib, num_obst, radius_obst, std)
    except RecursionError as e:
        print("RecursionError")
        sys.exit()
        
    return coords, size_obst
    
    
def check_intersection(centers, sizes):
    assert(centers.shape == (len(sizes), 2))
    intersect = False
    for i in range(len(sizes)):
        point_i = get_edges(centers[i, :], sizes[i])
        for j in range(i+1, len(sizes)):
            point_j = get_edges(centers[j, :], sizes[j])
            if not(point_i[0, 0] > point_j[1, 0] or point_j[0, 0] > point_i[1, 0]):
                if not(point_i[1, 1] > point_j[2, 1] or point_j[1, 1] > point_i[2, 1]):
                    intersect = True
                    return intersect
            
def get_edges(center, size):
    points_list = np.zeros((4, 2))  #only square shapes 
    
    points_list[0, 0] = center[0] - size   # bottom left point
    points_list[0, 1] = center[1] - size
    
    points_list[1, 0] = center[0] + size   # bottom right point
    points_list[1, 1] = center[1] - size

    points_list[2, 0] = center[0] + size   # top right point
    points_list[2, 1] = center[1] + size

    points_list[3, 0] = center[0] - size   # top left point
    points_list[3, 1] = center[1] + size   
    
    return points_list     


if __name__ == '__main__':
    
    strings = time.strftime("%m,%d,%H,%M")
    t = strings.split(',')
    
    config_file = sys.argv[1]
    
    script_path = path.dirname(path.realpath(__file__))
    if not path.isdir(script_path + "/auto_obstacles"):
        mkdir(script_path + "/auto_obstacles")
    newpath = Path(script_path + "/auto_obstacles")
                           
    with open(script_path +'/'+ config_file, 'r') as file:
        config = yaml.safe_load(file)
        
    if config['auto_buildings'] == False:
        print(script_path +'/'+ config_file)
        sys.exit()
    
    radius = config['arena_radius']
    ArenaCenterX = config['arena_center_x']
    ArenaCenterY = config['arena_center_y']
    distrib = config['distribution']
    numb_obst = config['numb_obst']
    radius_of_obst = config['radius_obst']
    std = config['radius_stdev_obst']
    height = config['height_obst']
    radius -= radius_of_obst + 30 #avoid obstacles on the edges of the arena
    filename = path.splitext(config_file)[0]+"_auto_obst_" + distrib + "_" + str(numb_obst) + "_" + t[0] + "_" + t[1] + "_"\
                                + t[2] + "_" + t[3] + ".yaml"    
    
    config['buildings'] = []
        
    coords, size_obst = gen_obstacles(distrib, numb_obst, radius_of_obst, std)
    plt.figure()
    for i in range(numb_obst):
        points = get_edges(coords[i, :], size_obst[i])
        plt.plot(points[:, 0], points[:, 1])
                    
        config['buildings'].append({'name': 'building_'+str(i), 'x': float(coords[i, 0]), 'y': float(coords[i, 1]), 'size_x': float(size_obst[i]), 'size_y': float(size_obst[i]), 'height': float(height)})
        
    with open(newpath / filename, 'w') as file:
        yaml.dump(config, file)
    
    chmod(newpath / filename, 0o666)
        
    print(newpath / filename)
    # plt.show()










