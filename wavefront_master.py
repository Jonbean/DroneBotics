#! usr/bin/python3

# inhouse pkgs
from modules import cell_size
from modules import grid_decomposition
from modules import wavefront
from modules import gradientascent
from modules.wavefront import get_valid_neighbors, is_valid_cell, get_directions
# std pkgs
import sys
import random
import numpy as np
# non std
import matplotlib.pyplot as plt
import matplotlib.collections as collections
import matplotlib
import matplotlib.patches as mpatches


"""
This master script impliments the wavefront algorithm based on input files:
"""

def path_between(from_point, to_point, step_size):
    '''
    Find path between point specified by a step size
    '''

    # TODO: Use a path resolution of 10 steps for computing a path between points
    
    # find the intermediate end positions 
    potential_path = []
    for d in range(len(from_point)):                                            
        dist = np.linspace(from_point[d], to_point[d], step_size, endpoint=True)
        potential_path.append(dist)                                                       
    potential_path = list(np.array(potential_path).T)                                          
    potential_path = [list(point) for point in potential_path]
    potential_path = [ (round(point[0],2), round(point[1],2)) for point in potential_path] 

    return potential_path


def find_intermediate_points(coverage_path):
    """
    This function finds linear points between each cell to help the 
    drone navigate without gaining too much momentum.
    """
    # init
    start_point = (0,0)
    drone_path = [start_point]
    
    for point in coverage_path:
        point = (-1*point[0], point[1]) # x needs to be mirrored for webots
        path_to_next = path_between(drone_path[-1], point, 10)
        for path in path_to_next:
            drone_path.append(path)
        
    return drone_path 

def main():
    """
    This main function controls the flow of the script
    """
    ####
    # PARAMETERS
    ####
    start_cell = (4,0)                                                          
    goal_cell = (1,3) 
    cell_width, cell_height = 1, 1
    aoi_height, aoi_width = 10, 10
    obstacles = [grid_decomposition.Obstacle(location=(1.0, 1.0), radius=0.5),  
                 grid_decomposition.Obstacle(location=(7.2, 6.8), radius=0.8)]

    ####
    # step 1: get the cell size
    ####
    #fov = cell_size.get_camera_fov(0.5, 1.5, 1.5) 
    #cell_width, cell_height = cell_size.get_cell_size(fov, 0)

    # TODO: Above doesn't work with grid decomposition

    ####
    # step 2: get the decomposed grid
    ####

    # init map constructor obj
    graph_map = grid_decomposition.Map_Constructor(img_width=aoi_width, 
                                                   img_height=aoi_height, 
                                                   cell_width=cell_width, 
                                                   cell_height=cell_height)

    # call member function to get coordinate matrix
    cell_center_matrix = graph_map.map_graph_construction()
    # call member function to get availability matrix
    availability_matrix = graph_map.map_obstacle_label(obstacles)

    # render results
    print("-"*30,"test start","-"*30)
    print("cell center coordinates matrix: ")
    print(cell_center_matrix)
    print("-"*80)
    print("cell availability matrix: ")
    print(availability_matrix)
    print("-"*80)

    ####
    # Visualize what decomposed graph looks like.
    ####
    # visualization with heat map
    state_bounds = np.array([[0, aoi_height], [0, aoi_width]])
    grid_decomposition.visualize_2D_graph(state_bounds, 
                                          availability_matrix, 
                                          obstacles, 
                                          cell_width, 
                                          cell_height)

    ####
    # Step 3: Wavefront Algorithm
    ####
    potential_func = wavefront.potential_function(availability_matrix, goal_cell)
    print(f"potential function: {potential_func}")

    ####                                                                        
    # Step 4: Gradient Ascent
    ####
    coverage_path = gradientascent.grad_ascent(potential_func, start_cell, goal_cell) 
    print(f"coverage_path: {coverage_path}") 
    grid_decomposition.visualize_2D_graph(state_bounds, 
                                          availability_matrix, 
                                          obstacles, 
                                          cell_width, 
                                          cell_height, 
                                          waypoints=coverage_path)
    ####
    # Step 5: Find points for the drone starting from 0,0
    ####
    drone_path = find_intermediate_points(coverage_path)
    print(drone_path)
    
if __name__ == "__main__":
    main()
