#! usr/bin/python3
import sys
import numpy as np
import random
#from wavefront import *
#from gradient_descent import *
from .wavefront import get_valid_neighbors, is_valid_cell, get_directions
from .gradient_descent import gradient_descent

"""
This module contains all the  necessary functions to impliment the method 
of gradient ascent to find the best coverage path with an input potential
function.
"""

def calculate_euc_dist(node_a, node_b):
    """ 
    calculates the manhattan distance.
    """
    x_dist = abs(node_a[0] - node_b[0])
    y_dist = abs(node_a[1] - node_b[1])
   
    return x_dist + y_dist


def find_highpot_unvisited(curr_node, visited_grid, potential_function):
    '''                                                                         
    Finds the node with the highest potential given that it is  unvisited.               
                                                                                
    Parameters                                                                  
    ---------- 
    curr_node: tuple
                                                                 
    potential_function: np.array
        np.array with numbers corresponding to the potential at that index      
                                                                                
    visited_grid: Binary np.array 
        np.array with True and False entries indicating if that cell has been
        added to the coverage path.                                                          
                                                                                
    Returns                                                                     
    -------                                                                     
    best_unvisited: a tuple indicating the index of the best next node. ex: (1,2)  
        An unvisited node with the highes potential                       
        If more than one with high potential choose closest
    '''
    non_true = np.where(visited_grid==False)
 
    # find nodes not visited or negative
    not_visited_or_boundry = {}
    for node_i in range(len(non_true[0])):
        x = non_true[0][node_i]
        y = non_true[1][node_i]
        if potential_function[x][y] != 0:
            pot = potential_function[x][y]
            node = (x,y)
            not_visited_or_boundry[node] = pot
    
    # find nodes with the highest potential.
    high_pot = 0
    not_visited_high = []
    for node, potent in not_visited_or_boundry.items():
        print(node, potent)
        if potent > high_pot:
            high_pot = potent
            not_visited_high = [node]
        elif potent == high_pot:
            not_visited_high.append(node)
        else:
            continue

    #find nodes with shortest distance
    not_visited_dist = []    
    shortest_dist = float("inf")
    for node in not_visited_high:
        dist_to_curr = calculate_euc_dist(node, curr_node)
        if dist_to_curr < shortest_dist:
            shortest_dist = dist_to_curr
            not_visited_dist = [node]
        elif dist_to_curr == shortest_dist:
            not_visited_dist.append(node)
        else:
            continue

    # if multiple left over, choose random
    if len(not_visited_dist) == 1:
        best_unvisited = not_visited_dist[0]
    elif len(not_visited_dist) > 1:
        best_unvisited = random.choice(not_visited_dist)
    else:
        print("ERROR: the not_visited_dist array in find_highpot_unvisited is empty!")

    return best_unvisited

def path_to_node(node_a, node_b, potential_function):
    '''                                                                         
    Computes the path from node_a to node_b.
    Would use gradient descent method most likely - but will not use yet.
    Returns                                                                     
    -------                                                                     
    path_to_node_b: array with size-2 tuples inside ex: [(1,1),(1,2),...,(5,5)]  
        This output array is the path specified by cells.                       
    ''' 
    path_to_node_b = [] 

    raise NotImplementedError

def nodes_to_still_visit(visited_grid, potential_function):
    """
    check number of nodes still needed to be visited
    """
    n_not_visited = abs(np.count_nonzero(visited_grid==0) -
                        np.count_nonzero(potential_function==0))

    return n_not_visited

def action_from_statetostate(node_a, node_b):
    """
    This returns the action needed to go from node_a to node_b
    """
    x = node_b[0] - node_a[0]
    y = node_b[1] - node_a[1]
    action_taken = (x,y)

    return action_taken

def node_with_least_turn(curr_cell, prev_action, possible):
    """
    The goal of this function is to return the next node which corresponds
    to the smallest turning angle need for reachability.
    """
    smallest_action_dist = float('inf')
    for node in possible:
        action_needed = action_from_statetostate(curr_cell, prev_action)
        action_dist_x = abs(prev_action[0] - action_needed[0])
        action_dist_y = abs(prev_action[1] - action_needed[1])
        action_dist = action_dist_x + action_dist_y
        if action_dist < smallest_action_dist:
            smallest_action_dist = action_dist
            node_w_least_turn = node

    return node_w_least_turn

def grad_ascent(potential_function, start_cell, goal_cell):
    '''
    Computes the path given the potential function for the grid.
    
    Parameters
    ----------
    potential_function: np.array
        np.array with numbers corresponding to the potential at that index
    
    start_cell: tuple
        cell where the drone is currently (or should start from)
        
    goal_cell: tuple
        cell where the drone should be at the end of the coverage path.

    Returns
    -------
    coverage_path: array with size-2 tuples inside ex: [(1,1),(1,2),...,(5,5)]
        This output array is the path specified by cells.
    '''
    # init data structures
    visited_grid = np.zeros_like(potential_function, dtype=bool)
    curr_cell = start_cell
    visited_grid[curr_cell[0]][curr_cell[1]] = True
    coverage_path = [curr_cell]
    iteration = 0

    ####
    # Gradient Ascent
    ####
    while nodes_to_still_visit(visited_grid, potential_function) > 0:
        high_potential = 0
        possible = []
        neighbors = get_valid_neighbors(curr_cell, visited_grid, potential_function)
        
        # find highest ranked neighbors 
        for n_cell in neighbors:
            n_row, n_col = n_cell
            potential = potential_function[n_row][n_col]
            if potential == high_potential:
                possible.append(n_cell)
            elif potential > high_potential:
                possible = [n_cell]
                high_potential = potential
            else:
                continue

        # from possible, choose a neighbor
        if len(possible) == 0:
            # find the highest node not yet visited
            print(f"potential_function: {potential_function}")
            if chosen == goal_cell:
                print("Goal found in len(possible) == 0:")
            chosen = find_highpot_unvisited(curr_cell, visited_grid, potential_function)

            # find viable path to chosen node
            # TODO: IMPLEMENT THIS
            path_to_chosen = gradient_descent(potential_function,curr_cell, chosen)
            coverage_path += path_to_chosen
            #for intermediate_n in path_to_chosen:
            #    coverage_path.append(intermediate_n)

        elif len(possible) == 1:
            # if this is the goal, make sure no others are unvisited!
            if possible[0] == goal_cell:
                if nodes_to_still_visit(visited_grid, potential_function) >= 1:
                    chosen = find_highpot_unvisited(curr_cell, visited_grid, potential_function)
                else:
                    print("GOAL FOUND!")
                    chosen = possible[0]
            else:
                chosen = possible[0]

        if len(possible) > 1: #choose random
            # if goal state in list, remove it
            if goal_cell in possible:
                possible.remove(goal_cell)
                
            # if first action choose random, else take lowest angle from previous    
            # direction.
            if iteration == 0:
                chosen = random.choice(possible)
            else:
                chosen = node_with_least_turn(curr_cell, prev_action, possible)

        # store the action taken to go from curr_cell to chosen
        # TODO: if path appended this will need updated
        prev_action = action_from_statetostate(curr_cell, chosen)
        # Store the next chosen state
        coverage_path.append(chosen)
        visited_grid[chosen[0]][chosen[1]] = True
        # update current cell
        curr_cell = chosen
        # increment iteration counter
        iteration += 1

    return coverage_path
 
####
# MAIN FUNCTION
####
def main():
    potential_func = np.array([[4,3,2,2,2,0],
                               [4,3,0,1,2,0],
                               [4,4,0,0,2,0],
                               [5,5,4,3,3,0],
                               [6,5,4,4,4,4]])
    coverage_path = grad_ascent(potential_func, (4,0), (1,3)) 
    print(coverage_path)
    
    ####
    # Testing other functions
    ####
    #find_highpot_unvisited((0,0), np.array([[False,False],[False,False]]), np.array([[1,1],[1,0]]))
    #find_highpot_unvisited((0,0), np.array([[True,False],[False,False]]), np.array([[1,1],[1,0]]))
    #action = action_from_statetostate((1,3), (2,4))
    #print(action)

if __name__ == "__main__":
    main()
