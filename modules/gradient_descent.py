import numpy as np
from .wavefront import potential_function, get_valid_neighbors, is_valid_cell, get_directions


def get_lowest_potential_neighbor(current_cell,visited_grid,potential_grid):
    '''
    Returns the lowest potential valid neighbor of the current cell

    Parameters
    ----------
    current_cell: tuple
        Tuple containing index of current cell in grid
    
    visited_grid: np.array
        Boolean grid of already visited cells

    potential_grid: np.array
        Potential grid   
        
    Returns
    -------
    next_cell: tuple
        Tuple containing index of next cell to visit in grid

    '''
    neighbors = get_valid_neighbors(current_cell,visited_grid,potential_grid)
    min_potential = float("inf")
    next_cell = None
    for x,y in neighbors:
        potential = potential_grid[x,y]
        if potential < min_potential:
            next_cell = (x,y)
            min_potential = potential
    return next_cell



def gradient_descent(potential_grid,start,goal):
    '''
    Finds the shortest path from start to goal in grid

    Parameters
    ----------
    potential_grid: np.array
        Potential grid  

    start: tuple
        Tuple containing index of start cell in grid
    
    goal: tuple
        Tuple containing index of goal cell in grid
 
        
    Returns
    -------
    path: list
        List of cells from (start, goal)

    '''
    visited_grid = np.zeros_like(potential_grid,dtype=bool)
    current_cell = start
    # Should path include start or goal cell? 
    path = []
    while current_cell != goal:
        next_cell = get_lowest_potential_neighbor(current_cell,visited_grid,potential_grid)
        visited_grid[next_cell[0],next_cell[1]] = True
        path.append(next_cell)
        current_cell = next_cell

    return path[:-1]


def main():
    grid = np.array([[1,1,1,1,1,0],[1,1,0,1,1,0],[1,1,0,0,1,0],[1,1,1,1,1,0],[1,1,1,1,1,1]])
    start = (2,1)
    goal = (4,4)
    # should be able to stop as soon as you reach the start state. Have to implement change for stopping in potential_function
    potential_grid = potential_function(grid,goal)
    path = gradient_descent(potential_grid,start,goal)

    print(grid)
    print(path)

    



if __name__ == "__main__":
    main()
    
    
