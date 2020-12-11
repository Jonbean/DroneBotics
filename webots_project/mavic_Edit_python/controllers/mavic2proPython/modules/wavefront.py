import numpy as np


def is_valid_cell(cell, num_rows, num_cols):
    '''
    Determines whether a cell is within the bounds of the grid

    Parameters
    ----------
    cell : tuple
        row and column index
    num_rows: int
    num_cols: int
        
    Returns
    -------
    bool
        True if cell is within grid bounds

    '''
    i, j = cell
    if i >= 0 and i < num_rows and j >=0 and j < num_cols:
        return True
    else:
        return False

# Could do something smarter in this function if you know the direction of the start position
def get_directions(type="diag"):
    '''
    Returns all of the possible directions to travel in the grid, given a type of neighborhood

    Parameters
    ----------
    type: string
        Gives the type of neighborhood (diag,square)
        
    Returns
    -------
    list of tuples
    '''
    if type == "diag":
        return [(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1),(1,0),(1,-1)]
    elif type =="square":
        return [(0,-1),(-1,0),(0,1),(1,0)]


def get_valid_neighbors(cur_cell, visited_grid, grid):

    
    '''
    Returns all the neighbors of a given cell that are valid cells, unvisited, and not an obstacle

    Parameters
    ----------
    cur_cell : tuple
        row and column index
    visited_grid: np.array
        Boolean grid with visited cells True
    grid: np.array
        Potential function grid
        
    Returns
    -------
    neighbors: list
        List of indices (tuples) of valid neighbors
    '''

    num_rows, num_cols = visited_grid.shape
    directions = get_directions()
    neighbors = []
    for dir in directions:
        nr = cur_cell[0] + dir[0]
        nc = cur_cell[1] + dir[1]
        if is_valid_cell((nr,nc),num_rows,num_cols) and not visited_grid[nr,nc] and grid[nr,nc]:
            neighbors.append((nr,nc))
    return neighbors
    


def potential_function(grid, goal_cell):
    '''
    Computes the potential function for the grid starting at the goal state
    
    Parameters
    ----------
    grid: np.array
        Binary 2d matrix with obstacles as 0
    
    goal_cell: tuple
        Index of goal cell in grid
        
    Returns
    -------
    grid: np.array
        Potential function on the grid

    '''
    row,col = goal_cell
    p = 1
    
    visited_grid = np.zeros_like(grid,dtype=bool)

    # initialize goal potential to 1
    grid[row,col] = p
    visited_grid[row,col] = True
    cur_set = [goal_cell]
    while cur_set:
        p += 1
        next_set = []
        for cell in cur_set:
            cr, cc = cell
            neighbors = get_valid_neighbors(cell, visited_grid, grid)
            for neighbor in neighbors:
                nr,nc = neighbor
                grid[nr,nc] = p
                visited_grid[nr,nc] = True
                next_set.append(neighbor)
        
        cur_set = next_set
    
    return grid

    

        
def main():
    grid_map = np.array([[1,1,1,1,1,0],[1,1,0,1,1,0],[1,1,0,0,1,0],[1,1,1,1,1,0],[1,1,1,1,1,1]])
    print(potential_function(grid_map,(1,3)))


if __name__ == "__main__":
    main()
