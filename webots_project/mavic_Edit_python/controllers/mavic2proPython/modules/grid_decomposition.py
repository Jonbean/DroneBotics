import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.collections as collections
import matplotlib
import matplotlib.patches as mpatches

import math

def rgb(value, minimum, scale):
    '''
    rgb function takes in scalar information and convert them into heat map rgb-color tuple

    Parameters
    ----------
    value : float
        Heat value of a cell
    minimum : float
        The min heat value among all cells
    scale: float
        The difference between the max and min heat value
        
    Returns
    -------
    r: float
        Red value in range (0,1) as requested by matplotlib color specification
    g: float
        Green value in range (0,1) as requested by matplotlib color specification
    b: float
        Blue value in range (0,1) as requested by matplotlib color specification
    '''
    ratio = 2 * (value-minimum) / scale
    b = int(max(0, 255*(1 - ratio)))
    r = int(max(0, 255*(ratio - 1)))
    g = 255 - b - r
    return r/255., g/255., b/255.

def plot_cell(xy, width, height, color):
    '''
    plot a rectangular cell on a map

    Parameters
    ----------
    xy : tule(float)
        the bottom left corner coordinate of the rectangular 
    width : float
        The width value of the rectangular
    height: float
        The height value of the rectangular
    color: tuple(float)
        The color tuple that contains (value, min_value, scale_value) for rgb function to calculate the heat rgb-color
        
    Returns
    -------
    rectangular_patch: matplotlib.patches
        the patch that need to be added to plot as a shape on the final plot
    '''
    r, g, b = rgb(color[0], color[1], color[2])
    final_color = (r,g,b,1.0)

    return mpatches.Rectangle(xy, width, height, facecolor=final_color)

def visualize_2D_graph(state_bounds, cells, obstacles, cell_width, cell_height, filename=None, waypoints=None):
    '''
    plot the cells and obstacles on a 2d canvas, this is just for visualization.

    Parameters
    ----------
    state_bounds : tule(float)
        The boundary values of the canvas, in the format as [[x_min, x_max], [y_min, y_max]]
    cells : 2d numpy array
        This matrix contains the heat value of each cell
    obstacles: List(Obstacle)
        A list of Obstacle objects
    cell_width: float
        the width of each cell on the canvas
    cell_height: float
        the height of each cell on the canvas
    filename(optional): path
        the path to save the ploted visualization
    Returns
    -------
    None
    '''

    
    fig, ax = plt.subplots(1,figsize=(6,6))

    ax.set_xlim(state_bounds[0,0], state_bounds[0,1])
    ax.set_ylim(state_bounds[1,0], state_bounds[1,1])
    patches = []
    # polygon = plt.Rectangle((-400, -400), 10, 10, color='yellow') #Rectangle((-400, -400), 10, 10, color='y')
    
    color_scale = np.max(cells) - np.min(cells)
    min_color = np.min(cells)
    for i in range(cells.shape[0]):
        for j in range(cells.shape[1]):
            xy = (i * cell_width, j * cell_height)

            # color_idx = (cells[i,j] - min_color) / color_scale
            
            polygon = plot_cell(xy, cell_width, cell_height, (cells[i,j], min_color, color_scale))
            # patches.append(polygon)
            ax.add_patch(polygon)

    for obs in obstacles:
        # circle = plot_circle(obs.location[0], obs.location[1], obs.radius)
        circle = mpatches.Circle([obs.location[0], obs.location[1]], radius = obs.radius, color = 'y')
        # patches.append(circle)
        ax.add_patch(circle)

    
    # if cover path points are provided, render with the cover path plots
    if waypoints != None:
        for i in range(len(waypoints)-1):
            
            # calculating cover path on the image
            x = waypoints[i][0] * cell_width + cell_width * 0.5
            y = waypoints[i][1] * cell_height + cell_height * 0.5
            dx = waypoints[i+1][0] * cell_width + cell_width * 0.5 - x
            dy = waypoints[i+1][1] * cell_height + cell_height * 0.5 - y
            vec = np.array([waypoints[i+1][0] - waypoints[i][0], waypoints[i+1][1] - waypoints[i][1]])

            # check heuristically if re-routing happened, if so plot with megenta arrows
            if np.linalg.norm(vec) >= 1.5:
                arrow = mpatches.Arrow(x, y, dx, dy, width=0.5, color='m')
            else:
                # regular path plot with yellow arrows
                arrow = mpatches.Arrow(x, y, dx, dy, width=0.5, color='y')
            ax.add_patch(arrow)
        


    if filename is not None:
        fig.savefig(filename)
    else:
        plt.show()



class Obstacle(object):
    """Obstacle Class is just a information container
    
    Attributes
    ----------
    location : tuple(float)
        the 2d coordinate of the center of the obstacle
    radius: float
        The radius of the obstacle(we assume the obstacles are in circle shape)
    """

    def __init__(self, location, radius):
        super(Obstacle, self).__init__()
        self.location = location
        self.radius = radius
        
class Map_Constructor(object):
    """
    Map_Constructor class is to hold map information and generate grid decomposition graph of it

    
    Attributes
    ----------
    img_width : float
        the width of the image of the map
    img_height: float
        The height of the image of the map
    cell_width: float
        The width of each survey cell predetermined by the height, overlap ratio and camera config of the UAV
    cell_height: float
        The height of each survey cell ...
    """
    def __init__(self, img_width, img_height, cell_width, cell_height):
        super(Map_Constructor, self).__init__()
        
        self.img_width = img_width
        self.img_height = img_height
        self.cell_width = cell_width
        self.cell_height = cell_height
        
        # zero_one_mat = self.map_obstacle_label(obstacles)

    def map_graph_construction(self):
        '''
        map_graph_construction function decompose the input image with respect to the cell size

        Parameters
        ----------
        None

        Returns
        -------
        coord_matrix: 3d numpy array
            The last dimension of this 3d array contains all the coordinate(vector) of the center of the cells. It can be thought of as a matrix of coordinates
        '''
        x_cells = self.img_width // self.cell_width
        y_cells = self.img_height // self.cell_height

        coord_matrix = np.swapaxes(np.mgrid[self.cell_width/2.0:self.img_width:self.cell_width, self.cell_height/2.0:self.img_height:self.cell_height], 0, 2)

        return coord_matrix

    def map_obstacle_label(self, obstacles):
        indices_map = np.ones((int(self.img_width//self.cell_width), int(self.img_height//self.cell_height)))

        for obs in obstacles:
            x_left = obs.location[0] - obs.radius
            x_right = obs.location[0] + obs.radius

            y_btm = obs.location[1] - obs.radius
            y_top = obs.location[1] + obs.radius

            x_left_idx = int(np.floor(x_left / self.cell_width))
            x_right_idx = int(np.floor(x_right / self.cell_width))

            y_top_idx = int(np.floor(y_top / self.cell_height))
            y_btm_idx = int(np.floor(y_btm / self.cell_height))

            print(x_left_idx, x_right_idx, y_btm_idx, y_top_idx)

            for i in range(x_left_idx, x_right_idx+1):
                for j in range(y_btm_idx, y_top_idx+1):
                    indices_map[i, j] = 0

            
        return indices_map


if __name__ == '__main__':
    
    # this script is for testing purpose

    # init 2 obstacles
    obstacles = [Obstacle(location=(1.0, 1.0), radius=0.5), Obstacle(location=(7.2, 6.8), radius=0.8)]

    # init cell size
    cell_width = 1.0
    cell_height = 1.0

    # init map constructor obj
    graph_map = Map_Constructor(img_width=10.0, img_height=10.0, cell_width=cell_width, cell_height=cell_height)

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

    # visualization with heat map
    state_bounds = np.array([[0, 10], [0, 10]])
    visualize_2D_graph(state_bounds, availability_matrix, obstacles, cell_width, cell_height)




