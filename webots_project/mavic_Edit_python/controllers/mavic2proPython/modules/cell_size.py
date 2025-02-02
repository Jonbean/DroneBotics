import numpy as np


def get_camera_fov(altitude, vert_angle, horiz_angle):
    '''
    Computes cameras field of view
    
    Parameters
    ----------
    altitude : float
        Height of the camera
    vert_angle : float
        Vertical degree of camera
    horiz_angle: float
        Horizontal degree of camera
        
    Returns
    -------
    width: float
        Gives camera field of view width
    length: float
        Gives camera field of view length
    '''
    width = 2*altitude * np.tan(vert_angle/2)
    length = 2*altitude * np.tan(horiz_angle/2)
    return width, length

def get_cell_size(fov, overlap_rate):
    '''
    Computes cell size for decomposition
    
    Parameters
    ----------
    fov : tuple
        (width,length) of the camera's field of view
    overlap_rate : float
        Value between 0 and 1 that gives the percent cell images overlap 

    Returns
    -------
    width: float
        Gives cells width
    length: float
        Gives cells length
    '''
    width = (1-overlap_rate) * fov[0]
    length = (1-overlap_rate) * fov[1]
    return width, length

####
# MAIN
####
def main():
    """
    Example is below
    """
    fov = get_camera_fov(2, 90, 90)
    width, length = get_cell_size(fov, 0.1)
    print(width, length)

if __name__ == "__main__":
    main()   
