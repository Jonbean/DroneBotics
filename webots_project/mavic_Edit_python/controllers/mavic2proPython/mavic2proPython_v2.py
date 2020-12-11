from controller import *
import mavic2proHelper
from simple_pid import PID
import csv
import struct

####
# Packages for Wavefront controller
####
# inhouse pkgs                                                                  
from modules import cell_size                                                   
from modules import grid_decomposition                                          
from modules import wavefront                                                   
from modules import gradientascent                                              
from modules.wavefront import get_valid_neighbors, is_valid_cell, get_directions
from modules.gradient_descent import gradient_descent                           
# std pkgs                                                                      
import sys                                                                      
import random                                                                   
import numpy as np                                                              
# non std                                                                       
import matplotlib.pyplot as plt                                                 
import matplotlib.collections as collections                                    
import matplotlib                                                               
import matplotlib.patches as mpatches
####
####




params = dict()
with open("../params.csv", "r") as f:
    lines = csv.reader(f)
    for line in lines:
        params[line[0]] = line[1]

TIME_STEP = int(params["QUADCOPTER_TIME_STEP"])
TAKEOFF_THRESHOLD_VELOCITY = int(params["TAKEOFF_THRESHOLD_VELOCITY"])
M_PI = 3.1415926535897932384626433

robot = Robot()

[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = mavic2proHelper.getMotorAll(robot)

timestep = int(robot.getBasicTimeStep())
mavic2proMotors = mavic2proHelper.getMotorAll(robot)
mavic2proHelper.initialiseMotors(robot, 0)
mavic2proHelper.motorsSpeed(robot, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY)

front_left_led = LED("front left led")
front_right_led = LED("front right led")
gps = GPS("gps")
gps.enable(TIME_STEP)
imu = InertialUnit("inertial unit")
imu.enable(TIME_STEP)
compass = Compass("compass")
compass.enable(TIME_STEP)
gyro = Gyro("gyro")
gyro.enable(TIME_STEP)

yaw_setpoint=-1

pitchPID = PID(float(params["pitch_Kp"]), float(params["pitch_Ki"]), float(params["pitch_Kd"]), setpoint=0.0)
rollPID = PID(float(params["roll_Kp"]), float(params["roll_Ki"]), float(params["roll_Kd"]), setpoint=0.0)
throttlePID = PID(float(params["throttle_Kp"]), float(params["throttle_Ki"]), float(params["throttle_Kd"]), setpoint=1)
yawPID = PID(float(params["yaw_Kp"]), float(params["yaw_Ki"]), float(params["yaw_Kd"]), setpoint=float(yaw_setpoint))

targetX, targetY, target_altitude = 1.0, 0.0, 10.0


#######
# Functions for the wavefront controller
#######

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

############

#######
# Wavefront
#######

# Parameters
start_cell = (4,0)                                                          
goal_cell = (1,3)                                                           
cell_width, cell_height = 1, 1                                              
aoi_height, aoi_width = 10, 10                                              
obstacles = [grid_decomposition.Obstacle(location=(1.0, 1.0), radius=0.5),  
             grid_decomposition.Obstacle(location=(7.2, 6.8), radius=0.8)] 

# step 2: get the decomposed grid                                           
# init map constructor obj                                                  
graph_map = grid_decomposition.Map_Constructor(img_width=aoi_width,         
                                               img_height=aoi_height,       
                                               cell_width=cell_width,       
                                               cell_height=cell_height)
# call member function to get coordinate matrix                             
cell_center_matrix = graph_map.map_graph_construction()                     
# call member function to get availability matrix                           
availability_matrix = graph_map.map_obstacle_label(obstacles)

# Step 3: Wavefront Algorithm                                               
potential_func = wavefront.potential_function(availability_matrix, goal_cell)

# Step 4: Gradient Ascent                                                   
coverage_path = gradientascent.grad_ascent(potential_func, start_cell, goal_cell)

# Step 5: Find points for the drone starting from 0,0                       
drone_path = find_intermediate_points(coverage_path)

############

#######
# This is where the output of the drone_path function could be used
# (in the wavefront_master.py)
#######
 
cell_numb = 0
targetX = drone_path[cell_numb][0]
targetY = drone_path[cell_numb][1]
time_of_last_change = robot.getTime()
prob_bound = 0.95
wait_time = 4
################

while (robot.step(timestep) != -1):

    led_state = int(robot.getTime()) % 2
    front_left_led.set(led_state)
    front_right_led.set(int(not(led_state)))

    roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = compass.getValues()[0]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    
    xGPS = gps.getValues()[2]
    yGPS = gps.getValues()[0]
    zGPS = gps.getValues()[1]
        
    #####
    # This is the controller code.
    #####
    current_time = robot.getTime()
    if ((xGPS >= drone_path[cell_numb][0]*prob_bound) or \
       (xGPS <= drone_path[cell_numb][0]*(2-prob_bound))) and \
       ((yGPS >= drone_path[cell_numb][1]*prob_bound) or \
       (yGPS <= drone_path[cell_numb][1]*(2-prob_bound))):  
        if (current_time - time_of_last_change) > wait_time:
            print(f"before: {targetX, targetY}")
            cell_numb += 1
            targetX = drone_path[cell_numb][0]
            targetY = drone_path[cell_numb][1]
            time_of_last_change = robot.getTime()
            print(f"after: {targetX, targetY}")
    else:
        None

    vertical_input = throttlePID(zGPS)
    yaw_input = yawPID(yaw)

    rollPID.setpoint = targetX
    pitchPID.setpoint = targetY
    
    roll_input = float(params["k_roll_p"]) * roll + roll_acceleration + rollPID(xGPS)
    pitch_input = float(params["k_pitch_p"]) * pitch - pitch_acceleration + pitchPID(-yGPS)

    front_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input + pitch_input + yaw_input

    mavic2proHelper.motorsSpeed(robot, front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)
