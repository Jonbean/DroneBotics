import sys
# Need to make sure path is correct for it to run
sys.path.append('/Users/rickgentry/Fall_2020/advanced_robotics/final_project/DroneBotics/modules')

from controller import *
from Trajectory import Trajectory
from Quadrotor import Quadrotor
import mavic2proHelper
from simple_pid import PID
import csv
import struct



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


# Trajectory generation using cubic spline
# waypoints = get_waypoints()
waypoints = [[0,0,0], [2,2,5], [5,10,5], [4,3,5], [4,4,0]]
T = 50
traj = Trajectory(waypoints,T)
traj.cubic_spline()

timestep_secs = timestep/1000

# For plotting trajectory
drone = Quadrotor(show_animation=False)

# Initializing vars used in control loop
i = 0
prob_bound = 0.1
plot_interval = 0.5
plot_time= -plot_interval

while (robot.step(timestep) != -1):
    current_time = robot.getTime()
    next_time = current_time + timestep_secs
    dt_plot = current_time - plot_time

    #print ("Time: %s, %s" % (current_time, next_time))
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

    if (dt_plot > plot_interval):
        plot_time = current_time
        drone.update_pose(xGPS,yGPS,zGPS,roll,pitch,yaw)


    if  i < len(waypoints) and \
        ((xGPS >= waypoints[i][0] - prob_bound) and \
        (xGPS <= waypoints[i][0] + prob_bound)) and \
        ((yGPS >= waypoints[i][1] - prob_bound) and \
        (yGPS <= waypoints[i][1] + prob_bound)) and \
        ((zGPS >= waypoints[i][2] - prob_bound) and \
        (zGPS <= waypoints[i][2] + prob_bound)):

        print(f"at: waypoint {i} at position {xGPS},{yGPS},{zGPS}")
        i += 1


    # Stop drone if we reach last waypoint
    elif i == len(waypoints):
        mavic2proHelper.motorsSpeed(robot, 0, 0, 0, 0)
        drone.plot_after(plot_pause=0.05)
        break

    # If we haven't reached the end of the trajectory, then set next position based on the target
    if (next_time < T):
        # Get next targets from trajectory
        pos = traj.get_pos(next_time)
        targetX, targetY, target_altitude = pos[0], -pos[1], pos[2]


    # Set desired next position
    rollPID.setpoint = targetX
    pitchPID.setpoint = targetY
    throttlePID.setpoint = target_altitude

    # Calculate inputs for motors
    vertical_input = throttlePID(zGPS)
    yaw_input = yawPID(yaw)
    roll_input = float(params["k_roll_p"]) * roll + roll_acceleration + rollPID(xGPS)
    pitch_input = float(params["k_pitch_p"]) * pitch - pitch_acceleration + pitchPID(-yGPS)

    front_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input + pitch_input + yaw_input

    mavic2proHelper.motorsSpeed(robot, front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)
