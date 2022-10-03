from tkinter import N
from KumpooterBision import controller_demo_cv
import numpy as np
import time
import math
import requests

goal_x = None
goal_y = None
goal_theta = None
robot_x = None
robot_y = None
robot_theta = None
target_x = None
target_y = None
target_theta = None
last_position_time = time.time()
wheel_base = 100 # mm # wheelbase is actually 100mm
half_wheel_base = 50 # mm # wheelbase is actually 100mm
wheel_radius = 30 # mm
feed_forward_velocity = 50 # mm/as
feed_forward_omega = 0 # radians

Px = -0.3
Py = 0.002
Pth = 0.01
I = 0
D = 0


def command_robot():
    # generate feedforward position
    # Leave alone

    error_v, error_w = get_error_robot_perspective()
    # print("error_v {}, error_w {}".format(error_v, error_w))

    error_comp_r = error_v + half_wheel_base * error_w
    error_comp_l = error_v - half_wheel_base * error_w

    v_r = feed_forward_velocity + half_wheel_base * feed_forward_omega + error_comp_r
    v_l = feed_forward_velocity - half_wheel_base * feed_forward_omega + error_comp_l
    # print("v_r {}, v_l {}".format(v_r, v_l))

    command_vel(v_l, v_r)


def get_error_robot_perspective():
    global target_x, target_y, target_theta, Px, Py, Pth, I, D, robot_x, robot_y, robot_theta

    error_robot_world_cords = np.array([robot_x - target_x, robot_y - target_y])
    rotation_matrix = np.array([[math.cos(robot_theta), -math.sin(robot_theta)], [math.sin(robot_theta), math.cos(robot_theta)]])
    inv_rotation_matrix = (rotation_matrix).T
    error_robot_robot_cords = inv_rotation_matrix @ error_robot_world_cords

    theta_error = target_theta - robot_theta
    theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

    error_v = error_robot_robot_cords[0] * Px
    error_omega = (error_robot_robot_cords[1] * Py) + theta_error
    # print("error_v {}, error_omega {}".format(error_v, error_omega))

    return error_v, error_omega




# where feedforward control thinks the robot is / should be 
def generate_feedforward_position():
    global target_x, target_y, target_theta, last_position_time
    feed_forward_velocity_l = feed_forward_velocity - (half_wheel_base * feed_forward_omega)
    feed_forward_velocity_r = feed_forward_velocity + (half_wheel_base * feed_forward_omega)

    delta_l = feed_forward_velocity_l * (time.time() - last_position_time)
    delta_r = feed_forward_velocity_r * (time.time() - last_position_time)
    
    delta_forward = (delta_l + delta_r) / 2
    delta_theta = (delta_r - delta_l) / wheel_base

    # use midpoint formula here
    target_theta = target_theta + (delta_theta/2)
    target_x = target_x + (delta_forward * math.cos(target_theta))
    target_y = target_y + (delta_forward * math.sin(target_theta))
    target_theta = target_theta + (delta_theta/2)

    return


# left, right are in mm/secs
def command_vel(v_left_lin, v_right_lin):
    v_left_ang = v_left_lin / wheel_radius
    v_right_ang = v_right_lin / wheel_radius

    # convert angular velocity to pwm
    scale_factor = 5
    left_pwm = v_left_ang * scale_factor + 90
    right_pwm = 90 - v_right_ang * scale_factor

    # clamp pwm between 0 and 180
    left_pwm = min(max(left_pwm, 0), 180)
    right_pwm = min(max(right_pwm, 0), 180)

    robot_url = "http://172.26.171.154"
    json = {"dtype": "speed", 
                "servo1": int(left_pwm),
                "servo2": int(right_pwm)
            }

    print(json)
    x = requests.post(robot_url, data=json)    

    
if __name__ == "__main__":
    # global goal_x, goal_y, goal_theta, robot_x, robot_y, robot_theta, target_x, target_y, target_theta, last_position_time
    while True:
        while (robot_x == None or robot_y == None or robot_theta == None):
            robot_x, robot_y, robot_theta = controller_demo_cv()
        
        # init a target position
        if (goal_x is None):
            goal_x, goal_y, goal_theta = robot_x, robot_y, robot_theta
            goal_y = goal_y + 200
            goal_x = goal_x + 200

        # init robot start point as actual position
        if (target_x is None):
            target_x, target_y, target_theta = robot_x, robot_y, robot_theta

        generate_feedforward_position()
        # print("feedforward position: ", target_x, target_y, target_theta)
        command_robot()
        # print("robot_x: ", robot_x, "robot_y: ", robot_y, "robot_theta: ", robot_theta)
        robot_x, robot_y, robot_theta = None, None, None
        last_position_time = time.time()