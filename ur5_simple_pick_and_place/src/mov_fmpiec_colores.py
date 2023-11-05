#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import math as m
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import DisplayTrajectory
import cv2 as cv
from pynput import keyboard
from read_camera_image_mult import image_read
import time as t

global current_pose, target_pose_correct_orien
global comp_orien
comp_orien = 1

def pos_take_images():
    global target_pose_correct_orien
    print("MOVING TO CAMERA POSITION...")
    images_goals = [1.3909180384024973, -1.2970095837866964, 1.4559604820238743, -1.75, -1.573950195686849, -0.1819244873697139]
    move_group_interface_arm.go(images_goals, wait=True)
    move_group_interface_arm.stop()
    target_pose_correct_orien = geometry_msgs.msg.Pose()
    target_pose_correct_orien = move_group_interface_arm.get_current_pose()

def home_pos():
    print("MOVING TO HOME POSITION...")
    joint_goals = [0.0, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
    move_group_interface_arm.go(joint_goals, wait=True)
    move_group_interface_arm.stop()

def home_pos_inv():
    print("MOVING TO HOME POSITION...")
    joint_goals = [-3.14, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
    move_group_interface_arm.go(joint_goals, wait=True)
    move_group_interface_arm.stop()

def close_gripper():
    print("CLOSING THE GRIPPER...")
    move_group_interface_gripper.go(gripper_close, wait=True)
    move_group_interface_gripper.stop()

def open_gripper():
    print("OPENING THE GRIPPER...")
    move_group_interface_gripper.go(gripper_open, wait=True)
    move_group_interface_gripper.stop()

# Approximate the gripper over the piece
def approximation(x, y):
    global target_pose, comp_orien, target_pose_correct_orien
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = 1.1
    if comp_orien > 2:
        target_pose.orientation = target_pose_correct_orien.pose.orientation
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()
    comp_orien = comp_orien + 1

# Take the piece with the gripper after approximation
def take_piece():
    global target_pose, comp_orien
    target_pose.position.z = 0.96
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

# Transform from image coordenates to world coordenates
def convert_m2w(p_px):
    Cx = 160
    Cy = 120.5
    Zc = 0.534395
    fx = 199.8938206925
    fy = 199.8938206925

    x_c = ((p_px[0]-Cx)/fx)*Zc
    y_c = ((p_px[1]-Cy)/fy)*Zc
    x_w = x_c + 0.004627
    y_w = 0.758958 - y_c

    return x_w, y_w

def red_pieces(c):
    global current_pose, target_pose
    global incr_r
    coord_red = []
    coord_red.append(c[0])
    coord_red.append(c[1])
    x_r, y_r = convert_m2w(coord_red)

    # Approx red piece
    approximation(x_r, y_r)

    # Open the gripper
    open_gripper()

    # Take red piece
    take_piece()

    # Close the gripper
    close_gripper()

    # Return to approximation pos
    approximation(x_r, y_r)

    # Move to home position
    home_pos()

    # Move the TCP over the red position
    approx_goals = [-0.9023952823594588, -0.7322973147618699, 1.2709732199546568, -2.115323358711743, -1.5688081113077317, -0.9020939621886495]
    move_group_interface_arm.go(approx_goals, wait=True)
    move_group_interface_arm.stop()

    # Leave the piece
    current_pose_2 = move_group_interface_arm.get_current_pose()
    target_pose.position = current_pose_2.pose.position
    target_pose.orientation = current_pose_2.pose.orientation
    target_pose.position.x = 0.549634458556 - incr_r
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    target_pose.position.z = 0.967
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    open_gripper()

    # Return over the red position
    target_pose.position.z = 1.1
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    pos_take_images()

def blue_pieces(c):
    global current_pose, target_pose
    global incr_b

    coord_blue = []
    coord_blue.append(c[0])
    coord_blue.append(c[1])

    x_b, y_b = convert_m2w(coord_blue)

    # Aprox blue piece
    approximation(x_b, y_b)

    # Take blue piece
    take_piece()

    close_gripper()

    approximation(x_b, y_b)

    home_pos()

    # Move the TCP above the blue position
    approx_goals = [-1.5860453154773948, -0.9473863999013306, 1.6479834864306415, -2.2721118993163927, -1.5642330045305393, -1.5840395145339077]
    move_group_interface_arm.go(approx_goals, wait=True)
    move_group_interface_arm.stop()

    # Leave blue piece
    current_pose_2 = move_group_interface_arm.get_current_pose()
    target_pose.position = current_pose_2.pose.position
    target_pose.orientation = current_pose_2.pose.orientation
    target_pose.position.x = 0.0998218605971 - incr_b
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    target_pose.position.z = 0.967
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    open_gripper()

    target_pose.position.z = 1.1
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    pos_take_images()

def green_pieces(c):
    global current_pose, target_pose
    global incr_g

    coord_green = []
    coord_green.append(c[0])
    coord_green.append(c[1])

    x_g, y_g = convert_m2w(coord_green)

    # Aprox green piece
    approximation(x_g, y_g)

    # Take blue piece
    take_piece()

    close_gripper()

    approximation(x_g, y_g)

    home_pos_inv()

    # Move the TCP above the green position
    approx_goals = [-2.3516030599181112, -1.0095631474481657, 1.7579949487353765, -2.31427905629861, -1.56649968952454, -2.3444216722389184]
    move_group_interface_arm.go(approx_goals, wait=True)
    move_group_interface_arm.stop()

    # Leave the green piece
    current_pose_2 = move_group_interface_arm.get_current_pose()
    target_pose.position = current_pose_2.pose.position
    target_pose.orientation = current_pose_2.pose.orientation
    target_pose.position.x = -0.350061671499 - incr_g
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    target_pose.position.z = 0.967
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    open_gripper()

    target_pose.position.z = 1.1
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    pos_take_images()
    

gripper_open = [0.005]
gripper_close = [0.24]

if __name__=='__main__':
    global target_pose, incr_r, incr_b, incr_g
    obj_img = image_read()  # Objeto de la clase image_read para acceder a sus funciones
    incr_r = 0
    incr_b = 0
    incr_g = 0

    # initialize moveit_comander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # create a RobotCommander object
    robot = moveit_commander.robot.RobotCommander()

    # create a MoveGroupCommander object
    # we will move the arm or the gripper with these objects
    group_name_1 = "ur5_arm"
    group_name_2 = "gripper"
    move_group_interface_arm = moveit_commander.move_group.MoveGroupCommander(group_name_1)
    move_group_interface_gripper = moveit_commander.move_group.MoveGroupCommander(group_name_2)

    print("-----------------------------------")
    print("Starting the pick and place program")
    print("-----------------------------------")   

    # Move to home position
    home_pos()

    # Move to take images position
    pos_take_images()

    current_pose = geometry_msgs.msg.Pose()
    current_pose = move_group_interface_arm.get_current_pose()

    t.sleep(2)
    coord = []

    n_b, n_g, n_r, coord = obj_img.rev_coord()  # Recibe las coordenadas de las piezas en la imagen
    print("************************")
    print("El numero de piezas de cada color es, rojo:{}, azul:{}, verde:{}".format(n_r, n_b, n_g))
    print("La cordenadas de los objetos son: {}".format(coord))
    print("************************")

    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation = current_pose.pose.orientation
    
    nt = n_r+n_b+n_g
    while(nt>0):
        if(n_r>0):
            red_pieces(coord[2])
            incr_r = incr_r + 0.1
        elif(n_b>0):
            blue_pieces(coord[0])
            incr_b = incr_b + 0.1
        elif(n_g>0):
            green_pieces(coord[1])
            incr_g = incr_g + 0.1
        t.sleep(2)
        n_b, n_g, n_r, coord = obj_img.rev_coord()
        print("************************")
        print("El numero de piezas de cada color es, rojo:{}, azul:{}, verde:{}".format(n_r, n_b, n_g))
        print("La cordenadas de los objetos son: {}".format(coord))
        print("************************")
        nt = n_r+n_b+n_g

    cv.destroyAllWindows()
