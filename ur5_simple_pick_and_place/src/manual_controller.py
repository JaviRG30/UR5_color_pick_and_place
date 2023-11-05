#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import math as m
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import DisplayTrajectory
import cv2 as cv
from pynput import keyboard
from read_camera_image_mult import image_read

def direct_kin():
    current_joint_values = move_group_interface_arm.get_current_joint_values()
    print('Current joint values:')
    print(current_joint_values)
    print('\nEnter new values:\n')
    try:
        joint_goals = [float(input("Enter joint " + str(i) + " value: ")) for i in range(6)]
    except:
        print("Error: bad joint_goals")
        exit()
    move_group_interface_arm.go(joint_goals, wait = True)
    print("New goals for the robot: " + str(joint_goals))
    move_group_interface_arm.stop

def on_press(key):
    global new_values
    try:
        current_joint_values = move_group_interface_arm.get_current_joint_values()
        print("\nMANUAL MODE")
        print("*********************************")
        print("CONTROLS:")
        print("Positive rotation of joints: 1, 2, 3, 4, 5, 6")
        print("Negative rotation of joints: q, w, e, r, t, y")
        print("Open the gripper: press 'o'")
        print("Close the gripper: press 'c'")
        print("Position AllZeros: press 'a'")
        print("Position Home: press 's'")
        print("See the current joint values: press 'd'")
        print("Close the manual mode: press 'f'")
        print("*********************************")
        # print(current_joint_values)
        print("Tecla pulsada: {}".format(key.char))
        if key.char == "1": 
            new_values[0] += 0.1121997376
            new_values[1:] = current_joint_values[1:]
        elif key.char == "2":
            new_values[0] = current_joint_values[0]
            new_values[1] += 0.1121997376
            new_values[2:] = current_joint_values[2:]
        elif key.char == "3":
            new_values[:1] = current_joint_values[:1]
            new_values[2] += 0.1121997376
            new_values[3:] = current_joint_values[3:]
        elif key.char == "4":
            new_values[:2] = current_joint_values[:2]
            new_values[3] += 0.1121997376
            new_values[4:] = current_joint_values[4:]
        elif key.char == "5":
            new_values[:3] = current_joint_values[:3]
            new_values[4] += 0.1121997376
            new_values[5] = current_joint_values[5]
        elif key.char == "6":
            new_values[:4] = current_joint_values[:4]
            new_values[5] += 0.1121997376
        elif key.char == "q":
            new_values[0] -= 0.1121997376
            new_values[1:] = current_joint_values[1:]
        elif key.char == "w":
            new_values[0] = current_joint_values[0]
            new_values[1] -= 0.1121997376
            new_values[2:] = current_joint_values[2:]
        elif key.char == "e":
            new_values[:1] = current_joint_values[:1]
            new_values[2] -= 0.1121997376
            new_values[3:] = current_joint_values[3:]
        elif key.char == "r":
            new_values[:2] = current_joint_values[:2]
            new_values[3] -= 0.1121997376
            new_values[4:] = current_joint_values[4:]
        elif key.char == "t":
            new_values[:3] = current_joint_values[:3]
            new_values[4] -= 0.1121997376
            new_values[5] = current_joint_values[5]
        elif key.char == "y":
            new_values[:4] = current_joint_values[:4]
            new_values[5] -= 0.1121997376
        elif key.char == "a":
            new_values = [0, 0, 0, 0, 0, 0]
        elif key.char == "s":
            new_values = [0, -1.4981833546922498, 1.5076458045811982, -0.0019684415915168785, 0, 0]
        elif key.char == "d":
            print('Current joint values:')
            print(current_joint_values)
        elif key.char == "o":
            move_group_interface_gripper.go(gripper_open, wait=True)
            move_group_interface_gripper.stop()
        elif key.char == "c":
            move_group_interface_gripper.go(gripper_close, wait=True)
            move_group_interface_gripper.stop()
        elif key.char == "f":
            sys.exit()
        move_group_interface_arm.go(new_values, wait = True)
        move_group_interface_arm.stop
    except AttributeError:
        print("You shouldn't press special characters")

def inverse_kin():
    current_pose = move_group_interface_arm.get_current_pose()
    print("Current values of end effector pose:")
    print(current_pose)
    print('\nEnter new end effector values:\n')

    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = float(input('Enter position x: '))
    pose_target.position.y = float(input('Enter position y: '))
    pose_target.position.z = float(input('Enter position z: '))
    z = int(input("Do you want to modify the orientation using a quaternion?(Yes(1) or No(2)): "))
    if z == 1:    
        pose_target.orientation.w = float(input('Enter value w: '))
        pose_target.orientation.x = float(input('Enter value x: '))
        pose_target.orientation.y = float(input('Enter value y: '))
        pose_target.orientation.z = float(input('Enter value z: '))
    elif z == 2:
        pose_target.orientation = current_pose.pose.orientation
                
    move_group_interface_arm.set_pose_target(pose_target)

    move_group_interface_arm.go(wait=True)
            
    move_group_interface_arm.stop()
    move_group_interface_arm.clear_pose_targets()

gripper_open = [0.005]
gripper_close = [0.24]

if __name__ == '__main__':
    obj_img = image_read()

    new_values = []
    # initialize moveit_comander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # instantiate a RobotCommander object
    robot = moveit_commander.robot.RobotCommander()

    # create a MoveGroupCommander object
    # we will move the arm or the gripper with these objects
    group_name_1 = "ur5_arm"
    group_name_2 = "gripper"
    move_group_interface_arm = moveit_commander.move_group.MoveGroupCommander(group_name_1)
    move_group_interface_gripper = moveit_commander.move_group.MoveGroupCommander(group_name_2)

    # Create DisplayTrajectory ROS publisher which is used to 
    # display trajectoris in Rviz
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)

    r = 0

    # print("End effector: " + str(move_group_interface_arm.get_end_effector_link()))
    
    while True:
        print('--------------------------------------------------')
        print('OPERATING MODES:')
        print('1: Direct kinematic')
        print('2: Direct kin - Manual mode')
        print('3: Inverse kinematic - MoveJ')
        print('4: Open gripper')
        print('5: Close gripper')
        print('6: Go Home')
        print('7: Go camera position')
        print('8: See the object coordenates')
        print('9: Close program')
        print('--------------------------------------------------')
        try:
            num = int(input('\nSelect one mode: '))
        except:
            print(rospy.logerr('Bad mode selection'))
            exit()

        if num == 1:
            print("\nDIRECT KINEMATIC")
            direct_kin()
        elif num == 2:
            print("\nMANUAL MODE")
            print("*********************************")
            print("CONTROLS:")
            print("Positive rotation of joints: 1, 2, 3, 4, 5, 6")
            print("Negative rotation of joints: q, w, e, r, t, y")
            print("Open the gripper: press 'o'")
            print("Close the gripper: press 'c'")
            print("Position AllZeros: press 'a'")
            print("Position Home: press 's'")
            print("See the current joint values: press 'd'")
            print("Close the manual mode: press 'f'")
            print("*********************************")
            current_joint_values = move_group_interface_arm.get_current_joint_values()
            new_values = current_joint_values
            with keyboard.Listener(
                    on_press=on_press) as listener:
                listener.join()
            print("Manual mode closed\n")
        elif num == 3:
            print('\nINVERSE KINEMATIC')
            inverse_kin()
        elif num == 4:
            print("OPENING THE GRIPPER")
            move_group_interface_gripper.go(gripper_open, wait=True)
            move_group_interface_gripper.stop()
        elif num == 5:
            print("CLOSING THE GRIPPER")
            move_group_interface_gripper.go(gripper_close, wait=True)
            move_group_interface_gripper.stop()
        elif num == 6:
            print("MOVING TO HOME POSITION")
            joint_goals = [0.0, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
            move_group_interface_arm.go(joint_goals, wait=True)
            move_group_interface_arm.stop()
        elif num == 7:
            print("MOVING TO THE POSITION WHERE WE PROCESS PICTURES")
            joint_goals = [1.3909180384024973, -1.2970095837866964, 1.4559604820238743, -1.75, -1.573950195686849, -0.1819244873697139]
            move_group_interface_arm.go(joint_goals, wait = True)
            move_group_interface_arm.stop()
        elif num == 8:
            coord = []
            _, _, _, coord = obj_img.rev_coord()
            print("Coordenadas piezas azules: ({})\nCoordendas piezas verdes: ({})\nCoordenadas piezas rojas: ({})".format(coord[0], coord[1], coord[2]))
        elif num == 9:
            print('Finishing the program')
            exit()
        else:
            print(rospy.logerr('Bad mode selection'))
            exit()
