#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
sys.path.append('/catkin_workspace/src/ros_kortex/kortex_examples/src/full_arm')
import rospy
import time
from robot import Robot
from kortex_driver.srv import *
from kortex_driver.msg import *
from task import open_actions
from task import peg_in
from task import peg_out
from task import pick_place
from task import screw
from task import screw_out


def main():
    ex = Robot()

# For testing purposes
    success = ex.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
    except:
        pass

    if success:
        #初始化成功
        #*******************************************************************************
        # Make sure to clear the robot's faults else it won't move if it's already in fault
        success &= ex.example_clear_faults()
        #*******************************************************************************
        
        #*******************************************************************************
        # Activate the action notifications
        success &= ex.example_subscribe_to_a_robot_notification()
        #*******************************************************************************

        #*******************************************************************************
        # 设定初始参数
        #*******************************************************************************
        task=ex.get_task_name()
        obj_pos=ex.get_input_pos()
        # obj_pos=[0.433,-0.148,0.029]
        angle=ex.get_turn_angle()  
        glass_vertical=ex.get_glass_vertical()

        place_position=[0.5370, -0.1388, 0.01] #放置插头，螺母，杯子的位置
        tar_pos=[0.4627765417098999,0.2663811445236206  , 0.214] #螺杆的位置
        hole_pos = [0.705,0.095,0.136]  #插板的位置
        #*******************************************************************************
        # Set the reference frame to "Mixed" 混合坐标系
        success &= ex.example_set_cartesian_reference_frame()
        #*******************************************************************************

        success=False
        print(task)
        if task=="hello":
            success=open_actions(robot=ex)
        elif task == "pick_up":
            success=pick_place(robot=ex,pick_position=obj_pos,place_position=place_position,obj_vertical=glass_vertical,obj_theta=angle)
        elif task == "screw":
            success=screw(robot=ex,nut_position=obj_pos,target_position=tar_pos)
        elif task == "screw_out":
            success=screw_out(robot=ex,nut_position=place_position,target_position=tar_pos)
        elif task == "peg_in":
            success=peg_in(robot=ex,peg_position=obj_pos,hole_position=hole_pos)
        elif task == "peg_out":
            success=peg_out(robot=ex,peg_position=place_position,hole_position=hole_pos)
        else :
            pass
        print('{{{'+str(success)+'}}}')
        
        # #获取当前坐标xyz theta_x&y&z
        # pos=ex.get_pose()
        # print('-------------------------++++++++++++++---------')
        # print(pos)
        # # #*******************************************************************************

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    main()
