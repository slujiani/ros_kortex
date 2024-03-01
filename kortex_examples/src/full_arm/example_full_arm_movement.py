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
        angle=ex.get_turn_angle()  #目前不用
        place_position=[0.5370, -0.1388, 0.1] #放置插头，螺母，杯子的位置
        tar_pos=[0.5860, 0.2068  , 0.20631255] #螺杆的位置
        hole_pos = [0.697,0.139,0.14]  #插板的位置
        #*******************************************************************************
        # Set the reference frame to "Mixed" 混合坐标系
        success &= ex.example_set_cartesian_reference_frame()
        #*******************************************************************************
        success=False
        print(task)
        if task=="hello":
            success=open_actions(robot=ex)
        elif task == "pick_up":
            success=pick_place(robot=ex,pick_position=obj_pos,place_position=place_position)
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
        #*******************************************************************************
        # open_actions(robot=ex)
        # peg_pos = [0.433,-0.148,0.029]  #[0.3123877  0.01808212 0.02797782 0.00698692] [0.31274772 0.01486048 0.02207625 0.00698692]   [0.3137,0.0207,0.03915]
        # hole_pos = [0.697,0.139,0.14]  #插进去的位置 [0.69520503 0.1399763  0.12625686 0.33187775] [ 0.60428723 -0.24222806  0.17437703  0.79301313] [ 0.48737724 -0.16999294  0.12956789  0.33537119][ 0.49367757 -0.14016773  0.12954721  0.3213974 ] [ 0.53476997 -0.14936443  0.12828383  0.33537119]  插头在插板前面的位置[ 0.49814716 -0.14806058  0.12354184  0.33187775]
        # # angles=[0.0,pi/4,0.0]
        # angles=[-180,0,-180]
        # angle=0
        # # for i in range(20):
        # #   rospy.loginfo('第'+str(i)+'次')
        # # peg_in(robot=ex,peg_position=peg_pos,hole_position=hole_pos)
        # # ex.mov_rot_action(position=peg_pos,theta=angles)
        # # peg_in_restraint(robot=ex,peg_position=peg_pos,steer_angle=angle,hole_position=hole_pos)
        # # peg_out(robot=ex,peg_position=peg_pos,hole_position=hole_pos)

        # # #grasp
        # pick_pos=[0.6570,-0.1388,0.1]#[0.5766, 0.0013208, 0.43364822]
        # place_pos=[0.5370, -0.1388, 0.1]
        # # pick_place(robot=ex,pick_position=pick_pos,place_position=place_pos)

        # # #screw
        # nut_pos=[0.3503, 0.04425, 0.02306067 ]  #[ 0.40719028 -0.05425203  0.02323915  0.51228071] [0.5000311  0.08270526 0.15489137 0.59298249] [0.31421369, 0.00457687, 0.02306067] 
        # tar_pos=[0.5860, 0.2068  , 0.20631255]  #[0.55387338 0.1342958  0.21441096 0.58689961] [0.55261421 0.12603921 0.21349069 0.59298249] [0.55699549 0.129932   0.2064488  0.59298249] 拧螺丝的杆的脑袋的位置
        # # screw(robot=ex,nut_position=nut_pos,target_position=tar_pos)
        # # screw_out(robot=ex,nut_position=nut_pos,target_position=tar_pos)

        # #获取当前坐标xyz theta_x&y&z
        # # pos=ex.get_pose()
        # # print('-------------------------++++++++++++++---------')
        # # print(pos)
        # #*******************************************************************************

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    main()