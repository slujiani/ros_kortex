import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from math import pi
from control_msgs.msg import *
from trajectory_msgs.msg import *
import actionlib
from std_srvs.srv import Empty
from tf import TransformListener
from robot import Robot
import time
import copy

gripper_vertical_theta=[-180,0,90]
gripper_horizontal_theta=[90,0,90]
peg_foot_len=0.075 #可以更改插入深度
screw_Quan=2  #拧螺母 转几个1/4圈
# 判断坐标是否在运动范围内
def pos_is_ok(pos=[0, 0, 0]):
    ret = True
    if pos[0] < 0 or pos[0] > 0.9:
        ret = False
    if ret == True and (pos[1] < -0.3 or pos[1] > 0.3):
        ret = False
    if ret == True and (pos[2] < 0 or pos[2] > 1):
        ret = False
    return ret

# 机械臂开机动画
def open_actions(robot:Robot):
    rospy.loginfo('the start of following tasks.')
    success = True
    success &= robot.example_home_the_robot()
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_send_gripper_command(0.8)
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_send_gripper_command(0.8)
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_retract_the_robot()
    rospy.loginfo('the open actions all done.')
    return success

# 抓不锈钢杯子
def pick_place(robot:Robot, pick_position=[0, 0, 0], place_position=[0, 0, 0],obj_vertical=True,obj_theta=0,success=True):
    place_position=[0.27, 0.233, 0.1]
    place_theta=[90,0,180]
    if pos_is_ok(pick_position) == False:
        rospy.loginfo('pick place is out of range.')
        return False
    if pos_is_ok(place_position) == False:
        rospy.loginfo('place place is out of range.')
        return False
    rospy.loginfo('Execute pick and place task...')
    rospy.loginfo('Go back to initial pose')
    #复位
    success &= robot.example_send_gripper_command(0)
    
    if obj_vertical==True:
        #夹爪状态横着的 杯子是竖着放的
        success &= robot.example_home_the_robot()
        gripper_theta=gripper_horizontal_theta
        pick_position[2]=0.1
        
    else:
        #夹爪状态竖着的 
        success &=robot.example_retract_the_robot()
        gripper_theta=gripper_vertical_theta
        gripper_theta[2]=obj_theta
        pick_position[2]=0.019
    
        
    #到杯子位置
    if obj_vertical== True:
        pick_position[0]-=0.05
    
    pick_position[2]+=0.1
    success &= robot.mov_rot_action(position=pick_position,theta=gripper_theta)

    if obj_vertical== True:
        pick_position[0]+=0.05

    pick_position[2]-=0.1
    success &= robot.mov_rot_action(position=pick_position,theta=gripper_theta)
    rospy.loginfo('Arrived object pose, prepare for grabing...')
    #闭合夹爪
    success &= robot.example_send_gripper_command(0.465)
    #机械臂抬高0.1
    pick_position[2]+=0.1
    success &= robot.mov_rot_action(position=pick_position,theta=gripper_theta)
    success &= robot.example_home_the_robot()
    #到达放置点  
    place_position[2]+=0.1 
    success &= robot.mov_rot_action(position=place_position,theta=place_theta)
    place_position[2]-=0.1
    success &= robot.mov_rot_action(position=place_position,theta=place_theta)
    #松开夹爪
    success &= robot.example_send_gripper_command(0)
    place_position[2]=0.30
    success &= robot.mov_rot_action(position=place_position,theta=place_theta)
    rospy.loginfo('Task finished, back to home')
    #复位
    success &= robot.example_retract_the_robot()
    return success

# 把螺母拧上
def screw(robot:Robot, nut_position=[0, 0, 0], target_position=[0, 0, 0], success=True):
    nut_position[2]=0.028
    if pos_is_ok(nut_position) == False:
            rospy.loginfo('nut position is out of range.')
            return False
    if pos_is_ok(target_position) == False:
            rospy.loginfo('target position is out of range.')
            return False

    rospy.loginfo('Execute screw task...')
    rospy.loginfo('Go back to initial pose')
    # 复位
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_retract_the_robot()
    #到达螺母位置
    print('==================')
    print(nut_position)
    success &= robot.mov_rot_action(position=nut_position,theta=gripper_vertical_theta)
    rospy.loginfo('Arrive object pose, prepare for grabing nut...')
    # 闭合夹爪
    success &= robot.example_send_gripper_command(0.9)
    rospy.loginfo('Go to target pose')
    # 抬高机械臂
    nut_position[2]+=0.1
    success &= robot.mov_rot_action(position=nut_position,theta=gripper_vertical_theta)
    target_position[2]+=0.1
    success &= robot.mov_rot_action(position=target_position,theta=gripper_vertical_theta)
    target_position[2]-=0.1
    success &= robot.mov_rot_action(position=target_position,theta=gripper_vertical_theta)
    # 拧
    screw_theta=copy.deepcopy(gripper_vertical_theta)
    rospy.loginfo('Screwing')
    for i in range(screw_Quan):
        screw_theta[2]-=90
        # if i ==4:
        #     target_position[2]-=0.005
        if screw_theta[2]==-360:
            screw_theta[2]==0
        success &= robot.mov_rot_action(position=target_position,theta=screw_theta)
        
    success &= robot.example_send_gripper_command(0)
    #抬高机械臂
    target_position[2]+=0.1
    success &= robot.mov_rot_action(position=target_position,theta=gripper_vertical_theta)
    rospy.loginfo('Task finished, back to home')
    # 复位
    
    success &= robot.example_retract_the_robot()
    return success

# 把螺母拧出来
def screw_out(robot:Robot, nut_position=[0, 0, 0], target_position=[0, 0, 0], success=True):
    nut_position[2]=0.028
    if pos_is_ok(nut_position) == False:
        rospy.loginfo('nut position is out of range.')
        return False
    if pos_is_ok(target_position) == False:
        rospy.loginfo('target position is out of range.')
        return False
    
    rospy.loginfo('Execute screw task...')
    rospy.loginfo('Go back to initial pose')
    # 复位
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_retract_the_robot()
    success &= robot.mov_rot_action(position=[0.3,0.0,0.3],theta=gripper_vertical_theta)
    #到达螺杆上方
    target_position[2]+=0.1
    success &= robot.mov_rot_action(position=target_position,theta=gripper_vertical_theta)
    #到达螺杆脑袋位置
    target_position[2]-=0.1
    success &= robot.mov_rot_action(position=target_position,theta=gripper_vertical_theta)

    rospy.loginfo('Arrive object pose, prepare for grabing nut...')
    #抓螺母
    success &= robot.example_send_gripper_command(0.8)
    rospy.loginfo('Screwing out')
    screw_theta=gripper_vertical_theta
    for _ in range(screw_Quan):
        screw_theta[2]+=90
        if screw_theta[2]==360:
            screw_theta[2]==0
        success &= robot.mov_rot_action(position=target_position,theta=screw_theta)
    #抬高机械臂
    target_position[2]+=0.1
    success &= robot.mov_rot_action(position=target_position,theta=screw_theta)
    success &= robot.mov_rot_action(position=[0.3,0.0,0.3],theta=gripper_vertical_theta)
    #到达螺母指定地点
    success &= robot.mov_rot_action(position=nut_position,theta=screw_theta)
    #复位
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_retract_the_robot()
    return success

  # 插头插进插板，停留，然后拔出复位
def peg_in(robot:Robot, peg_position, hole_position, peg_theta=[-180,0,90],success=True):
    #插头高度固定 
    peg_position[2]=0.051
    #插板高度固定 
    hole_position[2]=0.159
    if pos_is_ok(peg_position) == False:
        rospy.loginfo('peg position is out of range.')
        return False
    if pos_is_ok(hole_position) == False:
        rospy.loginfo('hole position is out of range.')
        return False
    rospy.loginfo('Execute peg in hole task...')
    rospy.loginfo('Go back to initial pose')
    #复位
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_retract_the_robot()

    # 移动到插头位置 带调整夹爪角度，默认是侧面垂直桌面（摄像头画面）
    peg_position[2]+=0.1
    success &= robot.mov_rot_action(position=peg_position,theta=peg_theta)
    rospy.loginfo('Arriving at peg position, perparing for grabing peg...')
    peg_position[2]-=0.1
    success &= robot.mov_rot_action(position=peg_position,theta=peg_theta)
    #抓
    success &= robot.example_send_gripper_command(0.465)  # 465
    #移动到插板位置
    print('moving to hole position')
    hole_theta=peg_theta  #插板放在摄像头正对面 需要机械臂j7-z逆时针旋转90度
    hole_theta[2]=-180
    success &= robot.mov_rot_action(position=hole_position,theta=hole_theta)
    #插入
    hole_position[1]+=peg_foot_len
    success &= robot.mov_rot_action(position=hole_position,theta=hole_theta)
    #复位
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_retract_the_robot()
    return success

def peg_out(robot:Robot, peg_position, hole_position, success=True):
    peg_position=[0.187,-0.197,0.051]
    if pos_is_ok(peg_position) == False:
        rospy.loginfo('peg position is out of range.')
        return False
    if pos_is_ok(hole_position) == False:
        rospy.loginfo('hole position is out of range.')
        return False

    rospy.loginfo('Execute peg out of hole task...')
    rospy.loginfo('Go back to initial pose')
    #复位
    success &= robot.example_send_gripper_command(0)
    success &= robot.example_retract_the_robot()
    #到达插板位置
    hole_theta=[-180,0,-180]
    success &= robot.mov_rot_action(position=hole_position,theta=hole_theta)
    hole_position[1]=hole_position[1]+peg_foot_len
    success &= robot.mov_rot_action(position=hole_position,theta=hole_theta)
    #抓插头
    success &= robot.example_send_gripper_command(0.465)  # 465
    rospy.loginfo('Start to peg out...')
    #后退
    hole_position[1]-=(0.1+peg_foot_len)
    success &= robot.mov_rot_action(position=hole_position,theta=hole_theta)
    #移动到插头的位置
    peg_theta=[-180,0,90]
    success &= robot.mov_rot_action(position=peg_position,theta=peg_theta)
    rospy.loginfo('Arriving at peg_position, ready to put peg down')
    #放下插头 复位
    success &= robot.example_send_gripper_command(0)
    print('task finished')
    success &= robot.example_retract_the_robot()
    return success

