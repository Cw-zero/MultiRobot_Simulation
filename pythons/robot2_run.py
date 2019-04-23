# -*- coding: utf-8 -*-

import rospy
from InitPose import SetInitPose
from GoPoint import GoToPose
from Getpos import Robot
from CtrlLift import CtrlLift

A = {"name":"1st_elevator_front","pose":(-7.809, -9.057, 0.0),"orientation":(0.0, 0.0, 0.999, -0.05)}
B = {"name":"1st_elevator_in","pose":(-9.39, -9.05, 0.0),"orientation":(0.0, 0.0, 0.001, 0.999)}
C = {"name":"2nd_elevator_in","pose":(-9.04, 25.485, 0.0),"orientation":(0.0, 0.0, -0.139, 0.990)}
D = {"name":"2nd_elevator_front","pose":(-7.809, 25.3, 0.0),"orientation":(0.0, 0.0, 0.999, -0.05)}
Locations = [A,B,C,D]

Robot1_Init = {"pose":(-11.007, 6.646, 0.0),"orientation":(0.0, 0.0, 1, -0.023)}
Robot2_Init = {"pose":(-1.43, 2.13, 0.0),"orientation":(0.0, 0.0, -0.694, -0.720)}
Robot3_Init = {"pose":(-5.32, 31.4, 0.0),"orientation":(0.0, 0.0, -0.694, -0.720)}

def Init_pose():
  SetInitPose(Robot1_Init["pose"],Robot1_Init["orientation"],"robot1")
  SetInitPose(Robot2_Init["pose"],Robot2_Init["orientation"],"robot2")
  SetInitPose(Robot3_Init["pose"],Robot3_Init["orientation"],"robot3")

def UpStairs(classNav,classPose):
  Nav_state = classNav.Point_Navigation(Locations[0])
  if Nav_state:
    CtrlLift(0)
    rospy.sleep(2) #wait elevator open
    Nav_state = classNav.Point_Navigation(Locations[1])
    if Nav_state:
      tmp_trans = None
      while tmp_trans == None:
        tmp_trans,tmp_rot = classPose.get_pos()
        tmp_trans[1] = tmp_trans[1]+34.65
      SetInitPose(tmp_trans,tmp_rot,"robot2")
      rospy.sleep(2)
      CtrlLift(1)
      rospy.sleep(0.5)
      CtrlLift(1)
      rospy.sleep(2) #wait elevator open
      Nav_state = classNav.Point_Navigation(Locations[3])

def DownStairs(classNav,classPose):
  Nav_state = classNav.Point_Navigation(Locations[3])
  if Nav_state:
    CtrlLift(1)
    rospy.sleep(2) #wait elevator open
    Nav_state = classNav.Point_Navigation(Locations[2]) #进电梯 D
    if Nav_state:
      tmp_trans = None
      while tmp_trans == None:
        tmp_trans,tmp_rot = classPose.get_pos()
        tmp_trans[1] = tmp_trans[1]-34.65
      SetInitPose(tmp_trans,tmp_rot,"robot2") #重定位 C
      CtrlLift(0)
      rospy.sleep(0.5)
      CtrlLift(0)
      rospy.sleep(2)
      Nav_state = classNav.Point_Navigation(Locations[0])

if __name__ == '__main__':
  A_1st = {"name":"A","pose":(-1.43, 2.13, 0.0),"orientation":(0.0, 0.0, -0.694, -0.720),"object":2} #init pose
  B_1st = {"name":"A","pose":(-11.007, 6.646, 0.0),"orientation":(0.0, 0.0, 1, -0.023),"object":2}
  A_2nd = {"name":"F","pose":(-4.86, 26, 0.0),"orientation":(0.0, 0.0, 0.997, -0.08),"object":0}
  rospy.init_node('robot2_display', anonymous=False)
  # Init_pose()

  navigation = GoToPose("robot2")
  robotpose = Robot("robot2")
  try:
    while True:
      Nav_state = navigation.Point_Navigation(B_1st)
      rospy.sleep(5)
      UpStairs(navigation,robotpose)
      Nav_state = navigation.Point_Navigation(A_2nd)
      DownStairs(navigation,robotpose)
      Nav_state = navigation.Point_Navigation(A_1st)
  except KeyboardInterrupt:
    navigation.shutdown()




  
