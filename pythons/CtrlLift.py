#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

class CtrlLift():
  def __init__(self,floor):
    if(floor == 0):
      topicname = '/switch0/vel_cmd'
    elif(floor == 1):
      topicname = '/switch1/vel_cmd'
    else:
      print("Error Floor")
      
    self.pub = rospy.Publisher(topicname, Float32, queue_size=2)
    self.CtrlLift()

  def CtrlLift(self):
    # r = rospy.Rate(5)
    # r.sleep()
    vectory = -2.0
    self.pub.publish(vectory)
    rospy.sleep(0.3)
    self.pub.publish(vectory)
    rospy.sleep(0.3)
    self.pub.publish(vectory)
    rospy.sleep(0.3)
    self.pub.publish(-vectory)
    rospy.sleep(0.3)
    self.pub.publish(-vectory)
    rospy.sleep(0.3)
    self.pub.publish(0)
    rospy.sleep(0.3)
    self.pub.publish(0)
    rospy.sleep(0.3)
    
    
    

# if __name__ == '__main__':
#   rospy.init_node('ctrlelevator', anonymous=False)
#   CtrlLift(0)