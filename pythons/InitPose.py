import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

class SetInitPose(object):
  def __init__(self, pos, quat,robotID):
    # rospy.init_node('test_initalpose', anonymous=False)
    rospy.loginfo("start test inital pose...")
    self.initialpose_topic = robotID + "/initialpose"
    self.frame_id = robotID + "_tf/map"
    self.setpose_pub = rospy.Publisher(self.initialpose_topic,PoseWithCovarianceStamped,latch=True, queue_size=1)
    self.set_pose = pos
    self.set_quat = quat
    self.test_set_pose_flag = True
    self.test_set_pose_cnt = 3
    while self.test_set_pose_flag == True:
      self.set_inital_pose()
      self.test_set_pose_cnt -= 1
      if self.test_set_pose_cnt == 0:
        self.test_set_pose_flag = False
      rospy.sleep(1)

  def set_inital_pose(self):
    rospy.loginfo("start set pose...")
    p = PoseWithCovarianceStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = self.frame_id

    p.pose.pose.position.x = self.set_pose[0]
    p.pose.pose.position.y = self.set_pose[1]
    p.pose.pose.position.z = self.set_pose[2]

    p.pose.pose.orientation.x = self.set_quat[0]
    p.pose.pose.orientation.y = self.set_quat[1]
    p.pose.pose.orientation.z = self.set_quat[2]
    p.pose.pose.orientation.w = self.set_quat[3]

    p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
    p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
    p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0

    self.setpose_pub.publish(p)