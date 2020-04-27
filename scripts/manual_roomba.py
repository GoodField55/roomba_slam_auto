#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Roomba():
  def __init__(self):
    self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

  def run(self):
    rate = rospy.Rate(10)	#(20)
    data = Twist()

    data.linear.x = 0.0		#0.1
    data.angular.z = 0.0
    self.cmd_vel.publish(data)   # motor stop

    while not rospy.is_shutdown():

      # wait key control

      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('roomba')
  Roomba().run()

