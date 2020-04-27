#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('keyboard_cmd_vel')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

while not rospy.is_shutdown():
  vel = Twist()
  direction = raw_input('k: forward, j: backward, h: left, l: right, return: stop > ')
  if 'k' in direction: vel.linear.x = 0.15
  if 'j' in direction: vel.linear.x = -0.15
  if 'h' in direction: vel.angular.z = 3.14/4
  if 'l' in direction: vel.angular.z = -3.14/4
  print vel
  pub.publish(vel)

