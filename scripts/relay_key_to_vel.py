#!/usr/bin/env python
#encoding: utf8
import sys, rospy
from geometry_msgs.msg import Twist

class Relay():
  def __init__(self):
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    self.sub_cmd_key = rospy.Subscriber('cmd_key', Twist, self.callback_cmd_key)
    self.vel = Twist()

  def callback_cmd_key(self,message):
    self.vel = message

  def send_vel(self):
    self.pub.publish(self.vel)

if __name__ == '__main__':
  rospy.init_node('relay_vel')
  relay = Relay()
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    relay.send_vel()
    rate.sleep()
