#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ca_msgs.msg import Bumper

class Roomba():
  def __init__(self):
    self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    self.sub_cmd_key = rospy.Subscriber('cmd_key', Twist, self.callback_cmd_key)
    self.vel = Twist()
    self.bumper = Bumper()
    rospy.Subscriber('/bumper', Bumper , self.callback_bumper)

  def callback_cmd_key(self,message):
    self.vel = message

  def send_vel(self):

    if self.vel.linear.x >0 or self.vel.angular.z != 0:

      # if near to wall, decelerate
      if self.bumper.is_light_left == True or self.bumper.is_light_front_left == True or self.bumper.is_light_center_left == True or self.bumper.is_light_right == True or self.bumper.is_light_front_right == True or self.bumper.is_light_center_right == True:
        self.vel.linear.x = 0.02

      # if bumper pressed, stop motor
      if self.bumper.is_left_pressed == True or self.bumper.is_right_pressed == True:
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

    self.cmd_vel.publish(self.vel)

  def callback_bumper(self, messages):
    self.bumper = messages

  def run(self):
    rate = rospy.Rate(10)

    self.vel.linear.x = 0.0
    self.vel.angular.z = 0.0
    self.send_vel()   # motor stop

    while not rospy.is_shutdown():
      
      self.send_vel()
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('roomba')
  Roomba().run()

