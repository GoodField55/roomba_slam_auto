#!/usr/bin/env python
import rospy,copy,math,random
from geometry_msgs.msg import Twist
from ca_msgs.msg import Bumper

class Roomba():
  def __init__(self):
    self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

    self.bumper_values = Bumper()
    rospy.Subscriber('bumper', Bumper, self.callback)

    self.right_flag = False  # True : under right turn
    self.left_flag = False   # True : under left turn

  def callback(self,messages):
    self.bumper_values = messages

  def wall_front(self,ls):
    return ls.light_signal_center_left > 100 or ls.light_signal_center_right > 100

  def too_right(self,ls):
    return ls.light_signal_front_right > 100

  def too_left(self,ls):
    return ls.light_signal_front_left > 100

  def bumper_pressed(self,ls):
    return ls.is_left_pressed == True or ls.is_right_pressed == True

  def run(self):
    rate = rospy.Rate(10)
    data = Twist()

    x_hi = 0.2   # high speed
    x_lo = 0.02  # low  speed
    data.linear.x = x_hi

    z_hi = math.pi / 3.0
    z_lo = math.pi / 15.0
    data.angular.z = z_hi

    while not rospy.is_shutdown():
      if self.bumper_values.is_left_pressed == True  : # left bumper pressed
        data.linear.x = -1 * x_lo
        data.angular.z = -1 * z_lo
        self.left_flag = False
        self.right_flag = True
      elif self.bumper_values.is_right_pressed == True  : # right bumper pressed
        data.linear.x = -1 * x_lo
        data.angular.z = z_lo
        self.left_flag = True
        self.right_flag = False
      else:  # bumper not pressed
        if self.wall_front(self.bumper_values):
          if self.right_flag:  # under right turn
              data.angular.z = -1 * z_hi
          elif self.left_flag:  # under left turn
              data.angular.z = z_hi
          elif random.random() > 0.5:
              data.angular.z = -1 * z_hi
              self.left_flag = False
              self.right_flag = True      
          else:
            data.angular.z = z_hi
            self.left_flag = True
            self.right_flag = False
        elif self.too_right(self.bumper_values):
          data.angular.z = z_hi
          self.left_flag = True
          self.right_flag = False
        elif self.too_left(self.bumper_values):
          data.angular.z = -1 * z_hi
          self.left_flag = False
          self.right_flag = True
        else:
          data.linear.x = x_hi
          data.angular.z = 0
          self.left_flag = False
          self.right_flag = False

      rospy.loginfo("vel=%f, ang=%f\n", data.linear.x, data.angular.z)
      self.cmd_vel.publish(data)
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('roomba')
  Roomba().run()

