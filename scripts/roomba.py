#!/usr/bin/env python
import rospy,copy,math,random
from geometry_msgs.msg import Twist
from ca_msgs.msg import Bumper
from ca_msgs.msg import Cliff

class Roomba():
  def __init__(self):
    self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

    self.bumper_values = Bumper()
    rospy.Subscriber('bumper', Bumper, self.callback_bumper)

    self.cliff_values = Cliff()
    rospy.Subscriber('cliff', Cliff, self.callback_cliff)

    self.right_flag = False  # True : under right turn
    self.left_flag = False   # True : under left turn

    self.right_bumper_back_flag = False  # True : under right bumper back process
    self.right_bumper_turn_flag = False  # True : under right bumper turn process
    self.left_bumper_back_flag = False  # True : under left bumper backprocess
    self.left_bumper_turn_flag = False  # True : under left bumper turn process

    self.right_bumper_phase = 0  # phase of right bumper process
    self.left_bumper_phase = 0  # phase of left bumper process

    self.right_cliff_back_flag = False  # True : under right cliff back process
    self.right_cliff_turn_flag = False  # True : under right cliff turn process
    self.left_cliff_back_flag = False  # True : under left cliff back process
    self.left_cliff_turn_flag = False  # True : under left cliff turn process

    self.right_cliff_phase = 0  # phase of right cliff process
    self.left_cliff_phase = 0  # phase of left cliff process
 
    self.front_right_cliff_back_flag = False  # True : under front right cliff back process
    self.front_right_cliff_turn_flag = False  # True : under front right cliff turn process
    self.front_left_cliff_back_flag = False  # True : under front left cliff back process
    self.front_left_cliff_turn_flag = False  # True : under front left cliff turn process

    self.front_right_cliff_phase = 0  # phase of front right cliff process
    self.front_left_cliff_phase = 0  # phase of front left cliff process

    self.goal_bumper_back = 0     # process counts to goal(bumper back)
    self.goal_bumper_turn = 0     # process counts to goal(bumper turn)
    self.goal_cliff_back = 0      # process counts to goal(cliff back)
    self.goal_cliff_turn = 0      # process counts to goal(cliff turn)

    self.goal_front_cliff_back = 0      # process counts to goal(front cliff back)
    self.goal_front_cliff_turn = 0      # process counts to goal(front cliff turn)

    self.counter = 0                    # process counter to goal 

  def callback_bumper(self,messages):
    self.bumper_values = messages

  def callback_cliff(self,messages):
    self.cliff_values = messages

  def wall_front(self,ls):
    return ls.light_signal_center_left > 100 or ls.light_signal_center_right > 100

  def wall_compare(self,ls):    # return true : left>right , false left<right
    return (ls.light_signal_left + ls.light_signal_front_left + ls.light_signal_center_left) > (ls.light_signal_right + ls.light_signal_front_right + ls.light_signal_center_right)

  def too_right(self,ls):
    return ls.light_signal_front_right > 100

  def too_left(self,ls):
    return ls.light_signal_front_left > 100

  def bumper_pressed(self,ls):
    return ls.is_left_pressed == True or ls.is_right_pressed == True

  def run(self):
    loop_hz = 10 * 10 # boost x10
    rate = rospy.Rate(loop_hz)
    data = Twist()

    x_hi = 0.1   # high speed
    x_lo = 0.05  # low  speed
    data.linear.x = x_hi

    z_hi = math.pi / 4.0
    z_lo = math.pi / 8.0
    data.angular.z = z_hi

    self.goal_bumper_back = loop_hz * 0.05 / x_hi            # counts to 0.05m
    self.goal_bumper_turn = loop_hz * math.pi / 4.0 / z_hi  # counts to 90degree
 
    self.goal_cliff_back = loop_hz * 0.1 / x_hi            # counts to 0.1m
    self.goal_cliff_turn = loop_hz * math.pi / 8.0 / z_hi  # counts to 45degree

    self.goal_front_cliff_back = loop_hz * 0.1 / x_hi            # counts to 0.1m
    self.goal_front_cliff_turn = loop_hz * math.pi / 4.0 / z_hi  # counts to 90degree

    while not rospy.is_shutdown():
      if self.cliff_values.is_cliff_front_left == True or self.front_left_cliff_back_flag == True or self.front_left_cliff_turn_flag == True : # front left cliff
        if self.front_left_cliff_phase == 0 :  # under back process
          if self.front_left_cliff_back_flag == False : # first time of cliff front left back
            self.counter = 0
          data.linear.x = -1 * x_hi
          data.angular.z = 0
          self.left_flag = False
          self.right_flag = False
          self.front_left_cliff_back_flag = True
          self.front_right_cliff_back_flag = False
          self.counter += 1
          rospy.loginfo("cliff_front_left back : %d",self.counter)
          if self.counter > self.goal_front_cliff_back :  # stop backward
            data.linear.x = 0
            data.angular.z = 0
            self.front_left_cliff_phase += 1
        else :                                 # under turn process
          if self.front_left_cliff_turn_flag == False : # first time of cliff front left turn
            self.counter = 0
          data.linear.x = 0
          data.angular.z = -1 * z_hi
          self.left_flag = False
          self.right_flag = True
          self.front_left_cliff_turn_flag = True
          self.front_right_cliff_tirn_flag = False
          self.front_left_cliff_back_flag = False     # clear back flag
          self.counter += 1
          rospy.loginfo("cliff_front_left turn : %d",self.counter)
          if self.counter > self.goal_front_cliff_turn :  # stop turn
            data.linear.x = x_hi
            data.angular.z = 0
            self.left_flag = False
            self.right_flag = False
            self.front_left_cliff_turn_flag = False
            self.front_right_cliff_turn_flag = False
            self.front_left_cliff_phase = 0                  # clear phase
      elif self.cliff_values.is_cliff_front_right == True or self.front_right_cliff_back_flag == True or self.front_right_cliff_turn_flag == True : # front right cliff
        if self.front_right_cliff_phase == 0 :  # under back process
          if self.front_right_cliff_back_flag == False : # first time of cliff front left back
            self.counter = 0
          data.linear.x = -1 * x_hi
          data.angular.z = 0
          self.left_flag = False
          self.right_flag = False
          self.front_left_cliff_back_flag = False
          self.front_right_cliff_back_flag = True
          self.counter += 1
          rospy.loginfo("cliff_front_right back : %d",self.counter)
          if self.counter > self.goal_front_cliff_back :  # stop backward
            data.linear.x = 0
            data.angular.z = 0
            self.front_right_cliff_phase += 1
        else :                                 # under turn process
          if self.front_right_cliff_turn_flag == False : # first time of cliff front right turn
            self.counter = 0
          data.linear.x = 0
          data.angular.z = z_hi
          self.left_flag = True
          self.right_flag = False
          self.front_left_cliff_turn_flag = False
          self.front_right_cliff_tirn_flag = True
          self.front_right_cliff_back_flag = False     # clear back flag
          self.counter += 1
          rospy.loginfo("cliff_front_left turn : %d",self.counter)
          if self.counter > self.goal_front_cliff_turn :  # stop turn
            data.linear.x = x_hi
            data.angular.z = 0
            self.left_flag = False
            self.right_flag = False
            self.front_left_cliff_turn_flag = False
            self.front_right_cliff_turn_flag = False
            self.front_right_cliff_phase = 0                  # clear phase

      elif self.cliff_values.is_cliff_left == True or self.left_cliff_back_flag == True or self.left_cliff_turn_flag == True : #left cliff
        if self.left_cliff_phase == 0 :  # under back process
          if self.left_cliff_back_flag == False : # first time of cliff left back
            self.counter = 0
          data.linear.x = -1 * x_hi
          data.angular.z = 0
          self.left_flag = False
          self.right_flag = False
          self.left_cliff_back_flag = True
          self.right_cliff_back_flag = False
          self.counter += 1
          rospy.loginfo("cliff_left back : %d",self.counter)
          if self.counter > self.goal_cliff_back :  # stop backward
            data.linear.x = 0
            data.angular.z = 0
            self.left_cliff_phase += 1
        else :                                 # under turn process
          if self.left_cliff_turn_flag == False : # first time of cliff left turn
            self.counter = 0
          data.linear.x = 0
          data.angular.z = -1 * z_hi
          self.left_flag = False
          self.right_flag = True
          self.left_cliff_turn_flag = True
          self.right_cliff_tirn_flag = False
          self.left_cliff_back_flag = False     # clear back flag
          self.counter += 1
          rospy.loginfo("cliff_left turn : %d",self.counter)
          if self.counter > self.goal_cliff_turn :  # stop turn
            data.linear.x = x_hi
            data.angular.z = 0
            self.left_flag = False
            self.right_flag = False
            self.left_cliff_turn_flag = False
            self.right_cliff_turn_flag = False
            self.left_cliff_phase = 0                  # clear phase
      elif self.cliff_values.is_cliff_right == True or self.right_cliff_back_flag == True or self.right_cliff_turn_flag == True : # right cliff
        if self.right_cliff_phase == 0 :  # under back process
          if self.right_cliff_back_flag == False : # first time of cliff right back
            self.counter = 0
          data.linear.x = -1 * x_hi
          data.angular.z = 0
          self.left_flag = False
          self.right_flag = False
          self.left_cliff_back_flag = False
          self.right_cliff_back_flag = True
          self.counter += 1
          rospy.loginfo("cliff_right back : %d",self.counter)
          if self.counter > self.goal_cliff_back :  # stop backward
            data.linear.x = 0
            data.angular.z = 0
            self.right_cliff_phase += 1
        else :                                 # under turn process
          if self.front_right_cliff_turn_flag == False : # first time of cliff right turn
            self.counter = 0
          data.linear.x = 0
          data.angular.z = z_hi
          self.left_flag = True
          self.right_flag = False
          self.left_cliff_turn_flag = False
          self.right_cliff_tirn_flag = True
          self.right_cliff_back_flag = False     # clear back flag
          self.counter += 1
          rospy.loginfo("cliff_right turn : %d",self.counter)
          if self.counter > self.goal_cliff_turn :  # stop turn
            data.linear.x = x_hi
            data.angular.z = 0
            self.left_flag = False
            self.right_flag = False
            self.left_cliff_turn_flag = False
            self.right_cliff_turn_flag = False
            self.right_cliff_phase = 0                  # clear phase

      elif self.bumper_values.is_left_pressed == True or self.left_bumper_back_flag == True  or self.left_bumper_turn_flag == True : # left bumper pressed
        if self.left_bumper_phase == 0 :  # under back process
          if self.left_bumper_back_flag == False : # first time of bumper left back
            self.counter = 0
          data.linear.x = -1 * x_hi
          data.angular.z = 0
          self.left_flag = False
          self.right_flag = False
          self.left_bumper_back_flag = True
          self.right_bumper_back_flag = False
          self.counter += 1
          rospy.loginfo("bumper_left back : %d",self.counter)
          if self.counter > self.goal_bumper_back :  # stop backward
            data.linear.x = 0
            data.angular.z = 0
            self.left_bumper_phase += 1
        else :                                 # under turn process
          if self.left_bumper_turn_flag == False : # first time of bumper left turn
            self.counter = 0
          data.linear.x = 0
          data.angular.z = -1 * z_hi
          self.left_flag = False
          self.right_flag = True
          self.left_bumper_turn_flag = True
          self.right_bumper_turn_flag = False
          self.left_bumper_back_flag = False     # clear back flag
          self.counter += 1
          rospy.loginfo("bumper_left turn : %d",self.counter)
          if self.counter > self.goal_bumper_turn :  # stop turn
            data.linear.x = x_hi
            data.angular.z = 0
            self.left_flag = False
            self.right_flag = False
            self.left_bumper_turn_flag = False
            self.right_bumper_turn_flag = False
            self.left_bumper_phase = 0                  # clear phase
      elif self.bumper_values.is_right_pressed == True or self.right_bumper_back_flag == True  or self.right_bumper_turn_flag == True : # right bumper pressed
        if self.right_bumper_phase == 0 :  # under back process
          if self.right_bumper_back_flag == False : # first time of bumper right back
            self.counter = 0
          data.linear.x = -1 * x_hi
          data.angular.z = 0
          self.left_flag = False
          self.right_flag = False
          self.left_bumper_back_flag = False
          self.right_bumper_back_flag = True
          self.counter += 1
          rospy.loginfo("bumper_right back : %d",self.counter)
          if self.counter > self.goal_bumper_back :  # stop backward
            data.linear.x = 0
            data.angular.z = 0
            self.right_bumper_phase += 1
        else :                                 # under turn process
          if self.right_bumper_turn_flag == False : # first time of bumper right turn
            self.counter = 0
          data.linear.x = 0
          data.angular.z = z_hi
          self.left_flag = True
          self.right_flag = False
          self.left_bumper_turn_flag = False
          self.right_bumper_turn_flag = True
          self.right_bumper_back_flag = False     # clear back flag
          self.counter += 1
          rospy.loginfo("bumper_right turn : %d",self.counter)
          if self.counter > self.goal_bumper_turn :  # stop turn
            data.linear.x = x_hi
            data.angular.z = 0
            self.left_flag = False
            self.right_flag = False
            self.left_bumper_turn_flag = False
            self.right_bumper_turn_flag = False
            self.right_bumper_phase = 0                  # clear phase

      else:  # bumper not pressed & not cliff
        if self.wall_front(self.bumper_values):
          if self.right_flag:  # under right turn
            data.linear.x = x_lo
            data.angular.z = -1 * z_lo
            rospy.loginfo("wall_front under right turn")
          elif self.left_flag:  # under left turn
            data.linear.x = x_lo
            data.angular.z = z_lo
            rospy.loginfo("wall_front under left turn")
          elif self.wall_compare(self.bumper_values):
            data.linear.x = x_lo
            data.angular.z = -1 * z_lo
            self.left_flag = False
            self.right_flag = True
            rospy.loginfo("wall_front right turn")
          else:
            data.linear.x = x_lo
            data.angular.z = z_lo
            self.left_flag = True
            self.right_flag = False
            rospy.loginfo("wall_front left turn")
        elif self.too_right(self.bumper_values):
          data.angular.z = z_hi
          self.left_flag = True
          self.right_flag = False
          rospy.loginfo("too_right")
        elif self.too_left(self.bumper_values):
          data.angular.z = -1 * z_hi
          self.left_flag = False
          self.right_flag = True
          rospy.loginfo("too_left")
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

