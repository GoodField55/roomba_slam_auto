#!/usr/bin/env python
import rospy,copy,math,random
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

class Roomba():
  def __init__(self):
#    self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

#    self.sensor_values = LightSensorValues()
#    rospy.Subscriber('lightsensors', LightSensorValues, self.callback)

    self.right_flag = False   # dummy
#    self.left_flag = False

#  def callback(self,messages):
#    self.sensor_values = messages

#  def wall_front(self,ls):
#    return ls.left_forward > 50 or ls.right_forward > 50

#  def too_right(self,ls):
#    return ls.right_side > 50

#  def too_left(self,ls):
#    return ls.left_side > 50

  def run(self):
    rate = rospy.Rate(10)	#(20)
    data = Twist()

    data.linear.x = 0.0		#0.1
    data.angular.z = 0.0

    z = 0.0			#0.5		#math.pi / 3.0

    data.angular.z = z


    while not rospy.is_shutdown():
#      if self.wall_front(self.sensor_values):
#        if self.right_flag:
#            data.angular.z = -1 * z
#        elif self.left_flag:
#            data.angular.z = z
#        elif random.random() > 0.5:
#            data.angular.z = -1 * z
#            self.left_flag = False
#            self.right_flag = True      
#        else:
#          data.angular.z = z
#          self.left_flag = True
#          self.right_flag = False
#      elif self.too_right(self.sensor_values):
#        data.angular.z = z
#        self.left_flag = True
#        self.right_flag = False
#      elif self.too_left(self.sensor_values):
#        data.angular.z = -1 * z
#        self.left_flag = False
#        self.right_flag = True
#      else:
#        data.angular.z = 0
#        self.left_flag = False
#        self.right_flag = False

#      rospy.loginfo("vel=%f, ang=%f\n", data.linear.x, data.angular.z)
#      self.cmd_vel.publish(data)
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('roomba')
#  rospy.wait_for_service('/motor_on')
#  rospy.wait_for_service('/motor_off')
#  rospy.on_shutdown(rospy.ServiceProxy('motor_off',Trigger).call)
#  rospy.ServiceProxy('motor_on',Trigger).call()
  Roomba().run()

