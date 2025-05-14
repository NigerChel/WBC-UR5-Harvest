#!/usr/bin/env python

import sys
sys.path.append("/home/dell/pinocchio/build/bindings/python/pinocchio")


import rospy
import pinocchio
from std_msgs.msg import String
from whole_body_state_msgs.msg import WholeBodyState
from whole_body_state_msgs.whole_body_state_publisher import WholeBodyStatePublisher

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

import numpy as np

urdf_filename = '/home/dell/ros_ws/src/whole_body_state_msgs/urdf/ur5.urdf'
#urdf_filename = '/home/niger/wbc-ur3_ws/src/whole_body_state_msgs/urdf/robotiq_85_gripper.xacro'

free_flyer = pinocchio.JointModelFreeFlyer()

topic = 'robot_states'
model = pinocchio.buildModelFromUrdf(urdf_filename, free_flyer)
#free_flyer
#model = pinocchio.buildModelFromXML(urdf_filename, free_flyer)
#pinocchio::urdf::buildModelFromXML(robot_model_, pinocchio::JointModelFreeFlyer(), model_);

# Create data required by the algorithms
data = model.createData()

frame_id = "base_link" 

wbs = WholeBodyStatePublisher(topic, model, frame_id)

print(model.nq)

q = np.zeros(model.nq)
v = np.zeros(model.nv)
tau = np.zeros(model.njoints - 2)

wrench_right_force =  np.zeros(3)
wrench_right_torque =  np.zeros(3)

wrench_left_force =  np.zeros(3)
wrench_left_torque =  np.zeros(3)

right_nsurf = [0, 0, 4]
right_frict_coeff = 3

left_nsurf = [0, 0, 4]
left_frict_coeff = 3

q[0] = 0.0
q[1] = 0.0
q[2] = 0.0
q[3] = 0.0
q[4] = 0.0
q[5] = 0.0
q[6] = 1.0


#Setup information of joints
def joint_state_callback(msg):
      q[7] = msg.position[3]
      q[8] = msg.position[2]
      q[9] = msg.position[0]
      q[10] = msg.position[4]
      q[11] = msg.position[5]
      q[12] = msg.position[6]
      #v[6] = msg.velocity[i]
      #tau[index - 7] = msg.effort[i]


def wrench_right_callback(msg):
   wrench_right_force[0] = msg.wrench.force.x
   wrench_right_force[1] = msg.wrench.force.y
   wrench_right_force[2] = msg.wrench.force.z

   wrench_right_torque[0] = msg.wrench.torque.x
   wrench_right_torque[1] = msg.wrench.torque.y
   wrench_right_torque[2] = msg.wrench.torque.z

def wrench_left_callback(msg):
   wrench_left_force[0] = msg.wrench.force.x
   wrench_left_force[1] = msg.wrench.force.y
   wrench_left_force[2] = msg.wrench.force.z

   wrench_left_torque[0] = msg.wrench.torque.x
   wrench_left_torque[1] = msg.wrench.torque.y
   wrench_left_torque[2] = msg.wrench.torque.z

class WrenchRightAnkle:
   linear = wrench_right_force   
   angular = wrench_right_torque

class WrenchLeftAnkle:
   linear = wrench_left_force   
   angular = wrench_left_torque
p=dict()
pd=dict()
s = {'wrist_3_joint':[left_nsurf, left_frict_coeff]}
f = {'wrist_3_joint':[0, WrenchLeftAnkle()]}

#s = {'right_sole_link':[right_nsurf, right_frict_coeff], 'left_sole_link':[left_nsurf, left_frict_coeff]}
#f = {'right_sole_link':[0, WrenchRightAnkle()], 'left_sole_link':[0, WrenchLeftAnkle()]}

def talker():
   rospy.init_node('wbs') #anonymous=True
   pub = rospy.Publisher('chatter', String, queue_size=10)
   rospy.Subscriber('/joint_states', JointState, joint_state_callback, queue_size=10)
   rospy.Subscriber('/right_ankle_ft', WrenchStamped, wrench_right_callback, queue_size=10)
   rospy.Subscriber('/left_ankle_ft', WrenchStamped, wrench_left_callback, queue_size=10)
   wbs._pub
   wbs._wb_iface
#   x = wbs._wb_iface._model.njoints - 2
#   print(x)
#   print(wbs._wb_iface._msg.joints)
   rate = rospy.Rate(10) # 10hz

   print(model.nq)
   while not rospy.is_shutdown():
      t =  rospy.Time.now().secs
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)
      pub.publish(hello_str)
      #wbs.publish(t, q, v, tau, p, pd, f, s)
      wbs.publish(t, q, v, tau, p, pd, f, s)
      rate.sleep()

if __name__ == '__main__':
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
      
