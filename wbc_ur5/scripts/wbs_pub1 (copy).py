#!/usr/bin/env python

import rospy
import pinocchio
from std_msgs.msg import String
from whole_body_state_msgs.whole_body_state_publisher import WholeBodyStatePublisher

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

import numpy as np

urdf_filename = '/home/niger/wbc-ur3_ws/src/whole_body_state_msgs/urdf/ur3e.urdf'
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



#Setup information of floating base
def floating_base_callback(msg):
   q[0] = msg.pose.pose.position.x
   q[1] = msg.pose.pose.position.y
   q[2] = msg.pose.pose.position.z
   q[3] = msg.pose.pose.orientation.x
   q[4] = msg.pose.pose.orientation.y
   q[5] = msg.pose.pose.orientation.z
   q[6] = msg.pose.pose.orientation.w

   v[0] = msg.twist.twist.linear.x
   v[1] = msg.twist.twist.linear.y
   v[2] = msg.twist.twist.linear.z
   v[3] = msg.twist.twist.angular.x
   v[4] = msg.twist.twist.angular.y
   v[5] = msg.twist.twist.angular.z

   position = np.array([0, 0, 0])
   #position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
   quaternion = pinocchio.Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
   conf_wrt_1 = pinocchio.SE3(quaternion, position).inverse()
   conf_wrt_1_action = conf_wrt_1.toActionMatrix()

   lin_vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
   ang_vel = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
   velocity_wrt_0 = pinocchio.Motion(lin_vel, ang_vel)

   #velocity_wrt_1 = pinocchio.SE3(quaternion, position).inverse().act(velocity_wrt_0)
   #velocity_wrt_1 = pinocchio.SE3(quaternion, position).actInv(velocity_wrt_0)
   #velocity_wrt_1 = data.oMi[1].actInv(velocity_wrt_0)

   # Perform the forward kinematics over the kinematic tree
   pinocchio.forwardKinematics(model,data,q)

   base_link_frame_id = model.getFrameId("base_link")

   #oMf = pinocchio.updateFramePlacement(model, data, base_link_frame_id)
   #velocity_wrt_1 = oMf.actInv(velocity_wrt_0)

   velocity_wrt_1 = data.oMf[base_link_frame_id].actInv(velocity_wrt_0)

   lin_vel_wrt_1 = data.oMi[1].rotation.T.dot(lin_vel)
   ang_vel_wrt_1 = data.oMi[1].rotation.T.dot(ang_vel)

   #print(velocity_wrt_1)
   print("R")
   print(data.oMi[1].rotation)
   print(data.oMi[1].rotation.T)

   from numpy import array
   from numpy.linalg import norm
   arr = array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
   print(arr)
   norm_l1 = norm(arr)
   print(norm_l1)


   #conf_wrt_1 = 
   '''v[0] = lin_vel_wrt_1[0]
   v[1] = lin_vel_wrt_1[1]
   v[2] = lin_vel_wrt_1[2]
   v[3] = ang_vel_wrt_1[0]
   v[4] = ang_vel_wrt_1[1]
   v[5] = ang_vel_wrt_1[2]'''

   '''v[0] = velocity_wrt_1.linear[0]
   v[1] = velocity_wrt_1.linear[1]
   v[2] = velocity_wrt_1.linear[2]
   v[3] = velocity_wrt_1.angular[0]
   v[4] = velocity_wrt_1.angular[1]
   v[5] = velocity_wrt_1.angular[2]'''

   '''v[0] = velocity_wrt_0.linear[0]
   v[1] = velocity_wrt_0.linear[1]
   v[2] = velocity_wrt_0.linear[2]
   v[3] = velocity_wrt_0.angular[0]
   v[4] = velocity_wrt_0.angular[1]
   v[5] = velocity_wrt_0.angular[2]'''


#Setup information of joints
def joint_state_callback(msg):
   '''for i in range(7,21):
      q[i] = msg.position[i+16-7]
   for i in range(0,17):
      q[i+13+7] = msg.position[i]

   for i in range(6,20):
      v[i] = msg.velocity[i+16-6]
   for i in range(0,17):
      v[i+13+6] = msg.velocity[i]'''

   #Define joints position
   leg_index = 7
   torso_index = 19
   arm_index = 21
   head_index = 35

   for i in range(len(msg.position)):

      #Arms information
      if(i < 14):

         #Calculate index
         index = i + arm_index

      #Head information
      elif(i < 16):

         #Calculate index
         index = i - 14 + head_index
      
      #Legs information
      elif (i < 28):

         #Calculate index
         index = i - 16 + leg_index

      #Torso information
      else:
         #Calculate index
         index = i - 28 + torso_index

      #Debug information
      #print(str(index) + ", " + str(i))

      #Setup information
      q[index] = msg.position[i]
      v[index - 1] = msg.velocity[i]
      tau[index - 7] = msg.effort[i]


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
   rospy.Subscriber('/floating_base_pose_simulated', Odometry, floating_base_callback, queue_size=10)
   rospy.Subscriber('/joint_states', JointState, joint_state_callback, queue_size=10)
   rospy.Subscriber('/right_ankle_ft', WrenchStamped, wrench_right_callback, queue_size=10)
   rospy.Subscriber('/left_ankle_ft', WrenchStamped, wrench_left_callback, queue_size=10)
   wbs._pub
   wbs._wb_iface
#   x = wbs._wb_iface._model.njoints - 2
#   print(x)
#   print(wbs._wb_iface._msg.joints)
   rate = rospy.Rate(10) # 10hz


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
      
