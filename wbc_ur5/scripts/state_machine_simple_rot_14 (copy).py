#!/usr/bin/env python

import rospy
import smach

from cgi import print_directory
import os
import unittest
from math import sqrt

import sys
sys.path.append("/home/niger/ndcurves/build/python/ndcurves")

import time
import eigenpy
import numpy as np

import yaml 
from yaml import load

from numpy import array, array_equal, isclose, random, zeros
from numpy.linalg import norm
import pickle

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

from ndcurves import exact_cubic, curve_constraints, Quaternion, SO3Linear, SE3Curve



from scipy.spatial.transform import Rotation
from scipy.spatial import transform
from pyquaternion import Quaternion

#######################################################################################
############################ IMPORTAR PARA PUBLICAR ###################################
#######################################################################################
import rospy
import rosbag
import pinocchio
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from whole_body_state_msgs.whole_body_state_publisher import WholeBodyStatePublisher
from whole_body_state_msgs.msg import WholeBodyState

######################################################################################
bag = rosbag.Bag('/home/niger/ros_ws/bagfiles/footprints_rot_go_right.bag')
#bag = rosbag.Bag('/home/niger/reemc_public_ws/bagfiles/footprints.bag')
for topic, msg, t in bag.read_messages(topics=['/vigir/footstep_planning/vis/step_plan_vis']):
    #pose = msg.markers[21].pose.position.x
    bag.close()
#pose1 = msg.markers[21].pose.position.x

h_des = 0.04
step_with = 0.075
#n = 9 # Number of steps
n = len(msg.markers)/2-2
rospy_sleep_com = 0.5
rospy_sleep_rfoot = 0.3
rospy_sleep_lfoot = 0.3

#Foot position
p_sub = dict()
p_sub_media  = dict()
p_sub[0] = np.array([0.0, -0.075, 0.0])
#p_sub[i+1] = np.array([msg2.markers[i+2].pose.position.x, msg2.markers[i+2].pose.position.y, msg2.markers[i+2].pose.position.z])
for i in range(0,n+2,1):
    p_sub[i+1] = np.array([msg.markers[2*i].pose.position.x, msg.markers[2*i].pose.position.y, msg.markers[2*i].pose.position.z - 0.0254])
    #print(p_sub)


for i in range(0,(n+1)/2,1):
    p_sub_media[2*i+1] = np.array([(p_sub[2*i][0] + p_sub[2*(i+1)][0])/2, (p_sub[2*i][1] + p_sub[2*(i+1)][1])/2, h_des])
    #print(p_sub_media)
    
for i in range(0,(n+1)/2,1):
    p_sub_media[2*i+2] = np.array([(p_sub[2*i+1][0] + p_sub[2*(i+1)+1][0])/2, (p_sub[2*i+1][1] + p_sub[2*(i+1)+1][1])/2, h_des])

#################################################################################################
#Foot orientation
p_sub_rot = dict()
p_sub_rot[0] = np.array([0.0, 0.0, 0.0, 1.0])
#p_sub[i+1] = np.array([msg2.markers[i+2].pose.position.x, msg2.markers[i+2].pose.position.y, msg2.markers[i+2].pose.position.z])
for i in range(0,n+2,1):
    p_sub_rot[i+1] = np.array([msg.markers[2*i].pose.orientation.x, msg.markers[2*i].pose.orientation.y, msg.markers[2*i].pose.orientation.z, msg.markers[2*i].pose.orientation.w])
    print(p_sub_rot)


curva_rot={}

for i in range(1,n+2,1):
    curva_rot[i] = [[0, 0, 0, 0],[0, 0, 0, 0]]

#Defining the target rotation for right foot
for i in range(0,(n+1)/2,1):
    curva_rot[2*i+1][0] = p_sub_rot[2*i]
    curva_rot[2*i+1][1] = p_sub_rot[2*(i+1)] # np.array([(p_sub[2*i][0] + p_sub[2*(i+1)][0])/2, (p_sub[2*i][1] + p_sub[2*(i+1)][1])/2, 0.1])
    print(curva_rot[2*i+1][0])
    print(curva_rot[2*i+1][1])

#Defining the target rotation for left foot
for i in range(0,(n+1)/2,1):
    curva_rot[2*i+2][0] = p_sub_rot[2*i+1]
    curva_rot[2*i+2][1] = p_sub_rot[2*(i+1)+1]

#################################################################################################

curva={}

for i in range(1,n+2,1):
    curva[i] = [[0, 0, 0],[0, 0, 0],[0, 0, 0]]

#Defining the target position for right foot
for i in range(0,(n+1)/2,1):
    curva[2*i+1][0] = p_sub[2*i]
    curva[2*i+1][1] = p_sub_media[2*i+1]
    curva[2*i+1][2] = p_sub[2*(i+1)] # np.array([(p_sub[2*i][0] + p_sub[2*(i+1)][0])/2, (p_sub[2*i][1] + p_sub[2*(i+1)][1])/2, 0.1])
    #print(p_sub_media)

#Defining the target position for left foot
for i in range(0,(n+1)/2,1):
    curva[2*i+2][0] = p_sub[2*i+1]
    curva[2*i+2][1] = p_sub_media[2*i+2]
    curva[2*i+2][2] = p_sub[2*(i+1)+1] #np.array([(p_sub[2*i+1][0] + p_sub[2*(i+1)+1][0])/2, (p_sub[2*i+1][1] + p_sub[2*(i+1)+1][1])/2, 0.1])
#######################################################################################
#######################################################################################
#######################################################################################

#Creation of exact cubic
'''T0=0.0
T1=1.0
T2=2.0'''

T0=0.0
T1=0.9
T2=1.8

#######################################################################################
########################################ICP############################################
#######################################################################################
## These come from a footstep planner
p = dict()
p[0] = np.array([0, 0, 0])
for i in range(1,n+1,1):
    p[i] = curva[i+1][0]
p[n+1] = curva[n][2]

g = 9.81 
he = 0.75 # Desired height of the Linear Inverted Pendulum (LIP)
w = np.sqrt(g/he) # Eigenfrequency of the pendulum

steptimes = 12

icp_eos = dict() # eos = end of step
icp_ini = dict() # ini = inicial of step

icp_eos[n] = (p[n+1] + p[n])/2

# Backward calculation
for i in range(n,-1,-1):
    icp_ini[i] = np.zeros(3)
    for j in range(3):
        icp_ini[i][j] = p[i][j] + 1/np.exp(T1*w) * (icp_eos[i][j] - p[i][j])      
    icp_eos[i-1] = icp_ini[i]
#####################################################
icp_ref = dict()
t = np.linspace(T0, T1, steptimes)
for i in range(0,n+1,1):
    icp_ref[i] = dict()
    for j in range(3):
        icp_ref[i][j] = np.zeros(steptimes)
        for k in range(steptimes): # k can be seen as the varying time
            icp_ref[i][j][k] = p[i][j] + np.exp(t[k]*w) * (icp_ini[i][j] - p[i][j]) 
######################################################

waypoints={}
timewaypoints={}
ec={}
NumberOfSplines={}
FirstSpline={}

res={}

Tmincheck={}

c={}
ec2={}
ec2_rot={}
steptime={}

##################
se3 = {}
rot_se3 = {}
pse3 = {}
pose = {}
##################

x={}
y={}
z={}
t={}

x_vel={}
y_vel={}
z_vel={}

x_acc={}
y_acc={}
z_acc={}

##############################
x_rot={}
y_rot={}
z_rot={}
w_rot={}

for j in range(n+1):
    waypoints[j]=np.matrix(curva[j+1]).transpose()
    timewaypoints[j]=np.matrix([T0,T1,T2]).transpose()
    ec[j]=exact_cubic(waypoints[j],timewaypoints[j])
    NumberOfSplines[j]=ec[j].getNumberSplines()#Get number of splines
    FirstSpline[j]=ec[j].getSplineAt(0)#Get first spline (polynomial)

    #Evaluationatt=0.5
    res[j]=ec[j](T1)

    #Derivativeorder1att=0.5
    res[j]=ec[j].derivate(T1,1)

    #Upperandlowerboundofdefinitioninterval
    Tmincheck[j]=ec[j].min()
    Tmincheck[j]=ec[j].max()

    #Creation of exact cubic with constraints
    c[j]=curve_constraints(3)
    c[j].init_vel=np.matrix([0.0,0.,0.0]).transpose()
    c[j].end_vel=np.matrix([0.,0.,0.]).transpose()
    c[j].init_acc=np.matrix([0.0,0.,0.0]).transpose()
    c[j].end_acc=np.matrix([0.,0.,0.]).transpose()
    ec2[j]=exact_cubic(waypoints[j],timewaypoints[j],c[j])

    ###################p##p#####################################################################################################
    ##########################################################COMENTAR########################################################
    ##########################################################################################################################
    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler('xyz', [0, 0, 0], degrees=True)
    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    print("El quaternion es")
    print(rot_quat)

    init_quat = pinocchio.Quaternion(curva_rot[j+1][0][3],curva_rot[j+1][0][0],curva_rot[j+1][0][1],curva_rot[j+1][0][2])
    #init_quat = pinocchio.Quaternion(0.38,0.0,0.0,0.92)
    end_quat = pinocchio.Quaternion(curva_rot[j+1][1][3],curva_rot[j+1][0][0],curva_rot[j+1][1][1],curva_rot[j+1][1][2])
    #end_quat = pinocchio.Quaternion(1.0,0.0,0.0,0.0)
    init_rot = init_quat.matrix()
    end_rot = end_quat.matrix()
    #ec2_rot[j] = SO3Linear(init_rot, end_rot, T0, T2)

    ec2_rot[j] = SO3Linear(init_rot, end_rot, T0, T2)


    #se3[j] = SE3Curve(ec2[j], ec2_rot[j])
    

    '''
    rot_se3[j] = se3[j].rotation_curve()
    quatttt = pinocchio.SE3(rot_se3[j].quaternion, np.zeros(3)) 
    #quatttt = pinocchio.SE3ToXYZQUAT(rot_se3[j], np.zeros(3))
    print("Value of the rotation matrix at initial time")
    print(rot_se3[j](T0))
    print("Value of the rotation matrix at midde time")
    print(rot_se3[j]((T0 + T2) / 2.0))
    print("Value of the rotation matrix at final time")

    print(rot_se3[j](T2))'''

    '''print(se3.derivate(T1, 1)[0:3])
    print(se3.derivate(T1, 1)[3:6])
    print(ec2[j].derivate(T1,1))'''
    #rot_quat_curve = pinocchio.SO3ToXYZQUAT(rot_se3[j])
    ##########################################################################################################################
    ##########################################################################################################################
    ##########################################################################################################################

    #Derivativeattimet
    res[j]=ec2[j].derivate(T0,1)#Equaltoinitvel
    res[j]=ec2[j].derivate(T0,2)#Equaltoinitacc
    res[j]=ec2[j].derivate(T2,1)#Equaltoendvel
    res[j]=ec2[j].derivate(T2,2)#Equaltoendacc

    steptime[j] = steptimes

    x[j]=np.zeros(steptime[j])
    y[j]=np.zeros(steptime[j])
    z[j]=np.zeros(steptime[j])

    x_vel[j]=np.zeros(steptime[j])
    y_vel[j]=np.zeros(steptime[j])
    z_vel[j]=np.zeros(steptime[j])

    x_acc[j]=np.zeros(steptime[j])
    y_acc[j]=np.zeros(steptime[j])
    z_acc[j]=np.zeros(steptime[j])

    t[j]=np.linspace(T0, T2, steptime[j])

    ###############################################

    x_rot[j]=np.zeros(steptime[j])
    y_rot[j]=np.zeros(steptime[j])
    z_rot[j]=np.zeros(steptime[j])
    w_rot[j]=np.zeros(steptime[j])


    #############################################

    for i in range(steptime[j]):
        x[j][i]=ec2[j](t[j][i])[0]
        y[j][i]=ec2[j](t[j][i])[1]
        z[j][i]=ec2[j](t[j][i])[2]

        # Getting values for rotation
        #x_rot[j]=se3[j](t[j][i])[3]

       # pse3[j][i] = se3[j].evaluateAsSE3(t[j][i])
       # pose[j][i] = pinocchio.SE3ToXYZQUAT(pse3[j][i])
        print("Desired pose")
        #print(pose[j])

        x_rot[j][i] = transform.Rotation.from_matrix(ec2_rot[j](t[j][i]))[2]
        y_rot[j][i] = 0
        z_rot[j][i] = 0
        w_rot[j][i] = 0
        print(ec2_rot[j](t[j][i]))
        print(x_rot[j])
        print(y_rot[j])
        print(z_rot[j])
        print(w_rot[j])



        x_vel[j][i]=ec2[j].derivate(t[j][i],1)[0]
        y_vel[j][i]=ec2[j].derivate(t[j][i],1)[1]
        z_vel[j][i]=ec2[j].derivate(t[j][i],1)[2]

        x_acc[j][i]=ec2[j].derivate(t[j][i],2)[0]
        y_acc[j][i]=ec2[j].derivate(t[j][i],2)[1]
        z_acc[j][i]=ec2[j].derivate(t[j][i],2)[2]


#print x
fig=plt.figure()
ax=plt.axes(projection ='3d')
plt.xlim(0,0.8)
plt.ylim(-0.3,0.1)
ax.set_zlim(0,0.06)

for i in range(0,n+1,1):
    ax.plot(x[i],y[i],z[i],label=str(i))

for i in range(0,n+1,1):
    ax.plot(icp_ref[i][0],icp_ref[i][1],icp_ref[i][2],label='icp_ref_'+str(i))


#ax.plot(icp_ref[1][0],icp_ref[1][1],icp_ref[1][2],label='icp_ref_1')

ax.legend()
plt.show()



# define state CoM
class CoM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'])
        #self.counter = 0
        self.n_step = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state CoM')
        if ((self.n_step < n+1) and (self.n_step % 2 == 0)):
            pub_icp_ref = rospy.Publisher('icp_ref', Pose, queue_size=10)
            mark_icp_ref = rospy.Publisher('mark_icp_ref', Marker, queue_size=10)
            
            pub_icp_ref_msg = Pose()
            
            
            for k in range(steptimes): # k can be seen as the varying time
                pub_icp_ref_msg.position.x = icp_ref[self.n_step][0][k] #icp_ref_x
                pub_icp_ref_msg.position.y = icp_ref[self.n_step][1][k] #icp_ref_y
                pub_icp_ref_msg.position.z = 0.75
                pub_icp_ref_msg.orientation.x = x_rot[self.n_step][k]
                pub_icp_ref_msg.orientation.y = y_rot[self.n_step][k]
                pub_icp_ref_msg.orientation.z = z_rot[self.n_step][k]
                pub_icp_ref_msg.orientation.w = w_rot[self.n_step][k]
                pub_icp_ref.publish(pub_icp_ref_msg)

                mark_icp_ref_msg = Marker()
                mark_icp_ref_msg.id = 2
                mark_icp_ref_msg.type = 2
                mark_icp_ref_msg.header.frame_id = "base_link"

                mark_icp_ref_msg.scale.x = 0.05
                mark_icp_ref_msg.scale.y = 0.05
                mark_icp_ref_msg.scale.z = 0.05
                mark_icp_ref_msg.color.r = 255.0
                mark_icp_ref_msg.color.g = 0.0
                mark_icp_ref_msg.color.b = 0.0
                mark_icp_ref_msg.color.a = 1.0
                mark_icp_ref_msg.header.stamp = rospy.get_rostime()

                mark_icp_ref_msg = Marker()
                mark_icp_ref_msg.id = 2
                mark_icp_ref_msg.type = 2
                mark_icp_ref_msg.header.frame_id = "base_link"

                mark_icp_ref_msg.scale.x = 0.05
                mark_icp_ref_msg.scale.y = 0.05
                mark_icp_ref_msg.scale.z = 0.05
                mark_icp_ref_msg.color.r = 255.0
                mark_icp_ref_msg.color.g = 0.0
                mark_icp_ref_msg.color.b = 0.0
                mark_icp_ref_msg.color.a = 1.0

                mark_icp_ref_msg.pose.position.x = icp_ref[self.n_step][0][k] #pos_left[0]
                mark_icp_ref_msg.pose.position.y = icp_ref[self.n_step][1][k] #pos_left[1]
                mark_icp_ref_msg.pose.position.z = 0.0 #pos_left[2]
                mark_icp_ref.publish(mark_icp_ref_msg)


                rospy.sleep(rospy_sleep_com)
            #self.counter += 1
            self.n_step += 1
            return 'outcome1'
        elif ((self.n_step < n+1) and (self.n_step % 2 == 1)):
            pub_icp_ref = rospy.Publisher('icp_ref', Pose, queue_size=10)
            mark_icp_ref = rospy.Publisher('mark_icp_ref', Marker, queue_size=10)
            
            pub_icp_ref_msg = Pose()
            
            
            for k in range(steptimes): # k can be seen as the varying time
                pub_icp_ref_msg.position.x = icp_ref[self.n_step][0][k] #icp_ref_x
                pub_icp_ref_msg.position.y = icp_ref[self.n_step][1][k] #icp_ref_y
                pub_icp_ref_msg.position.z = 0.75
                pub_icp_ref_msg.orientation.x = x_rot[self.n_step][k]
                pub_icp_ref_msg.orientation.y = y_rot[self.n_step][k]
                pub_icp_ref_msg.orientation.z = z_rot[self.n_step][k]
                pub_icp_ref_msg.orientation.w = w_rot[self.n_step][k]
                pub_icp_ref.publish(pub_icp_ref_msg)

                mark_icp_ref_msg = Marker()
                mark_icp_ref_msg.id = 2
                mark_icp_ref_msg.type = 2
                mark_icp_ref_msg.header.frame_id = "base_link"

                mark_icp_ref_msg.scale.x = 0.05
                mark_icp_ref_msg.scale.y = 0.05
                mark_icp_ref_msg.scale.z = 0.05
                mark_icp_ref_msg.color.r = 255.0
                mark_icp_ref_msg.color.g = 0.0
                mark_icp_ref_msg.color.b = 0.0
                mark_icp_ref_msg.color.a = 1.0
                mark_icp_ref_msg.header.stamp = rospy.get_rostime()

                mark_icp_ref_msg = Marker()
                mark_icp_ref_msg.id = 2
                mark_icp_ref_msg.type = 2
                mark_icp_ref_msg.header.frame_id = "base_link"

                mark_icp_ref_msg.scale.x = 0.05
                mark_icp_ref_msg.scale.y = 0.05
                mark_icp_ref_msg.scale.z = 0.05
                mark_icp_ref_msg.color.r = 255.0
                mark_icp_ref_msg.color.g = 0.0
                mark_icp_ref_msg.color.b = 0.0
                mark_icp_ref_msg.color.a = 1.0

                mark_icp_ref_msg.pose.position.x = icp_ref[self.n_step][0][k] #pos_left[0]
                mark_icp_ref_msg.pose.position.y = icp_ref[self.n_step][1][k] #pos_left[1]
                mark_icp_ref_msg.pose.position.z = 0.0 #pos_left[2]
                mark_icp_ref.publish(mark_icp_ref_msg)


                rospy.sleep(rospy_sleep_com)
            #self.counter += 1
            self.n_step += 1
            return 'outcome3'
        else:
            return 'outcome2'

    '''def execute(self, userdata):
        rospy.loginfo('Executing state CoM')
        if ((self.counter < steptimes) and (self.n_step < n-4)):
            pub_icp_ref = rospy.Publisher('icp_ref', Accel, queue_size=10)
            rospy.sleep(10)
            pub_icp_ref_msg = Accel()
            for i in range(0,n+1,1):
                for k in range(steptimes): # k can be seen as the varying time
                    pub_icp_ref_msg.linear.x = icp_ref[i][0][k] #icp_ref_x
                    pub_icp_ref_msg.linear.y = icp_ref[i][1][k] #icp_ref_y
                    pub_icp_ref_msg.linear.z = 0.0
                    pub_icp_ref_msg.angular.x = 0.0
                    pub_icp_ref_msg.angular.y = 0.0
                    pub_icp_ref_msg.angular.z = 0.0

                    pub_icp_ref.publish(pub_icp_ref_msg)
            self.counter += 1
            self.n_step += 1
            return 'outcome1'
        else:
            return 'outcome2' '''


# define state RightFoot
class RightFoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])
        self.right_curve = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state RightFoot')
        if self.right_curve < (n+1)/2:
            pub_multi_refence_right = rospy.Publisher('multi_reference_right', Pose, queue_size=10)
            mark_pos_right = rospy.Publisher('mark_pos_right', Marker, queue_size=10)
            
            mark_pos_right_msg = Marker()
            p_multi_refence_right = Pose()
            
            
            for k in range(steptimes): # k can be seen as the varying time
                #p[2*self.right_curve] = se3[j].evaluateAsSE3(t[2*self.right_curve][k])
                #pose[2*self.right_curve] = pinocchio.SE3ToXYZQUAT(p[2*self.right_curve])

                p_multi_refence_right.position.x = x[2*self.right_curve][k]
                p_multi_refence_right.position.y = y[2*self.right_curve][k]
                p_multi_refence_right.position.z = z[2*self.right_curve][k] 
                p_multi_refence_right.orientation.x = x_rot[2*self.right_curve][k] #pose[2*self.right_curve][k][3]
                p_multi_refence_right.orientation.y = y_rot[2*self.right_curve][k] #pose[2*self.right_curve][k][4]
                p_multi_refence_right.orientation.z = z_rot[2*self.right_curve][k] #pose[2*self.right_curve][k][5]
                p_multi_refence_right.orientation.w = w_rot[2*self.right_curve][k] #pose[2*self.right_curve][k][6]

                pub_multi_refence_right.publish(p_multi_refence_right)

                mark_pos_right_msg.id = 0
                mark_pos_right_msg.type = 2
                mark_pos_right_msg.header.frame_id = "base_link"

                mark_pos_right_msg.scale.x = 0.05
                mark_pos_right_msg.scale.y = 0.05
                mark_pos_right_msg.scale.z = 0.05
                mark_pos_right_msg.color.r = 0.0
                mark_pos_right_msg.color.g = 0.0
                mark_pos_right_msg.color.b = 1.0
                mark_pos_right_msg.color.a = 1.0

                mark_pos_right_msg.header.stamp = rospy.get_rostime()

                mark_pos_right_msg.pose.position.x = x[2*self.right_curve][k]
                mark_pos_right_msg.pose.position.y = y[2*self.right_curve][k]
                mark_pos_right_msg.pose.position.z = z[2*self.right_curve][k]

    

                mark_pos_right.publish(mark_pos_right_msg)


                rospy.sleep(rospy_sleep_rfoot)
            #self.counter += 1
            self.right_curve += 1
            return 'outcome2'
        else:
            return 'outcome2'

# define state LeftFoot
class LeftFoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])
        self.left_curve = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state RightFoot')
        if self.left_curve < (n+1)/2 - 1: # take out the -1 if you want to see the final swing phase, but for the moment it makes the robot fall
            pub_multi_refence_left = rospy.Publisher('multi_reference_left', Pose, queue_size=10)
            mark_pos_left = rospy.Publisher('mark_pos_left', Marker, queue_size=10)
            mark_pos_left_msg = Marker()
            p_multi_refence_left = Pose()
            
            for k in range(steptimes):
                #p[2*self.left_curve + 1] = se3[j].evaluateAsSE3(t[2*self.left_curve + 1][k])
                #pose[2*self.left_curve + 1] = pinocchio.SE3ToXYZQUAT(p[2*self.left_curve + 1])
                print("Desired pose")
                #print(pose[2*self.left_curve + 1])

                '''x_rot[2*self.left_curve + 1] = pose[2*self.left_curve + 1][3]
                y_rot[2*self.left_curve + 1] = pose[2*self.left_curve + 1][4]
                z_rot[2*self.left_curve + 1] = pose[2*self.left_curve + 1][5]
                w_rot[2*self.left_curve + 1] = pose[2*self.left_curve + 1][6]'''

                p_multi_refence_left.position.x = x[2*self.left_curve + 1][k]
                p_multi_refence_left.position.y = y[2*self.left_curve + 1][k]
                p_multi_refence_left.position.z = z[2*self.left_curve + 1][k] 
                p_multi_refence_left.orientation.x = x_rot[2*self.left_curve + 1][k] #pose[2*self.left_curve + 1][k][3]
                p_multi_refence_left.orientation.y = y_rot[2*self.left_curve + 1][k] #pose[2*self.left_curve + 1][k][4]
                p_multi_refence_left.orientation.z = z_rot[2*self.left_curve + 1][k] #pose[2*self.left_curve + 1][k][5]
                p_multi_refence_left.orientation.w = w_rot[2*self.left_curve + 1][k] #pose[2*self.left_curve + 1][k][6]
                pub_multi_refence_left.publish(p_multi_refence_left)

                print(x_rot[2*self.left_curve + 1][k])
                #print(pose[2*self.left_curve + 1][6])

                mark_pos_left_msg.id = 1
                mark_pos_left_msg.type = 2
                mark_pos_left_msg.header.frame_id = "base_link"

                mark_pos_left_msg.scale.x = 0.05
                mark_pos_left_msg.scale.y = 0.05
                mark_pos_left_msg.scale.z = 0.05
                mark_pos_left_msg.color.r = 0.0
                mark_pos_left_msg.color.g = 0.0
                mark_pos_left_msg.color.b = 1.0
                mark_pos_left_msg.color.a = 1.0

                mark_pos_left_msg.header.stamp = rospy.get_rostime()

                '''mark_pos_left_msg.pose.position.x = 0.2 #pos_left[0]
                mark_pos_left_msg.pose.position.y = 0.2 #pos_left[1]
                mark_pos_left_msg.pose.position.z = 0.2 #pos_left[2]'''

                mark_pos_left_msg.pose.position.x = x[2*self.left_curve + 1][k]
                mark_pos_left_msg.pose.position.y = y[2*self.left_curve + 1][k]
                mark_pos_left_msg.pose.position.z = z[2*self.left_curve + 1][k]
    
    
                mark_pos_left.publish(mark_pos_left_msg)
                rospy.sleep(rospy_sleep_lfoot)
                #self.counter += 1
            self.left_curve += 1
            return 'outcome2'
        else:
            return 'outcome2'
        



# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CoM', CoM(), 
                               transitions={'outcome1':'RightFoot','outcome3':'LeftFoot','outcome2':'outcome4'})
        smach.StateMachine.add('RightFoot', RightFoot(), 
                               transitions={'outcome2':'CoM'})
        smach.StateMachine.add('LeftFoot', LeftFoot(), 
                               transitions={'outcome2':'CoM'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
