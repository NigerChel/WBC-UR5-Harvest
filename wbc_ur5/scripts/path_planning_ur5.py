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
from pyquaternion import Quaternion

#######################################################################################
############################ IMPORTAR PARA PUBLICAR ###################################
#######################################################################################
import rospy
import rosbag
import pinocchio
import std_msgs.msg
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
    #Foot position
n = 1

steptimes = 10

p_sub = dict()
p_sub_media  = dict()
p_sub[0] = np.array([-0.255772825346, 0.1, 0.72])


p_sub_rot = dict()
p_sub_rot[0] = np.array([0.0, 0.0, 0.0, 1.0])

def contacts(msg):
    p_sub[0] = np.array([msg.contacts[0].pose.position.x, msg.contacts[0].pose.position.y, msg.contacts[0].pose.position.z])

    


##################################################################################
##################################################################################

# Target pose from vision system Eduardo 

def callback(msg4):
    print(msg4)

p_sub[1] = np.array([0, 0, -4]) # x y z components of the strawberry (position)
p_sub_rot[1] = np.array([0.0, 0.0, 0.0, 1.0]) # x y z w quaternion components of the strawberry (rotation)

def target_callback(msg5):
    p_sub[1] = np.array([msg5.position.x, msg5.position.y, msg5.position.z]) 
    print(p_sub[1])


    ################################################################################
    ################################################################################

'''def interpolation(cPosition,tPosition):
    x = {}
    y = {}
    z = {}


    trajectory.x = x
    trajectory.y = y
    trajectory.z = z
    return trajectory '''


# main
def main():
    rospy.init_node('smach_example_state_machine')

    pub_multi_refence_left = rospy.Publisher('multi_reference_left', Pose, queue_size=10)
    mark_pos_left = rospy.Publisher('mark_pos_left', Marker, queue_size=10)
    rospy.Subscriber('robot_states', WholeBodyState, contacts, queue_size=10)
    rospy.Subscriber('target_pose', Pose, target_callback, queue_size=10)
    mark_pos_left_msg = Marker()
    p_multi_refence_left = Pose()

    sub = rospy.Subscriber('pub', std_msgs.msg.Int16, callback)

    rospy.sleep(1.5)



    #p_sub[i+1] = np.array([msg2.markers[i+2].pose.position.x, msg2.markers[i+2].pose.position.y, msg2.markers[i+2].pose.position.z])


    for i in range(n):
        p_sub_media[i] = np.array([(p_sub[i][0] + p_sub[i+1][0])/2, (p_sub[i][1] + p_sub[i+1][1])/2, (p_sub[i][2] + p_sub[i+1][2])/2])
        #print(p_sub_media)

    curva={}

    for i in range(n):
        curva[i] = [[0, 0, 0],[0, 0, 0],[0, 0, 0]]

    #Defining the target position for left foot
    for i in range(n):
        curva[i][0] = p_sub[i]
        curva[i][1] = p_sub_media[i]
        curva[i][2] = p_sub[i+1] #np.array([(p_sub[2*i+1][0] + p_sub[2*(i+1)+1][0])/2, (p_sub[2*i+1][1] + p_sub[2*(i+1)+1][1])/2, 0.1])

    #################################################################################################


    curva_rot={}

    for i in range(n):
        curva_rot[i] = [[0, 0, 0, 0],[0, 0, 0, 0]]

    #Defining the target rotation for right foot
    for i in range(n):
        curva_rot[i][0] = p_sub_rot[i]
        curva_rot[i][1] = p_sub_rot[i+1]


    #################################################################################################
    #######################################################################################

    #Creation of exact cubic
    '''T0=0.0
    T1=1.0
    T2=2.0'''

    T0=0.0
    T1=0.9
    T2=1.8


    #######################################################################################
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

    for j in range(n):
        waypoints[j]=np.matrix(curva[j]).transpose()
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

        init_quat = pinocchio.Quaternion(curva_rot[j][0][3],curva_rot[j][0][0],curva_rot[j][0][1],curva_rot[j][0][2])
        #init_quat = pinocchio.Quaternion(0.38,0.0,0.0,0.92)
        end_quat = pinocchio.Quaternion(curva_rot[j][1][3],curva_rot[j][1][0],curva_rot[j][1][1],curva_rot[j][1][2])
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

            x_rot[j][i] = 0 # transform.Rotation.from_matrix(ec2_rot[j](t[j][i]))[2]
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
    plt.xlim(-5,5)
    plt.ylim(-5,5)
    ax.set_zlim(-5,5)

    for i in range(n):
        ax.plot(x[i],y[i],z[i],label=str(i))


    #ax.plot(icp_ref[1][0],icp_ref[1][1],icp_ref[1][2],label='icp_ref_1')

    ax.legend()
    plt.show()
    

    for k in range(steptimes):
        #p[2*self.left_curve + 1] = se3[j].evaluateAsSE3(t[2*self.left_curve + 1][k])
        #pose[2*self.left_curve + 1] = pinocchio.SE3ToXYZQUAT(p[2*self.left_curve + 1])
        print("Desired pose")
        #print(pose[2*self.left_curve + 1])

        '''x_rot[2*self.left_curve + 1] = pose[2*self.left_curve + 1][3]
        y_rot[2*self.left_curve + 1] = pose[2*self.left_curve + 1][4]
        z_rot[2*self.left_curve + 1] = pose[2*self.left_curve + 1][5]
        w_rot[2*self.left_curve + 1] = pose[2*self.left_curve + 1][6]'''

        p_multi_refence_left.position.x = x[0][k]
        p_multi_refence_left.position.y = y[0][k]
        p_multi_refence_left.position.z = z[0][k] 
        p_multi_refence_left.orientation.x = x_rot[0][k] #pose[2*self.left_curve + 1][k][3]
        p_multi_refence_left.orientation.y = y_rot[0][k] #pose[2*self.left_curve + 1][k][4]
        p_multi_refence_left.orientation.z = z_rot[0][k] #pose[2*self.left_curve + 1][k][5]
        p_multi_refence_left.orientation.w = w_rot[0][k] #pose[2*self.left_curve + 1][k][6]
        pub_multi_refence_left.publish(p_multi_refence_left)


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

        mark_pos_left_msg.pose.position.x = x[0][k]
        mark_pos_left_msg.pose.position.y = y[0][k]
        mark_pos_left_msg.pose.position.z = z[0][k]
    
    
        mark_pos_left.publish(mark_pos_left_msg)
        rospy.sleep(1.5)



if __name__ == '__main__':
    main()
