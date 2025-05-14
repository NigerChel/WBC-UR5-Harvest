#!/usr/bin/env python
import os
import unittest
from math import sqrt

import sys
sys.path.append("/home/niger/ndcurves/build/python/ndcurves")

import eigenpy
import numpy as np
from numpy import array, array_equal, isclose, random, zeros
from numpy.linalg import norm
import pickle

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

from ndcurves import exact_cubic, curve_constraints

#######################################################################################
############################ IMPORTAR PARA PUBLICAR ###################################
#######################################################################################
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
#######################################################################################
#######################################################################################
#######################################################################################

curva={}
curva[1]=[[0,-0.1,0],[0.16,-0.1,0.08],[0.32,-0.1,0]]
curva[2]=[[0,0.1,0],[0.34,0.1,0.08],[0.68,0.1,0]]
curva[3]=[curva[1][2],[curva[2][2][0],-0.1,0.08],[1.04,-0.1,0]]
curva[4]=[curva[2][2],[curva[3][2][0],0.1,0.08],[1.400,0.1,0]]
curva[5]=[curva[3][2],[curva[4][2][0],-0.1,0.08],[1.76,-0.1,0]]
curva[6]=[curva[4][2],[curva[5][2][0],0.1,0.08],[2.12,0.1,0]]
curva[7]=[curva[5][2],[curva[6][2][0],-0.1,0.08],[2.48,-0.1,0]]
curva[8]=[curva[6][2],[curva[7][2][0],0.1,0.08],[2.76,0.1,0]]
curva[9]=[curva[7][2],[curva[8][2][0],-0.1,0.08],[3,-0.1,0]]
curva[10]=[curva[8][2],[2.88,0.1,0.08],[3,0.1,0]]


#curva={}
#curva[1]=[[0,-0.1,0],[0.32,-0.1,0]]
#curva[2]=[[0,0.1,0],[0.68,0.1,0]]
#curva[3]=[curva[1][1],[1.04,-0.1,0]]
#curva[4]=[curva[2][1],[1.400,0.1,0]]
#curva[5]=[curva[3][1],[1.76,-0.1,0]]
#curva[6]=[curva[4][1],[2.12,0.1,0]]
#curva[7]=[curva[5][1],[2.48,-0.1,0]]
#curva[8]=[curva[6][1],[2.76,0.1,0]]
#curva[9]=[curva[7][1],[3,-0.1,0]]
#curva[10]=[curva[8][1],[3,0.1,0]]


#Creation of exact cubic
T0=0.0;
T1=1.0;
T2=2.0;

waypoints={}
timewaypoints={}
ec={}
NumberOfSplines={}
FirstSpline={}

res={}

Tmincheck={}

c={}
ec2={}
steptime={}

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

joint="Left"


numero=0

for j in range(10):
    waypoints[j]=np.matrix(curva[j+1]).transpose()
    timewaypoints[j]=np.matrix([T0,T1,T2]).transpose()
    ec[j]=exact_cubic(waypoints[j],timewaypoints[j])
    NumberOfSplines[j]=ec[j].getNumberSplines()#Get number of splines
    FirstSpline[j]=ec[j].getSplineAt(0)#Get first spline (polynomial)

    #Evaluationatt=0.5
    res[j]=ec[j](0.5)

    #Derivativeorder1att=0.5
    res[j]=ec[j].derivate(0.5,1)

    #Upperandlowerboundofdefinitioninterval
    Tmincheck[j]=ec[j].min()
    Tmincheck[j]=ec[j].max()

    #Creation of exact cubic with constraints
    c[j]=curve_constraints(3)
    c[j].init_vel=np.matrix([0.1,0.,0.1]).transpose()
    c[j].end_vel=np.matrix([0.,0.,0.]).transpose()
    c[j].init_acc=np.matrix([0.1,0.,0.1]).transpose()
    c[j].end_acc=np.matrix([0.,0.,0.]).transpose()
    ec2[j]=exact_cubic(waypoints[j],timewaypoints[j],c[j])
 
    #Derivativeattimet
    res[j]=ec2[j].derivate(T0,1)#Equaltoinitvel
    #print(res[j])
    res[j]=ec2[j].derivate(0.5,1)[0]#Equaltoinitvel#################
    #print(res[j])################################################
    res[j]=ec2[j].derivate(T0,2)#Equaltoinitacc
    #print(res[j])
    res[j]=ec2[j].derivate(T2,1)#Equaltoendvel
    #print(res[j])
    res[j]=ec2[j].derivate(T2,2)#Equaltoendacc
    #print(res[j])
    #print("-----------")

    steptime[j]=10

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


    for i in range(steptime[j]):
        x[j][i]=ec2[j](t[j][i])[0]
        y[j][i]=ec2[j](t[j][i])[1]
        z[j][i]=ec2[j](t[j][i])[2]

        x_vel[j][i]=ec2[j].derivate(t[j][i],1)[0]
        y_vel[j][i]=ec2[j].derivate(t[j][i],1)[1]
        z_vel[j][i]=ec2[j].derivate(t[j][i],1)[2]

        x_acc[j][i]=ec2[j].derivate(t[j][i],2)[0]
        y_acc[j][i]=ec2[j].derivate(t[j][i],2)[1]
        z_acc[j][i]=ec2[j].derivate(t[j][i],2)[2]



#print x
fig=plt.figure()
ax=plt.axes(projection ='3d')
plt.xlim(0,3)
plt.ylim(-0.3,0.3)
ax.set_zlim(0, 1)

ax.plot(x[0],y[0],z[0],label='1')
ax.plot(x[1],y[1],z[1],label='2')
ax.plot(x[2],y[2],z[2],label='3')
ax.plot(x[3],y[3],z[3],label='4')
ax.plot(x[4],y[4],z[4],label='5')
ax.plot(x[5],y[5],z[5],label='6')
ax.plot(x[6],y[6],z[6],label='7')
ax.plot(x[7],y[7],z[7],label='8')
ax.plot(x[8],y[8],z[8],label='9')
ax.plot(x[9],y[9],z[9],label='10')
ax.legend()
plt.show()




#######################################################################################
################################### CREAR PUBLISHER ###################################
#######################################################################################
def talker():
    pub_pos = rospy.Publisher('chatter_pos', Point, queue_size=10)
    pub_vel = rospy.Publisher('chatter_vel', Twist, queue_size=10)
    pub_acc = rospy.Publisher('chatter_acc', Accel, queue_size=10)
    pub_joint  = rospy.Publisher('chatter_joint', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    contador=1
   
    for g in range (10):
        for h in range (10):
            ###########################################################################
            #################### PUBLICA HASTA LLEGAR AL FINAL ########################
            ###########################################################################
            #while not rospy.is_shutdown():
                p=Point()
                p_vel=Twist()
                p_acc=Accel()

                p.x=x[g][h]
                p.y=y[g][h]
                p.z=z[g][h]
                
                p_vel.linear.x=x_vel[g][h]
                p_vel.linear.y=y_vel[g][h]
                p_vel.linear.z=z_vel[g][h]
                p_vel.angular.x=0
                p_vel.angular.y=0
                p_vel.angular.z=0

                p_acc.linear.x=x_acc[g][h]
                p_acc.linear.y=y_acc[g][h]
                p_acc.linear.z=z_acc[g][h]
                p_acc.angular.x=0
                p_acc.angular.y=0
                p_acc.angular.z=0


                while(pub_pos.get_num_connections() == 0):
                    rate.sleep()

                pub_pos.publish(p)
                pub_vel.publish(p_vel)
                pub_acc.publish(p_acc)

                equis="Coordenada x: %f" %p.x
                ye   ="Coordenada y: %f" %p.y
                zeta ="Coordenada z: %f \n" %p.z

                equis_vel="Velocidad x: %f" %p_vel.linear.x
                ye_vel   ="Velocidad y: %f" %p_vel.linear.y
                zeta_vel ="Velocidad z: %f \n" %p_vel.linear.z

                equis_acc="Aceleracion x: %f" %p_acc.linear.x
                ye_acc   ="Aceleracion y: %f" %p_acc.linear.y
                zeta_acc ="Aceleracion z: %f \n" %p_acc.linear.z

                print(contador)
                
                global joint
            
                if((contador%10==1) and (joint=="Right")):
                    joint="Left"

                elif ((contador%10==1) and (joint=="Left")):
                    joint="Right"
                
                print(joint)
                pub_joint.publish(joint)

                rospy.loginfo(equis)
                rospy.loginfo(ye)
                rospy.loginfo(zeta)

                rospy.loginfo(equis_vel)
                rospy.loginfo(ye_vel)
                rospy.loginfo(zeta_vel)

                rospy.loginfo(equis_acc)
                rospy.loginfo(ye_acc)
                rospy.loginfo(zeta_acc)

                contador=contador+1

                rate.sleep()
            ###########################################################################
            ###########################################################################
            ###########################################################################

    ###################################################################################
    ############################ PUBLICA COORDENADAS FINALES ##########################
    ###################################################################################
    while not rospy.is_shutdown():

        if((contador%10==1) and (joint=="Right")):
            joint="Left"
            g=10
            
        elif ((contador%10==1) and (joint=="Left")):
            joint="Right"
            g=9

        p=Point()
        p_vel=Twist()
        p_acc=Accel()
        
        print(g-1)

        p.x=x[g-1][h]
        p.y=y[g-1][h]
        p.z=z[g-1][h]
        
        p_vel.linear.x=0 #x_vel[g][h]
        p_vel.linear.y=0 #y_vel[g][h]
        p_vel.linear.z=0 #z_vel[g][h]
        p_vel.angular.x=0
        p_vel.angular.y=0
        p_vel.angular.z=0

        p_acc.linear.x=0 #x_acc[g][h]
        p_acc.linear.y=0 #y_acc[g][h]
        p_acc.linear.z=0 #z_acc[g][h]
        p_acc.angular.x=0
        p_acc.angular.y=0
        p_acc.angular.z=0

        while(pub_pos.get_num_connections() == 0):
            rate.sleep()
        
        pub_pos.publish(p)
        pub_vel.publish(p_vel)
        pub_acc.publish(p_acc)

        equis="Coordenada x: %f" %p.x
        ye   ="Coordenada y: %f" %p.y
        zeta ="Coordenada z: %f \n" %p.z

        equis_vel="Velocidad x: %f" %p_vel.linear.x
        ye_vel   ="Velocidad y: %f" %p_vel.linear.y
        zeta_vel ="Velocidad z: %f \n" %p_vel.linear.z

        equis_acc="Aceleracion x: %f" %p_acc.linear.x
        ye_acc   ="Aceleracion y: %f" %p_acc.linear.y
        zeta_acc ="Aceleracion z: %f \n" %p_acc.linear.z

        print(contador)
        
        '''
        if((contador%10==1) and (joint=="Right")):
            joint="Left"
            g=10
            
        elif ((contador%10==1) and (joint=="Left")):
            joint="Right"
            g=9
        '''

        print(joint)
        pub_joint.publish(joint)

        rospy.loginfo(equis)
        rospy.loginfo(ye)
        rospy.loginfo(zeta)

        rospy.loginfo(equis_vel)
        rospy.loginfo(ye_vel)
        rospy.loginfo(zeta_vel)

        rospy.loginfo(equis_acc)
        rospy.loginfo(ye_acc)
        rospy.loginfo(zeta_acc)

        contador=contador+1

        rate.sleep()
    ###################################################################################
    ###################################################################################
    ###################################################################################
      
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#######################################################################################
#######################################################################################
#######################################################################################


