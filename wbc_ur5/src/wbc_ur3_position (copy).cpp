/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov. Licensed under the Apache License,
    Version 2.0. (see LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iostream>
#include <sstream>
#include <Eigen/Eigen>
#include <qpmad/solver.h>

//Standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/skew.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "ros/ros.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>

#include <whole_body_state_msgs/WholeBodyState.h>
#include <whole_body_state_conversions/whole_body_state_interface.h>

//Headers
#include <whole_body_state_msgs/JointState.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>

#include <yaml-cpp/yaml.h>

using namespace Eigen;
using namespace pinocchio;	

//typedef Matrix<double, 30, 1> tau;
 //typedef Matrix<float, 30, 1> tau_des;


//Robot articulations
whole_body_state_msgs::WholeBodyState r_joints;
std::vector<std::string> art_names; 
int art_counter;

//Node configuration
std::string node_path;
int mode;

//Init publishers
ros::Publisher effort_pub; 

/*double x1_clb_1 = 0.0;
double x1_clb_2 = 0.0;
double x1_clb_3 {}; //= 0.0;

double x2_clb_1 = 0.0;
double x2_clb_2 = 0.0;
double x2_clb_3 = 0.0;

int switch1 = 0;

void left_foot_subs(const geometry_msgs::Point &msg1)
		{
			switch1 = 1;
			x1_clb_3 = msg1.z;
			
		}*/
		
double d1_clb_0 {};
double d1_clb_1 {};
double d1_clb_2 {}; //= 0.0

double d1_clb_3 {};
double d1_clb_4 {};
double d1_clb_5 {}; //= 0.0;;
double d1_clb_6 {};

double x2_clb_1 = 0.0;
double x2_clb_2 = 0.0;
double x2_clb_3 = 0.0;

int switch1 = 0;

double d2_clb_0 {};
double d2_clb_1 {};
double d2_clb_2 {}; //= 0.0

double d2_clb_3 {};
double d2_clb_4 {};
double d2_clb_5 {}; //= 0.0;;
double d2_clb_6 {}; 

/*double x2_clb_1 = 0.0;
double x2_clb_2 = 0.0;
double x2_clb_3 = 0.0;*/

int switch2 = 0;

double d3_clb_0 {};
double d3_clb_1 {};
double d3_clb_2 {}; //= 0.0

double d3_clb_3 {};
double d3_clb_4 {};
double d3_clb_5 {}; //= 0.0;;
double d3_clb_6 {};

/*double x2_clb_1 = 0.0;
double x2_clb_2 = 0.0;
double x2_clb_3 = 0.0;*/

int switch3 = 0;

void left_foot_subs(const geometry_msgs::Pose &msg1)
		{
			switch1 = 1;
			d1_clb_0 = msg1.position.x;
			d1_clb_1 = msg1.position.y;
			d1_clb_2 = msg1.position.z;
			d1_clb_3 = msg1.orientation.x;
			d1_clb_4 = msg1.orientation.y;
			d1_clb_5 = msg1.orientation.z;
			d1_clb_6 = msg1.orientation.w;
			
		}
		
void right_foot_subs(const geometry_msgs::Pose &msg2)
		{
			switch2 = 1;
			d2_clb_0 = msg2.position.x;
			d2_clb_1 = msg2.position.y;
			d2_clb_2 = msg2.position.z;
			d2_clb_3 = msg2.orientation.x;
			d2_clb_4 = msg2.orientation.y;
			d2_clb_5 = msg2.orientation.z;
			d2_clb_6 = msg2.orientation.w;
		}
		
void icp_ref_subs(const geometry_msgs::Pose &msg3)
		{
			switch3 = 1;
			d3_clb_0 = msg3.position.x;
			d3_clb_1 = msg3.position.y;
			d3_clb_2 = msg3.position.z;
			d3_clb_3 = msg3.orientation.x;
			d3_clb_4 = msg3.orientation.y;
			d3_clb_5 = msg3.orientation.z;
			d3_clb_6 = msg3.orientation.w;
		}
////////////////////////////////////////////////////////////
//class server {
//	public:
	
 void robot_state_callback(const whole_body_state_msgs::WholeBodyState &msg)
  {
	// You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename = "/home/niger/reemc_public_ws/src/reemc_simulation/reemc_gazebo/models/reemc_full/reemc_full_ft_hey5.urdf.urdf";
  
  // Load the urdf model
  Model model;
	pinocchio::JointModelFreeFlyer root_joint;
  pinocchio::urdf::buildModel(urdf_filename, root_joint, model);
	
	// Create data required by the algorithms
  Data data(model);
  
  Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  
  //////////////////////////////////
  // Retrieve the generalized position and velocity, and joint torques
  	/*q(3) = msg.centroidal.base_orientation.w;
    q(4) = msg.centroidal.base_orientation.x;
    q(5) = msg.centroidal.base_orientation.y;
    q(6) = msg.centroidal.base_orientation.z;*/
    
    q(3) = msg.centroidal.base_orientation.x;
    q(4) = msg.centroidal.base_orientation.y;
    q(5) = msg.centroidal.base_orientation.z;
    q(6) = msg.centroidal.base_orientation.w;
    
    v(3) = msg.centroidal.base_angular_velocity.x;
    v(4) = msg.centroidal.base_angular_velocity.y;
    v(5) = msg.centroidal.base_angular_velocity.z;

    for (std::size_t j = 0; j < msg.joints.size(); ++j) {
      // TODO: Generalize to different floating-base types!
      // TODO: Check if joint exists!
      auto jointId = model.getJointId(msg.joints[j].name) - 2;
      q(jointId + 7) = msg.joints[j].position;
      v(jointId + 6) = msg.joints[j].velocity;
      tau(jointId) = msg.joints[j].effort;
    }
    pinocchio::normalize(model, q);
    pinocchio::centerOfMass(model, data, q, v);
    q(0) = msg.centroidal.com_position.x - data.com[0](0);
    q(1) = msg.centroidal.com_position.y - data.com[0](1);
    q(2) = msg.centroidal.com_position.z - data.com[0](2);
    v(0) = msg.centroidal.com_velocity.x - data.vcom[0](0);
    v(1) = msg.centroidal.com_velocity.y - data.vcom[0](1);
    v(2) = msg.centroidal.com_velocity.z - data.vcom[0](2);
    
    v.head<3>() = Eigen::Quaterniond(q(6), q(3), q(4), q(5)).toRotationMatrix().transpose() * v.head<3>();  // local frame
    
    // Retrieve the contact information
    std::map<std::string, whole_body_state_conversions::ContactState> contacts;
    for (const auto &contact : msg.contacts) {
      // Contact pose
      contacts[contact.name].position =
          pinocchio::SE3(Eigen::Quaterniond(contact.pose.orientation.w, contact.pose.orientation.x,
                                            contact.pose.orientation.y, contact.pose.orientation.z),
                         Eigen::Vector3d(contact.pose.position.x, contact.pose.position.y, contact.pose.position.z));
      // Contact velocity
      contacts[contact.name].velocity = pinocchio::Motion(
          Eigen::Vector3d(contact.velocity.linear.x, contact.velocity.linear.y, contact.velocity.linear.z),
          Eigen::Vector3d(contact.velocity.angular.x, contact.velocity.angular.y, contact.velocity.angular.z));
      // Contact wrench
      contacts[contact.name].force =
          pinocchio::Force(Eigen::Vector3d(contact.wrench.force.x, contact.wrench.force.y, contact.wrench.force.z),
                           Eigen::Vector3d(contact.wrench.torque.x, contact.wrench.torque.y, contact.wrench.torque.z));
      // Surface normal and friction coefficient
      contacts[contact.name].surface_normal.x() = contact.surface_normal.x;
      contacts[contact.name].surface_normal.y() = contact.surface_normal.y;
      contacts[contact.name].surface_normal.z() = contact.surface_normal.z;
      contacts[contact.name].surface_friction = contact.friction_coefficient;
    }
    
		// Declare canonical QP elements
    Eigen::VectorXd         x;
    Eigen::MatrixXd         H;
    Eigen::VectorXd         h;
    Eigen::MatrixXd         A;
    Eigen::VectorXd         Alb;
    Eigen::VectorXd         Aub;
    Eigen::VectorXd         lb;
    Eigen::VectorXd         ub;
		
		int dof = model.nv; //Number of actuated joints plus 6-dof floating base
		int c = 2; //Number of contacs
		int m = 4; //Number of basis of friction cone
		
		qpmad::MatrixIndex size = dof + m*c; // x dimension
    qpmad::MatrixIndex num_ctr = 6 + 30 + 30 + c*m + 3*c; // Number of constraints that will be taken in the QP plus 6*c to take into acccount the rigid contacts
    
		//Read YAML file
	  YAML::Node conf = YAML::LoadFile("/home/niger/reemc_public_ws/src/wbc/config/param_position_control_wto_constrain14.yaml"); 
		/// Weights for QP ///
		double w1 = conf["qp_weight"]["weights"]["w1"].as<double>(); //10.0; //1000.0; 
		double w2 = conf["qp_weight"]["weights"]["w2"].as<double>(); //1000.0;
		double w3 = conf["qp_weight"]["weights"]["w3"].as<double>(); //500.0;
		double w4 = conf["qp_weight"]["weights"]["w4"].as<double>();
		double w5 = conf["qp_weight"]["weights"]["w5"].as<double>();
		double w6 = conf["qp_weight"]["weights"]["w6"].as<double>();
		double dt = conf["torque_control"]["gains"]["dt"].as<double>();
		
		/*double w1 = 10.0; //10.0; //1000.0; 
		double w2 = 10.0; //1000.0;
		double w3 = 200.0; //500.0;
		double w4 = 200.0;*/
		
		double gamma1 = conf["qp_weight"]["weights"]["gamma1"].as<double>();
		double gamma2 = conf["qp_weight"]["weights"]["gamma2"].as<double>();
		
		// Selection matrix
		Eigen::MatrixXd S_l = Eigen::MatrixXd::Zero(3,6);
    S_l.block<3,3>(0,0).setIdentity(3,3);
    Eigen::MatrixXd S_a = Eigen::MatrixXd::Zero(3,6);
    S_a.block<3,3>(0,3).setIdentity(3,3);
    
    Eigen::MatrixXd S_act_ddq = Eigen::MatrixXd::Zero(model.nv-6,model.nv); // Select the actuated part of all joints of the humanoid from the joint acceleration
    S_act_ddq.block<30,30>(0,6).setIdentity(model.nv-6,model.nv-6);
		
		 //
		////////////// Tasks (Ji*dv = di) ///////////////
		
		/* Task 1 (Spatial acceleration for left foot pose)*/
		// Get frame_id for left_sole_link
  	std::string FRAME_ID_LEFT = "left_sole_link"; //left_sole_link
  	pinocchio::FrameIndex frame_id_left = model.getFrameId(FRAME_ID_LEFT);
  	
  	// Get frame Jacobian 
  	pinocchio::Data::Matrix6x J1(6,model.nv);
  	J1.setZero();
  	pinocchio::computeJointJacobians(model, data, q); // this also compute the forward kinematics for joints
		pinocchio::framesForwardKinematics(model, data, q); // this update the frames placement
		pinocchio::getFrameJacobian(model, data, frame_id_left, pinocchio::LOCAL_WORLD_ALIGNED, J1);
		
		//Desired task
		Eigen::MatrixXd d1; 
		d1.setConstant(6,1,0);
		
		// Gains PD
		double k1_px = conf["leg_left"]["gains"]["px"].as<double>(); //50.0;
		double k1_py = conf["leg_left"]["gains"]["py"].as<double>();
		double k1_pz = conf["leg_left"]["gains"]["pz"].as<double>();
    double k1_dx = 2.0*sqrt(k1_px);
    double k1_dy = 2.0*sqrt(k1_py);
    double k1_dz = 2.0*sqrt(k1_pz);
    
    /*// Desired position
    double x1 = 0.00429;
    double y1 = 0.07609;
    double z1 = -0.00209;*/
    
    /*/ Desired position
    double x1 = 0.17;
    double y1 = 0.12;
    double z1 = 0.3;*/
    //Read YAML file

	  // Desired position
		double x1 = conf["leg_left"]["position_ref"]["x"].as<double>(); //-0.019774711256;
		double y1 = conf["leg_left"]["position_ref"]["y"].as<double>(); //0.0904920792963; 
    double z1 = conf["leg_left"]["position_ref"]["z"].as<double>();
    // ros::NodeHandle nh;
   // std::string s;
   
     /*  if (switch1 == true)
        {
          z1 = x1_clb_3;
       } else {
         z1 = conf["leg_left"]["position_ref"]["z"].as<double>();
       }*/
       
    /* x: 0.00429689194564
        y: 0.0760919763934
        z: -0.00209538474092
		*/
		pinocchio::forwardKinematics(model,data,q,v,0*v);
		pinocchio::computeJointJacobians(model,data,q);
    pinocchio::framesForwardKinematics(model,data,q);
		
		const pinocchio::Motion &a_drift_leg_left = pinocchio::getFrameClassicalAcceleration(model, data, frame_id_left, pinocchio::LOCAL_WORLD_ALIGNED);
		
		bool lin_planning1_x = conf["leg_left"]["path_planning"]["lin_x"].as<bool>();
		bool lin_planning1_y = conf["leg_left"]["path_planning"]["lin_y"].as<bool>();
		bool lin_planning1_z = conf["leg_left"]["path_planning"]["lin_z"].as<bool>();
		
		  if (switch1 == true && lin_planning1_x == true)
        {
        d1(0) = - k1_dx*contacts[FRAME_ID_LEFT].velocity.linear().x() + k1_px*(d1_clb_0-contacts[FRAME_ID_LEFT].position.translation().x()) - a_drift_leg_left.linear().x();
       } else {
       
       d1(0) = - k1_dx*contacts[FRAME_ID_LEFT].velocity.linear().x() + k1_px*(x1-contacts[FRAME_ID_LEFT].position.translation().x()) - a_drift_leg_left.linear().x();
       }
       
       if (switch1 == true && lin_planning1_y == true)
        {
        d1(1) = - k1_dy*contacts[FRAME_ID_LEFT].velocity.linear().y() + k1_py*(d1_clb_1-contacts[FRAME_ID_LEFT].position.translation().y()) - a_drift_leg_left.linear().y();
       } else {
       
		d1(1) = - k1_dy*contacts[FRAME_ID_LEFT].velocity.linear().y() + k1_py*(y1-contacts[FRAME_ID_LEFT].position.translation().y()) - a_drift_leg_left.linear().y();
       }
       
       if (switch1 == true && lin_planning1_z == true)
        {
          d1(2) = - k1_dz*contacts[FRAME_ID_LEFT].velocity.linear().z() + k1_pz*(d1_clb_2-contacts[FRAME_ID_LEFT].position.translation().z()) - a_drift_leg_left.linear().z();
       } else {
       
		 d1(2) = - k1_dz*contacts[FRAME_ID_LEFT].velocity.linear().z() + k1_pz*(z1-contacts[FRAME_ID_LEFT].position.translation().z()) - a_drift_leg_left.linear().z();
       }
    
   /* // Get the difference between a reference rotation and the state rotation under log function
		//pinocchio::Quaternion quat_ref_2(1, 0, 0, 0)
		// const pinocchio::SE3::Quaternion quat_ref_1(Eigen::Quaterniond(1, 0, 0, 0));// Eigen::Quaterniond(1, 0, 0, 0)
		Matrix3d rot_ref_1 = Matrix3d::Identity();
		Matrix3d rot_diff_1 = rot_ref_1 * contacts[FRAME_ID_LEFT].position.rotation().transpose();
		Vector3d log3_1 = pinocchio::log3(rot_diff_1);
		
		d1(3) = - k1_d*contacts[FRAME_ID_LEFT].velocity.angular().x() + k1_p*(log3_1(0)) - a_drift_leg_left.angular().x();
		d1(4) = - k1_d*contacts[FRAME_ID_LEFT].velocity.angular().y() + k1_p*(log3_1(1)) - a_drift_leg_left.angular().y();
		d1(5) = - k1_d*contacts[FRAME_ID_LEFT].velocity.angular().z() + k1_p*(log3_1(2)) - a_drift_leg_left.angular().z();*/
		
		// Desired orientation
    double orientation_w1 = conf["leg_left"]["orientation_ref"]["w"].as<double>();
		double orientation_x1 = conf["leg_left"]["orientation_ref"]["x"].as<double>(); //-0.019774711256;
		double orientation_y1 = conf["leg_left"]["orientation_ref"]["y"].as<double>(); //0.0904920792963; 
    double orientation_z1 = conf["leg_left"]["orientation_ref"]["z"].as<double>();
    
    const pinocchio::SE3 left_foot_ref = pinocchio::SE3(Eigen::Quaterniond(orientation_w1, orientation_x1, orientation_y1, orientation_z1), Eigen::Vector3d(0, 0, 0));
    
    // Get the difference between a reference rotation and the state rotation under log function
		//pinocchio::Quaternion quat_ref_2(1, 0, 0, 0);
		
		/*const pinocchio::SE3::Quaternion quat_ref_1(Eigen::Quaterniond(1, 0, 0, 0));// Eigen::Quaterniond(1, 0, 0, 0);
		std::cout << "Normalizes quaternion pin" << quat_ref_2 << std::endl;*/
		
		Matrix3d rot_ref_1 = left_foot_ref.rotation(); //Matrix3d::Identity();
		// Matrix3d rot_diff_1 = rot_ref_1 * contacts[FRAME_ID_LEFT].position.rotation().transpose();
		Matrix3d rot_diff_1 = contacts[FRAME_ID_LEFT].position.rotation() * rot_ref_1.transpose();
		Vector3d log3_1 = pinocchio::log3(rot_diff_1);
		
		double k1_ptheta1 = conf["leg_left"]["gains"]["ptheta1"].as<double>(); //50.0;
		double k1_ptheta2 = conf["leg_left"]["gains"]["ptheta2"].as<double>();
		double k1_ptheta3 = conf["leg_left"]["gains"]["ptheta3"].as<double>();
    double k1_dtheta1 = 2.0*sqrt(k1_ptheta1);
    double k1_dtheta2 = 2.0*sqrt(k1_ptheta2);
    double k1_dtheta3 = 2.0*sqrt(k1_ptheta3);
		
		/*double k1_p_ang = conf["leg_left"]["gains"]["p_angular"].as<double>(); //50.0;
    double k1_d_ang = 2.0*sqrt(k1_p_ang);*/
		
		/*d1(3) = - k1_dtheta1*contacts[FRAME_ID_LEFT].velocity.angular().x() + k1_ptheta1*(log3_1(0)) - a_drift_leg_left.angular().x();
		d1(4) = - k1_dtheta2*contacts[FRAME_ID_LEFT].velocity.angular().y() + k1_ptheta2*(log3_1(1)) - a_drift_leg_left.angular().y();
		d1(5) = - k1_dtheta3*contacts[FRAME_ID_LEFT].velocity.angular().z() + k1_ptheta3*(log3_1(2)) - a_drift_leg_left.angular().z();*/
		
		/*****d1(3) = - k1_dtheta1*contacts[FRAME_ID_LEFT].velocity.angular().x() - k1_ptheta1*(log3_1(0)) - a_drift_leg_left.angular().x();
		d1(4) = - k1_dtheta2*contacts[FRAME_ID_LEFT].velocity.angular().y() - k1_ptheta2*(log3_1(1)) - a_drift_leg_left.angular().y();
		d1(5) = - k1_dtheta3*contacts[FRAME_ID_LEFT].velocity.angular().z() - k1_ptheta3*(log3_1(2)) - a_drift_leg_left.angular().z();****/
		
  ////////////////////////////////////////////////////////////////////////////////
  
  const pinocchio::SE3 left_foot_ref_clb = pinocchio::SE3(Eigen::Quaterniond(d1_clb_6, d1_clb_3, d1_clb_4, d1_clb_5), Eigen::Vector3d(0, 0, 0));
  
  Matrix3d rot_ref_1_clb = left_foot_ref_clb.rotation(); 
		Matrix3d rot_diff_1_clb= contacts[FRAME_ID_LEFT].position.rotation() * rot_ref_1_clb.transpose();
		Vector3d log3_1_clb = pinocchio::log3(rot_diff_1_clb);
		
		bool ang_planning1_x = conf["leg_left"]["path_planning"]["ang_x"].as<bool>();
		bool ang_planning1_y = conf["leg_left"]["path_planning"]["ang_y"].as<bool>();
		bool ang_planning1_z = conf["leg_left"]["path_planning"]["ang_z"].as<bool>();
  
  if (switch1 == true && ang_planning1_x == true)
        {
        d1(3) = - k1_dtheta1*contacts[FRAME_ID_LEFT].velocity.angular().x() - k1_ptheta1*(log3_1_clb(0)) - a_drift_leg_left.angular().x();
       } else {
       
       d1(3) = - k1_dtheta1*contacts[FRAME_ID_LEFT].velocity.angular().x() - k1_ptheta1*(log3_1(0)) - a_drift_leg_left.angular().x();
       }
       
       if (switch1 == true && ang_planning1_y == true)
        {
        d1(4) = - k1_dtheta2*contacts[FRAME_ID_LEFT].velocity.angular().y() - k1_ptheta2*(log3_1_clb(1)) - a_drift_leg_left.angular().y();
       } else {
       
		d1(4) = - k1_dtheta2*contacts[FRAME_ID_LEFT].velocity.angular().y() - k1_ptheta2*(log3_1(1)) - a_drift_leg_left.angular().y();
       }
       
       if (switch1 == true && ang_planning1_z == true)
        {
         d1(5) = - k1_dtheta3*contacts[FRAME_ID_LEFT].velocity.angular().z() - k1_ptheta3*(log3_1_clb(2)) - a_drift_leg_left.angular().z();
       } else {
       
		 d1(5) = - k1_dtheta3*contacts[FRAME_ID_LEFT].velocity.angular().z() - k1_ptheta3*(log3_1(2)) - a_drift_leg_left.angular().z();
       }
       
    //////////////////////////////////////////////////////////////////////////
		
		bool S1_linear = conf["leg_left"]["selection_matrix"]["linear"].as<bool>();
    bool S1_angular = conf["leg_left"]["selection_matrix"]["angular"].as<bool>();
    
    /*// Selection matrix for getting linear or angular task
    Eigen::MatrixXd S1 = Eigen::MatrixXd::Zero(6,6);
    
    if (S1_linear == true && S1_angular == true) {
    S1.block<3,3>(0,0).setIdentity(3,3);
    S1.block<3,3>(3,3).setIdentity(3,3);
    } else if (S1_linear == true && S1_angular == false) {
    S1.block<3,3>(0,0).setIdentity(3,3);
    } else if (S1_linear == false && S1_angular == true) {
    S1.block<3,3>(3,3).setIdentity(3,3);
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular hip control in the yaml file!!! ");
    }*/
    
     // Selection matrix for getting linear or angular task
    Eigen::MatrixXd S1 = Eigen::MatrixXd::Identity(6,6);
    
    if (S1_linear == true && S1_angular == true) {
    Eigen::MatrixXd S1 = Eigen::MatrixXd::Identity(6,6);
    } else if (S1_linear == true && S1_angular == false) {
    S1 = S_l;
    } else if (S1_linear == false && S1_angular == true) {
    S1 = S_a;
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular com control in the yaml file!!! ");
    }
    
    /* Task 2 (Spatial acceleration for right foot pose)*/
    // Get frame_id for right_sole_link
    std::string FRAME_ID_RIGHT = "right_sole_link"; //left_sole_link
  	pinocchio::FrameIndex frame_id_right = model.getFrameId(FRAME_ID_RIGHT);
  	
  	// Get frame Jacobian 
  	pinocchio::Data::Matrix6x J2(6,model.nv);
  	J2.setZero();
  	pinocchio::computeJointJacobians(model, data, q);   // this also compute the forward kinematics for joints
		pinocchio::framesForwardKinematics(model, data, q); // this update the frames placement
		pinocchio::getFrameJacobian(model, data, frame_id_right, pinocchio::LOCAL_WORLD_ALIGNED, J2);
		
 		//Desired task
		Eigen::MatrixXd d2; 
		d2.setConstant(6,1,0);
    
    // Gains PD
		/*double k2_p = conf["leg_right"]["gains"]["p"].as<double>();//50.0;
    double k2_d = 2.0*sqrt(k2_p);*/
    
    double k2_px = conf["leg_right"]["gains"]["px"].as<double>(); //50.0;
		double k2_py = conf["leg_right"]["gains"]["py"].as<double>();
		double k2_pz = conf["leg_right"]["gains"]["pz"].as<double>();
    double k2_dx = 2.0*sqrt(k2_px);
    double k2_dy = 2.0*sqrt(k2_py);
    double k2_dz = 2.0*sqrt(k2_pz);
    
    /*// Desired position
    double x2 = 0.00432;
    double y2 = -0.07558;
    double z2 = -0.00209;*/
    
    /*// Desired position
    double x2 = 0.17;
    double y2 = -0.12;
    double z2 = 0.3;*/
    
    // Desired position
    double x2 = conf["leg_right"]["position_ref"]["x"].as<double>(); //-0.0203258336221;
    double y2 = conf["leg_right"]["position_ref"]["y"].as<double>(); //-0.0621026780498;
    double z2 = conf["leg_right"]["position_ref"]["z"].as<double>(); //0.4;
   
    //z2 = x2_clb_3;
    /*
        x: 0.00432559746305
        y: -0.0755890513575
        z: -0.00209827029282

		*/
		pinocchio::forwardKinematics(model,data,q,v,0*v);
		pinocchio::computeJointJacobians(model,data,q);
    pinocchio::framesForwardKinematics(model,data,q);
		
		const pinocchio::Motion &a_drift_leg_right = pinocchio::getFrameClassicalAcceleration(model, data, frame_id_right, pinocchio::LOCAL_WORLD_ALIGNED);
		
		bool lin_planning2_x = conf["leg_right"]["path_planning"]["lin_x"].as<bool>();
		bool lin_planning2_y = conf["leg_right"]["path_planning"]["lin_y"].as<bool>();
		bool lin_planning2_z = conf["leg_right"]["path_planning"]["lin_z"].as<bool>();
		
		 if (switch2 == true && lin_planning2_x == true)
        {
        
        d2(0) = - k2_dx*contacts[FRAME_ID_RIGHT].velocity.linear().x() + k2_px*(d2_clb_0-contacts[FRAME_ID_RIGHT].position.translation().x()) - a_drift_leg_right.linear().x();
		
       } else {
       
       d2(0) = - k2_dx*contacts[FRAME_ID_RIGHT].velocity.linear().x() + k2_px*(x2-contacts[FRAME_ID_RIGHT].position.translation().x()) - a_drift_leg_right.linear().x();
		
       }
       
       if (switch2 == true && lin_planning2_y == true)
        {
		d2(1) = - k2_dy*contacts[FRAME_ID_RIGHT].velocity.linear().y() + k2_py*(d2_clb_1-contacts[FRAME_ID_RIGHT].position.translation().y()) - a_drift_leg_right.linear().y();
		
       } else {
		
		d2(1) = - k2_dy*contacts[FRAME_ID_RIGHT].velocity.linear().y() + k2_py*(y2-contacts[FRAME_ID_RIGHT].position.translation().y()) - a_drift_leg_right.linear().y();
       }
       
       if (switch2 == true && lin_planning2_z == true)
        {
          d2(2) = - k2_dz*contacts[FRAME_ID_RIGHT].velocity.linear().z() + k2_pz*(d2_clb_2-contacts[FRAME_ID_RIGHT].position.translation().z()) - a_drift_leg_right.linear().z();
       } else {
       
		d2(2) = - k2_dz*contacts[FRAME_ID_RIGHT].velocity.linear().z() + k2_pz*(z2-contacts[FRAME_ID_RIGHT].position.translation().z()) - a_drift_leg_right.linear().z();
       }
       
       
       // Desired orientation
    double orientation_w2 = conf["leg_right"]["orientation_ref"]["w"].as<double>();
		double orientation_x2 = conf["leg_right"]["orientation_ref"]["x"].as<double>(); //-0.019774711256;
		double orientation_y2 = conf["leg_right"]["orientation_ref"]["y"].as<double>(); //0.0904920792963; 
    double orientation_z2 = conf["leg_right"]["orientation_ref"]["z"].as<double>();
    
    const pinocchio::SE3 right_foot_ref = pinocchio::SE3(Eigen::Quaterniond(orientation_w2, orientation_x2, orientation_y2, orientation_z2), Eigen::Vector3d(0, 0, 0));
    
    // Get the difference between a reference rotation and the state rotation under log function
		//pinocchio::Quaternion quat_ref_2(1, 0, 0, 0);
		
		/*const pinocchio::SE3::Quaternion quat_ref_1(Eigen::Quaterniond(1, 0, 0, 0));// Eigen::Quaterniond(1, 0, 0, 0);
		std::cout << "Normalizes quaternion pin" << quat_ref_2 << std::endl;*/
		
		Matrix3d rot_ref_2 = right_foot_ref.rotation(); //Matrix3d::Identity();
		//Matrix3d rot_diff_2 = rot_ref_2 * contacts[FRAME_ID_RIGHT].position.rotation().transpose();
		Matrix3d rot_diff_2 = contacts[FRAME_ID_RIGHT].position.rotation() * rot_ref_2.transpose();
		Vector3d log3_2 = pinocchio::log3(rot_diff_2);
		
		double k2_ptheta1 = conf["leg_right"]["gains"]["ptheta1"].as<double>(); //50.0;
		double k2_ptheta2 = conf["leg_right"]["gains"]["ptheta2"].as<double>();
		double k2_ptheta3 = conf["leg_right"]["gains"]["ptheta3"].as<double>();
    double k2_dtheta1 = 2.0*sqrt(k2_ptheta1);
    double k2_dtheta2 = 2.0*sqrt(k2_ptheta2);
    double k2_dtheta3 = 2.0*sqrt(k2_ptheta3);
		
		/*double k1_p_ang = conf["leg_left"]["gains"]["p_angular"].as<double>(); //50.0;
    double k1_d_ang = 2.0*sqrt(k1_p_ang);*/
		
		/*d2(3) = - k2_dtheta1*contacts[FRAME_ID_RIGHT].velocity.angular().x() + k2_ptheta1*(log3_1(0)) - a_drift_leg_right.angular().x();
		d2(4) = - k2_dtheta2*contacts[FRAME_ID_RIGHT].velocity.angular().y() + k2_ptheta2*(log3_1(1)) - a_drift_leg_right.angular().y();
		d2(5) = - k2_dtheta3*contacts[FRAME_ID_RIGHT].velocity.angular().z() + k2_ptheta3*(log3_1(2)) - a_drift_leg_right.angular().z();*/
		
		/*****d2(3) = - k2_dtheta1*contacts[FRAME_ID_RIGHT].velocity.angular().x() - k2_ptheta1*(log3_2(0)) - a_drift_leg_right.angular().x();
		d2(4) = - k2_dtheta2*contacts[FRAME_ID_RIGHT].velocity.angular().y() - k2_ptheta2*(log3_2(1)) - a_drift_leg_right.angular().y();
		d2(5) = - k2_dtheta3*contacts[FRAME_ID_RIGHT].velocity.angular().z() - k2_ptheta3*(log3_2(2)) - a_drift_leg_right.angular().z();***/
		
		 ////////////////////////////////////////////////////////////////////////////////
  
   const pinocchio::SE3 right_foot_ref_clb = pinocchio::SE3(Eigen::Quaterniond(d2_clb_6, d2_clb_3, d2_clb_4, d2_clb_5), Eigen::Vector3d(0, 0, 0));
    
		Matrix3d rot_ref_2_clb = right_foot_ref_clb.rotation();
		Matrix3d rot_diff_2_clb = contacts[FRAME_ID_RIGHT].position.rotation() * rot_ref_2_clb.transpose();
		Vector3d log3_2_clb = pinocchio::log3(rot_diff_2_clb);
		
		bool ang_planning2_x = conf["leg_right"]["path_planning"]["ang_x"].as<bool>();
		bool ang_planning2_y = conf["leg_right"]["path_planning"]["ang_y"].as<bool>();
		bool ang_planning2_z = conf["leg_right"]["path_planning"]["ang_z"].as<bool>();
  
  if (switch2 == true && ang_planning2_x == true)
        {
        d2(3) = - k2_dtheta1*contacts[FRAME_ID_RIGHT].velocity.angular().x() - k2_ptheta1*(log3_2_clb(0)) - a_drift_leg_right.angular().x();
       } else {
       
       d2(3) = - k2_dtheta1*contacts[FRAME_ID_RIGHT].velocity.angular().x() - k2_ptheta1*(log3_2(0)) - a_drift_leg_right.angular().x();
       }
       
       if (switch2 == true && ang_planning2_y == true)
        {
        d2(4) = - k2_dtheta2*contacts[FRAME_ID_RIGHT].velocity.angular().y() - k2_ptheta2*(log3_2_clb(1)) - a_drift_leg_right.angular().y();
       } else {
       
		d2(4) = - k2_dtheta2*contacts[FRAME_ID_RIGHT].velocity.angular().y() - k2_ptheta2*(log3_2(1)) - a_drift_leg_right.angular().y();
       }
       
       if (switch2 == true && ang_planning2_z == true)
        {
         d2(5) = - k2_dtheta3*contacts[FRAME_ID_RIGHT].velocity.angular().z() - k2_ptheta3*(log3_2_clb(2)) - a_drift_leg_right.angular().z();
       } else {
       
		 d2(5) = - k2_dtheta3*contacts[FRAME_ID_RIGHT].velocity.angular().z() - k2_ptheta3*(log3_2(2)) - a_drift_leg_right.angular().z();
       }
       
    //////////////////////////////////////////////////////////////////////////
		
		bool S2_linear = conf["leg_right"]["selection_matrix"]["linear"].as<bool>();
    bool S2_angular = conf["leg_right"]["selection_matrix"]["angular"].as<bool>();
    
    /*// Selection matrix for getting linear or angular task
    Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(6,6);
    
    if (S2_linear == true && S2_angular == true) {
    S2.block<3,3>(0,0).setIdentity(3,3);
    S2.block<3,3>(3,3).setIdentity(3,3);
    } else if (S2_linear == true && S2_angular == false) {
    S2.block<3,3>(0,0).setIdentity(3,3);
    } else if (S2_linear == false && S2_angular == true) {
    S2.block<3,3>(3,3).setIdentity(3,3);
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular hip control in the yaml file!!! ");
    }*/
    
     // Selection matrix for getting linear or angular task
    Eigen::MatrixXd S2 = Eigen::MatrixXd::Identity(6,6);
    
    if (S2_linear == true && S2_angular == true) {
    Eigen::MatrixXd S2 = Eigen::MatrixXd::Identity(6,6);
    } else if (S2_linear == true && S2_angular == false) {
    S2 = S_l;
    } else if (S2_linear == false && S2_angular == true) {
    S2 = S_a;
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular com control in the yaml file!!! ");
    }
    
       
    /* Task 3 (Momentum rate for CoM) */ 
    pinocchio::computeCentroidalMapTimeVariation(model, data, q, v);
    Eigen::VectorXd dAv = data.dAg*v;
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3,6);
    S.block<3,3>(0,0).setIdentity(3,3);
    const pinocchio::Data::Matrix6x &J3 = pinocchio::computeCentroidalMap(model, data, q);
    Eigen::VectorXd d3 = Eigen::VectorXd::Zero(6);
    double M = pinocchio::computeTotalMass(model); // Mass in kg
    //M = 80;
    const pinocchio::Force &momenta_rate_drift = pinocchio::computeCentroidalMomentumTimeVariation(model, data,q,v,0*v);
    
    /*double kp_3 = conf["com"]["gains"]["p"].as<double>()/M;
    double kd_3 = 2.0*sqrt(kp_3)/M;*/
    
    /*double k3_px = conf["com"]["gains"]["px"].as<double>()/M; //50.0;
		double k3_py = conf["com"]["gains"]["py"].as<double>()/M;
		double k3_pz = conf["com"]["gains"]["pz"].as<double>()/M;
    double k3_dx = 2.0*sqrt(k3_px)/M;
    double k3_dy = 2.0*sqrt(k3_py)/M;
    double k3_dz = 2.0*sqrt(k3_pz)/M;*/
    
    double k3_px = conf["com"]["gains"]["px"].as<double>(); //50.0;
		double k3_py = conf["com"]["gains"]["py"].as<double>();
		double k3_pz = conf["com"]["gains"]["pz"].as<double>();
    double k3_dx = 2.0*sqrt(k3_px);
    double k3_dy = 2.0*sqrt(k3_py);
    double k3_dz = 2.0*sqrt(k3_pz);
    
    double x3 = conf["com"]["position_ref"]["x"].as<double>();
    double y3 = conf["com"]["position_ref"]["y"].as<double>();
    double z3 = conf["com"]["position_ref"]["z"].as<double>(); // Desired height
    
    
    //double kd_3 = 0.0;
    //d3(2) = M * (ddz_d + g_z) + b_z(dz_d - dz) + k_z (z_d - z);
    /*d3(0) = M*(- kd_3*data.vcom[0](0) + kp_3*(- data.com[0](0)) - dAv(0));
    d3(1) = M*(- kd_3*data.vcom[0](1) + kp_3*(- data.com[0](1)) - dAv(1));
    d3(2) = M*( - 9.81 - kd_3*data.vcom[0](2) + kp_3*(z_d - data.com[0](2)) - dAv(2));*/
    
    /*d3(0) = M*(- kd_3*data.vcom[0](0) + kp_3*(- data.com[0](0))) - dAv(0);
    d3(1) = M*(- kd_3*data.vcom[0](1) + kp_3*(- data.com[0](1))) - dAv(1);
    d3(2) = M*(- kd_3*data.vcom[0](2) + kp_3*(z_d - data.com[0](2))) - dAv(2);*/
    
    /*d3(0) = 1.0*(- kd_3*data.vcom[0](0) + kp_3*(- data.com[0](0))) - dAv(0);
    d3(1) = 1.0*(- kd_3*data.vcom[0](1) + kp_3*(- data.com[0](1))) - dAv(1);
    d3(2) = 1.0*(- kd_3*data.vcom[0](2) + kp_3*(z_d - data.com[0](2))) - dAv(2);
    */
    
    // linear
   /* double omega_squared = 9.81/z3;
    Eigen::Vector3d p_des((1 + kp_3/sqrt(omega_squared))* (msg.centroidal.com_position.x + 1/sqrt(omega_squared) * msg.centroidal.com_velocity.x - 0), (1 + kp_3/omega_squared )* (msg.centroidal.com_position.y + 1/sqrt(omega_squared) * msg.centroidal.com_velocity.y - 0), 0);
    
    d3(0) = M*(9.81/z3)*(msg.centroidal.com_position.x - p_des(0)) - dAv(0);
    d3(1) = M*(9.81/z3)*(msg.centroidal.com_position.y - p_des(1)) - dAv(1);
    d3(2) = - kd_3*msg.centroidal.momenta.linear.z + M*kp_3*(z3 - msg.centroidal.com_position.z) - dAv(2);*/
    
    /*d3(0) = - kd_3*msg.centroidal.com_velocity.x + kp_3*(x3 - msg.centroidal.com_position.x) - dAv(0);
    d3(1) = - kd_3*msg.centroidal.com_velocity.y + kp_3*(y3 - msg.centroidal.com_position.y) - dAv(1);
    d3(2) = - kd_3*msg.centroidal.com_velocity.z + kp_3*(z3 - msg.centroidal.com_position.z) - dAv(2);*/
    
    pinocchio::SE3 oM_com = pinocchio::SE3(Eigen::Quaterniond(1, 0.0, 0.0, 0.0), data.com[0]);
    //oM_com.toDualActionMatrix();
    
    /*d3(0) = - kd_3*msg.centroidal.momenta.linear.x + M*kp_3*(x3 - msg.centroidal.com_position.x) - dAv(0);
    d3(1) = - kd_3*msg.centroidal.momenta.linear.y + M*kp_3*(y3 - msg.centroidal.com_position.y) - dAv(1);
    d3(2) = - kd_3*msg.centroidal.momenta.linear.z + M*kp_3*(z3 - msg.centroidal.com_position.z) - dAv(2);*/
    
    /*d3(0) = - k3_dx*msg.centroidal.momenta.linear.x + M*k3_px*(x3 - msg.centroidal.com_position.x) - momenta_rate_drift.linear().x();
    d3(1) = - k3_dy*msg.centroidal.momenta.linear.y + M*k3_py*(y3 - msg.centroidal.com_position.y) - momenta_rate_drift.linear().y();
    d3(2) = - k3_dz*msg.centroidal.momenta.linear.z + M*k3_pz*(z3 - msg.centroidal.com_position.z) - momenta_rate_drift.linear().z();*/
    
    /*d3(0) = - k3_dx*msg.centroidal.momenta.linear.x - M*k3_px*(msg.centroidal.com_position.x - x3) - momenta_rate_drift.linear().x();
    d3(1) = - k3_dy*msg.centroidal.momenta.linear.y - M*k3_py*(msg.centroidal.com_position.y - y3) - momenta_rate_drift.linear().y();
    d3(2) = - k3_dz*msg.centroidal.momenta.linear.z - M*k3_pz*(msg.centroidal.com_position.z - z3) - momenta_rate_drift.linear().z();*/
    
   /* bool lin_planning_3 = conf["com"]["path_planning"]["linear"].as<bool>();
    
    d3(0) = - k3_dx*msg.centroidal.momenta.linear.x - M*k3_px*(msg.centroidal.com_position.x - x3) - momenta_rate_drift.linear().x();
    
    if (switch3 == true && lin_planning_3 == true)
        {  
         d3(1) = - k3_dy*msg.centroidal.momenta.linear.y - M*k3_py*(msg.centroidal.com_position.y - d3_clb_1) - momenta_rate_drift.linear().y();
       } else { 
       	 d3(1) = - k3_dy*msg.centroidal.momenta.linear.y - M*k3_py*(msg.centroidal.com_position.y - y3) - momenta_rate_drift.linear().y();
       	 }
    d3(2) = - k3_dz*msg.centroidal.momenta.linear.z - M*k3_pz*(msg.centroidal.com_position.z - z3) - momenta_rate_drift.linear().z();*/
    
    bool lin_planning_3 = conf["com"]["path_planning"]["linear"].as<bool>();
    ///////////////////////////////////////////////////////////////////////////////////
    /*d3(0) = - k3_dx*msg.centroidal.momenta.linear.x + M*k3_px*(x3 - msg.centroidal.com_position.x) - momenta_rate_drift.linear().x();
    
    if (switch3 == true && lin_planning_3 == true)
        {  
         d3(1) = - k3_dy*msg.centroidal.momenta.linear.y + M*k3_py*(d3_clb_1 - msg.centroidal.com_position.y) - momenta_rate_drift.linear().y();
       } else { 
       	 d3(1) = - k3_dy*msg.centroidal.momenta.linear.y + M*k3_py*(y3 - msg.centroidal.com_position.y) - momenta_rate_drift.linear().y();
       	 }
    d3(2) = - k3_dz*msg.centroidal.momenta.linear.z + M*k3_pz*(z3 - msg.centroidal.com_position.z) - momenta_rate_drift.linear().z();*/
   //////////////////////////////////////////////////////////////////////////////////////
   
   d3(0) = M*(- k3_dx*data.vcom[0](0) + k3_px*(x3 - msg.centroidal.com_position.x)) - dAv(0);
    
    if (switch3 == true && lin_planning_3 == true)
        {  
         d3(1) = M*(- k3_dy*data.vcom[0](1)  + k3_py*(d3_clb_1 - msg.centroidal.com_position.y)) - dAv(1);
       } else { 
       	 d3(1) = M*(- k3_dy*data.vcom[0](1) + k3_py*(y3 - msg.centroidal.com_position.y)) - dAv(1);
       	 }
    d3(2) = M*(- k3_dz*data.vcom[0](2) + k3_pz*(z3 - msg.centroidal.com_position.z)) - dAv(2); 
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////
    // Computing the ICP
    
    double height = abs(msg.centroidal.com_position.z);
    //gravity_ = model.gravity.linear().norm();
    double omega = sqrt(model.gravity.linear().norm() / height);
    Eigen::Vector3d com_pos = Eigen::Vector3d(msg.centroidal.com_position.x,
                                              msg.centroidal.com_position.y,
                                              msg.centroidal.com_position.z);
    Eigen::Vector3d com_vel(msg.centroidal.com_velocity.x,
                          msg.centroidal.com_velocity.y,
                          msg.centroidal.com_velocity.z);
    Eigen::Vector3d icp_pos = com_pos + com_vel / omega;
    double kp_xi = conf["icp"]["gains"]["p"].as<double>();
    Eigen::Vector3d p_des(0.0 + (1 + kp_xi / omega)* (icp_pos(0) - 0), -0.06 + (1 + kp_xi / omega)* (icp_pos(1) - 0), 0);
    
    /*d3(0) = M*omega*(com_pos(0) - p_des(0)) - momenta_rate_drift.linear().x();
    d3(1) = M*omega*(com_pos(1) - p_des(1)) - momenta_rate_drift.linear().y();
    d3(2) = - k3_dz*msg.centroidal.momenta.linear.z + M*k3_pz*(z3 - msg.centroidal.com_position.z) - momenta_rate_drift.linear().z();*/
    /////////////////////////////////////////////////////////////////////////////////////////////
    double x3_angular = 0.0; //-0.000725885563308;
    double y3_angular = 0.0; //0.1462162639;
    double z3_angular = 0.0; //0.000822523838352;
    //angular
    /*d3(3) = kd_3*(x3_angular - msg.centroidal.momenta.angular.x) - dAv(3);
    d3(4) = kd_3*(y3_angular - msg.centroidal.momenta.angular.y) - dAv(4);
    d3(5) = kd_3*(z3_angular - msg.centroidal.momenta.angular.z) - dAv(5);*/
    
    //kp_3 = conf["com"]["gains"]["p_angular"].as<double>();
    
    double k3_ptheta1 = conf["com"]["gains"]["ptheta1"].as<double>(); //50.0;
		double k3_ptheta2 = conf["com"]["gains"]["ptheta2"].as<double>();
		double k3_ptheta3 = conf["com"]["gains"]["ptheta3"].as<double>();
    double k3_dtheta1 = 2.0*sqrt(k3_ptheta1);
    double k3_dtheta2 = 2.0*sqrt(k3_ptheta2);
    double k3_dtheta3 = 2.0*sqrt(k3_ptheta3);
    
    d3(3) = k3_dtheta1*(x3_angular - msg.centroidal.momenta.angular.x) - momenta_rate_drift.angular().x();
    d3(4) = k3_dtheta2*(y3_angular - msg.centroidal.momenta.angular.y) - momenta_rate_drift.angular().y();
    d3(5) = k3_dtheta3*(z3_angular - msg.centroidal.momenta.angular.z) - momenta_rate_drift.angular().z();
 		
 		
 		bool S3_linear = conf["com"]["selection_matrix"]["linear"].as<bool>();
    bool S3_angular = conf["com"]["selection_matrix"]["angular"].as<bool>();
    
    /*// Selection matrix for getting linear or angular task
    Eigen::MatrixXd S3 = Eigen::MatrixXd::Zero(6,6);
    
    if (S3_linear == true && S3_angular == true) {
    S3.block<3,3>(0,0).setIdentity(3,3);
    S3.block<3,3>(3,3).setIdentity(3,3);
    } else if (S3_linear == true && S3_angular == false) {
    S3.block<3,3>(0,0).setIdentity(3,3);
    } else if (S3_linear == false && S3_angular == true) {
    S3.block<3,3>(3,3).setIdentity(3,3);
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular com control in the yaml file!!! ");
    }*/
    
    // Selection matrix for getting linear or angular task
    Eigen::MatrixXd S3 = Eigen::MatrixXd::Identity(6,6);
    
    if (S3_linear == true && S3_angular == true) {
    Eigen::MatrixXd S3 = Eigen::MatrixXd::Identity(6,6);
    } else if (S3_linear == true && S3_angular == false) {
    S3 = S_l;
    } else if (S3_linear == false && S3_angular == true) {
    S3 = S_a;
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular com control in the yaml file!!! ");
    }
    
    /* Task 4 (Joint acceleration for upper body posture) */
    
    
    /*Eigen::MatrixXd J4 = Eigen::MatrixXd::Zero(30,model.nv); // Selection matrix
    //J4.block<1,1>(2,8).setIdentity(1,1);
    //J4.block<1,1>(8,14).setIdentity(1,1);
    
    //J4.block<1,1>(0,6).setIdentity(1,1);
    //J4.block<1,1>(6,12).setIdentity(1,1);
    J4.block<18,18>(12,18).setIdentity(18,18);
    Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(30);
    q_ref.tail(18)(0) = conf["q_joint_ref"]["torso"]["j1"].as<double>();
    q_ref.tail(18)(1) = conf["q_joint_ref"]["torso"]["j2"].as<double>();
    
    for (int i = 1; i < 8; ++i) {
    	q_ref.tail(18)(1+i) = conf["q_joint_ref"]["arm_left"]["j" + std::to_string(i)].as<double>();
    	q_ref.tail(18)(8+i) = conf["q_joint_ref"]["arm_right"]["j" + std::to_string(i)].as<double>();
    }
    
    q_ref.tail(18)(16) = conf["q_joint_ref"]["head"]["j1"].as<double>();
    q_ref.tail(18)(17) = conf["q_joint_ref"]["head"]["j2"].as<double>();
    
    //Setting PD
    
    Eigen::VectorXd k4_pjoint = Eigen::VectorXd::Zero(model.nv-6);
    
    k4_pjoint(0) = conf["q_joint_ref"]["leg_left_gains"]["pj1"].as<double>();
    k4_pjoint(6) = conf["q_joint_ref"]["leg_right_gains"]["pj1"].as<double>();
    k4_pjoint.tail(18)(0) = conf["q_joint_ref"]["torso_gains"]["pj1"].as<double>();
    k4_pjoint.tail(18)(1) = conf["q_joint_ref"]["torso_gains"]["pj2"].as<double>();
    
    for (int i = 1; i < 8; ++i) {
    	k4_pjoint.tail(18)(1+i) = conf["q_joint_ref"]["arm_left_gains"]["pj" + std::to_string(i)].as<double>();
    	k4_pjoint.tail(18)(8+i) = conf["q_joint_ref"]["arm_right_gains"]["pj" + std::to_string(i)].as<double>();
    }
    
    k4_pjoint.tail(18)(16) = conf["q_joint_ref"]["head_gains"]["pj1"].as<double>();
    k4_pjoint.tail(18)(17) = conf["q_joint_ref"]["head_gains"]["pj2"].as<double>();
    
    //Eigen::DiagonalMatrix<double, 30> mat = k4_pjoint.asDiagonal();
    
    Eigen::MatrixXd k4_p;
    k4_p.setConstant(30,30,0);
    
    //const Eigen::MatrixXd  = Eigen::MatrixXd::Zero(30,30);
    
    for (int i = 0; i < 30; ++i) {
    	k4_p.diagonal()[i] = k4_pjoint(i);
    }
    
    Eigen::MatrixXd k4_d;
    k4_d.setConstant(30,30,0);
    for (int i = 0; i < 30; ++i) {
    	k4_d.diagonal()[i] = 2.0*sqrt(k4_pjoint(i));
    }
    
    //double k4_p = conf["q_joint_ref"]["gains"]["p"].as<double>();
    //double k4_d = 2.0*sqrt(k4_p); //6;
    
    Eigen::VectorXd d4;
    // d4(0) = kp*(q_ref - q(24)) - kd*v(23);  
    d4 = -k4_p*(q.tail(30) - q_ref) - k4_d*v.tail(30); 
    */
    
    Eigen::MatrixXd J4 = Eigen::MatrixXd::Zero(model.nv-6,model.nv); // Select the actuated part of all joints of the humanoid 
    Eigen::MatrixXd S4 = Eigen::MatrixXd::Zero(18,model.nv-6); // Selection matrix which selects the joints that we want to control
    //J4.block<1,1>(2,8).setIdentity(1,1);
    //J4.block<1,1>(8,14).setIdentity(1,1);
    
    //J4.block<1,1>(0,6).setIdentity(1,1);
    //J4.block<1,1>(6,12).setIdentity(1,1);
    J4.block<30,30>(0,6).setIdentity(model.nv-6,model.nv-6);
    S4.block<18,18>(0,12).setIdentity(18,18);
    Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(30);
    q_ref.tail(18)(0) = conf["q_joint_ref"]["torso"]["j1"].as<double>();
    q_ref.tail(18)(1) = conf["q_joint_ref"]["torso"]["j2"].as<double>();
    
    for (int i = 1; i < 8; ++i) {
    	q_ref.tail(18)(1+i) = conf["q_joint_ref"]["arm_left"]["j" + std::to_string(i)].as<double>();
    	q_ref.tail(18)(8+i) = conf["q_joint_ref"]["arm_right"]["j" + std::to_string(i)].as<double>();
    }
    
    q_ref.tail(18)(16) = conf["q_joint_ref"]["head"]["j1"].as<double>();
    q_ref.tail(18)(17) = conf["q_joint_ref"]["head"]["j2"].as<double>();
    
    //Setting PD
    
    Eigen::VectorXd k4_pjoint = Eigen::VectorXd::Zero(model.nv-6);
    
    k4_pjoint(0) = conf["q_joint_ref"]["leg_left_gains"]["pj1"].as<double>();
    k4_pjoint(6) = conf["q_joint_ref"]["leg_right_gains"]["pj1"].as<double>();
    k4_pjoint.tail(18)(0) = conf["q_joint_ref"]["torso_gains"]["pj1"].as<double>();
    k4_pjoint.tail(18)(1) = conf["q_joint_ref"]["torso_gains"]["pj2"].as<double>();
    
    for (int i = 1; i < 8; ++i) {
    	k4_pjoint.tail(18)(1+i) = conf["q_joint_ref"]["arm_left_gains"]["pj" + std::to_string(i)].as<double>();
    	k4_pjoint.tail(18)(8+i) = conf["q_joint_ref"]["arm_right_gains"]["pj" + std::to_string(i)].as<double>();
    }
    
    k4_pjoint.tail(18)(16) = conf["q_joint_ref"]["head_gains"]["pj1"].as<double>();
    k4_pjoint.tail(18)(17) = conf["q_joint_ref"]["head_gains"]["pj2"].as<double>();
    
    //Eigen::DiagonalMatrix<double, 30> mat = k4_pjoint.asDiagonal();
    
    Eigen::MatrixXd k4_p;
    k4_p.setConstant(30,30,0);
    
    //const Eigen::MatrixXd  = Eigen::MatrixXd::Zero(30,30);
    
    for (int i = 0; i < 30; ++i) {
    	k4_p.diagonal()[i] = k4_pjoint(i);
    }
    
    Eigen::MatrixXd k4_d;
    k4_d.setConstant(30,30,0);
    for (int i = 0; i < 30; ++i) {
    	k4_d.diagonal()[i] = 2.0*sqrt(k4_pjoint(i));
    }
    
    /*double k4_p = conf["q_joint_ref"]["gains"]["p"].as<double>();
    double k4_d = 2.0*sqrt(k4_p); //6;*/
    
    Eigen::VectorXd d4;
    // d4(0) = kp*(q_ref - q(24)) - kd*v(23);  
    d4 = -k4_p*(q.tail(30) - q_ref) - k4_d*v.tail(30); 
    
  
    
    ////////////////////////////////////////////////////////////////
    
    /*Eigen::MatrixXd J4 = Eigen::MatrixXd::Zero(30,model.nv); // Selection matrix
    //J4.block<1,1>(2,8).setIdentity(1,1);
    //J4.block<1,1>(8,14).setIdentity(1,1);
    J4.block<1,1>(0,6).setIdentity(1,1);
    J4.block<1,1>(6,12).setIdentity(1,1);
    J4.block<18,18>(12,18).setIdentity(18,18);
    Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(30);
    q_ref.tail(18)(0) = conf["q_joint_ref"]["torso"]["j1"].as<double>();
    q_ref.tail(18)(1) = conf["q_joint_ref"]["torso"]["j2"].as<double>();
    
    for (int i = 1; i < 8; ++i) {
    	q_ref.tail(18)(1+i) = conf["q_joint_ref"]["arm_left"]["j" + std::to_string(i)].as<double>();
    	q_ref.tail(18)(8+i) = conf["q_joint_ref"]["arm_right"]["j" + std::to_string(i)].as<double>();
    }
    
    q_ref.tail(18)(16) = conf["q_joint_ref"]["head"]["j1"].as<double>();
    q_ref.tail(18)(17) = conf["q_joint_ref"]["head"]["j2"].as<double>();
    
    Eigen::VectorXd d4;
    //Eigen::VectorXd kp = Eigen::VectorXd::Zero(1);
    double k4_p = conf["q_joint_ref"]["gains"]["p"].as<double>();
    double k4_d = 2.0*sqrt(k4_p); //6;
    
    // d4(0) = kp*(q_ref - q(24)) - kd*v(23);  
    d4 = -k4_p*(q.tail(30) - q_ref) - k4_d*v.tail(30); 
    
    //double t1 = conf["q_joint_ref"]["torso_gains"]["p"].as<double>();*/
    
    /////////////////////////////////////////////////////////
    
    /*Eigen::MatrixXd J4 = Eigen::MatrixXd::Zero(18,model.nv); // Selection matrix
    J4.block<18,18>(0,18).setIdentity(18,18);
    Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(18);
    q_ref(0) = conf["q_joint_ref"]["torso"]["j1"].as<double>();
    q_ref(1) = conf["q_joint_ref"]["torso"]["j2"].as<double>();
    
    for (int i = 1; i < 8; ++i) {
    	q_ref(1+i) = conf["q_joint_ref"]["arm_left"]["j" + std::to_string(i)].as<double>();
    	q_ref(8+i) = conf["q_joint_ref"]["arm_right"]["j" + std::to_string(i)].as<double>();
    }
    
    q_ref(16) = conf["q_joint_ref"]["head"]["j1"].as<double>();
    q_ref(17) = conf["q_joint_ref"]["head"]["j2"].as<double>();
    
    Eigen::VectorXd d4;
    //Eigen::VectorXd kp = Eigen::VectorXd::Zero(1);
    double k4_p = conf["q_joint_ref"]["gains"]["p"].as<double>();
    double k4_d = 2.0*sqrt(k4_p); //6;
    
    // d4(0) = kp*(q_ref - q(24)) - kd*v(23);  
    d4 = -k4_p*(q.tail(18) - q_ref) - k4_d*v.tail(18);  */
    
    
    /* Task 5 (Spatial acceleration for right base_link)*/
    // Get frame_id for right_sole_link
    std::string FRAME_ID_BASE_LINK = "base_link"; //left_sole_link
  	pinocchio::FrameIndex frame_id_base_link = model.getFrameId(FRAME_ID_BASE_LINK);
  	
  	// Get frame Jacobian 
  	pinocchio::Data::Matrix6x J5(6,model.nv);
  	J5.setZero();
  	pinocchio::computeJointJacobians(model, data, q);   // this also compute the forward kinematics for joints
		pinocchio::framesForwardKinematics(model, data, q); // this update the frames placement
		pinocchio::getFrameJacobian(model, data, frame_id_base_link, pinocchio::LOCAL_WORLD_ALIGNED, J5);
		
 		//Desired task
		Eigen::MatrixXd d5; 
		d5.setConstant(6,1,0);
    
    // Gains PD
		/*double k5_p = conf["hip"]["gains"]["p"].as<double>();//50.0;
    double k5_d = 2.0*sqrt(k5_p);*/
    
    double k5_px = conf["hip"]["gains"]["px"].as<double>(); //50.0;
		double k5_py = conf["hip"]["gains"]["py"].as<double>();
		double k5_pz = conf["hip"]["gains"]["pz"].as<double>();
    double k5_dx = 2.0*sqrt(k5_px);
    double k5_dy = 2.0*sqrt(k5_py);
    double k5_dz = 2.0*sqrt(k5_pz);
    
    /*// Desired position
    double x2 = 0.00432;
    double y2 = -0.07558;
    double z2 = -0.00209;*/
    
    /*// Desired position
    double x2 = 0.17;
    double y2 = -0.12;
    double z2 = 0.3;*/
    
    // Desired position
    double x5 = conf["hip"]["position_ref"]["x"].as<double>(); //-0.0203258336221;
    double y5 = conf["hip"]["position_ref"]["y"].as<double>(); //-0.0621026780498;
    double z5 = conf["hip"]["position_ref"]["z"].as<double>(); //0.4;
   
    //z2 = x2_clb_3;
    /*
        x: 0.00432559746305
        y: -0.0755890513575
        z: -0.00209827029282

		*/
		pinocchio::forwardKinematics(model,data,q,v,0*v);
		pinocchio::computeJointJacobians(model,data,q);
    pinocchio::framesForwardKinematics(model,data,q);
    
    bool lin_planning5_x = conf["hip"]["path_planning"]["lin_x"].as<bool>();
    bool lin_planning5_y = conf["hip"]["path_planning"]["lin_y"].as<bool>();
    bool lin_planning5_z = conf["hip"]["path_planning"]["lin_z"].as<bool>();
		
		const pinocchio::Motion &a_drift_base_link = pinocchio::getFrameClassicalAcceleration(model, data, frame_id_base_link, pinocchio::LOCAL_WORLD_ALIGNED);
		/*d5(0) = - k5_dx*contacts[FRAME_ID_BASE_LINK].velocity.linear().x() + k5_px*(x5-contacts[FRAME_ID_BASE_LINK].position.translation().x()) - a_drift_base_link.linear().x();*/
		
		 if (switch3 == true && lin_planning5_x == true)
        {
        d5(0) = - k5_dx*contacts[FRAME_ID_BASE_LINK].velocity.linear().x() + k5_px*(d3_clb_0-contacts[FRAME_ID_BASE_LINK].position.translation().x()) - a_drift_base_link.linear().x();
      
       } else {
       d5(0) = - k5_dx*contacts[FRAME_ID_BASE_LINK].velocity.linear().x() + k5_px*(x5-contacts[FRAME_ID_BASE_LINK].position.translation().x()) - a_drift_base_link.linear().x();
      }
       
       if (switch3 == true && lin_planning5_y == true)
        {
         d5(1) = - k5_dy*contacts[FRAME_ID_BASE_LINK].velocity.linear().y() + k5_py*(d3_clb_1-contacts[FRAME_ID_BASE_LINK].position.translation().y()) - a_drift_base_link.linear().y();
       } else {
       	 d5(1) = - k5_dy*contacts[FRAME_ID_BASE_LINK].velocity.linear().y() + k5_py*(y5-contacts[FRAME_ID_BASE_LINK].position.translation().y()) - a_drift_base_link.linear().y();
       	 }
       
       if (switch3 == true && lin_planning5_z == true)
        {
        d5(2) = - k5_dz*contacts[FRAME_ID_BASE_LINK].velocity.linear().z() + k5_pz*(d3_clb_2-contacts[FRAME_ID_BASE_LINK].position.translation().z()) - a_drift_base_link.linear().z();
       } else {
      d5(2) = - k5_dz*contacts[FRAME_ID_BASE_LINK].velocity.linear().z() + k5_pz*(z5-contacts[FRAME_ID_BASE_LINK].position.translation().z()) - a_drift_base_link.linear().z();
       }
		
		
		
		
		/*// Get the difference between a reference rotation and the state rotation under log function
		//pinocchio::Quaternion quat_ref_2(1, 0, 0, 0)
		// const pinocchio::SE3::Quaternion quat_ref_1(Eigen::Quaterniond(1, 0, 0, 0));// Eigen::Quaterniond(1, 0, 0, 0)
		Matrix3d rot_ref_5= Matrix3d::Identity();
		Matrix3d rot_diff_5 = rot_ref_5 * contacts[FRAME_ID_BASE_LINK].position.rotation().transpose();
		Vector3d log3_5 = pinocchio::log3(rot_diff_5);
		
		double k5_ptheta1 = conf["hip"]["gains"]["ptheta1"].as<double>(); //50.0;
		double k5_ptheta2 = conf["hip"]["gains"]["ptheta2"].as<double>();
		double k5_ptheta3 = conf["hip"]["gains"]["ptheta3"].as<double>();
    double k5_dtheta1 = 2.0*sqrt(k5_ptheta1);
    double k5_dtheta2 = 2.0*sqrt(k5_ptheta2);
    double k5_dtheta3 = 2.0*sqrt(k5_ptheta3);
		
		d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() + k5_ptheta1*(log3_5(0)) - a_drift_base_link.angular().x();
		d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() + k5_ptheta2*(log3_5(1)) - a_drift_base_link.angular().y();
		d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() + k5_ptheta3*(log3_5(2)) - a_drift_base_link.angular().z();*/
		
		// Desired orientation
    double orientation_w5 = conf["hip"]["orientation_ref"]["w"].as<double>();
		double orientation_x5 = conf["hip"]["orientation_ref"]["x"].as<double>(); //-0.019774711256;
		double orientation_y5 = conf["hip"]["orientation_ref"]["y"].as<double>(); //0.0904920792963; 
    double orientation_z5 = conf["hip"]["orientation_ref"]["z"].as<double>();
    
    const pinocchio::SE3 base_link_ref = pinocchio::SE3(Eigen::Quaterniond(orientation_w5, orientation_x5, orientation_y5, orientation_z5), Eigen::Vector3d(0, 0, 0));
    
    // Get the difference between a reference rotation and the state rotation under log function
		//pinocchio::Quaternion quat_ref_2(1, 0, 0, 0);
		
		/*const pinocchio::SE3::Quaternion quat_ref_1(Eigen::Quaterniond(1, 0, 0, 0));// Eigen::Quaterniond(1, 0, 0, 0);
		std::cout << "Normalizes quaternion pin" << quat_ref_2 << std::endl;*/
		
		Matrix3d rot_ref_5 = base_link_ref.rotation(); //Matrix3d::Identity();
		//Matrix3d rot_diff_5 = rot_ref_5 * contacts[FRAME_ID_BASE_LINK].position.rotation().transpose();
		Matrix3d rot_diff_5 = contacts[FRAME_ID_BASE_LINK].position.rotation() * rot_ref_5.transpose();
		Vector3d log3_5 = pinocchio::log3(rot_diff_5);
		
		double k5_ptheta1 = conf["hip"]["gains"]["ptheta1"].as<double>(); //50.0;
		double k5_ptheta2 = conf["hip"]["gains"]["ptheta2"].as<double>();
		double k5_ptheta3 = conf["hip"]["gains"]["ptheta3"].as<double>();
    double k5_dtheta1 = 2.0*sqrt(k5_ptheta1);
    double k5_dtheta2 = 2.0*sqrt(k5_ptheta2);
    double k5_dtheta3 = 2.0*sqrt(k5_ptheta3);
		
		/*d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() + k5_ptheta1*(log3_5(0)) - a_drift_base_link.angular().x();
		d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() + k5_ptheta2*(log3_5(1)) - a_drift_base_link.angular().y();
		d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() + k5_ptheta3*(log3_5(2)) - a_drift_base_link.angular().z();*/
		
		/****d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() - k5_ptheta1*(log3_5(0)) - a_drift_base_link.angular().x();
		d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() - k5_ptheta2*(log3_5(1)) - a_drift_base_link.angular().y();
		d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() - k5_ptheta3*(log3_5(2)) - a_drift_base_link.angular().z();*****/
		
		 ////////////////////////////////////////////////////////////////////////////////
  
  const pinocchio::SE3 base_link_ref_clb = pinocchio::SE3(Eigen::Quaterniond(d3_clb_6, d3_clb_3, d3_clb_4, d3_clb_5), Eigen::Vector3d(0, 0, 0));
    
		
		Matrix3d rot_ref_5_clb = base_link_ref_clb.rotation(); 
		Matrix3d rot_diff_5_clb = contacts[FRAME_ID_BASE_LINK].position.rotation() * rot_ref_5_clb.transpose();
		Vector3d log3_5_clb = pinocchio::log3(rot_diff_5_clb);
		
		bool ang_planning5_x = conf["hip"]["path_planning"]["ang_x"].as<bool>();
		bool ang_planning5_y = conf["hip"]["path_planning"]["ang_y"].as<bool>();
		bool ang_planning5_z = conf["hip"]["path_planning"]["ang_z"].as<bool>();
  
  if (switch3 == true && ang_planning5_x == true)
        {
        d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() - k5_ptheta1*(log3_5_clb(0)) - a_drift_base_link.angular().x();
       } else {
       d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() - k5_ptheta1*(log3_5(0)) - a_drift_base_link.angular().x();
       }
       
       if (switch3 == true && ang_planning5_y == true)
        {
        d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() - k5_ptheta2*(log3_5_clb(1)) - a_drift_base_link.angular().y();
       } else {
       
		d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() - k5_ptheta2*(log3_5(1)) - a_drift_base_link.angular().y();
       }
       
       if (switch3 == true && ang_planning5_z == true)
        {
         d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() - k5_ptheta3*(log3_5_clb(2)) - a_drift_base_link.angular().z();
       } else {
       d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() - k5_ptheta3*(log3_5(2)) - a_drift_base_link.angular().z();
       }
   
   
  /* bool ang_planning5_x = conf["hip"]["path_planning"]["ang_x"].as<bool>();
   if (switch3 == true && ang_planning5_x == true)
   {
     const pinocchio::SE3 base_link_ref_clb = pinocchio::SE3(Eigen::Quaterniond(d3_clb_6,d3_clb_3,d3_clb_4, d3_clb_5), Eigen::Vector3d(0, 0, 0));
    
		
		Matrix3d rot_ref_5_clb = base_link_ref_clb.rotation(); 
		Matrix3d rot_diff_5_clb = contacts[FRAME_ID_BASE_LINK].position.rotation() * rot_ref_5_clb.transpose();
		Vector3d log3_5_clb = pinocchio::log3(rot_diff_5_clb);
        
        d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() - k5_ptheta1*(log3_5_clb(0)) - a_drift_base_link.angular().x();
        d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() + k5_ptheta2*(log3_5_clb(1)) - a_drift_base_link.angular().y();
         d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() - k5_ptheta3*(log3_5_clb(2)) - a_drift_base_link.angular().z();
       } else {
       d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() - k5_ptheta1*(log3_5(0)) - a_drift_base_link.angular().x();
       	d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() - k5_ptheta2*(log3_5(1)) - a_drift_base_link.angular().y();
		d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() - k5_ptheta3*(log3_5(2)) - a_drift_base_link.angular().z();
       }*/
    //////////////////////////////////////////////////////////////////////////
		
		Eigen::MatrixXd S_ang = Eigen::MatrixXd::Zero(3,6);
    S_ang.block<3,3>(0,3).setIdentity(3,3);
    
    bool S5_linear = conf["hip"]["selection_matrix"]["linear"].as<bool>();
    bool S5_angular = conf["hip"]["selection_matrix"]["angular"].as<bool>();
    
    /*// Selection matrix for getting linear or angular task
    Eigen::MatrixXd S5 = Eigen::MatrixXd::Zero(6,6);
    
    if (S5_linear == true && S5_angular == true) {
    S5.block<3,3>(0,0).setIdentity(3,3);
    S5.block<3,3>(3,3).setIdentity(3,3);
    } else if (S5_linear == true && S5_angular == false) {
    S5.block<3,3>(0,0).setIdentity(3,3);
    } else if (S5_linear == false && S5_angular == true) {
    S5.block<3,3>(3,3).setIdentity(3,3);
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular hip control in the yaml file!!! ");
    }*/
    
     // Selection matrix for getting linear or angular task
    Eigen::MatrixXd S5 = Eigen::MatrixXd::Identity(6,6);
    
    if (S5_linear == true && S5_angular == true) {
    Eigen::MatrixXd S5 = Eigen::MatrixXd::Identity(6,6);
    } else if (S5_linear == true && S5_angular == false) {
    S5 = S_l;
    } else if (S5_linear == false && S5_angular == true) {
    S5 = S_a;
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular com control in the yaml file!!! ");
    }
    
    /* Task 6 (Spatial acceleration for torso_2_link)*/
    // Get frame_id for torso_2_link
    std::string FRAME_ID_TORSO_2_LINK = "torso_2_link"; // torso_2_link
  	pinocchio::FrameIndex frame_id_torso_2_link = model.getFrameId(FRAME_ID_TORSO_2_LINK);
  	
  	// Get frame Jacobian 
  	pinocchio::Data::Matrix6x J6(6,model.nv);
  	J6.setZero();
  	pinocchio::computeJointJacobians(model, data, q);   // this also compute the forward kinematics for joints
		pinocchio::framesForwardKinematics(model, data, q); // this update the frames placement
		pinocchio::getFrameJacobian(model, data, frame_id_torso_2_link, pinocchio::LOCAL_WORLD_ALIGNED, J6);
		
 		//Desired task
		Eigen::MatrixXd d6; 
		d6.setConstant(6,1,0);
    
    // Gains PD
		/*double k5_p = conf["hip"]["gains"]["p"].as<double>();//50.0;
    double k5_d = 2.0*sqrt(k5_p);*/
    
    double k6_px = conf["torso"]["gains"]["px"].as<double>(); //50.0;
		double k6_py = conf["torso"]["gains"]["py"].as<double>();
		double k6_pz = conf["torso"]["gains"]["pz"].as<double>();
    double k6_dx = 2.0*sqrt(k6_px);
    double k6_dy = 2.0*sqrt(k6_py);
    double k6_dz = 2.0*sqrt(k6_pz);
    
    /*// Desired position
    double x2 = 0.00432;
    double y2 = -0.07558;
    double z2 = -0.00209;*/
    
    /*// Desired position
    double x2 = 0.17;
    double y2 = -0.12;
    double z2 = 0.3;*/
    
    // Desired position
    double x6 = conf["torso"]["position_ref"]["x"].as<double>(); //-0.0203258336221;
    double y6 = conf["torso"]["position_ref"]["y"].as<double>(); //-0.0621026780498;
    double z6 = conf["torso"]["position_ref"]["z"].as<double>(); //0.4;
   
    //z2 = x2_clb_3;
    /*
        x: 0.00432559746305
        y: -0.0755890513575
        z: -0.00209827029282

		*/
		pinocchio::forwardKinematics(model,data,q,v,0*v);
		pinocchio::computeJointJacobians(model,data,q);
    pinocchio::framesForwardKinematics(model,data,q);
		
		const pinocchio::Motion &a_drift_torso_2_link = pinocchio::getFrameClassicalAcceleration(model, data, frame_id_torso_2_link, pinocchio::LOCAL_WORLD_ALIGNED);
		
		
		/***d6(0) = - k6_dx*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().x() + k6_px*(x6-contacts[FRAME_ID_TORSO_2_LINK].position.translation().x()) - a_drift_torso_2_link.linear().x();
		d6(1) = - k6_dy*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().y() + k6_py*(y6-contacts[FRAME_ID_TORSO_2_LINK].position.translation().y()) - a_drift_torso_2_link.linear().y();
		d6(2) = - k6_dz*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().z() + k6_pz*(z6-contacts[FRAME_ID_TORSO_2_LINK].position.translation().z()) - a_drift_torso_2_link.linear().z();
		***/
		
			bool lin_planning6_x = conf["torso"]["path_planning"]["lin_x"].as<bool>();
    bool lin_planning6_y = conf["torso"]["path_planning"]["lin_y"].as<bool>();
    bool lin_planning6_z = conf["torso"]["path_planning"]["lin_z"].as<bool>();
    
		 if (switch3 == true && lin_planning6_x == true)
        {
        d6(0) = - k6_dx*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().x() + k6_px*(d3_clb_0-contacts[FRAME_ID_TORSO_2_LINK].position.translation().x()) - a_drift_torso_2_link.linear().x();
      
       } else {
       d6(0) = - k6_dx*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().x() + k6_px*(x6-contacts[FRAME_ID_TORSO_2_LINK].position.translation().x()) - a_drift_torso_2_link.linear().x();
      }
       
       if (switch3 == true && lin_planning6_y == true)
        {
         d6(1) = - k6_dy*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().y() + k6_py*(d3_clb_1-contacts[FRAME_ID_TORSO_2_LINK].position.translation().y()) - a_drift_torso_2_link.linear().y();
       } else {
       	 d6(1) = - k6_dy*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().y() + k6_py*(y6-contacts[FRAME_ID_TORSO_2_LINK].position.translation().y()) - a_drift_torso_2_link.linear().y();
       	 }
       
       if (switch3 == true && lin_planning6_z == true)
        {
        d6(2) = - k6_dz*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().z() + k6_pz*(d3_clb_2-contacts[FRAME_ID_TORSO_2_LINK].position.translation().z()) - a_drift_torso_2_link.linear().z();
       } else {
      d6(2) = - k6_dz*contacts[FRAME_ID_TORSO_2_LINK].velocity.linear().z() + k6_pz*(z6-contacts[FRAME_ID_TORSO_2_LINK].position.translation().z()) - a_drift_torso_2_link.linear().z();
       }
		
		
		/*// Get the difference between a reference rotation and the state rotation under log function
		//pinocchio::Quaternion quat_ref_2(1, 0, 0, 0)
		// const pinocchio::SE3::Quaternion quat_ref_1(Eigen::Quaterniond(1, 0, 0, 0));// Eigen::Quaterniond(1, 0, 0, 0)
		Matrix3d rot_ref_5= Matrix3d::Identity();
		Matrix3d rot_diff_5 = rot_ref_5 * contacts[FRAME_ID_BASE_LINK].position.rotation().transpose();
		Vector3d log3_5 = pinocchio::log3(rot_diff_5);
		
		double k5_ptheta1 = conf["hip"]["gains"]["ptheta1"].as<double>(); //50.0;
		double k5_ptheta2 = conf["hip"]["gains"]["ptheta2"].as<double>();
		double k5_ptheta3 = conf["hip"]["gains"]["ptheta3"].as<double>();
    double k5_dtheta1 = 2.0*sqrt(k5_ptheta1);
    double k5_dtheta2 = 2.0*sqrt(k5_ptheta2);
    double k5_dtheta3 = 2.0*sqrt(k5_ptheta3);
		
		d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() + k5_ptheta1*(log3_5(0)) - a_drift_base_link.angular().x();
		d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() + k5_ptheta2*(log3_5(1)) - a_drift_base_link.angular().y();
		d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() + k5_ptheta3*(log3_5(2)) - a_drift_base_link.angular().z();*/
		
		// Desired orientation
    double orientation_w6 = conf["torso"]["orientation_ref"]["w"].as<double>();
		double orientation_x6 = conf["torso"]["orientation_ref"]["x"].as<double>(); //-0.019774711256;
		double orientation_y6 = conf["torso"]["orientation_ref"]["y"].as<double>(); //0.0904920792963; 
    double orientation_z6 = conf["torso"]["orientation_ref"]["z"].as<double>();
    
    const pinocchio::SE3 torso_2_link_ref = pinocchio::SE3(Eigen::Quaterniond(orientation_w6, orientation_x6, orientation_y6, orientation_z6), Eigen::Vector3d(0, 0, 0));
    
    // Get the difference between a reference rotation and the state rotation under log function
		//pinocchio::Quaternion quat_ref_2(1, 0, 0, 0);
		
		/*const pinocchio::SE3::Quaternion quat_ref_1(Eigen::Quaterniond(1, 0, 0, 0));// Eigen::Quaterniond(1, 0, 0, 0);
		std::cout << "Normalizes quaternion pin" << quat_ref_2 << std::endl;*/
		
		Matrix3d rot_ref_6 = torso_2_link_ref.rotation(); //Matrix3d::Identity();
		//Matrix3d rot_diff_5 = rot_ref_5 * contacts[FRAME_ID_BASE_LINK].position.rotation().transpose();
		Matrix3d rot_diff_6 = contacts[FRAME_ID_TORSO_2_LINK].position.rotation() * rot_ref_6.transpose();
		Vector3d log3_6 = pinocchio::log3(rot_diff_6);
		
		double k6_ptheta1 = conf["torso"]["gains"]["ptheta1"].as<double>(); //50.0;
		double k6_ptheta2 = conf["torso"]["gains"]["ptheta2"].as<double>();
		double k6_ptheta3 = conf["torso"]["gains"]["ptheta3"].as<double>();
    double k6_dtheta1 = 2.0*sqrt(k6_ptheta1);
    double k6_dtheta2 = 2.0*sqrt(k6_ptheta2);
    double k6_dtheta3 = 2.0*sqrt(k6_ptheta3);
		
		/*d5(3) = - k5_dtheta1*contacts[FRAME_ID_BASE_LINK].velocity.angular().x() + k5_ptheta1*(log3_5(0)) - a_drift_base_link.angular().x();
		d5(4) = - k5_dtheta2*contacts[FRAME_ID_BASE_LINK].velocity.angular().y() + k5_ptheta2*(log3_5(1)) - a_drift_base_link.angular().y();
		d5(5) = - k5_dtheta3*contacts[FRAME_ID_BASE_LINK].velocity.angular().z() + k5_ptheta3*(log3_5(2)) - a_drift_base_link.angular().z();*/
		
		d6(3) = - k6_dtheta1*contacts[FRAME_ID_TORSO_2_LINK].velocity.angular().x() - k6_ptheta1*(log3_6(0)) - a_drift_torso_2_link.angular().x();
		d6(4) = - k6_dtheta2*contacts[FRAME_ID_TORSO_2_LINK].velocity.angular().y() - k6_ptheta2*(log3_6(1)) - a_drift_torso_2_link.angular().y();
		d6(5) = - k6_dtheta3*contacts[FRAME_ID_TORSO_2_LINK].velocity.angular().z() - k6_ptheta3*(log3_6(2)) - a_drift_torso_2_link.angular().z();
		
    bool S6_linear = conf["torso"]["selection_matrix"]["linear"].as<bool>();
    bool S6_angular = conf["torso"]["selection_matrix"]["angular"].as<bool>();
    
    /*// Selection matrix for getting linear or angular task
    Eigen::MatrixXd S6 = Eigen::MatrixXd::Zero(6,6);
    
    if (S6_linear == true && S6_angular == true) {
    S6.block<3,3>(0,0).setIdentity(3,3);
    S6.block<3,3>(3,3).setIdentity(3,3);
    } else if (S6_linear == true && S6_angular == false) {
    S6.block<3,3>(0,0).setIdentity(3,3);
    } else if (S6_linear == false && S6_angular == true) {
    S6.block<3,3>(3,3).setIdentity(3,3);
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular torso control in the yaml file!!! ");
    }*/
    
     // Selection matrix for getting linear or angular task
    Eigen::MatrixXd S6 = Eigen::MatrixXd::Identity(6,6);
    
    if (S6_linear == true && S6_angular == true) {
    Eigen::MatrixXd S6 = Eigen::MatrixXd::Identity(6,6);
    } else if (S6_linear == true && S6_angular == false) {
    S6 = S_l;
    } else if (S6_linear == false && S6_angular == true) {
    S6 = S_a;
    } else {
    ROS_INFO("You must switch at least ON, either for linear or angular com control in the yaml file!!! ");
    }
    

  	/////////////////////////////////////////////

    // Declare blocks
    Eigen::MatrixXd block_1;
    Eigen::MatrixXd block_2;
    Eigen::MatrixXd block_3;
    Eigen::MatrixXd block_4;
    
    Eigen::MatrixXd h2; // Declare block h2 of h

    // Initializing 
    block_1.setIdentity(dof,dof);
    block_2.setConstant(dof,m*c,0);
    block_3.setConstant(m*c,dof,0);
    block_4.setIdentity(m*c,m*c);
		
		// Giving values for the hessian H
		H.setIdentity(size, size);
		
		////////////////////////////////////////////////////////////////////////////////////////////
		H.block<36,36>(0,0) = (2.0*gamma1*block_1) + (2.0*w4*J4.transpose()*S4.transpose()*S4*J4) + (2.0*w3*J3.transpose()*S3.transpose()*S3*J3) + (2.0*w1*J1.transpose()*S1.transpose()*S1*J1) + (2.0*w2*J2.transpose()*S2.transpose()*S2*J2) + (2.0*w5*J5.transpose()*S5.transpose()*S5*J5) + (2.0*w6*J6.transpose()*S6.transpose()*S6*J6);
		///////////////////////////////////////////////////////////////////////////////////////////
		
		// This is when the Hessian matrix is multiplied by a constant 1/2
		/*H.block<36,36>(0,0) = (gamma1*block_1) + (w4*J4.transpose()*S4.transpose()*S4*J4) + (w3*J3.transpose()*S3.transpose()*S3*J3) + (w1*J1.transpose()*S1.transpose()*S1*J1) + (w2*J2.transpose()*S2.transpose()*S2*J2) + (w5*J5.transpose()*S5.transpose()*S5*J5) + (w6*J6.transpose()*S6.transpose()*S6*J6);*/
		
		/*H.block<36,36>(0,0) = (2.0*gamma1*block_1) + (2.0*w4*J4.transpose()*J4) + (2.0*w3*J3.transpose()*S3.transpose()*S3*J3) + (2.0*w1*J1.transpose()*S1.transpose()*S1*J1) + (2.0*w2*J2.transpose()*S2.transpose()*S2*J2) + (2.0*w5*J5.transpose()*S5.transpose()*S5*J5);*/
		
		/*H.block<36,36>(0,0) = (2.0*gamma1*block_1) + (2.0*w4*J4.transpose()*J4) + (2.0*w3*J3.transpose()*S.transpose()*S*J3) + (2.0*w1*J1.transpose()*J1) + (2.0*w2*J2.transpose()*J2) + (2.0*w5*J5.transpose()*S_ang.transpose()*S_ang*J5);*/
		
		/*H.block<36,36>(0,0) = (2.0*gamma1*block_1) + (2.0*w4*J4.transpose()*J4) + (2.0*w3*J3.transpose()*S.transpose()*S*J3) + (2.0*w1*J1.transpose()*S.transpose()*S*J1) + (2.0*w2*J2.transpose()*S.transpose()*S*J2) + (2.0*w5*J5.transpose()*S_ang.transpose()*S_ang*J5);*/
		
    /*H.block<36,36>(0,0) = (2.0*gamma1*block_1) + (2.0*w4*J4.transpose()*J4) + (2.0*w3*J3.transpose()*J3) + (2.0*w1*J1.transpose()*S.transpose()*S*J1) + (2.0*w2*J2.transpose()*S.transpose()*S*J2) + (2.0*w5*J5.transpose()*J5);*/
    
   /*H.block<36,36>(0,0) = (2.0*gamma1*block_1) + (2.0*w4*J4.transpose()*J4) + (2.0*w3*J3.transpose()*J3) + (2.0*w1*J1.transpose()*S.transpose()*S*J1) + (2.0*w2*J2.transpose()*S.transpose()*S*J2) + (2.0*w5*J5.transpose()*S.transpose()*S*J5);*/
    
     //+ (2*w4*J4.transpose()*J4);
    H.block<36,8>(0,dof) = block_2;
    H.block<8,36>(dof,0) = block_3;
    H.block<8,8>(dof,dof) = (2.0*gamma2)*block_4;
		
		// Giving values of the gradient h
    h.setRandom(size);
    
    ///////////////////////////////////////////////////////////////////////////////////////////////
    h.block<36,1>(0,0) = -2.0*w4*J4.transpose()*S4.transpose()*S4*d4 - 2.0*w3*J3.transpose()*S3.transpose()*S3*d3 - 2.0*w1*J1.transpose()*S1.transpose()*S1*d1 - 2.0*w2*J2.transpose()*S2.transpose()*S2*d2 - 2.0*w5*J5.transpose()*S5.transpose()*S5*d5 - 2.0*w6*J6.transpose()*S6.transpose()*S6*d6;
    //////////////////////////////////////////////////////////////////////////////////////////////
    
    // This is when the gradient matrix is multiplied by 1/2
    /*h.block<36,1>(0,0) = -w4*J4.transpose()*S4.transpose()*S4*d4 - w3*J3.transpose()*S3.transpose()*S3*d3 - w1*J1.transpose()*S1.transpose()*S1*d1 - w2*J2.transpose()*S2.transpose()*S2*d2 - w5*J5.transpose()*S5.transpose()*S5*d5 - w6*J6.transpose()*S6.transpose()*S6*d6;*/
    
    /*h.block<36,1>(0,0) = -2.0*w4*J4.transpose()*d4 - 2.0*w3*J3.transpose()*S3.transpose()*S3*d3 - 2.0*w1*J1.transpose()*S1.transpose()*S1*d1 - 2.0*w2*J2.transpose()*S2.transpose()*S2*d2 - 2.0*w5*J5.transpose()*S5.transpose()*S5*d5;*/
    
    /*h.block<36,1>(0,0) = -2.0*w4*J4.transpose()*d4 - 2.0*w3*J3.transpose()*S.transpose()*S*d3 - 2.0*w1*J1.transpose()*d1 - 2.0*w2*J2.transpose()*d2 - 2.0*w5*J5.transpose()*S_ang.transpose()*S_ang*d5;*/
    
    /*h.block<36,1>(0,0) = -2.0*w4*J4.transpose()*d4 - 2.0*w3*J3.transpose()*S.transpose()*S*d3 - 2.0*w1*J1.transpose()*S.transpose()*S*d1 - 2.0*w2*J2.transpose()*S.transpose()*S*d2 - 2.0*w5*J5.transpose()*S_ang.transpose()*S_ang*d5;*/
    
		/*h.block<36,1>(0,0) = -2.0*w4*J4.transpose()*d4 - 2.0*w3*J3.transpose()*d3 - 2.0*w1*J1.transpose()*S.transpose()*S*d1 - 2.0*w2*J2.transpose()*S.transpose()*S*d2 - 2.0*w5*J5.transpose()*d5;*/
		
		/*h.block<36,1>(0,0) = -2.0*w4*J4.transpose()*d4 - 2.0*w3*J3.transpose()*d3 - 2.0*w1*J1.transpose()*S.transpose()*S*d1 - 2.0*w2*J2.transpose()*S.transpose()*S*d2 - 2.0*w5*J5.transpose()*S.transpose()*S*d5;*/
		
		 //- (2*w4*J4.transpose()*d4); 
		h2.setConstant(m*c,1,0);
		h.block<8,1>(dof,0) = h2; 
		
		
    // Base of polyhedral
    double mu = 0.8; // Friction
    Eigen::MatrixXd U(3,4);
    U << mu, 0, -mu,  0,
    		 0,  mu,  0, -mu,
    		 1,  1,   1,  1;
    		 
    for (int i = 0; i < U.cols(); i++)
    		U.col(i).normalize();
    
    // Getting grasp map Q1 for right foot
    Eigen::MatrixXd Q1;
    Q1.setConstant(6,4,1);
    Q1.block<3,4>(0,0) = U;
    Eigen::Vector3d p1(contacts[FRAME_ID_LEFT].position.translation().x() - data.com[0](0), contacts[FRAME_ID_LEFT].position.translation().y() - data.com[0](1), contacts[FRAME_ID_LEFT].position.translation().z() - data.com[0](2));
    
    for(int i = 0; i < 4; ++i)
    {
    Q1.block<3,1>(3,i) = p1.cross(U.block<3,1>(0,i));
    }
  	
  	// Getting grasp map Q2 for left foot
    Eigen::MatrixXd Q2;
    Q2.setConstant(6,4,1);
    Q2.block<3,4>(0,0) = U;
    Eigen::Vector3d p2(contacts[FRAME_ID_RIGHT].position.translation().x() - data.com[0](0), contacts[FRAME_ID_RIGHT].position.translation().y() - data.com[0](1), contacts[FRAME_ID_RIGHT].position.translation().z() - data.com[0](2));
    
    for(int i = 0; i < 4; ++i)
    {
    Q2.block<3,1>(3,i) = p2.cross(U.block<3,1>(0,i));
    }
    
    // Giving values of A from data.Ag, Q1, Q2 blocks 
    A.resize(num_ctr, size);
    A.setZero();
    // Setting the values for equality constraints
    A.block<6,36>(0,0) = data.Ag; //data.dAg
    A.block<6,4>(0,36) = -Q1;
    A.block<6,4>(0,40) = -Q2;
    // Setting the values for inequality constraints for joint position q
    A.block<30,36>(6,0) = S_act_ddq;
    A.block<30,4>(6,36) = Eigen::MatrixXd::Zero(30,4);
		A.block<30,4>(6,40) = Eigen::MatrixXd::Zero(30,4);
		// Setting the values for inequality constraints for joint velocity dq
		A.block<30,36>(36,0) = S_act_ddq;
    A.block<30,4>(36,36) = Eigen::MatrixXd::Zero(30,4);
		A.block<30,4>(36,40) = Eigen::MatrixXd::Zero(30,4);
		// Setting the values for inequality constraints for force intensities rho
    A.block<4,4>(66,36) = Eigen::MatrixXd::Identity(4,4);
		A.block<4,4>(70,40) = Eigen::MatrixXd::Identity(4,4);
		// Taking into account rigid contacts 
		A.block<3,36>(74,0) = J1.block<3,36>(0,0);
		A.block<3,36>(77,0) = J2.block<3,36>(0,0);
		
		/*A.block<6,36>(74,0) = J1;
		A.block<6,36>(80,0) = J2;*/
		
    Alb.resize(num_ctr);
    Aub.resize(num_ctr);
    Eigen::VectorXd Wg(6,1);
    Wg << 0, 0, -M*9.81, 0, 0, 0; // Gravity force
    
    Eigen::VectorXd Wc1(6,1);
    Wc1.block<3,1>(0,0) = contacts["left_sole_link"].force.linear();
    Wc1.block<3,1>(3,0) = contacts["left_sole_link"].force.angular();
    
    Eigen::VectorXd Wc2(6,1);
    Wc2.block<3,1>(0,0) = contacts["right_sole_link"].force.linear();
    Wc2.block<3,1>(3,0) = contacts["right_sole_link"].force.angular();
    
    /*Alb << Wg + Wc1 + Wc2 - data.dAg * v;
    Aub << Wg + Wc1 + Wc2 - data.dAg * v;
    */
    Eigen::VectorXd momenta_drift(6,1);
    momenta_drift << momenta_rate_drift.linear().x(), momenta_rate_drift.linear().y(), momenta_rate_drift.linear().z(), momenta_rate_drift.angular().x(), momenta_rate_drift.angular().y(), momenta_rate_drift.angular().z();
    
    /*Alb << Wg - data.dAg * v;
    Aub << Wg - data.dAg * v;*/
    
    
    

    /*lb.resize(size);
    ub.resize(size);
    lb.head(36).setConstant(-10000);
    lb.tail(8).setConstant(0);
    ub.head(36).setConstant(10000);
    ub.tail(8).setConstant(10000000);*/
    
   	double lb_joint = conf["q_joint_ref"]["lb_angle"]["lb"].as<double>();
   	double ub_joint = conf["q_joint_ref"]["ub_angle"]["ub"].as<double>();
   	
   	// Setting the bounds for joint position 
    Eigen::VectorXd lb_bound;
    lb_bound.setConstant(30,lb_joint);
    for (int i = 0; i < 30; ++i) {
    	lb_bound(i) = conf["q_joint_ref"]["lb_joint_angle"]["j" + std::to_string(i+1)].as<double>()*(3.14/180.0);
    }
    
    Eigen::VectorXd ub_bound;
    ub_bound.setConstant(30,ub_joint);
    for (int i = 0; i < 30; ++i) {
    	ub_bound(i) = conf["q_joint_ref"]["ub_joint_angle"]["j" + std::to_string(i+1)].as<double>()*(3.14/180.0);
    }
    
    //Setting the bounds for joint velocity
    Eigen::VectorXd lb_bound_dq;
    lb_bound_dq.setConstant(30,lb_joint);
    
    Eigen::VectorXd ub_bound_dq;
    ub_bound_dq.setConstant(30,ub_joint);
    
    lb.resize(size);
    ub.resize(size);
    lb.head(36).setConstant(-10000);
    lb.tail(8).setConstant(0);
    ub.head(36).setConstant(10000);
    ub.tail(8).setConstant(10000000);
    
    //q.tail(30) + v.tail(30)*dt + a_des.tail(30)*(dt*dt/2)
    
    
    /*lb.segment(6,30) << (2.0/dt*dt) * (lb_bound - q.tail(30) - v.tail(30)*dt);
    ub.segment(6,30) << (2.0/dt*dt) * (ub_bound - q.tail(30) - v.tail(30)*dt);*/
		
		// Setting values for the inequality constrains //
		
		Alb.block<6,1>(0,0) = Wg - momenta_drift;
    Alb.block<30,1>(6,0) = (2.0/dt*dt) * (lb_bound - q.tail(30) - v.tail(30)*dt);
    Alb.block<30,1>(36,0) = (1.0/dt) * (lb_bound_dq - v.tail(30));
    Alb.block<4,1>(66,0) = Eigen::VectorXd::Constant(4,0.0);
    Alb.block<4,1>(70,0) = Eigen::VectorXd::Constant(4,0.0);  //a_drift_leg_left.linear()
    // Lower bound contact inequality
    Alb.block<3,1>(74,0) = -a_drift_leg_left.linear();
		Alb.block<3,1>(77,0) = -a_drift_leg_right.linear();
		/*Alb.block<3,1>(74,0) = -a_drift_leg_left.linear();
		Alb.block<3,1>(77,0) = -a_drift_leg_left.angular();
		Alb.block<3,1>(80,0) = -a_drift_leg_right.linear();
		Alb.block<3,1>(83,0) = -a_drift_leg_right.angular();*/
		
    Aub.block<6,1>(0,0) = Wg - momenta_drift;
    Aub.block<30,1>(6,0) = (2.0/dt*dt) * (ub_bound - q.tail(30) - v.tail(30)*dt);
    Aub.block<30,1>(36,0) = (1.0/dt) * (ub_bound_dq - v.tail(30));
    Alb.block<4,1>(66,0) = Eigen::VectorXd::Constant(4,1000000.0);
    Alb.block<4,1>(70,0) = Eigen::VectorXd::Constant(4,1000000.0);
    // Upper bound contact inequality
    Aub.block<3,1>(74,0) = -a_drift_leg_left.linear();
		Aub.block<3,1>(77,0) = -a_drift_leg_right.linear();
		/*Aub.block<3,1>(74,0) = -a_drift_leg_left.linear();
		Aub.block<3,1>(77,0) = -a_drift_leg_left.angular();
		Aub.block<3,1>(80,0) = -a_drift_leg_right.linear();
		Aub.block<3,1>(83,0) = -a_drift_leg_right.angular();*/
		
		/*
    lb << -20000, -5, -5, -5, -5, -5, -20000, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, 0, 0, 0, 0, 0, 0, 0, 0;
    ub << 5, 5, 5, 5, 5, 5, 100000, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 20, 20, 20, 20, 20, 20, 20, 20;
		*/

    qpmad::Solver solver;

    qpmad::Solver::ReturnStatus status = solver.solve(x, H, h, A, Alb, Aub);
    Eigen::VectorXd a_des;
    a_des = x.head(36);
    
    //////////////////////////////////////////////////////////////////////////////////////
    
     // Getting grasp map Q1 for right foot
   
    Q1.block<3,4>(0,0) = U;
    p1(0) = contacts[FRAME_ID_LEFT].position.translation().x();
    p1(1) = contacts[FRAME_ID_LEFT].position.translation().y();
    p1(2) = contacts[FRAME_ID_LEFT].position.translation().z();
    
    for(int i = 0; i < 4; ++i)
    {
    Q1.block<3,1>(3,i) = p1.cross(U.block<3,1>(0,i));
    }
  	
  	// Getting grasp map Q2 for left foot
   
    Q2.block<3,4>(0,0) = U;
    p2(0) = contacts[FRAME_ID_RIGHT].position.translation().x();
    p2(1) = contacts[FRAME_ID_RIGHT].position.translation().y();
    p2(2) = contacts[FRAME_ID_RIGHT].position.translation().z();
    
    for(int i = 0; i < 4; ++i)
    {
    Q2.block<3,1>(3,i) = p2.cross(U.block<3,1>(0,i));
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////
    
    Eigen::VectorXd Wgr_1 = Q1*x.segment(36,4);
    Eigen::VectorXd Wgr_2 = Q2*x.segment(40,4);
    
    pinocchio::Force F1 = pinocchio::Force(Wgr_1.segment(0,3), Wgr_1.segment(3,3));
    pinocchio::Force F2 = pinocchio::Force(Wgr_2.segment(0,3), Wgr_2.segment(3,3));
    /*Eigen::VectorXd fex = Eigen::VectorXd::Zero(3);
    fex(0) = 1;
    fex(1) = 1;*/
  	
  		/*
    const container::aligned_vector<Force> & fext = Eigen::VectorXd::Zero(36); // (size_t)model.njoints
  
    
  
  	fext = Eigen::VectorXd::Zero((size_t)model.njoints);
  	
  	for (ForceVector::iterator it = fext.begin(); it ! = fext.end(); ++it)
  	(*it).setRandom();
  	*/
  	
  	/*const Eigen::VectorXd & fex = J1.transpose().block<36,3>(0,0)*contacts["left_sole_link"].force.linear()
  															+ J2.transpose().block<36,3>(0,0)*contacts["right_sole_link"].force.linear();*/
  	
  
  										
  	/*const Eigen::VectorXd & fex = J1.transpose().block<36,3>(0,0)*contacts["left_sole_link"].force.linear()
  															+ J2.transpose().block<36,3>(0,0)*contacts["right_sole_link"].force.linear();*/
  															
  	//const Eigen::VectorXd & fex = J1.transpose().block<36,3>(0,0)*F2.linear() + J2.transpose().block<36,3>(0,0)*F1.linear();
  															
  															
  	const Eigen::VectorXd & fex = J1.transpose()*Wgr_1 + J2.transpose()*Wgr_2;
  	
  	typedef container::aligned_vector<Force> ForceVector;
  
  	ForceVector fext((size_t)model.njoints - 2);
  	ForceVector::iterator it;
  	for (it = fext.begin(); it != fext.end(); ++it) {
  	(*it).setZero();
  	//std::cout << "it" << *it << std::endl;
  	}
  	//fext[3] = contacts["left_sole_link"].force;
  	//fext[9] = contacts["right_sole_link"].force;
  	
  	fext[5] = F1; //contacts["left_sole_link"].force;
  	fext[11] = F2; // contacts["right_sole_link"].force;
  	
  	/*for (it = fext.begin(); it != fext.end(); ++it) {
  	//(*it).setZero();
  	std::cout << "it" << *it << std::endl;
  	}*/
  	
  	
  	/*for (it = fext.begin(); it != fext.end(); ++it) {
  	if (it = 5) {
  	*it = contacts["left_sole_link"].force.linear();
  	}
  	else {
  	(*it).setZero();
  	}
  	std::cout << "it" << *it << std::endl;
  	}*/
  															
    const Eigen::VectorXd &tau_des = pinocchio::rnea(model, data, q, v, a_des, fext);// + fex;
    const Eigen::VectorXd &tau_d = pinocchio::rnea(model, data, q, v, a_des) - fex;
    const Eigen::VectorXd &a_d = pinocchio::aba(model, data, q, v, tau, fext);
    
    /*for(int i = 0; i < r_joints.joints.size(); i++)
			{
				//Setup effort
				r_joints.joints[i].effort = tau_des(i+6);
			}
		*/
			
   // Torque control along with feedback
   	
   /*	Eigen::VectorXd v_des = v + a_des*dt;
   	const Eigen::VectorXd &q_des = pinocchio::integrate(model, q, v_des*dt);*/
   	
   	//Eigen::VectorXd q_des = q.tail(30) + dq_des*dt;
   	
   	///////////////////////////////////////////////////////
   	Eigen::VectorXd v_mean = v + 0.5*a_des*dt;
   	Eigen::VectorXd v_des = v + a_des*dt;
   	//v += dt*a_des;
   	const Eigen::VectorXd &q_des = pinocchio::integrate(model, q, v_mean*dt);
   	
   	//const Eigen::VectorXd &q_des = q.tail(30) + v.tail(30)*dt + a_des.tail(30)*(dt*dt/2);
   	
   	Eigen::VectorXd dq_des = v.tail(30) + a_des.tail(30)*dt; //+ a_des.tail(30)*(dt*dt/2)
   	Eigen::VectorXd q_d = q.tail(30) + dq_des*dt;
    /*const Eigen::VectorXd &dq_des = pinocchio::integrate(model, v.tail(30), a_des.tail(30)*dt);
    const Eigen::VectorXd &q_d = pinocchio::integrate(model, q.tail(30), dq_des*dt);*/
    
    double p_torque = conf["torque_control"]["gains"]["p"].as<double>();
    double d_torque = 2.0*sqrt(p_torque);
    //double d_torque = conf["torque_control"]["gains"]["d"].as<double>();
    
   	const Eigen::VectorXd &tau_control = tau_d.tail(30) + p_torque*(q_des.tail(30) - q.tail(30)) + d_torque*(v_des.tail(30) - v.tail(30)); 
    	
    	// Get the desired joint position
    	for(int i = 0; i < r_joints.joints.size(); i++)
			{
				//Setup effort
				//r_joints.joints[i].position = q_des.tail(30)(i);
				r_joints.joints[i].position = q_d(i);
			}
			
			/*r_joints.joints[2].position = 0.0;
			r_joints.joints[8].position = 0.0;*/
			
			/*
			r_joints.joints[3].position = 0.0;
			r_joints.joints[10].position = 0.0;*/
			
			
			/*r_joints.joints[4].position = 0.0;
			r_joints.joints[11].position = 0.0;*/
			
			
			// Get the desired torque
			for(int i = 0; i < r_joints.joints.size(); i++)
			{
				//Setup effort
				r_joints.joints[i].effort = tau_control(i);
			}
			
			///////////////////////////////////////////////////7
				r_joints.header.stamp = ros::Time::now();
			effort_pub.publish(r_joints);
			
			
			
			//const Eigen::VectorXd & a = pinocchio::computeForwardDynamics(model,data,q,v,tau);

    
   /* 
    std::cout << " Basic Jacobian " << data.J << std::endl;
    std::cout << "  Size of basic Jacobian " << data.J.size() << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << data.J.block<6,6>(0,0) << std::endl;
    std::cout << "  S2 to S7 " << std::endl;
    std::cout << data.J.block<6,6>(0,1) << std::endl;
    std::cout << " A " << A << std::endl;
    std::cout << " Alb " << Alb << std::endl;
    std::cout << " tau " << tau << std::endl;
    std::cout << " Q1 " << Q1 << std::endl;
    std::cout << " Q2 " << Q2 << std::endl;
    std::cout << " Jacobian left foot " << data.a[JOINT_ID]  << std::endl; 
    */
    
   // std::cout << " f_ex dim " << fex.size() << std::endl;
   
   ////////////////////
   /*
   // Print out the placement of each joint of the kinematic tree
   for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "
              << std::fixed << std::setprecision(2)
              << data.oMi[joint_id] //.translation().transpose()
              << std::endl;
              
    std::cout << " Inverse of placement oM3" << data.oMi[3].inverse() << std::endl;
    //std::cout << " Id" << data.oMi[3] *data.oMi[3].inverse() << std::endl;
    std::cout << " 0M4" << data.oMi[3] * data.oMi[3].inverse() * data.oMi[4]  << std::endl;
    std::cout << " 1M2" << data.oMi[1].inverse() * data.oMi[2]  << std::endl;
    std::cout << " 3M4" << data.oMi[3].inverse() * data.oMi[4]  << std::endl;
    std::cout << " 5M6" << data.oMi[5].inverse() * data.oMi[6]  << std::endl;
    
    std::cout << " Adjoint Matrix for Motion 0M1" << std::endl; // Adjoint Matrix for Motion
    std::cout << data.oMi[1].toActionMatrix() << std::endl;
    
    std::cout << " Adjoint Matrix for force 0M1" << std::endl; // Adjoint Matrix for Force
    std::cout << data.oMi[1].toActionMatrix().inverse() << std::endl;
    
    std::cout << " frame_id_left " << frame_id_left << std::endl;
    std::cout << " frame_id_right " << frame_id_right << std::endl;
    
    // Print spatial and hybrid Jacobians for left_sole_link
    std::cout << " Left spatial Jacobian " << J_spatial_left << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << J_spatial_left.block<6,20>(0,0) << std::endl;
    
    std::cout << " Left hybrid Jacobian " << J_hybrid_left << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << J_hybrid_left.block<6,20>(0,0) << std::endl;
    
    // Print spatial and hybrid Jacobians for right_sole_link
    std::cout << " Right spatial Jacobian " << J_spatial_right << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << J_spatial_right.block<6,20>(0,0) << std::endl;
    
    std::cout << " Right hybrid Jacobian " << J_hybrid_right << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << J_hybrid_right.block<6,20>(0,0) << std::endl;
    
    // Dual action matrix
    std::cout << " Dual action matrix " << std::endl;
    std::cout << data.oMf[18].toDualActionMatrix() << std::endl;
    
    // Contact force magnitude
    std::cout << " Contact force magnitude " << std::endl;
    std::cout << data.lambda_c << std::endl;
    
    // Force 
    //std::cout << " Force " << std::endl;
    //std::cout << fext.size() << std::endl;
    
    // Frame Jacobian time variation for left_sole_link
    std::cout << " Left hybrid Jacobian time variation " << dJ_hybrid_left << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << dJ_hybrid_left.block<6,20>(0,0) << std::endl;
    
    // Frame Jacobian time variation for right_sole_link
    std::cout << " Right hybrid Jacobian time variation " << dJ_hybrid_right << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << dJ_hybrid_right.block<6,20>(0,0) << std::endl;
    
    // Frame Jacobian time variation for arm_right_7_link
    std::cout << " Right arm Jacobian " << J_arm_right << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << J_arm_right.block<6,30>(0,0) << std::endl;
    
    // Frame Jacobian time variation for arm_right_7_link
    std::cout << " Right arm Jacobian time variation " << dJ_arm_right << std::endl;
    std::cout << "  S1 " << std::endl;
    std::cout << dJ_arm_right.block<6,30>(0,0) << std::endl;
    */
    // Centroidal drift 
    /*std::cout << "  dAg*v " << std::endl;
    std::cout << data.dAg * v << std::endl;
    
    pinocchio::computeCentroidalMomentumTimeVariation(model, data, q, v, 0*a_des);
    std::cout << " Drift dhg " << std::endl;
    std::cout << data.dhg << std::endl;
    
    const pinocchio::Force & dmomentum = pinocchio::computeCentroidalMomentumTimeVariation(model, data);
    std::cout << " dhg " << std::endl;
    std::cout << dmomentum.linear() << std::endl;
    std::cout << dmomentum.angular() << std::endl;
    
    
    
    std::cout << " size " << std::endl;
    
    std::cout << v.tail(18).size() << std::endl;
    
    std::cout << "H" << std::endl;
    std::cout << H << std::endl;
    std::cout << "h is:" << h.transpose() << std::endl;
    std::cout << "x is:" << x.transpose() << std::endl;
    std::cout << " The size of x is:" << x.size() << std::endl;
    
    std::cout << "d1" << d1 << std::endl;
    
    std::cout << "Contact" << contacts["left_sole_link"].force << std::endl;*/
    
    
    
    std::cout << " The size of q is:" << q.size() << std::endl;
    std::cout << "Mass" << M << std::endl;
    /*std::cout << "x" << x << std::endl;
    std::cout << "Wgr_1" << Wgr_1 << std::endl;
    //std::cout << "F1" << F1 << std::endl;
    std::cout << "tau_d" << tau_d << std::endl;
    std::cout << "U" << std::endl;
    std::cout << U << std::endl;
    std::cout << "Net force" << Wgr_1 + Wgr_2 << std::endl;
    std::cout << "Gravity" << model.gravity << std::endl;
    std::cout << "Mass * gravity_norm " << M * model.gravity.linear().norm() << std::endl;
    
    std::cout << "x1_clb_3:" << x1_clb_3 << std::endl;
    std::cout << "switch1:" << switch1 << std::endl;*/
    
   
    
    
    //std::cout << "fext[3]" << fext[3] << std::endl;
    /*std::cout << "d2" << d2 << std::endl;
    
    std::cout << "S" << S << std::endl;
    std::cout << "d3(2)" << d3(2) << std::endl;*/
    
     // Perform the forward kinematics over the kinematic tree
  /*forwardKinematics(model,data,q);
  // Print out the placement of each joint of the kinematic tree
  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "
              << std::fixed << std::setprecision(2)
              << data.oMi[joint_id].translation().transpose()
              << std::endl;*/
     /*std::cout << "r_joints" << r_joints.joints.size() << std::endl;      
     std::cout << "q" << q_des << std::endl;
     std::cout << "v" << v << std::endl;
     std::cout << "d3" << d3 << std::endl;
     std::cout << "toActionMatrix" << std::endl;
     std::cout <<  oM_com.toActionMatrix() << std::endl;
     std::cout << "toDualActionMatrix" << std::endl;
     std::cout << oM_com.toDualActionMatrix() << std::endl;*/
     
    std::cout << " S5 " << std::endl;
    std::cout << S5 << std::endl;

		std::cout << "d1: \n" << d1 << std::endl;
		std::cout << "d2: \n" << d2 << std::endl;
		std::cout << "d3_clb_1:" << d3_clb_1 << std::endl;
    std::cout << lb.segment(6,30).size() << std::endl;
    std::cout << lb(35) << std::endl;
   /* std::cout << " SE3" << std::endl;
    std::cout << base_link_ref_clb << std::endl;*/
    std::cout << lb(36) << std::endl;
    
 }
//};
	
int main(int argc, char **argv)
{
	//server observer;
	// You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename = "/home/niger/reemc_public_ws/src/reemc_simulation/reemc_gazebo/models/reemc_full/reemc_full_ft_hey5.urdf.urdf";
  
  // Load the urdf model
  Model model;
	pinocchio::JointModelFreeFlyer root_joint;
  pinocchio::urdf::buildModel(urdf_filename, root_joint, model);
	
	// Create data required by the algorithms
  Data data(model);
  
  //Get joints number
			int j_number = (size_t)model.njoints - 2;

			//Get parameters for articulations
			for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints -2 ; joint_id++)
			{
				whole_body_state_msgs::JointState joint;

				//Get joint name
				joint.name = model.names[joint_id + 2]; 

				//Save on vector
				r_joints.joints.push_back(joint);				
			}
  
	std::string effort_topic;
	 
  // Initializing ROS node with a name of demo_topic_subscriber
  ros::init(argc, argv,"qp_solver"); //
  // Created a nodehandle object
  ros::NodeHandle nh;
  // Create a publisher object
  
  ros::Subscriber left_foot_subscriber = nh.subscribe("/multi_reference_left", 1, left_foot_subs);
	ros::Subscriber right_foot_subscriber = nh.subscribe("/multi_reference_right", 1, right_foot_subs);
	ros::Subscriber icp_ref_subscriber = nh.subscribe("/icp_ref", 1, icp_ref_subs);
	//ros::Subscriber left_foot_subscriber = nh.subscribe("/Position_left", 1, left_foot_subs);
  ros::Subscriber joint_state_subscriber = nh.subscribe("/robot_states", 1, robot_state_callback);
  
  
  
  switch(mode)
	{
		case 0:

			//Init publishers
			effort_pub = nh.advertise<whole_body_state_msgs::WholeBodyState>("/reemc/positions", 1); //effort_topic
			
	}
  std::cout << "r_joints_size" << r_joints.joints[1] << std::endl;
  //Ros rate as fps
	ros::Rate loop_rate(100);
	
  //Spinning the node
   while(ros::ok)
   {
    	//if(mode == 0)
		//{
			
			
		// }
		ros::spinOnce();
		//Induced delay
		loop_rate.sleep();
		}
  
  return 0;
}
