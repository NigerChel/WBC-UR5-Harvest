#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Eigen>
#include <qpmad/solver.h>

//Standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
#include <cmath>  // For sqrt

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

#include <urdf/model.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <whole_body_state_msgs/WholeBodyState.h>
#include <whole_body_state_conversions/whole_body_state_interface.h>

//Headers
#include <whole_body_state_msgs/JointState.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>

#include <yaml-cpp/yaml.h>

#include <deque>  // For circular buffer
#include <Eigen/Dense>


using namespace Eigen;
using namespace pinocchio;	

//Read YAML file
	  YAML::Node conf = YAML::LoadFile("/home/niger/ros_ws/src/wbc_ur5/config/real_param_position_ur5_param_pir.yaml"); 

// Estructura para almacenar datos con timestamp
struct TimedData {
    ros::Time timestamp;
    Eigen::Vector3d position; // Posición (x, y, z)
};

// Buffers para almacenar valores históricos
std::deque<TimedData> d1_clb_buffer;
std::deque<TimedData> contact_buffer;
std::deque<std::pair<double, Eigen::Vector3d>> error_buffer; // Buffer de errores

//double delay_time = 0.0; // Tiempo de retraso en segundos}

double delay_time = conf["torque_control"]["gains"]["delay_time"].as<double>();

// Función para obtener un valor retrasado de un buffer con interpolación
Eigen::Vector3d getDelayedValue(std::deque<TimedData>& buffer, double delay) {
    ros::Time target_time = ros::Time::now() - ros::Duration(delay);

    for (size_t i = 1; i < buffer.size(); ++i) {
        if (buffer[i].timestamp >= target_time) {
            // Si coincide exacto con el timestamp
            if (buffer[i].timestamp == target_time) {
                return buffer[i].position;
            } else {
                // Interpolación lineal
                double alpha = (target_time - buffer[i - 1].timestamp).toSec() /
                               (buffer[i].timestamp - buffer[i - 1].timestamp).toSec();
                return (1 - alpha) * buffer[i - 1].position + alpha * buffer[i].position;
            }
        }
    }
    // Si no hay suficiente historial, devolver el más antiguo
    return buffer.front().position;
}

Eigen::Vector3d d1_clb_position; // Stores the latest d1_clb position
Eigen::Vector3d contact_position; // Stores the latest contact position

// Callback para almacenar la posición de d1_clb_*
void contact_ref_subs(const geometry_msgs::Pose &msg1) {
    d1_clb_position = Eigen::Vector3d(msg1.position.x, msg1.position.y, msg1.position.z);
    d1_clb_buffer.push_back({ros::Time::now(), d1_clb_position});

    // Eliminar datos viejos
    while (!d1_clb_buffer.empty() &&
           (ros::Time::now() - d1_clb_buffer.front().timestamp).toSec() > delay_time) {
        d1_clb_buffer.pop_front();
    }
}


// Función para almacenar la posición de contacto
void contact_subs(const whole_body_state_msgs::WholeBodyState &msg) {
	contact_position = Eigen::Vector3d(msg.contacts[0].pose.position.x, 
                                       msg.contacts[0].pose.position.y, 
                                       msg.contacts[0].pose.position.z);
    contact_buffer.push_back({ros::Time::now(), contact_position});

    // Eliminar datos viejos
    while (!contact_buffer.empty() &&
           (ros::Time::now() - contact_buffer.front().timestamp).toSec() > delay_time) {
        contact_buffer.pop_front();
    }
}

// Función para estimar la integral del error usando la Regla del Trapecio
Eigen::Vector3d estimateIntegralError() {
    if (error_buffer.size() < 2) return Eigen::Vector3d::Zero();  // Evitar cálculos si no hay datos suficientes

    Eigen::Vector3d integral_error = Eigen::Vector3d::Zero();
    
    for (size_t i = 1; i < error_buffer.size(); ++i) {
        double dt = error_buffer[i].first - error_buffer[i - 1].first;
        Eigen::Vector3d avg_error = 0.5 * (error_buffer[i].second + error_buffer[i - 1].second);
        integral_error += avg_error * dt;
    }

    return integral_error;
}


// Global variables
Eigen::Vector3d current_error;
Eigen::Vector3d delayed_error;
Eigen::Vector3d integral_error;

// Define ROS publishers
ros::Publisher current_error_pub;
ros::Publisher delayed_error_pub;

// Función para calcular los errores retrasados e integrarlos
void computeDelayedError() {
    double current_time = ros::Time::now().toSec();
    
    Eigen::Vector3d delayed_d1_clb = getDelayedValue(d1_clb_buffer, delay_time);
    Eigen::Vector3d delayed_contact = getDelayedValue(contact_buffer, delay_time);
	 current_error = d1_clb_position - contact_position;
    delayed_error = delayed_d1_clb - delayed_contact;

    // Store the delayed error in the buffer
    error_buffer.push_back({current_time, delayed_error});
    if (error_buffer.size() > 1000) error_buffer.pop_front();  // Limit buffer size

    // Calcular la integral del error
    integral_error = estimateIntegralError();
    
    // Publish current error
    geometry_msgs::Vector3 current_error_msg;
    current_error_msg.x = current_error.x();
    current_error_msg.y = current_error.y();
    current_error_msg.z = current_error.z();
    current_error_pub.publish(current_error_msg);

    // Publish delayed error
    geometry_msgs::Vector3 delayed_error_msg;
   delayed_error_msg.x = delayed_error.x();
    delayed_error_msg.y = delayed_error.y();
    delayed_error_msg.z = delayed_error.z();
    delayed_error_pub.publish(delayed_error_msg);

   ROS_INFO("Delayed errors: ex_t: %f, ey_t: %f, ez_t: %f", delayed_error.x(), delayed_error.y(), delayed_error.z());
    ROS_INFO("Integral error: Int_ex: %f, Int_ey: %f, Int_ez: %f", integral_error.x(), integral_error.y(), integral_error.z());
}




// Define buffers to store past errors
std::deque<double> ex_buffer, ey_buffer, ez_buffer;

// Define integral terms
double integral_ex = 0.0, integral_ey = 0.0, integral_ez = 0.0;

// Control parameters
const double h = 0.02;  // Time step
const double h_past = 0.4;  // Time in the past for t-h
const int buffer_size = static_cast<int>(h_past / h);  // Number of steps to store

//Robot articulations
whole_body_state_msgs::WholeBodyState r_joints;
std::vector<std::string> art_names; 
int art_counter;

//Node configuration
std::string node_path;
int mode;

//Init publishers
ros::Publisher effort_pub; 


		
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
		
///////////////////////////////////////////////////////////
// Define a global variable to store the Twist message
geometry_msgs::Twist ref_twist;

// Callback function to handle the incoming Twist messages
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ref_twist = *msg;
  ROS_INFO("Received Twist message: linear_x=%f, angular_z=%f",
           ref_twist.linear.x, ref_twist.angular.z);
}

// Define a global variable to store the Accel message
geometry_msgs::Accel ref_accel;

// Callback function to handle the incoming Accel messages
void accelCallback(const geometry_msgs::Accel::ConstPtr& msg)
{
  ref_accel = *msg;
  ROS_INFO("Received Accel message: linear_x=%f, angular_z=%f",
           ref_accel.linear.x, ref_accel.angular.z);
}


////////////////////////////////////////////////////////////
//class server {
//	public:
	
 void robot_state_callback(const whole_body_state_msgs::WholeBodyState &msg)
  {
	// You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename_ = "/home/niger/ros_ws/src/wbc_ur5/urdf/ur5e.urdf";

////////////////////////////////////////////////////////////////////
 ros::NodeHandle nh;
 // Get the URDF string from the parameter server
    std::string urdf_string;
    if (!nh.getParam("/robot_description", urdf_string)) {
        ROS_ERROR("Failed to get /robot_description parameter");
        return;
    }

    // Specify the temporary file path
    std::string urdf_filename = "/tmp/robot.urdf";

    // Write the URDF string to the file
    std::ofstream urdf_file(urdf_filename);
    if (!urdf_file) {
        ROS_ERROR("Failed to open file: %s", urdf_filename.c_str());
        return;
    }
    urdf_file << urdf_string;
    urdf_file.close();

    // Print the content of the URDF file
    std::ifstream urdf_file_read(urdf_filename);
    if (!urdf_file_read) {
        ROS_ERROR("Failed to open file: %s", urdf_filename.c_str());
        return;
    }

    std::string line;
    while (std::getline(urdf_file_read, line)) {
        std::cout << line << std::endl;
    }
    urdf_file_read.close();
////////////////////////////////////////////////////////////////////
/*std::string urdf_string;
if (!ros::param::get("/robot_description", urdf_string)) {
    ROS_ERROR("Failed to get /robot_description from parameter server");
    return;
}

std::string urdf_filename = "/tmp/robot.urdf";
std::ofstream urdf_file(urdf_filename);

if (!urdf_file.is_open()) {
    ROS_ERROR("Failed to open file: %s", urdf_filename.c_str());
    return;
}

urdf_file << urdf_string;
urdf_file.close();*/

//////////////////////////////////////////////////////////
  
  // Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
	
	// Create data required by the algorithms
  Data data(model);
  
  Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);

  std::cout << model.nv << std::endl;
  
  //////////////////////////////////
  // Retrieve the generalized position and velocity, and joint torques

   for (std::size_t j = 0; j < msg.joints.size(); ++j) {
      // TODO: Generalize to different floating-base types!
      // TODO: Check if joint exists!
      auto jointId = model.getJointId(msg.joints[j].name)-1;
      q(jointId) = msg.joints[j].position;
      v(jointId) = msg.joints[j].velocity;
      tau(jointId) = msg.joints[j].effort;
    }
    
  
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


		int dof = model.nv; //Number of actuated joints
		int dof_act = model.nv; //DoF of the number of actuated part
		
		qpmad::MatrixIndex size = dof; // x dimension
    qpmad::MatrixIndex num_ctr = 6 + dof_act + dof_act; // Number of constraints that will be taken in the QP
    
		
		/// Weights for QP ///
		double w1 = conf["qp_weight"]["weights"]["w1"].as<double>(); //10.0; //1000.0; 
		double w2 = conf["qp_weight"]["weights"]["w2"].as<double>(); //1000.0;
		double w3 = conf["qp_weight"]["weights"]["w3"].as<double>(); //500.0;
		double w4 = conf["qp_weight"]["weights"]["w4"].as<double>();
		double w5 = conf["qp_weight"]["weights"]["w5"].as<double>();
		double w6 = conf["qp_weight"]["weights"]["w6"].as<double>();
		double dt = conf["torque_control"]["gains"]["dt"].as<double>();
		
		
		double gamma1 = conf["qp_weight"]["weights"]["gamma1"].as<double>();
		double gamma2 = conf["qp_weight"]["weights"]["gamma2"].as<double>();
		
		// Selection matrix
		Eigen::MatrixXd S_l = Eigen::MatrixXd::Zero(3,6);
    S_l.block<3,3>(0,0).setIdentity(3,3);
    Eigen::MatrixXd S_a = Eigen::MatrixXd::Zero(3,6);
    S_a.block<3,3>(0,3).setIdentity(3,3);
    
    Eigen::MatrixXd S_act_ddq = Eigen::MatrixXd::Zero(model.nv, model.nv); // Select the actuated part of all joints  from the joint acceleration
    //S_act_ddq.block(0, 0, dof_act, dof_act).setIdentity();
    S_act_ddq.block(0, 0, 1, 1).setIdentity();
    // Print the resulting matrix
    std::cout << "S_act_ddq with identity block set:\n" << S_act_ddq << std::endl;
		
		 //
		////////////// Tasks (Ji*dv = di) ///////////////
		
		/* Task 1 (Spatial acceleration for left foot pose)*/
		// Get frame_id for left_sole_link
  	//std::string FRAME_ID_LEFT = "camera_link"; //left_sole_link
  	std::string FRAME_ID_LEFT = "gripper_link";  // "wrist_3_joint"; //left_sole_link
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
		
		double ix = conf["leg_left"]["gains"]["ix"].as<double>(); //50.0;
		double iy = conf["leg_left"]["gains"]["iy"].as<double>();
		double iz = conf["leg_left"]["gains"]["iz"].as<double>();
		
		double k1_dx = conf["leg_left"]["gains"]["kx"].as<double>(); //50.0;
		double k1_dy = conf["leg_left"]["gains"]["ky"].as<double>();
		double k1_dz = conf["leg_left"]["gains"]["kz"].as<double>();
    /*double k1_dx = 2.0*sqrt(k1_px);
    double k1_dy = 2.0*sqrt(k1_py);


    double k1_dz = 2.0*sqrt(k1_pz);*/
    
    //Read YAML file

	  // Desired position
		double x1 = conf["leg_left"]["position_ref"]["x"].as<double>(); //-0.019774711256;
		double y1 = conf["leg_left"]["position_ref"]["y"].as<double>(); //0.0904920792963; 
    double z1 = conf["leg_left"]["position_ref"]["z"].as<double>();
    // Desired velocity
		double vx1 = conf["leg_left"]["velocity_ref"]["x"].as<double>(); //-0.019774711256;
		double vy1 = conf["leg_left"]["velocity_ref"]["y"].as<double>(); //0.0904920792963; 
    double vz1 = conf["leg_left"]["velocity_ref"]["z"].as<double>();
  
		pinocchio::forwardKinematics(model,data,q,v,0*v);
		pinocchio::computeJointJacobians(model,data,q);
    pinocchio::framesForwardKinematics(model,data,q);
		
		const pinocchio::Motion &a_drift_leg_left = pinocchio::getFrameClassicalAcceleration(model, data, frame_id_left, pinocchio::LOCAL_WORLD_ALIGNED);
		
		bool lin_planning1_x = conf["leg_left"]["path_planning"]["lin_x"].as<bool>();
		bool lin_planning1_y = conf["leg_left"]["path_planning"]["lin_y"].as<bool>();
		bool lin_planning1_z = conf["leg_left"]["path_planning"]["lin_z"].as<bool>();
		
		//Calculate current errors
    double ex_t = d1_clb_0 - contacts[FRAME_ID_LEFT].position.translation().x();
    double ey_t = d1_clb_1 - contacts[FRAME_ID_LEFT].position.translation().y();
    double ez_t = d1_clb_2 - contacts[FRAME_ID_LEFT].position.translation().z();
    std::cout << typeid(ex_t).name() << std::endl;
    
    /*double ex_t = static_cast<double>(d1_clb_0) - static_cast<double>(contacts[FRAME_ID_LEFT].position.translation().x());
	double ey_t = static_cast<double>(d1_clb_1) - static_cast<double>(contacts[FRAME_ID_LEFT].position.translation().y());
	double ez_t = static_cast<double>(d1_clb_2) - static_cast<double>(contacts[FRAME_ID_LEFT].position.translation().z());*/
	
	//Eigen::VectorXd integral_ex = Eigen::VectorXd::Zero(1);

    //double integral_ex = 0.0;
    // Update integral terms
    integral_ex += ex_t* 0.02;
    integral_ey += ey_t* 0.02;
    integral_ez += ez_t* 0.02;

    // Add current errors to buffers
    ex_buffer.push_back(d1_clb_0 - contacts[FRAME_ID_LEFT].position.translation().x());
    ey_buffer.push_back(d1_clb_1 - contacts[FRAME_ID_LEFT].position.translation().y());
    ez_buffer.push_back(d1_clb_2 - contacts[FRAME_ID_LEFT].position.translation().z());

    // Ensure buffer size does not exceed the required size
    if (ex_buffer.size() > buffer_size) ex_buffer.pop_front();
    if (ey_buffer.size() > buffer_size) ey_buffer.pop_front();
    if (ez_buffer.size() > buffer_size) ez_buffer.pop_front();

    // Retrieve errors from t-h
    double ex_t_h = (ex_buffer.size() >= buffer_size) ? ex_buffer.front() : 0.0;
    double ey_t_h = (ey_buffer.size() >= buffer_size) ? ey_buffer.front() : 0.0;
    double ez_t_h = (ez_buffer.size() >= buffer_size) ? ez_buffer.front() : 0.0;
    const Eigen::VectorXd h_eigen = Eigen::VectorXd::Constant(1, 0.01);
		
		  if (switch1 == true && lin_planning1_x == true)
        {
        d1(0) = ref_accel.linear.x + k1_px * current_error.x() - k1_dx * delayed_error.x()  + ix * integral_error.x()  - a_drift_leg_left.linear().x();
       } else {
       
       d1(0) = k1_dx*(vx1 -contacts[FRAME_ID_LEFT].velocity.linear().x()) + k1_px*(x1-contacts[FRAME_ID_LEFT].position.translation().x()) - a_drift_leg_left.linear().x();
       }
       
       if (switch1 == true && lin_planning1_y == true)
        {
        d1(1) = ref_accel.linear.y + k1_py * current_error.y() - k1_dy * delayed_error.y()  + iy * integral_error.y() - a_drift_leg_left.linear().y();
       } else {
       
		d1(1) = k1_dy * (vy1 - contacts[FRAME_ID_LEFT].velocity.linear().y()) + k1_py*(y1-contacts[FRAME_ID_LEFT].position.translation().y()) - a_drift_leg_left.linear().y();
       }
       
       if (switch1 == true && lin_planning1_z == true)
        {
          d1(2) = ref_accel.linear.z + k1_pz * current_error.z() - k1_dz * delayed_error.z()  + iz * integral_error.z()  - a_drift_leg_left.linear().z();
       } else {
       
		 d1(2) = k1_dz*(vz1 - contacts[FRAME_ID_LEFT].velocity.linear().z()) + k1_pz*(z1-contacts[FRAME_ID_LEFT].position.translation().z()) - a_drift_leg_left.linear().z();
       }
    
		
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
   
  ///////////////////////////////////////////////////////////////////////////////////////////////// 
      /* Task 4 (Joint acceleration for controlling joint space) */
    
    
	Eigen::MatrixXd J4 = Eigen::MatrixXd::Zero(model.nv, model.nv); 
	Eigen::MatrixXd S4 = Eigen::MatrixXd::Zero(6, model.nv);
	//J4.block<6,6>(0,6).setIdentity();
	J4.setIdentity();
	//S4.setIdentity(); // assuming you want S4 to be a full 6x6 identity matrix
	S4.block(0, 0, 1, 1).setIdentity();
	Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(6);

	for (int i = 0; i < 6; ++i) {
		 q_ref(i) = conf["q_joint_ref"]["arm_left"]["j" + std::to_string(i + 1)].as<double>();
	}

	Eigen::VectorXd k4_pjoint(6);
	for (int i = 0; i < 6; ++i) {
		 k4_pjoint(i) = conf["q_joint_ref"]["arm_left_gains"]["pj" + std::to_string(i + 1)].as<double>();
	}

	Eigen::MatrixXd k4_p = k4_pjoint.asDiagonal();
	Eigen::MatrixXd k4_d = (2.0 * k4_pjoint.array().sqrt()).matrix().asDiagonal();

	Eigen::VectorXd d4 = Eigen::VectorXd::Zero(6);
	d4.noalias() = -k4_p * (q.tail(6) - q_ref) - k4_d * v.tail(6);

    
//////////////////////////////////////////////////////////////////

 Eigen::MatrixXd block_1;
  block_1.setIdentity(dof,dof);
		
		// Giving values for the hessian H
		H.setIdentity(size, size);
		
		////////////////////////////////////////////////////////////////////////////////////////////
		 H = (2.0*gamma1*block_1) + (2.0*w1*J1.transpose()*S1.transpose()*S1*J1) + (2.0*w4*J4.transpose()*S4.transpose()*S4*J4);
		///////////////////////////////////////////////////////////////////////////////////////////
		
		// Giving values of the gradient h
    h.setRandom(size);
    
    ///////////////////////////////////////////////////////////////////////////////////////////////
      h = - 2.0*w1*J1.transpose()*S1.transpose()*S1*d1 - 2.0*w4*J4.transpose()*S4.transpose()*S4*d4;
    //////////////////////////////////////////////////////////////////////////////////////////////		
 
    Eigen::Vector3d p1(contacts[FRAME_ID_LEFT].position.translation().x() - data.com[0](0), contacts[FRAME_ID_LEFT].position.translation().y() - data.com[0](1), contacts[FRAME_ID_LEFT].position.translation().z() - data.com[0](2));
    
	//float M = 0.000000000001;
	
	float M = pinocchio::computeTotalMass(model);
	Eigen::VectorXd gravity1(6,1);
	gravity1 = model.gravity.linear();
	std::cout << "Total mass = :\n" << M	 << std::endl;
	std::cout << "Gravity= :\n" << gravity1	 << std::endl;
 
    Eigen::VectorXd Wg(6,1);
    Wg << 0, 0, -M*9.81, 0, 0, 0; // Gravity force
    
   const pinocchio::Force &momenta_rate_drift = pinocchio::computeCentroidalMomentumTimeVariation(model, data,q,v,0*v);
    Eigen::VectorXd momenta_drift(6,1);
    momenta_drift << momenta_rate_drift.linear().x(), momenta_rate_drift.linear().y(), momenta_rate_drift.linear().z(), momenta_rate_drift.angular().x(), momenta_rate_drift.angular().y(), momenta_rate_drift.angular().z();
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	double lb_joint = conf["q_joint_ref"]["lb_angle"]["lb"].as<double>();
   	double ub_joint = conf["q_joint_ref"]["ub_angle"]["ub"].as<double>();
   	
   	// Setting the bounds for joint position 
    Eigen::VectorXd lb_bound;
    lb_bound.setConstant(dof_act,lb_joint);
    for (int i = 0; i < dof_act; ++i) {
    	lb_bound(i) = conf["q_joint_ref"]["lb_joint_angle"]["j" + std::to_string(i+1)].as<double>()*(3.14/180.0);
    }
    
    Eigen::VectorXd ub_bound;
    ub_bound.setConstant(dof_act,ub_joint);
    for (int i = 0; i < dof_act; ++i) {
    	ub_bound(i) = conf["q_joint_ref"]["ub_joint_angle"]["j" + std::to_string(i+1)].as<double>()*(3.14/180.0);
    }
    
    //Setting the bounds for joint velocity
    Eigen::VectorXd lb_bound_dq;
    lb_bound_dq.setConstant(dof_act,lb_joint);
    
    Eigen::VectorXd ub_bound_dq;
    ub_bound_dq.setConstant(dof_act,ub_joint);
    
    lb.resize(size);
    ub.resize(size);
  
	// First, compute the forward kinematics
	pinocchio::forwardKinematics(model, data, q, v);

	// Compute the centroidal momentum matrix
	//pinocchio::computeCentroidalMap(model, data, q);
	
	// Giving values of A from data.Ag, Q1, Q2 blocks 
    A.resize(num_ctr, size);
     //A.setZero();
   
    // Setting the values for equality constraints
  /*  A.block(0,0,3,dof) = data.Ag.topRows(3); //data.dAg
    // Setting the values for inequality constraints for joint position q
    A.block(6,0,dof_act,dof) = S_act_ddq;
		// Setting the values for inequality constraints for joint velocity dq
		A.block(dof + dof, 0, dof_act,dof) = S_act_ddq;*/
		
	
	/* A.resize(num_ctr - 6, size);
    A.setZero();
    // Setting the values for equality constrai
    // Setting the values for inequality constraints for joint position q
    A.block(0,0,dof_act,dof) = S_act_ddq;
		// Setting the values for inequality constraints for joint velocity dq
		A.block(dof, 0, dof_act,dof) = S_act_ddq;*/
		
		std::cout << "A = :\n" << A << std::endl;
		std::cout << "S_act = :\n" << S_act_ddq << std::endl;
		
		std::cout << "Ag = :\n" << data.Ag << std::endl;
		
		
   Alb.resize(num_ctr);
    Aub.resize(num_ctr);
  
// Setting values for the inequality constrains //
		
   /* Alb.block(0,0,6,1) = Wg - momenta_drift;
    Alb.block(6,0,dof_act,1) = (2.0/dt*dt) * (lb_bound - q.tail(dof_act) - v.tail(dof_act)*dt);
    Alb.block(dof_act + dof_act, 0,dof_act,1) = (1.0/dt) * (lb_bound_dq - v.tail(dof_act));
    // Lower bound contact inequality

		
    Aub.block(0,0,6,1) = Wg - momenta_drift;
    Aub.block(6,0,dof_act,1) = (2.0/dt*dt) * (ub_bound - q.tail(dof_act) - v.tail(dof_act)*dt);
    Aub.block(dof_act + dof_act, 0,dof_act, 1) = (1.0/dt) * (ub_bound_dq - v.tail(dof_act));*/
    
    ///////////////////////////////////////////////////////////////////////////////////////7
    
    /* Alb.resize(num_ctr -6);
    Aub.resize(num_ctr - 6);
// Setting values for the inequality constrains //
		
    Alb.block(0,0,dof_act,1) = (2.0/dt*dt) * (lb_bound - q.tail(dof_act) - v.tail(dof_act)*dt);
    Alb.block(dof_act, 0,dof_act,1) = (1.0/dt) * (lb_bound_dq - v.tail(dof_act));
    // Lower bound contact inequality

		
    Aub.block(0,0,dof_act,1) = (2.0/dt*dt) * (ub_bound - q.tail(dof_act) - v.tail(dof_act)*dt);
    Aub.block(dof_act, 0,dof_act, 1) = (1.0/dt) * (ub_bound_dq - v.tail(dof_act));*/
    
    
    ////////////////////////////////////////////////////////////////////////////////////////
    
   
	// Calculate the distance between the first components of lb_bound and q
    double distance_first_component = std::abs(lb_bound(0) - q(0));
    
    // Define a threshold value for the condition
    //double threshold = 0.01;
    
    double threshold = conf["torque_control"]["gains"]["threshold"].as<double>();
    
    if (distance_first_component < threshold) {
        std::cout << "First component distance is less than threshold." << std::endl;
        A.setZero();
        
        // A.block(0,0,3,dof) = data.Ag.topRows(3); //data.dAg
    // Setting the values for inequality constraints for joint position q
    A.block(6,0,dof_act,dof) = S_act_ddq;
		// Setting the values for inequality constraints for joint velocity dq
		A.block(dof + dof, 0, dof_act,dof) = S_act_ddq;
		
		Alb.setZero();
   	Aub.setZero();
		//Alb.block(0,0,6,1) = Wg - momenta_drift;
    Alb.block(6,0,dof_act,1) = S_act_ddq * (2.0/dt*dt) * (lb_bound - q.tail(dof_act) - v.tail(dof_act)*dt);
    Alb.block(dof_act + dof_act, 0,dof_act,1) = S_act_ddq * (1.0/dt) * (lb_bound_dq - v.tail(dof_act));
    // Lower bound contact inequality

	 
    //Aub.block(0,0,6,1) = Wg - momenta_drift;
    Aub.block(6,0,dof_act,1) = S_act_ddq * (2.0/dt*dt) * (ub_bound - q.tail(dof_act) - v.tail(dof_act)*dt);
    Aub.block(dof_act + dof_act, 0,dof_act, 1) = S_act_ddq * (1.0/dt) * (ub_bound_dq - v.tail(dof_act));
		
		
    } else {
        std::cout << "First component distance is greater than or equal to threshold." << std::endl;
        
         A.setZero();
          Alb.setZero();
   		 Aub.setZero();
    }
  
    /////////////////////////////////////////////////////////
    lb.block(0,0,dof_act,1) = (2.0/dt*dt) * (lb_bound - q.tail(dof_act) - v.tail(dof_act)*dt);
	 ub.block(0,0,dof_act,1) = (2.0/dt*dt) * (ub_bound - q.tail(dof_act) - v.tail(dof_act)*dt);
	 
	 std::cout << "Alb = :\n" << Alb << std::endl;
	 std::cout << "Aub = :\n" << Aub << std::endl;
	 
 qpmad::Solver solver;

    //qpmad::Solver::ReturnStatus status = solver.solve(x, H, h, lb, ub);
    qpmad::Solver::ReturnStatus status = solver.solve(x, H, h, A, Alb, Aub);
    //qpmad::Solver::ReturnStatus status = solver.solve(x, H, h, lb, ub, A, Alb, Aub);
    Eigen::VectorXd a_des;
    a_des = x;


	Eigen::VectorXd dq_des = v + a_des*dt; //+ a_des.tail(30)*(dt*dt/2)
   	Eigen::VectorXd q_d = q + dq_des*dt;


    	// Get the desired joint position
    	for(int i = 0; i < r_joints.joints.size(); i++)
			{
				//Setup effort
				//r_joints.joints[i].position = q_des.tail(30)(i);
				r_joints.joints[i].position = q_d(i);
			}
			
			
			// Get the desired torque
			for(int i = 0; i < r_joints.joints.size(); i++)
			{
				//Setup effort
				r_joints.joints[i].effort = 0.0; //tau_control(i);
			}
			
			///////////////////////////////////////////////////7
				r_joints.header.stamp = ros::Time::now();
			effort_pub.publish(r_joints);
			
			
			
			//const Eigen::VectorXd & a = pinocchio::computeForwardDynamics(model,data,q,v,tau);

    
   
  
   std::cout << q << std::endl;
   std::cout << ref_twist << std::endl;

    
 }
//};
	
int main(int argc, char **argv)
{

 // Initializing ROS node with a name of demo_topic_subscriber
  ros::init(argc, argv,"qp_solver");

	////////////////////////////////////////////////////////////////////
 ros::NodeHandle nh;
 // Get the URDF string from the parameter server
    std::string urdf_string;
    if (!nh.getParam("/robot_description", urdf_string)) {
        ROS_ERROR("Failed to get /robot_description parameter");
        return -1;
    }

    // Specify the temporary file path
    std::string urdf_filename = "/tmp/robot.urdf";

    // Write the URDF string to the file
    std::ofstream urdf_file(urdf_filename);
    if (!urdf_file) {
        ROS_ERROR("Failed to open file: %s", urdf_filename.c_str());
        return -1;
    }
    urdf_file << urdf_string;
    urdf_file.close();

    // Print the content of the URDF file
    std::ifstream urdf_file_read(urdf_filename);
    if (!urdf_file_read) {
        ROS_ERROR("Failed to open file: %s", urdf_filename.c_str());
        return -1;
    }

    std::string line;
    while (std::getline(urdf_file_read, line)) {
        std::cout << line << std::endl;
    }
    urdf_file_read.close();
////////////////////////////////////////////////////////////////////
	//server observer;
	// You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename_ = "/home/niger/ros_ws/src/wbc_ur5/urdf/ur5e.urdf";
  
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
	 
 
  // Create a publisher object
  
  ros::Subscriber left_foot_subscriber = nh.subscribe("/multi_reference_left", 10, left_foot_subs);
  ros::Subscriber joint_state_subscriber = nh.subscribe("/robot_states", 10, robot_state_callback);
 ros::Subscriber twist_sub = nh.subscribe("cmd_vel", 10, twistCallback);
ros::Subscriber accel_sub = nh.subscribe("cmd_accel", 10, accelCallback);
ros::Subscriber contact_ref_subscriber = nh.subscribe("/multi_reference_left", 10, contact_ref_subs);
ros::Subscriber contact_subscriber = nh.subscribe("/robot_states", 10, contact_subs);

// Initialize publishers
    current_error_pub = nh.advertise<geometry_msgs::Vector3>("current_error", 10);
    delayed_error_pub = nh.advertise<geometry_msgs::Vector3>("delayed_error", 10);

  
  
  
  switch(mode)
	{
		case 0:

			//Init publishers
			effort_pub = nh.advertise<whole_body_state_msgs::WholeBodyState>("/reemc/positions", 10); //effort_topic
			
	}
  std::cout << "r_joints_size" << r_joints.joints[1] << std::endl;
  // Temporizador para calcular errores cada 10ms
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), [](const ros::TimerEvent& event) {
        computeDelayedError();
    });
  //Ros rate as fps
	ros::Rate loop_rate(10);
	
  //Spinning the node
   while(ros::ok)
   {
		ros::spinOnce();
		//Induced delay
		loop_rate.sleep();
		}
  
  return 0;
}
