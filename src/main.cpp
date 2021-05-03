#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <stdlib.h>

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "gazebo_msgs/SetModelState.h"
#include <gazebo/transport/transport.hh>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <math.h>

#include "boost/thread.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"

#include "cin_diretta.h" 
#include "quadruped.h"
#include "prob_quadratico.h"
#include "poligono_supporto.h"
#include "traj_planner.h"
#include "capture_point.h"
#include "prob_quadratico_cp.h"
#include "quadruped_control.h"
#include "quadruped_step.h"

//#include "eseguo_traj.h"

//includo librerie per il calcolo della traiettoria
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include <chrono>  


#include <fstream>
#include <sstream>

using namespace Eigen;
using namespace std;


// Variabili globali

bool joint_state_available = false, joint_base_available = false;
bool contact_fr, contact_fl, contact_br, contact_bl;
VectorXd hp(4), kp(4), he(4), rp(4);
Vector3d eef_bl, eef_br, eef_fl, eef_fr;
Vector3d coo_ee_bl, coo_ee_br, coo_ee_fl, coo_ee_fr;
Matrix3d rot_world_virtual_base;
Vector3d gravity1(0,0, -9.81); //mi serve solo per inizializzare la classe quadruped->update vedi se va bene dichiararlo qui
Matrix4d world_H_base; //come sopra
VectorXd q_joints(12,1);
VectorXd dq_joints(12,1);
Matrix<double,6,1> basevel;
Vector3d floating_base_position;//posso inserirla come variabile locale alla callback modelState
Matrix<double, 18,1> q_joints_total, dq_joints_total;
Matrix<double,6,1> floating_base_pose, floating_base_posed;
Matrix<double,3,1> floating_base_orientation; //posso inserirla come variabile locale alla callback modelState
double com_zdes = 0.4;
double t_step = 0.4;
float w=sqrt(9.81/com_zdes);
float  cpx, cpy;
ros::Time ts;
ros::Time begin0;
gazebo::transport::PublisherPtr pub;
gazebo::msgs::WorldControl stepper;
ros::Publisher _tau_pub;
//Segnale di controllo
//std_msgs::Float64MultiArray msg_ctrl;
std_msgs::Float64MultiArray tau1_msg;
Eigen::VectorXd tau;
 Eigen::MatrixXd Kcom;
Eigen::MatrixXd Dcom;
Eigen::Matrix<double,3,1>  force_br, force_bl, force_fl, force_fr;

//Vector3d com_pos,com_vel;

towr::SplineHolder solution;
towr::NlpFormulation formulation;
int flag;
//ignition::transport::Node node_ign;
//ignition::msgs::Marker markerMsg;

string modelFile="/home/salvatore/ros_ws/src/DogBotV4/ROS/src/dogbot_description/urdf/dogbot.urdf";
	std::ofstream com_file("com_file.txt");
	std::ofstream foothold_file("foothold.txt");
	std::ofstream foothold_br_file("foothold_br_file.txt");
	std::ofstream foothold_fl_file("foothold_fl_file.txt");
	std::ofstream foothold_des_file("foothold_des.txt");
	std::ofstream capture_point_file("capture_point.txt");
	std::ofstream tempo_simulazione_file("tempo_simulazione.txt");
	std::ofstream com_traiettoria_des_file("com_traiettoria_des.txt");
	std::ofstream tau_file("tau.txt");
	std::ofstream x_lim_file("x_lim_poligono.txt");

	

	QUADRUPED *doggo;
	

	PROB_QUAD *ottim;
	

	PROB_QUAD_CP *ottim_cp;
	
	
	TrajPlanner *traiettoria;
	
	//ESEGUOtraj *eseguo_traiettoria;
	//eseguo_traiettoria = new ESEGUOtraj(*doggo);
	
	// Set controller Viviana
    QUADRUPEDController *doggoControl; //controller_(doggo);
	

	//Step Control
	QUADRUPEDStep *doggoStep;
	



void Joint_cb(sensor_msgs::JointStateConstPtr js){
	joint_state_available = true;
	
	
	//memo pitch e hip coincidono
	hp<<js->position[1],js->position[4],js->position[7],js->position[10]; //rispettivamente bl,br,fl,fr
	he<<js->effort[1],js->effort[4],js->effort[7],js->effort[10];
	kp<<js->position[0],js->position[3],js->position[6],js->position[9];
	rp<<js->position[2],js->position[5],js->position[8],js->position[11];


	q_joints  << js->position[11], js->position[8], js->position[2], js->position[5], js->position[4],  js->position[3], js->position[1],  js->position[0] , js->position[7], js->position[6], js->position[10], js->position[9];
	dq_joints << js->velocity[11], js->velocity[8], js->velocity[2], js->velocity[5], js->velocity[4],  js->velocity[3], js->velocity[1],  js->velocity[0] , js->velocity[7], js->velocity[6], js->velocity[10],  js->velocity[9];

		
}

//posizione del contatto e misura della forza
void eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){

	if(eebl->states.empty()){ 
		contact_bl = false;
	}
	else{
		contact_bl = true;
		eef_bl<<eebl->states[0].total_wrench.force.x,
		eebl->states[0].total_wrench.force.y, eebl->states[0].total_wrench.force.z;

		coo_ee_bl<<eebl->states[0].contact_positions[0].x, eebl->states[0].contact_positions[0].y, eebl->states[0].contact_positions[0].z;
	}
}
//posizione del contatto e misura della forza
void eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){

	if(eebr->states.empty()){
		contact_br = false;
	}
	else{
		contact_br = true;
		eef_br<<eebr->states[0].total_wrench.force.x,
		eebr->states[0].total_wrench.force.y, eebr->states[0].total_wrench.force.z;

		coo_ee_br<<eebr->states[0].contact_positions[0].x, eebr->states[0].contact_positions[0].y,eebr->states[0].contact_positions[0].z;
	}
}
//posizione del contatto e misura della forza
void eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){
	if(eefl->states.empty()){
		contact_fl = false;
	}
	else{
		contact_fl = true;
		eef_fl<<eefl->states[0].total_wrench.force.x,
		eefl->states[0].total_wrench.force.y, eefl->states[0].total_wrench.force.z;

		coo_ee_fl<< eefl->states[0].contact_positions[0].x, eefl->states[0].contact_positions[0].y, eefl->states[0].contact_positions[0].z;
	}
}
//posizione del contatto e misura della forza
void eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){
	if(eefr->states.empty()){
		contact_fr = false;
	}
	else{
		contact_fr = true;
		eef_fr<<eefr->states[0].total_wrench.force.x,
		eefr->states[0].total_wrench.force.y, eefr->states[0].total_wrench.force.z;

		coo_ee_fr<<eefr->states[0].contact_positions[0].x,eefr->states[0].contact_positions[0].y, eefr->states[0].contact_positions[0].z;
	}
}





void modelState_cb( const gazebo_msgs::ModelStates &pt){
	joint_base_available = true;
	world_H_base.setIdentity();
	
	double roll, pitch, yaw;
	//double yaw_eu, pitch_eu, roll_eu;

	//quaternion
	tf::Quaternion q(pt.pose[1].orientation.x, pt.pose[1].orientation.y, pt.pose[1].orientation.z, pt.pose[1].orientation.w);
	Matrix<double,3,3> rot;
    tf::matrixTFToEigen(tf::Matrix3x3(q),rot);

	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	rot_world_virtual_base << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll),
							sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
							-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll);

	
	floating_base_position<<pt.pose[1].position.x, pt.pose[1].position.y, pt.pose[1].position.z;
	basevel<<pt.twist[1].linear.x, pt.twist[1].linear.y, pt.twist[1].linear.z, pt.twist[1].angular.x, pt.twist[1].angular.y, pt.twist[1].angular.z;
	dq_joints_total<<basevel,dq_joints;
	floating_base_orientation<< roll, pitch, yaw;//roll_eu, pitch_eu, yaw_eu;
	q_joints_total<<floating_base_position, floating_base_orientation, q_joints;
	floating_base_pose<< floating_base_position, floating_base_orientation;
	floating_base_posed<<floating_base_pose[0], floating_base_pose[1],com_zdes, floating_base_pose[3],floating_base_pose[4],floating_base_pose[5];

	//Set transformation matrix
    world_H_base.block(0,0,3,3)= rot;
    world_H_base.block(0,3,3,1)= floating_base_pose.block(0,0,3,1);

}

void stand_phase( ros::Rate loop_rate, double duration )
{ while((ros::Time::now()-begin0).toSec() < duration)
   if (joint_state_available && joint_base_available)
      {  
        
        // Update robot
          
         doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
      
        // Time
         double t = (ros::Time::now()-begin0).toSec();
         int idx=std::round( t*1000);
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;

         
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
         
        std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
        std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
        std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;

      // Compute control torque
      
      
    Eigen::Matrix<double,6,1> com_vel1= doggo->getCOMvel();
 


      Eigen::Matrix<double,12,1> w3=Eigen::Matrix<double,12,1>::Zero();

       tau = doggoStep->Cntr(composdes, comveldes, comaccdes,
                                 Kcom, Dcom,w3);
   

      // Set command message
       tau1_msg.data.clear();
      std::vector<double> ta(12,0.0);

      // torques in right order
       ta[11]=tau(7);
       ta[10]=tau(6);
       ta[9]=tau(2);
       ta[8]=tau(5);
       ta[7]=tau(4);
       ta[6]=tau(3);
       ta[5]=tau(9);
       ta[4]=tau(8);
       ta[3]=tau(1);
       ta[2]=tau(11);
       ta[1]=tau(10);
       ta[0]=tau(0);


      // Fill Command message
      for(int i=0; i<12; i++)
      {
   
        tau1_msg.data.push_back(ta[i]);
       }

       //Sending command
        _tau_pub.publish(tau1_msg);
        
    
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    	pub->Publish(stepper);
   
		foothold_br_file<<coo_ee_br(0)<<" "<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<"\n";
		foothold_br_file.flush();
		foothold_fl_file<<coo_ee_fl(0)<<" "<<coo_ee_fl(1)<<" "<<coo_ee_fl(2)<<" "<<"\n";
		foothold_fl_file.flush();
   	 	Eigen::MatrixXd com= doggo->getCOMpos();
    	Eigen::MatrixXd com_vel= doggo->getCOMvel();

  
        ros::spinOnce();
        loop_rate.sleep();
      }
}

//swing_phase br e fl (trot)
void swing_phase( ros::Rate loop_rate, double duration , double duration_prev)
{	

	while ((ros::Time::now()-begin0).toSec() < duration && flag==0 )
    {   auto start2 = std::chrono::high_resolution_clock::now();
       	//markerMsg.set_id(2);

      	if (joint_state_available && joint_base_available)
      	{	/*markerMsg.set_id(6);
      		markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    		markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
          	markerMsg.clear_point();
			*/
        	Eigen::VectorXd posfoot=Eigen::VectorXd::Zero(3);

			// Update robot
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			

			// Time
			double t = (ros::Time::now()-begin0).toSec();

				
			int idx=std::round( (t+0.225-0.01)*1000);
      	
			// Set desired vectors
			iDynTree::Vector6 composdes, comveldes, comaccdes;
    
         
			toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
			toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
			toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
			std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
			std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			// std::cout<<"posdesbr"<<trajsw.pos(0,idx)<<" "<<trajsw.pos(1,idx)<<" "<<trajsw.pos(2,idx)<<std::endl;
			std::cout<<"veldesbr"<<solution.ee_motion_.at(1)->GetPoint(t).v()<<std::endl;
			std::cout<<"accdesbr"<<solution.ee_motion_.at(2)->GetPoint(t).v()<<std::endl; //non dovrebbe essere .at(1)?
			/* std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;*/
			Eigen::Matrix<double,6,1> accd;
			if (flag==0)
       		{
				Eigen::Matrix<double,6,1> accd;
      
				accd<< solution.ee_motion_.at(1)->GetPoint(t).a(),
						solution.ee_motion_.at(2)->GetPoint(t).a();

				Eigen::Matrix<double,6,1> posdelta;
				posdelta<< solution.ee_motion_.at(1)->GetPoint(t).p()-doggo->getBRpos(),
              	solution.ee_motion_.at(2)->GetPoint(t).p()-doggo->getFLpos();

				Eigen::Matrix<double,6,1> veldelta;
				veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-doggo->getBRvel(),
					solution.ee_motion_.at(2)->GetPoint(t).v()-doggo->getFLvel();
        
				Eigen::MatrixXd Kp;
				Kp=250*Eigen::MatrixXd::Identity(6,6);
				Eigen::MatrixXd Kd;
				Kd=50*Eigen::MatrixXd::Identity(6,6);

				Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;

   
      
         		Eigen::Matrix<double,12,18> J=doggo->getJacobianCOM_linear();
            	Eigen::Matrix<double,6,12> Jcom;
				Jcom.block(0,0,3,12)=J.block(0,6,3,12);
				Jcom.block(3,0,3,12)=J.block(6,6,3,12);
  
     
				tau = doggoStep->CntrBr(composdes, comveldes, comaccdes,
												Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L2, Eigen::Matrix<double,12,1>::Zero());
            	std::cout<<"accd"<<accd<<std::endl;
               
			}
       		else if (flag==1)
       		{
				Eigen::Matrix<double,3,1> accd;
       			accd<< solution.ee_motion_.at(1)->GetPoint(t).a();
      		}
       		else if (flag==2)
       		{
				Eigen::Matrix<double,3,1> accd;
    			accd<< solution.ee_motion_.at(2)->GetPoint(t).a();
   			}
			
			// Compute control torque
		
			std::cout<<"tau"<<tau<<std::endl;

			// Set command message
			tau1_msg.data.clear();
			std::vector<double> ta(12,0.0);

			// torques in right order
			ta[11]=tau(7);
			ta[10]=tau(6);
			ta[9]=tau(2);
			ta[8]=tau(5);
			ta[7]=tau(4);
			ta[6]=tau(3);
			ta[5]=tau(9);
			ta[4]=tau(8);
			ta[3]=tau(1);
			ta[2]=tau(11);
			ta[1]=tau(10);
			ta[0]=tau(0);


      		// Fill Command message
      		for(int i=0; i<12; i++)
      		{
   				tau1_msg.data.push_back(ta[i]);
       		}

       		//Sending command
        	_tau_pub.publish(tau1_msg);

    
      		// One step in gazebo world ( to use if minqp problem takes too long for control loop)
     		pub->Publish(stepper);

       		////////

			Eigen::MatrixXd com= doggo->getCOMpos();
			Eigen::MatrixXd com_vel= doggo->getCOMvel();

			foothold_br_file<<coo_ee_br(0)<<" "<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<"\n";
			foothold_br_file.flush();
			
        	ros::spinOnce();
       		if(t>duration-0.05)
        	{
				//node_ign.Request("/marker", markerMsg);
			}

       		if(contact_br==false && contact_fl==true && t>duration-0.1)
      		{
				flag=1;
       			std::cout<<"contact"<<contact_fl<<std::endl;
			}
      		else if(contact_br==true && contact_fl==false && t>duration-0.1)
      		{
				flag=2;
       			std::cout<<"contact"<<contact_br<<std::endl;
			}
      		else if(contact_br==true && contact_fl==true && t>duration-0.1)
      		{
				flag=3;
				std::cout<<"contact"<<contact_br<<std::endl;
				std::cout<<"contact"<<contact_fl<<std::endl;
			}
        	loop_rate.sleep();
		}
    }
}

//swing_phase bl e fr (trot)
void swing_phase2( ros::Rate loop_rate, double duration , double duration_prev)
{	

	while ((ros::Time::now()-begin0).toSec() < duration && flag==0 )
    {   

      	if (joint_state_available && joint_base_available)
      	{	
        	Eigen::VectorXd posfoot=Eigen::VectorXd::Zero(3);

			// Update robot
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			

			// Time
			double t = (ros::Time::now()-begin0).toSec();

				
			int idx=std::round( (t+0.225-0.01)*1000);
      	
			// Set desired vectors
			iDynTree::Vector6 composdes, comveldes, comaccdes;
    
         
			toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
			toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
			toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
			std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
			std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			// std::cout<<"posdesbr"<<trajsw.pos(0,idx)<<" "<<trajsw.pos(1,idx)<<" "<<trajsw.pos(2,idx)<<std::endl;
			std::cout<<"veldesbr"<<solution.ee_motion_.at(0)->GetPoint(t).v()<<std::endl;
			std::cout<<"accdesbr"<<solution.ee_motion_.at(3)->GetPoint(t).v()<<std::endl; //non dovrebbe essere .at(1)?
			/* std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;*/
			Eigen::Matrix<double,6,1> accd;
			if (flag==0)
       		{
				Eigen::Matrix<double,6,1> accd;
      
				accd<< solution.ee_motion_.at(0)->GetPoint(t).a(),
						solution.ee_motion_.at(3)->GetPoint(t).a();

				Eigen::Matrix<double,6,1> posdelta;
				posdelta<< solution.ee_motion_.at(0)->GetPoint(t).p()-doggo->getBLpos(),
              	solution.ee_motion_.at(3)->GetPoint(t).p()-doggo->getFRpos();

				Eigen::Matrix<double,6,1> veldelta;
				veldelta<< solution.ee_motion_.at(0)->GetPoint(t).v()-doggo->getBLvel(),
					solution.ee_motion_.at(3)->GetPoint(t).v()-doggo->getFRvel();
        
				Eigen::MatrixXd Kp;
				Kp=250*Eigen::MatrixXd::Identity(6,6);
				Eigen::MatrixXd Kd;
				Kd=50*Eigen::MatrixXd::Identity(6,6);

				Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;

   
      
         		Eigen::Matrix<double,12,18> J=doggo->getJacobianCOM_linear();
            	Eigen::Matrix<double,6,12> Jcom;
				Jcom.block(0,0,3,12)=J.block(0,6,3,12);
				Jcom.block(3,0,3,12)=J.block(6,6,3,12);
  
     
				tau = doggoStep->CntrBr(composdes, comveldes, comaccdes,
												Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L3, Eigen::Matrix<double,12,1>::Zero());
            	std::cout<<"accd"<<accd<<std::endl;
               
			}
       		else if (flag==1)
       		{
				Eigen::Matrix<double,3,1> accd;
       			accd<< solution.ee_motion_.at(0)->GetPoint(t).a();
      		}
       		else if (flag==2)
       		{
				Eigen::Matrix<double,3,1> accd;
    			accd<< solution.ee_motion_.at(3)->GetPoint(t).a();
   			}
			
			// Compute control torque
		
			std::cout<<"tau"<<tau<<std::endl;

			// Set command message
			tau1_msg.data.clear();
			std::vector<double> ta(12,0.0);

			// torques in right order
			ta[11]=tau(7);
			ta[10]=tau(6);
			ta[9]=tau(2);
			ta[8]=tau(5);
			ta[7]=tau(4);
			ta[6]=tau(3);
			ta[5]=tau(9);
			ta[4]=tau(8);
			ta[3]=tau(1);
			ta[2]=tau(11);
			ta[1]=tau(10);
			ta[0]=tau(0);


      		// Fill Command message
      		for(int i=0; i<12; i++)
      		{
   				tau1_msg.data.push_back(ta[i]);
       		}

       		//Sending command
        	_tau_pub.publish(tau1_msg);

    
      		// One step in gazebo world ( to use if minqp problem takes too long for control loop)
     		pub->Publish(stepper);

       		////////

			Eigen::MatrixXd com= doggo->getCOMpos();
			Eigen::MatrixXd com_vel= doggo->getCOMvel();

			//foothold_br_file<<coo_ee_br(0)<<" "<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<"\n";
			//foothold_br_file.flush();
			
        	ros::spinOnce();
       		if(t>duration-0.05)
        	{
				//node_ign.Request("/marker", markerMsg);
			}

       		if(contact_bl==false && contact_fr==true && t>duration-0.1)
      		{
				flag=1;
       			std::cout<<"contact"<<contact_fr<<std::endl;
			}
      		else if(contact_bl==true && contact_fr==false && t>duration-0.1)
      		{
				flag=2;
       			std::cout<<"contact"<<contact_bl<<std::endl;
			}
      		else if(contact_bl==true && contact_fr==true && t>duration-0.1)
      		{
				flag=3;
				std::cout<<"contact"<<contact_bl<<std::endl;
				std::cout<<"contact"<<contact_fr<<std::endl;
			}
        	loop_rate.sleep();
		}
    }
}
//swing_phase fl fr (bound)
void swing_phase_b( ros::Rate loop_rate, double duration , double duration_prev)
{	

	while ((ros::Time::now()-begin0).toSec() < duration && flag==0 )
    {   auto start2 = std::chrono::high_resolution_clock::now();
       	//markerMsg.set_id(2);

      	if (joint_state_available && joint_base_available)
      	{	/*markerMsg.set_id(6);
      		markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    		markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
          	markerMsg.clear_point();
			*/
        	Eigen::VectorXd posfoot=Eigen::VectorXd::Zero(3);

			// Update robot
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			

			// Time
			double t = (ros::Time::now()-begin0).toSec();

				
			int idx=std::round( (t+0.225-0.01)*1000);
      	
			// Set desired vectors
			iDynTree::Vector6 composdes, comveldes, comaccdes;
    
         
			toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
			toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
			toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
			std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
			std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			// std::cout<<"posdesbr"<<trajsw.pos(0,idx)<<" "<<trajsw.pos(1,idx)<<" "<<trajsw.pos(2,idx)<<std::endl;
			std::cout<<"veldesfl"<<solution.ee_motion_.at(2)->GetPoint(t).v()<<std::endl;
			std::cout<<"veldesfr"<<solution.ee_motion_.at(3)->GetPoint(t).v()<<std::endl;
			/* std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;*/
			Eigen::Matrix<double,6,1> accd;
			if (flag==0)
       		{
				Eigen::Matrix<double,6,1> accd;
      
				accd<< solution.ee_motion_.at(2)->GetPoint(t).a(),
						solution.ee_motion_.at(3)->GetPoint(t).a();

				Eigen::Matrix<double,6,1> posdelta;
				posdelta<< solution.ee_motion_.at(2)->GetPoint(t).p()-doggo->getFLpos(),
              	solution.ee_motion_.at(3)->GetPoint(t).p()-doggo->getFRpos();

				Eigen::Matrix<double,6,1> veldelta;
				veldelta<< solution.ee_motion_.at(2)->GetPoint(t).v()-doggo->getFLvel(),
					solution.ee_motion_.at(3)->GetPoint(t).v()-doggo->getFRvel();
        
				Eigen::MatrixXd Kp;
				Kp=250*Eigen::MatrixXd::Identity(6,6);
				Eigen::MatrixXd Kd;
				Kd=50*Eigen::MatrixXd::Identity(6,6);

				Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;

   
      
         		Eigen::Matrix<double,12,18> J=doggo->getJacobianCOM_linear();
            	Eigen::Matrix<double,6,12> Jcom;
				Jcom.block(0,0,3,12)=J.block(0,6,3,12);
				Jcom.block(3,0,3,12)=J.block(6,6,3,12);
  
     
				tau = doggoStep->CntrBr(composdes, comveldes, comaccdes,
												Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L6, Eigen::Matrix<double,12,1>::Zero());
            	std::cout<<"accd"<<accd<<std::endl;
               
			}
       		else if (flag==1)
       		{
				Eigen::Matrix<double,3,1> accd;
       			accd<< solution.ee_motion_.at(2)->GetPoint(t).a();
      		}
       		else if (flag==2)
       		{
				Eigen::Matrix<double,3,1> accd;
    			accd<< solution.ee_motion_.at(3)->GetPoint(t).a();
   			}
			
			// Compute control torque
		
			std::cout<<"tau"<<tau<<std::endl;

			// Set command message
			tau1_msg.data.clear();
			std::vector<double> ta(12,0.0);

			// torques in right order
			ta[11]=tau(7);
			ta[10]=tau(6);
			ta[9]=tau(2);
			ta[8]=tau(5);
			ta[7]=tau(4);
			ta[6]=tau(3);
			ta[5]=tau(9);
			ta[4]=tau(8);
			ta[3]=tau(1);
			ta[2]=tau(11);
			ta[1]=tau(10);
			ta[0]=tau(0);


      		// Fill Command message
      		for(int i=0; i<12; i++)
      		{
   				tau1_msg.data.push_back(ta[i]);
       		}

       		//Sending command
        	_tau_pub.publish(tau1_msg);

    
      		// One step in gazebo world ( to use if minqp problem takes too long for control loop)
     		pub->Publish(stepper);

       		////////

			Eigen::MatrixXd com= doggo->getCOMpos();
			Eigen::MatrixXd com_vel= doggo->getCOMvel();
			foothold_fl_file<<coo_ee_fl(0)<<" "<<coo_ee_fl(1)<<" "<<coo_ee_fl(2)<<" "<<"\n";
			foothold_fl_file.flush();

   
        	ros::spinOnce();
       		if(t>duration-0.05)
        	{
				//node_ign.Request("/marker", markerMsg);
			}

       		if(contact_fl==false && contact_fr==true && t>duration-0.1)
      		{
				flag=1;
       			std::cout<<"contact"<<contact_fr<<std::endl;
			}
      		else if(contact_fl==true && contact_fr==false && t>duration-0.1)
      		{
				flag=2;
       			std::cout<<"contact"<<contact_fl<<std::endl;
			}
      		else if(contact_fl==true && contact_fr==true && t>duration-0.1)
      		{
				flag=3;
				std::cout<<"contact"<<contact_fl<<std::endl;
				std::cout<<"contact"<<contact_fr<<std::endl;
			}
        	loop_rate.sleep();
		}
    }
}

void swing_phase_b2( ros::Rate loop_rate, double duration , double duration_prev)
{	

	while ((ros::Time::now()-begin0).toSec() < duration && flag==0 )
    {   auto start2 = std::chrono::high_resolution_clock::now();
       	//markerMsg.set_id(2);

      	if (joint_state_available && joint_base_available)
      	{	/*markerMsg.set_id(6);
      		markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    		markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
          	markerMsg.clear_point();
			*/
        	Eigen::VectorXd posfoot=Eigen::VectorXd::Zero(3);

			// Update robot
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			

			// Time
			double t = (ros::Time::now()-begin0).toSec();

				
			int idx=std::round( (t+0.225-0.01)*1000);
      	
			// Set desired vectors
			iDynTree::Vector6 composdes, comveldes, comaccdes;
    
         
			toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
			toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
			toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
			std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
			std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			// std::cout<<"posdesbr"<<trajsw.pos(0,idx)<<" "<<trajsw.pos(1,idx)<<" "<<trajsw.pos(2,idx)<<std::endl;
			std::cout<<"veldesbr"<<solution.ee_motion_.at(0)->GetPoint(t).v()<<std::endl;
			std::cout<<"accdesbr"<<solution.ee_motion_.at(1)->GetPoint(t).v()<<std::endl;
			/* std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;*/
			Eigen::Matrix<double,6,1> accd;
			if (flag==0)
       		{
				Eigen::Matrix<double,6,1> accd;
      
				accd<< solution.ee_motion_.at(0)->GetPoint(t).a(),
						solution.ee_motion_.at(1)->GetPoint(t).a();

				Eigen::Matrix<double,6,1> posdelta;
				posdelta<< solution.ee_motion_.at(0)->GetPoint(t).p()-doggo->getBLpos(),
              	solution.ee_motion_.at(1)->GetPoint(t).p()-doggo->getBRpos();

				Eigen::Matrix<double,6,1> veldelta;
				veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-doggo->getBRvel(),
					solution.ee_motion_.at(0)->GetPoint(t).v()-doggo->getBLvel();
        
				Eigen::MatrixXd Kp;
				Kp=250*Eigen::MatrixXd::Identity(6,6);
				Eigen::MatrixXd Kd;
				Kd=50*Eigen::MatrixXd::Identity(6,6);

				Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;

   
      
         		Eigen::Matrix<double,12,18> J=doggo->getJacobianCOM_linear();
            	Eigen::Matrix<double,6,12> Jcom;
				Jcom.block(0,0,3,12)=J.block(0,6,3,12);
				Jcom.block(3,0,3,12)=J.block(6,6,3,12);
  
     
				tau = doggoStep->CntrBr(composdes, comveldes, comaccdes,
												Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L7, Eigen::Matrix<double,12,1>::Zero());
            	std::cout<<"accd"<<accd<<std::endl;
               
			}
       		else if (flag==1)
       		{
				Eigen::Matrix<double,3,1> accd;
       			accd<< solution.ee_motion_.at(0)->GetPoint(t).a();
      		}
       		else if (flag==2)
       		{
				Eigen::Matrix<double,3,1> accd;
    			accd<< solution.ee_motion_.at(1)->GetPoint(t).a();
   			}
			
			// Compute control torque
		
			std::cout<<"tau"<<tau<<std::endl;

			// Set command message
			tau1_msg.data.clear();
			std::vector<double> ta(12,0.0);

			// torques in right order
			ta[11]=tau(7);
			ta[10]=tau(6);
			ta[9]=tau(2);
			ta[8]=tau(5);
			ta[7]=tau(4);
			ta[6]=tau(3);
			ta[5]=tau(9);
			ta[4]=tau(8);
			ta[3]=tau(1);
			ta[2]=tau(11);
			ta[1]=tau(10);
			ta[0]=tau(0);


      		// Fill Command message
      		for(int i=0; i<12; i++)
      		{
   				tau1_msg.data.push_back(ta[i]);
       		}

       		//Sending command
        	_tau_pub.publish(tau1_msg);

    
      		// One step in gazebo world ( to use if minqp problem takes too long for control loop)
     		pub->Publish(stepper);

       		////////

			Eigen::MatrixXd com= doggo->getCOMpos();
			Eigen::MatrixXd com_vel= doggo->getCOMvel();
			foothold_br_file<<coo_ee_br(0)<<" "<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<"\n";
			foothold_br_file.flush();

   
        	ros::spinOnce();
       		if(t>duration-0.05)
        	{
				//node_ign.Request("/marker", markerMsg);
			}

       		if(contact_bl==false && contact_br==true && t>duration-0.1)
      		{
				flag=1;
       			std::cout<<"contact"<<contact_br<<std::endl;
			}
      		else if(contact_bl==true && contact_br==false && t>duration-0.1)
      		{
				flag=2;
       			std::cout<<"contact"<<contact_bl<<std::endl;
			}
      		else if(contact_bl==true && contact_br==true && t>duration-0.1)
      		{
				flag=3;
				std::cout<<"contact"<<contact_bl<<std::endl;
				std::cout<<"contact"<<contact_br<<std::endl;
			}
        	loop_rate.sleep();
		}
    }
}

//swing_phase br fr (pace)
void swing_phase_p( ros::Rate loop_rate, double duration , double duration_prev)
{	

	while ((ros::Time::now()-begin0).toSec() < duration && flag==0 )
    {   auto start2 = std::chrono::high_resolution_clock::now();
       	//markerMsg.set_id(2);

      	if (joint_state_available && joint_base_available)
      	{	/*markerMsg.set_id(6);
      		markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    		markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
          	markerMsg.clear_point();
			*/
        	Eigen::VectorXd posfoot=Eigen::VectorXd::Zero(3);

			// Update robot
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			

			// Time
			double t = (ros::Time::now()-begin0).toSec();

				
			int idx=std::round( (t+0.225-0.01)*1000);
      	
			// Set desired vectors
			iDynTree::Vector6 composdes, comveldes, comaccdes;
    
         
			toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
			toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
			toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
			std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
			std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			// std::cout<<"posdesbr"<<trajsw.pos(0,idx)<<" "<<trajsw.pos(1,idx)<<" "<<trajsw.pos(2,idx)<<std::endl;
			std::cout<<"veldesbr"<<solution.ee_motion_.at(1)->GetPoint(t).v()<<std::endl;
			std::cout<<"accdesbr"<<solution.ee_motion_.at(3)->GetPoint(t).v()<<std::endl;
			/* std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;*/
			Eigen::Matrix<double,6,1> accd;
			if (flag==0)
       		{
				Eigen::Matrix<double,6,1> accd;
      
				accd<< solution.ee_motion_.at(1)->GetPoint(t).a(),
						solution.ee_motion_.at(3)->GetPoint(t).a();

				Eigen::Matrix<double,6,1> posdelta;
				posdelta<< solution.ee_motion_.at(1)->GetPoint(t).p()-doggo->getBRpos(),
              	solution.ee_motion_.at(3)->GetPoint(t).p()-doggo->getFRpos();

				Eigen::Matrix<double,6,1> veldelta;
				veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-doggo->getBRvel(),
					solution.ee_motion_.at(3)->GetPoint(t).v()-doggo->getFRvel();
        
				Eigen::MatrixXd Kp;
				Kp=250*Eigen::MatrixXd::Identity(6,6);
				Eigen::MatrixXd Kd;
				Kd=50*Eigen::MatrixXd::Identity(6,6);

				Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;

   
      
         		Eigen::Matrix<double,12,18> J=doggo->getJacobianCOM_linear();
            	Eigen::Matrix<double,6,12> Jcom;
				Jcom.block(0,0,3,12)=J.block(0,6,3,12);
				Jcom.block(3,0,3,12)=J.block(6,6,3,12);
  
     
				tau = doggoStep->CntrBr(composdes, comveldes, comaccdes,
												Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L4, Eigen::Matrix<double,12,1>::Zero());
            	std::cout<<"accd"<<accd<<std::endl;
               
			}
       		else if (flag==1)
       		{
				Eigen::Matrix<double,3,1> accd;
       			accd<< solution.ee_motion_.at(1)->GetPoint(t).a();
      		}
       		else if (flag==2)
       		{
				Eigen::Matrix<double,3,1> accd;
    			accd<< solution.ee_motion_.at(3)->GetPoint(t).a();
   			}
			
			// Compute control torque
		
			std::cout<<"tau"<<tau<<std::endl;

			// Set command message
			tau1_msg.data.clear();
			std::vector<double> ta(12,0.0);

			// torques in right order
			ta[11]=tau(7);
			ta[10]=tau(6);
			ta[9]=tau(2);
			ta[8]=tau(5);
			ta[7]=tau(4);
			ta[6]=tau(3);
			ta[5]=tau(9);
			ta[4]=tau(8);
			ta[3]=tau(1);
			ta[2]=tau(11);
			ta[1]=tau(10);
			ta[0]=tau(0);


      		// Fill Command message
      		for(int i=0; i<12; i++)
      		{
   				tau1_msg.data.push_back(ta[i]);
       		}

       		//Sending command
        	_tau_pub.publish(tau1_msg);

    
      		// One step in gazebo world ( to use if minqp problem takes too long for control loop)
     		pub->Publish(stepper);

       		////////

			Eigen::MatrixXd com= doggo->getCOMpos();
			Eigen::MatrixXd com_vel= doggo->getCOMvel();
			foothold_br_file<<coo_ee_br(0)<<" "<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<"\n";
			foothold_br_file.flush();

   
        	ros::spinOnce();
       		if(t>duration-0.05)
        	{
				//node_ign.Request("/marker", markerMsg);
			}

       		if(contact_br==false && contact_fr==true && t>duration-0.1)
      		{
				flag=1;
       			std::cout<<"contact"<<contact_fr<<std::endl;
			}
      		else if(contact_br==true && contact_fr==false && t>duration-0.1)
      		{
				flag=2;
       			std::cout<<"contact"<<contact_br<<std::endl;
			}
      		else if(contact_br==true && contact_fr==true && t>duration-0.1)
      		{
				flag=3;
				std::cout<<"contact"<<contact_br<<std::endl;
				std::cout<<"contact"<<contact_fr<<std::endl;
			}
        	loop_rate.sleep();
		}
    }
}

void swing_phase_p2( ros::Rate loop_rate, double duration , double duration_prev)
{	

	while ((ros::Time::now()-begin0).toSec() < duration && flag==0 )
    {   auto start2 = std::chrono::high_resolution_clock::now();
       	//markerMsg.set_id(2);

      	if (joint_state_available && joint_base_available)
      	{	/*markerMsg.set_id(6);
      		markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    		markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
          	markerMsg.clear_point();
			*/
        	Eigen::VectorXd posfoot=Eigen::VectorXd::Zero(3);

			// Update robot
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			

			// Time
			double t = (ros::Time::now()-begin0).toSec();

				
			int idx=std::round( (t+0.225-0.01)*1000);
      	
			// Set desired vectors
			iDynTree::Vector6 composdes, comveldes, comaccdes;
    
         
			toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
			toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
			toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
			std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
			std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			// std::cout<<"posdesbr"<<trajsw.pos(0,idx)<<" "<<trajsw.pos(1,idx)<<" "<<trajsw.pos(2,idx)<<std::endl;
			std::cout<<"veldesbr"<<solution.ee_motion_.at(0)->GetPoint(t).v()<<std::endl;
			std::cout<<"accdesbr"<<solution.ee_motion_.at(2)->GetPoint(t).v()<<std::endl;
			/* std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;*/
			Eigen::Matrix<double,6,1> accd;
			if (flag==0)
       		{
				Eigen::Matrix<double,6,1> accd;
      
				accd<< solution.ee_motion_.at(0)->GetPoint(t).a(),
						solution.ee_motion_.at(2)->GetPoint(t).a();

				Eigen::Matrix<double,6,1> posdelta;
				posdelta<< solution.ee_motion_.at(0)->GetPoint(t).p()-doggo->getBLpos(),
              	solution.ee_motion_.at(2)->GetPoint(t).p()-doggo->getFLpos();

				Eigen::Matrix<double,6,1> veldelta;
				veldelta<< solution.ee_motion_.at(0)->GetPoint(t).v()-doggo->getBLvel(),
					solution.ee_motion_.at(2)->GetPoint(t).v()-doggo->getFLvel();
        
				Eigen::MatrixXd Kp;
				Kp=250*Eigen::MatrixXd::Identity(6,6);
				Eigen::MatrixXd Kd;
				Kd=50*Eigen::MatrixXd::Identity(6,6);

				Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;

   
      
         		Eigen::Matrix<double,12,18> J=doggo->getJacobianCOM_linear();
            	Eigen::Matrix<double,6,12> Jcom;
				Jcom.block(0,0,3,12)=J.block(0,6,3,12);
				Jcom.block(3,0,3,12)=J.block(6,6,3,12);
  
     
				tau = doggoStep->CntrBr(composdes, comveldes, comaccdes,
												Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L5, Eigen::Matrix<double,12,1>::Zero());
            	std::cout<<"accd"<<accd<<std::endl;
               
			}
       		else if (flag==1)
       		{
				Eigen::Matrix<double,3,1> accd;
       			accd<< solution.ee_motion_.at(0)->GetPoint(t).a();
      		}
       		else if (flag==2)
       		{
				Eigen::Matrix<double,3,1> accd;
    			accd<< solution.ee_motion_.at(2)->GetPoint(t).a();
   			}
			
			// Compute control torque
		
			std::cout<<"tau"<<tau<<std::endl;

			// Set command message
			tau1_msg.data.clear();
			std::vector<double> ta(12,0.0);

			// torques in right order
			ta[11]=tau(7);
			ta[10]=tau(6);
			ta[9]=tau(2);
			ta[8]=tau(5);
			ta[7]=tau(4);
			ta[6]=tau(3);
			ta[5]=tau(9);
			ta[4]=tau(8);
			ta[3]=tau(1);
			ta[2]=tau(11);
			ta[1]=tau(10);
			ta[0]=tau(0);


      		// Fill Command message
      		for(int i=0; i<12; i++)
      		{
   				tau1_msg.data.push_back(ta[i]);
       		}

       		//Sending command
        	_tau_pub.publish(tau1_msg);

    
      		// One step in gazebo world ( to use if minqp problem takes too long for control loop)
     		pub->Publish(stepper);

       		////////

			Eigen::MatrixXd com= doggo->getCOMpos();
			Eigen::MatrixXd com_vel= doggo->getCOMvel();
			foothold_fl_file<<coo_ee_fl(0)<<" "<<coo_ee_fl(1)<<" "<<coo_ee_fl(2)<<" "<<"\n";
			foothold_fl_file.flush();

   
        	ros::spinOnce();
       		if(t>duration-0.05)
        	{
				//node_ign.Request("/marker", markerMsg);
			}

       		if(contact_bl==false && contact_fl==true && t>duration-0.1)
      		{
				flag=1;
       			std::cout<<"contact"<<contact_fl<<std::endl;
			}
      		else if(contact_bl==true && contact_fl==false && t>duration-0.1)
      		{
				flag=2;
       			std::cout<<"contact"<<contact_bl<<std::endl;
			}
      		else if(contact_bl==true && contact_fl==true && t>duration-0.1)
      		{
				flag=3;
				std::cout<<"contact"<<contact_bl<<std::endl;
				std::cout<<"contact"<<contact_fl<<std::endl;
			}
        	loop_rate.sleep();
		}
    }
}

void swing_phasebr( ros::Rate loop_rate, double duration , double duration_prev)
{
	cout<<"tempo corrente: "<<ros::Time::now()-begin0<<endl;
	cout<<"Duration: "<<duration<<endl;
	while ((ros::Time::now()-begin0).toSec() < duration)// &  contact_br == false)
    { 	 
		
      if (joint_state_available && joint_base_available)
      	{  
			// Update robot
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);

			// Time
			double t = (ros::Time::now()-begin0).toSec();

			
			int idx=std::round( (t)*1000);
		
			// Set desired vectors
			iDynTree::Vector6 composdes, comveldes, comaccdes;
		
			cout<<"prova"<<endl;
			toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
			toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
			toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
			std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
			std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			//std::cout<<"posdesbr"<<trajsw.pos(0,idx)<<" "<<trajsw.pos(1,idx)<<" "<<trajsw.pos(2,idx)<<std::endl;
			std::cout<<"veldesbr"<<solution.ee_motion_.at(1)->GetPoint(t).v()<<std::endl;
			std::cout<<"accdesbr"<<solution.ee_motion_.at(2)->GetPoint(t).v()<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			
			Eigen::Matrix<double,3,1> accd;
			cout<<"accd: "<<endl<<solution.ee_motion_.at(1)->GetPoint(t).a()<<endl;
			accd<< solution.ee_motion_.at(1)->GetPoint(t).a();  
			cout<<"prova1"<<endl;
			Eigen::Matrix<double,3,1> posdelta;
			posdelta<< solution.ee_motion_.at(1)->GetPoint(t).p()-doggo->getBRpos();
			cout<<"prova2"<<endl;
			Eigen::Matrix<double,3,1> veldelta;
			veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-doggo->getBRvel();
			cout<<"prova3"<<endl;
			Eigen::MatrixXd Kp;
			Kp=250*Eigen::MatrixXd::Identity(3,3);
			Eigen::MatrixXd Kd;
			Kd=50*Eigen::MatrixXd::Identity(3,3);
			cout<<"prova4"<<endl;
			Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;
			cout<<"prova5"<<endl;


			Eigen::Matrix<double,6,1> com_vel1= doggo->getCOMvel();
			Eigen::Matrix<double,12,18> J=doggo->getJacobianCOM_linear();
			Eigen::Matrix<double,3,12> Jcom;
			Jcom.block(0,0,3,12)=J.block(0,6,3,12);

			cout<<"prova6"<<endl;
			tau = doggoStep->CntrOl(composdes, comveldes, comaccdes,
									Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::BR, Eigen::Matrix<double,12,1>::Zero()); 
		
			cout<<"prova7"<<endl;
			std::cout<<"tau"<<tau<<std::endl;

			// Set command message
			tau1_msg.data.clear();
			std::vector<double> ta(12,0.0);

			// torques in right order
			ta[11]=tau(7);
			ta[10]=tau(6);
			ta[9]=tau(2);
			ta[8]=tau(5);
			ta[7]=tau(4);
			ta[6]=tau(3);
			ta[5]=tau(9);
			ta[4]=tau(8);
			ta[3]=tau(1);
			ta[2]=tau(11);
			ta[1]=tau(10);
			ta[0]=tau(0);


			// Fill Command message
			for(int i=0; i<12; i++)
			{
				tau1_msg.data.push_back(ta[i]);
			}
			cout<<"prova8"<<endl;
			//Sending command
			_tau_pub.publish(tau1_msg);

		
			// One step in gazebo world ( to use if minqp problem takes too long for control loop)
			pub->Publish(stepper);

			cout<<"prova9"<<endl;

			Eigen::MatrixXd com= doggo->getCOMpos();
			Eigen::MatrixXd com_vel= doggo->getCOMvel();
			foothold_br_file<<coo_ee_br(0)<<" "<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<"\n";
			foothold_br_file.flush();
			
	
			ros::spinOnce();
			loop_rate.sleep();
    	} 
	}
}

void swing_phasefr( ros::Rate loop_rate, double duration , double duration_prev)
{
	while ((ros::Time::now()-begin0).toSec() < duration &  contact_fr == false)
    { 

    	if (joint_state_available && joint_base_available)
      	{  
			// Update robot
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);

			// Time
			double t = (ros::Time::now()-begin0).toSec();

			
			int idx=std::round( (t)*1000);
		
			// Set desired vectors
			iDynTree::Vector6 composdes, comveldes, comaccdes;
			/*toEigen(composdes)<<traj.pos(0,idx),traj.pos(1,idx),traj.pos(2,idx), 0,  0, 0;
			toEigen(comveldes)<<traj.vel(0,idx),  traj.vel(1,idx), traj.vel(2,idx), 0,  0, 0;
			toEigen(comaccdes) <<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx), 0,  0, 0;*/
			
			toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
			toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
			toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
			std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
			std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
			std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
			
			//std::cout<<"posdesbr"<<trajsw.pos(0,idx)<<" "<<trajsw.pos(1,idx)<<" "<<trajsw.pos(2,idx)<<std::endl;
			std::cout<<"veldesbr"<<solution.ee_motion_.at(3)->GetPoint(t).v()<<std::endl;
			std::cout<<"accdesbr"<<solution.ee_motion_.at(3)->GetPoint(t).v()<<std::endl;
			
			Eigen::Matrix<double,3,1> accd;
			accd<< solution.ee_motion_.at(3)->GetPoint(t).a();

			Eigen::Matrix<double,3,1> posdelta;
			posdelta<< solution.ee_motion_.at(3)->GetPoint(t).p()-doggo->getFRpos();

			Eigen::Matrix<double,3,1> veldelta;
			veldelta<< solution.ee_motion_.at(3)->GetPoint(t).v()-doggo->getFRvel();
			
			Eigen::MatrixXd Kp;
			Kp=250*Eigen::MatrixXd::Identity(3,3);
			Eigen::MatrixXd Kd;
			Kd=50*Eigen::MatrixXd::Identity(3,3);

			Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;

			Eigen::Matrix<double,6,1> com_vel1= doggo->getCOMvel();
		
		
			Eigen::Matrix<double,12,18> J=doggo->getJacobianCOM_linear();
			Eigen::Matrix<double,3,12> Jcom;
			Jcom.block(0,0,3,12)=J.block(9,6,3,12);
			//  Jcom.block(3,0,3,12)=J.block(9,6,3,12);

		
			tau = doggoStep->CntrOl(composdes, comveldes, comaccdes,
									Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::FR, Eigen::Matrix<double,12,1>::Zero()); 
			std::cout<<"accd"<<accd<<std::endl;

			std::cout<<"tau"<<tau<<std::endl;

			// Set command message
			tau1_msg.data.clear();
			std::vector<double> ta(12,0.0);

			// torques in right order
			ta[11]=tau(7);
			ta[10]=tau(6);
			ta[9]=tau(2);
			ta[8]=tau(5);
			ta[7]=tau(4);
			ta[6]=tau(3);
			ta[5]=tau(9);
			ta[4]=tau(8);
			ta[3]=tau(1);
			ta[2]=tau(11);
			ta[1]=tau(10);
			ta[0]=tau(0);


			// Fill Command message
			for(int i=0; i<12; i++)
			{
				tau1_msg.data.push_back(ta[i]);
			}

			//Sending command
			_tau_pub.publish(tau1_msg);

		
			// One step in gazebo world ( to use if minqp problem takes too long for control loop)
			pub->Publish(stepper);

			Eigen::MatrixXd com= doggo->getCOMpos();
			Eigen::MatrixXd com_vel= doggo->getCOMvel();



			ros::spinOnce();
			loop_rate.sleep();
    	}
	}
}

towr::NlpFormulation computetrajecotry(int gait_flag)
{    towr::NlpFormulation formulation_;
    // terrain
  formulation_.terrain_ = std::make_shared<towr::FlatGround>(0.0);

  // Kinematic limits and dynamic parameters of the hopper
  formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot);

  formulation_.initial_base_.lin.at(towr::kPos) << doggo->getCOMpos().block(0,0,3,1);
  formulation_.initial_base_.ang.at(towr::kPos) << doggo->getCOMpos().block(3,0,3,1);
  formulation_.initial_base_.lin.at(towr::kVel) << doggo->getCOMvel().block(0,0,3,1);
  formulation_.initial_base_.ang.at(towr::kVel) << doggo->getCOMvel().block(3,0,3,1);


  
  //else if((ros::Time::now()).toSec()<5){
	  												//0.2,0,0;
   // formulation_.final_base_.lin.at(towr::kPos) << 0.0, formulation_.initial_base_.lin.at(towr::kPos)[1]-0.05, 0.40229;
    //formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
 // }
 	if(gait_flag<4){
		//formulation_.final_base_.lin.at(towr::kPos) << 0.0, formulation_.initial_base_.lin.at(towr::kPos)[1]+0.05, 0.0;//dopo rimetti formulation_.initial_base_.lin.at(towr::kPos)[1]+0.05, 0
		//formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
		formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[1]+0.05, 0, 0.0;
		formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
	}
	else if(gait_flag==4 ){
		//formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0], 0, 0.0; //cosa significa quell'1 tra parentesi?
		//formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
		formulation_.final_base_.lin.at(towr::kPos) << 0,formulation_.initial_base_.lin.at(towr::kPos)[1], 0.0; //cosa significa quell'1 tra parentesi?
		formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
	}
	else if(gait_flag==5)
	{
		formulation_.final_base_.lin.at(towr::kPos) << 0.0, formulation_.initial_base_.lin.at(towr::kPos)[1]+0.05, 0.0;
		formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
	}
	else if(gait_flag==6)
	{
		formulation_.final_base_.lin.at(towr::kPos) << 0.0, formulation_.initial_base_.lin.at(towr::kPos)[1], 0.0;
		formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
	}
	if(gait_flag==7){
		formulation_.final_base_.lin.at(towr::kPos) << 0.0, formulation_.initial_base_.lin.at(towr::kPos)[1], 0.0;
		formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
	}

   auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();
  formulation_.initial_ee_W_ = nominal_stance_B;  
  std::cout<<"Phase durations"<<formulation_.initial_ee_W_ .size()<<std::endl;

  Eigen::Vector3d pos_ee;
  for (int ee=0; ee<4; ee++){
    switch(ee){
     case 0: pos_ee=doggo->getBLpos();
     break;
     case 1: pos_ee=doggo->getBRpos();
     break;
     case 2: pos_ee=doggo->getFLpos();
     break;
     case 3: pos_ee=doggo->getFRpos();
     break;
    }

    formulation_.initial_ee_W_.at(ee)[0]=pos_ee[0];
    formulation_.initial_ee_W_.at(ee)[1]=pos_ee[1];

  }

  std::for_each(formulation_.initial_ee_W_.begin(), formulation_.initial_ee_W_.end(),
                  [&](Eigen::Vector3d& p){  p[2]= 0.0; } );
  
  std::cout<<"size"<<formulation_.initial_ee_W_.at(0)<<std::endl;
  std::cout<<"size"<<formulation_.initial_ee_W_.at(1)<<std::endl;
  std::cout<<"size"<<formulation_.initial_ee_W_.at(2)<<std::endl;
  std::cout<<"size"<<formulation_.initial_ee_W_.at(3)<<std::endl;
  
  
   auto gait_gen_ = towr::GaitGenerator::MakeGaitGenerator(4);
   /*
   if (gait_flag==1){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C1);
  gait_gen_->SetCombo(id_gait);
   }
    else if (gait_flag==2){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C5);
  gait_gen_->SetCombo(id_gait);
   }
   */
  	std::cout<<"a"<<endl;
	if (gait_flag==1){ //muovo solo la gamba br
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C7);
  	gait_gen_->SetCombo(id_gait);
   }
   if (gait_flag==2){ //muovo br e fl (trot)
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C5);
  	gait_gen_->SetCombo(id_gait);
   }
    if (gait_flag==3){//muovo br e fr (pace)
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C8);
  	gait_gen_->SetCombo(id_gait);
   }
    if (gait_flag==4){//muovo bl e fl (pace)
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C9);
  	gait_gen_->SetCombo(id_gait);
   }
   cout<<"a2"<<endl;
   if (gait_flag==5){//muovo fl e fr (bound)
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C10);
  	gait_gen_->SetCombo(id_gait);
   }
     if (gait_flag==6){//muovo br e bl (bound)
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C11);
  	gait_gen_->SetCombo(id_gait);
   }
   if (gait_flag==7){//muovo br e fl (trot)
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C12);
  	gait_gen_->SetCombo(id_gait);
   }
	cout<<"a3"<<endl;
    formulation_.params_.ee_phase_durations_.clear();
    std::cout<<"Phase durations"<<formulation_.params_.ee_phase_durations_.size()<<std::endl;
    
    for (int ee=0; ee<4; ++ee) {
      formulation_.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(t_step, ee)); //t_step è la durata della traiettoria 
      formulation_.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
      std::cout<<"ciao"<<std::endl;
    }


  ifopt::Problem nlp;
  
  for (auto c : formulation_.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  std::cout<<"ciao"<<std::endl;
  for (auto c : formulation_.GetConstraints(solution))
    nlp.AddConstraintSet(c);
    std::cout<<"ciao"<<std::endl;
  for (auto c : formulation_.GetCosts())
    nlp.AddCostSet(c);
    std::cout<<"ciao"<<std::endl;
     std::cout<<"size"<<formulation_.params_.GetEECount()<<std::endl;
 
 //cout<<"formulation:"<<endl<<formulation_.GetVariableSets(solution)<<endl;
 
 for (int ee=0; ee<4; ++ee) {
    for (int i=0; i<formulation_.params_.ee_phase_durations_.at(ee).size(); ++i){
  std::cout<<"ee: "<<ee<<std::endl;
  std::cout<<"phase duration: "<<formulation_.params_.ee_phase_durations_.at(ee)[i]<<std::endl;
  }}
	std::cout<<"a3"<<endl;
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  std::cout<<"a4"<<endl;
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  std::cout<<"a4.1"<<endl;
  solver->SetOption("max_cpu_time", 20.0);
	std::cout<<"a4.2"<<endl;
  solver->Solve(nlp);//eigen error
  
   std::cout<<"a5"<<endl;
  return formulation_;
	std::cout<<"a5.1"<<endl;
}

int main(int argc, char **argv){
	ros::Time begin;
	doggo = new QUADRUPED(modelFile);

	
	ottim = new PROB_QUAD;

	
	ottim_cp = new PROB_QUAD_CP; 
	
	TrajPlanner *traiettoria;
	doggoControl = new QUADRUPEDController(*doggo);

	//Step Control
	
	doggoStep = new QUADRUPEDStep(*doggo);
	
	//Start node
	ros::init(argc, argv, "ros_control_node");
	ros::NodeHandle _nh;
	
	// Gazebo node 
     gazebo::transport::NodePtr node(new gazebo::transport::Node());
     node->Init();

	// Rate
      ros::Rate loop_rate(1000);
      gazebo::client::setup(argc,argv);

    
	//Subscriber
	ros::Subscriber _jointstate_sub = _nh.subscribe ("/dogbot/joint_states", 1, Joint_cb);
	ros::Subscriber _eebl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",1, eebl_cb);
	ros::Subscriber _eebr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",1, eebr_cb);
	ros::Subscriber _eefl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",1, eefl_cb);
	ros::Subscriber _eefr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",1, eefr_cb);
	ros::Subscriber _modelState_sub = _nh.subscribe("/gazebo/model_states", 1, modelState_cb);
	
	//Publisher
	//ros::Publisher _tau_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command",1);
	_tau_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command",1);
	
	
	
	//Service 
	ros::ServiceClient set_model_configuration_srv = _nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient set_model_state_srv = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	ros::ServiceClient pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
		 
	// Gazebo publisher in case the qp problem takes too long for the control loop
      //gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
	  pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
      pub->WaitForConnection();
	  
      //gazebo::msgs::WorldControl stepper;
     // Set multi-step to requested iterations
       stepper.set_step(1);
	   

	//Segnale di controllo
		std_msgs::Float64MultiArray msg_ctrl;
		//std_msgs::Float64MultiArray tau1_msg;
		std_msgs::Float64MultiArray tau_step_msg;
	
	    // Start the robot in position (stand up) 
      gazebo_msgs::SetModelConfiguration robot_init_config;
      robot_init_config.request.model_name = "dogbot";
      robot_init_config.request.urdf_param_name = "robot_description";
      robot_init_config.request.joint_names.push_back("back_left_roll_joint");
      robot_init_config.request.joint_names.push_back("back_left_pitch_joint");
      robot_init_config.request.joint_names.push_back("back_left_knee_joint");
      robot_init_config.request.joint_names.push_back("back_right_roll_joint");
      robot_init_config.request.joint_names.push_back("back_right_pitch_joint");
      robot_init_config.request.joint_names.push_back("back_right_knee_joint");
      robot_init_config.request.joint_names.push_back("front_left_roll_joint");
      robot_init_config.request.joint_names.push_back("front_left_pitch_joint");
      robot_init_config.request.joint_names.push_back("front_left_knee_joint");
      robot_init_config.request.joint_names.push_back("front_right_roll_joint");
      robot_init_config.request.joint_names.push_back("front_right_pitch_joint");
      robot_init_config.request.joint_names.push_back("front_right_knee_joint");
      robot_init_config.request.joint_positions.push_back(0.0037);
      robot_init_config.request.joint_positions.push_back(-0.8426);
      robot_init_config.request.joint_positions.push_back(-1.6216);
      robot_init_config.request.joint_positions.push_back(0.0039);
      robot_init_config.request.joint_positions.push_back(0.8421);
      robot_init_config.request.joint_positions.push_back(1.6165);
      robot_init_config.request.joint_positions.push_back(-0.0032);
      robot_init_config.request.joint_positions.push_back(-0.847);
      robot_init_config.request.joint_positions.push_back(-1.6302);
      robot_init_config.request.joint_positions.push_back(-0.0034);
      robot_init_config.request.joint_positions.push_back(0.8467);
      robot_init_config.request.joint_positions.push_back(1.6256);
      if(set_model_configuration_srv.call(robot_init_config))
        ROS_INFO("Robot configuration set.");
      else
        ROS_INFO("Failed to set robot configuration.");

      gazebo_msgs::SetModelState robot_init_state;
      robot_init_state.request.model_state.model_name = "dogbot";
      robot_init_state.request.model_state.reference_frame = "world";
      robot_init_state.request.model_state.pose.position.x=-0.00182;
      robot_init_state.request.model_state.pose.position.y=-0.0106414;
      robot_init_state.request.model_state.pose.position.z=0.426005;
      robot_init_state.request.model_state.pose.orientation.x=0.0;
      robot_init_state.request.model_state.pose.orientation.y=0.0;
      robot_init_state.request.model_state.pose.orientation.z=0.0;
      robot_init_state.request.model_state.pose.orientation.w=0.0;
	
      if(set_model_state_srv.call(robot_init_state))
        ROS_INFO("Robot state set.");
      else
        ROS_INFO("Failed to set robot state.");

	  
	   
	   std_srvs::Empty pauseSrv;
		 
		 while(!joint_state_available && !joint_base_available )
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        ros::spinOnce();
    }
	
	//pause gazebo
	pauseGazebo.call(pauseSrv); //da commentare 

	//Update robot 
	


	doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
	
	//traiettoria
	double ti=0.0, tf=0.3, t=0.0; //rimetti tf 1.5
	
	Matrix<double,6,1> init_pos, end_pos, init_vel, end_vel, init_acc, end_acc;

	// Initial position
      init_pos= doggo->getCOMpos();
    
    // Desired position
      end_pos<<0.0, -0.0, 0.4,0,0,0;
   
    // Initial velocity
      init_vel= doggo->getCOMvel();
	

	end_vel = Matrix<double,6,1>::Zero();
	end_vel = init_acc = end_acc;

	traiettoria = new TrajPlanner(ti, tf, init_pos, end_pos, init_vel, end_vel, init_acc, end_acc);
	
	trajectory_point traj;
     traj = traiettoria->getTraj();
	 
	 // Set Gain Matrix per controllo 
    Kcom=2500*Eigen::MatrixXd::Identity(6,6);
    Dcom=50*Eigen::MatrixXd::Identity(6,6);

		// initial simulation time 
    begin = ros::Time::now();
	 ts = ros::Time::now();
     ROS_INFO_STREAM_ONCE("Starting control loop ...");
	bool cpok = true; 
	  while ((ros::Time::now()-begin).toSec() < tf-0.001 && cpok) //dopo rimetti cpok senza !
    { 
		//prendo punto della traiettorie nell'ista desi 
	
		cout<<"joint_state_available: "<<joint_state_available<<endl;
			if(joint_state_available==true && joint_base_available==true)
			{
				
				doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
				VectorXd b=doggo->getBiasMatrix();
				Matrix<double,18,18> M=doggo->getMassMatrix();
				Matrix<double,24,18> Jc=doggo->getJacobian();
				Matrix<double,24,1> Jcdqd=doggo->getBiasAcc();
				Matrix<double,18,18> T=doggo->getTransMatrix();
				Matrix<double,18,18> T_dot=doggo->getTdotMatrix();
			    Matrix<double,6,18> Jcom=doggo->getJCOMMatrix();				
                Matrix<double,6,18> Jcomdot=doggo->getJCOMDot();				
				MatrixXd com_pos=doggo->getCOMpos();				
                MatrixXd com_vel=doggo->getCOMvel();
				com_file<<com_pos(0)<<" "<<com_pos(1)<<" "<<com_pos(2)<<" "<<com_pos(3)<<" "<<com_pos(4)<<" "<<com_pos(5)<<"\n";
				com_file.flush();
				ts = ros::Time::now();
				tempo_simulazione_file<<ts<<"\n";
				tempo_simulazione_file.flush();
				//cout<<"b:"<<endl<<b<<endl;				
				// Time
         		t = (ros::Time::now()-begin).toSec();
         		int idx= std::round( t*1000);
				
				//Calcolo vettori desiderati prendo solo la posizione e non l'orientamento
				Matrix<double,6,1> composdes, comveldes, comaccdes;
				composdes<<traj.pos(0,idx), traj.pos(1,idx), traj.pos(2,idx),MatrixXd::Zero(3,1);
				comveldes<<traj.vel(0,idx), traj.vel(1,idx), traj.vel(2,idx),MatrixXd::Zero(3,1);
				
				comaccdes<<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx),MatrixXd::Zero(3,1);
				com_traiettoria_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
				com_traiettoria_des_file.flush();

			
			
				//stampo giunti che passo al controllo ottimo
				for (int i=0; i<18; i++){
					cout<<"joint "<<i<<": "<<q_joints_total[i]<<endl;
				}
				//controllo senza capture point
				//ottim->CalcoloProbOttimo(b, M, Jc, Jcdqd, T, T_dot, q_joints_total, dq_joints_total, composdes, comveldes, com_pos, com_vel, Jcom, Jcomdot);
				//vector<double> tau = ottim->getTau();
				float x_inf, x_sup, y_inf, y_sup;
				
				x_inf = coo_ee_bl(0);
				x_sup = coo_ee_br(0);
    			y_inf = coo_ee_bl(1);
    			y_sup = coo_ee_fr(1);
				//Controllo con capture point
				//ottim_cp->CalcoloProbOttimoCP(b, M, Jc, Jcdqd, T, T_dot, q_joints_total, dq_joints_total, composdes, comveldes, com_pos, com_vel, Jcom, Jcomdot,
				//x_inf, x_sup, y_inf, y_sup);
				//vector<double> tau = ottim_cp->getTau();

				//controllo sul capture point e traiettoria
				
				cpx = com_pos(0)+com_vel(0)/w;
				cpy = com_pos(1)+com_vel(1)/w;
				cout<<"cpx-main: "<<cpx<<endl;
				cout<<"cpy-main: "<<cpy<<endl;
				cout<<"y_sup: "<<y_sup<<endl;
				cout<<"y_inf: "<<y_inf<<endl;
				//supponendo la spinta con la stessa direzione dell'asse x
			
				capture_point_file<<cpx<<" "<<cpy<<"\n";
				capture_point_file.flush();
				
				if((cpx > x_sup-0.02 || cpy > y_sup-0.035)&&coo_ee_fr(1)>0.001)
				{	
					cpok = false;
					cpx = com_pos(0)+com_vel(0)/w;
					cpy = com_pos(1)+com_vel(1)/w;
					
					Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
					Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
					Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
					Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
					
					force_br = Tbr * eef_br;
					force_bl = Tbl * eef_bl;
					force_fr = Tfr * eef_fr;
					force_fl = Tfl * eef_fl;
					
					cout<<"cpx: "<<cpx<<endl;
					cout<<"cpy: "<<cpy<<endl;
					
					float copx = (force_bl(2) * coo_ee_bl(0) + force_br(2) * coo_ee_br(0) + force_fr(2) * coo_ee_fr(0) + force_fl(2) * coo_ee_fl(0))/(force_bl(2)+force_br(2)+force_fr(2)+force_fl(2));
					float copy = (force_bl(2) * coo_ee_bl(1) + force_br(2) * coo_ee_br(1) + force_fr(2) * coo_ee_fr(1) + force_fl(2) * coo_ee_fl(1))/(force_bl(2)+force_br(2)+force_fr(2)+force_fl(2));
					float cpx_f = (cpx-copx)* pow(M_E,(w*t_step)) + copx;
					float cpy_f = (cpy-copy)* pow(M_E,(w*t_step)) + copy;

					cout<<"copx: "<<copx<<endl;
					cout<<"copy: "<<copy<<endl;
					cout<<"cpx_f: "<<cpx_f<<endl;
					cout<<"cpy_f: "<<cpy_f<<endl;
					
				}

				foothold_file<<coo_ee_bl(0)<<" "<<coo_ee_bl(1)<<" "<<coo_ee_bl(2)<<" "<<coo_ee_br(0)<<" "
				<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<coo_ee_fl(0)<<" "<<coo_ee_fl(1)<<" "<<coo_ee_fl(2)<<" "
				<<coo_ee_fr(0)<<" "<<coo_ee_fr(1)<<" "<<coo_ee_fr(2)<<" "<<"\n";
				foothold_file.flush();
				x_lim_file<<x_sup<<"\n";
				x_lim_file.flush();

				//--------------------pubblico le coppie calcolate --------------------
				/*
				// copy in the data
				
				msg_ctrl.data.clear();
				//msg_ctrl.data.insert(msg_ctrl.data.end(), tau.begin(), tau.end());
				//stampa tau e controlla anche il topic command
				for(int i =0; i<12; i++){
					cout<<"tau: "<<tau[11-i]<<endl;
					tau_file<<tau[11-i]<<" ";
					msg_ctrl.data.push_back(tau[i]);
				}
				tau_file<<"\n";
				tau_file.flush();
				_tau_pub.publish(msg_ctrl);
				*/
				//-------------------------------coppie per controllo Viviana
				
				// Compute control torque
				// control vector
      			//Eigen::VectorXd tau; non serve piú perchè l'ho dichiarato globale 
      			tau.resize(12);
       			tau = doggoControl->Cntr(composdes, comveldes, comaccdes, Kcom, Dcom, 
				   x_inf, x_sup, y_inf, y_sup);
				
      			 std::cout<<"tau"<<tau<<std::endl;
				// Set command message
				tau1_msg.data.clear();
				std::vector<double> ta(12,0.0);

				// torques in right order
				ta[11]=tau(7);
				ta[10]=tau(6);
				ta[9]=tau(2);
				ta[8]=tau(5);
				ta[7]=tau(4);
				ta[6]=tau(3);
				ta[5]=tau(9);
				ta[4]=tau(8);
				ta[3]=tau(1);
				ta[2]=tau(11);
				ta[1]=tau(10);
				ta[0]=tau(0);


				// Fill Command message
				for(int i=0; i<12; i++)
				{
			
					tau1_msg.data.push_back(ta[i]);
				}

				//Sending command
					_tau_pub.publish(tau1_msg);
				
				//-----------------------------------------------------
			}
				// One step in gazebo world ( to use if minqp problem takes too long for control loop)
				pub->Publish(stepper);
				
				ros::spinOnce();
				
				loop_rate.sleep();
		
		
		
	}
	
		cout<<"cpok: "<<cpok<<endl;
		if(cpok==false)//memo rimetti la condizione uguale a false 
		{
				doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			
				cout<<"if1"<<endl;
				MatrixXd com_pos=doggo->getCOMpos();
				MatrixXd com_vel=doggo->getCOMvel();

			
				cpx = com_pos(0)+com_vel(0)/w;
				cpy = com_pos(1)+com_vel(1)/w;
				/*

				Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
      			Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
      			Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
      			Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
				
				force_br = Tbr * eef_br;
				force_bl = Tbl * eef_bl;
				force_fr = Tfr * eef_fr;
				force_fl = Tfl * eef_fl;
      			
				float copx = (force_bl(2) * coo_ee_bl(0) + force_br(2) * coo_ee_br(0) + force_fr(2) * coo_ee_fr(0) + force_fl(2) * coo_ee_fl(0))/(force_bl(2)+force_br(2)+force_fr(2)+force_fl(2));
				float copy = (force_bl(2) * coo_ee_bl(1) + force_br(2) * coo_ee_br(1) + force_fr(2) * coo_ee_fr(1) + force_fl(2) * coo_ee_fl(1))/(force_bl(2)+force_br(2)+force_fr(2)+force_fl(2));
				float cpx_f = (cpx-copx)* pow(M_E,(w*t_step)) + copx;
				float cpy_f = (cpy-copy)* pow(M_E,(w*t_step)) + copy;
				*/

				//while((ros::Time::now()-begin0).toSec() < t_fin2-0.001)// && (!contact_br))//rimetti fr al posto di fl !contact_fl && !contact_br
				//{	
					
					int movimento=5;

					if (movimento == 1){//muovo solo br
						formulation=computetrajecotry(1);
						begin0 = ros::Time::now();	
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(1)[0]);
						swing_phasebr(loop_rate, formulation.params_.ee_phase_durations_.at(1)[0]+formulation.params_.ee_phase_durations_.at(1)[1] , formulation.params_.ee_phase_durations_.at(1)[0]);
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(1)[0]+ formulation.params_.ee_phase_durations_.at(1)[1] + formulation.params_.ee_phase_durations_.at(1)[2]);
					}
					else if (movimento == 2){ //muovo br e fl (trot)
						
						formulation=computetrajecotry(2);
						
						// initial simulation time 
						begin0 = ros::Time::now();
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(1)[0]);
						swing_phase(loop_rate, formulation.params_.ee_phase_durations_.at(1)[0]+formulation.params_.ee_phase_durations_.at(1)[1] , formulation.params_.ee_phase_durations_.at(1)[0]);
						//stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(1)[0]+ formulation.params_.ee_phase_durations_.at(1)[1] + formulation.params_.ee_phase_durations_.at(1)[2]);
						
						//completo il trotto
						formulation=computetrajecotry(7);
						
						// initial simulation time 
						begin0 = ros::Time::now();
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(0)[0]);
						swing_phase2(loop_rate,formulation.params_.ee_phase_durations_.at(0)[0]+formulation.params_.ee_phase_durations_.at(0)[1], formulation.params_.ee_phase_durations_.at(0)[0]);
						stand_phase(loop_rate, formulation.params_.ee_phase_durations_.at(0)[0]+formulation.params_.ee_phase_durations_.at(0)[1]+ formulation.params_.ee_phase_durations_.at(0)[2]);
						
					}
					else if (movimento == 3){ //muovo br e fr (pace)
						formulation=computetrajecotry(3);
						// initial simulation time 
						begin0 = ros::Time::now();
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(1)[0]);
						swing_phase_p(loop_rate, formulation.params_.ee_phase_durations_.at(1)[0]+formulation.params_.ee_phase_durations_.at(1)[1] , formulation.params_.ee_phase_durations_.at(1)[0]);
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(1)[0]+ formulation.params_.ee_phase_durations_.at(1)[1] + formulation.params_.ee_phase_durations_.at(1)[2]);
						//riporto il dogbot nella posizione nominale
						formulation=computetrajecotry(4); //pace bl e fl
						// initial simulation time 
						begin0 = ros::Time::now();
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(0)[0]);
						swing_phase_p2(loop_rate, formulation.params_.ee_phase_durations_.at(0)[0]+formulation.params_.ee_phase_durations_.at(0)[1] , formulation.params_.ee_phase_durations_.at(0)[0]);
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(0)[0]+ formulation.params_.ee_phase_durations_.at(0)[1] + formulation.params_.ee_phase_durations_.at(0)[2]);
					}
					else if (movimento == 5){
						
						formulation=computetrajecotry(5);//bound fr e fl
						
						// initial simulation time 
						begin0 = ros::Time::now();
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(2)[0]);
						swing_phase_b(loop_rate, formulation.params_.ee_phase_durations_.at(2)[0]+formulation.params_.ee_phase_durations_.at(2)[1] , formulation.params_.ee_phase_durations_.at(2)[0]);
						//stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(2)[0]+ formulation.params_.ee_phase_durations_.at(2)[1] + formulation.params_.ee_phase_durations_.at(2)[2]);
						
						//riporto il dogbot nella posizione nominale
						formulation=computetrajecotry(6);//bound br e bl
						// initial simulation time
						begin0 = ros::Time::now();
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(1)[0]);
						swing_phase_b2(loop_rate, formulation.params_.ee_phase_durations_.at(1)[0]+formulation.params_.ee_phase_durations_.at(1)[1] , formulation.params_.ee_phase_durations_.at(1)[0]);
						stand_phase(loop_rate,formulation.params_.ee_phase_durations_.at(1)[0]+ formulation.params_.ee_phase_durations_.at(1)[1] + formulation.params_.ee_phase_durations_.at(1)[2]);					
					}
					if(pauseGazebo.call(pauseSrv))
        				ROS_INFO("Simulation paused.");
    				else
        				ROS_INFO("Failed to pause simulation.");
					
					
					
				//}
		}
 		
 //pase gazebo call
 pauseGazebo.call(pauseSrv);
 	
	
	
		return(0);	

}