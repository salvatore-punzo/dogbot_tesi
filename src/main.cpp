#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <stdlib.h>

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ModelStates.h"
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


using namespace Eigen;
using namespace std;


// Variabili globali

bool joint_state_available = false, joint_base_available = false;

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
Vector3d com_pos,com_vel;


void Joint_cb(sensor_msgs::JointStateConstPtr js){
	joint_state_available = true;
	
	
	//memo pitch e hip coincidono
	hp<<js->position[1],js->position[4],js->position[7],js->position[10]; //rispettivamente bl,br,fl,fr
	he<<js->effort[1],js->effort[4],js->effort[7],js->effort[10];
	kp<<js->position[0],js->position[3],js->position[6],js->position[9];
	rp<<js->position[2],js->position[5],js->position[8],js->position[11];
/*
	float _rblp = js->position[2]; //back_left_roll_joint
	float _rbrp = js->position[5];
	float _rflp = js->position[8];
	float _rfrp = js->position[11];

	float _hblv = js->velocity[1];
	float _hbrv = js->velocity[4];
	float _hflv = js->velocity[7];
	float _hfrv = js->velocity[10];

	float _kblv = js->velocity[0];
	float _kbrv = js->velocity[3];
	float _kflv = js->velocity[6];
	float _kfrv = js->velocity[9];

	float _rblv = js->velocity[2];
	float _rbrv = js->velocity[5];
	float _rflv = js->velocity[8];
	float _rfrv = js->velocity[11];
*/

	q_joints << js->position[11], js->position[8], js->position[2], js->position[5], js->position[4],  js->position[3], js->position[1],  js->position[0] , js->position[7], js->position[6], js->position[10], js->position[9];
	dq_joints<< js->velocity[11], js->velocity[8], js->velocity[2], js->velocity[5], js->velocity[4],  js->velocity[3], js->velocity[1],  js->velocity[0] , js->velocity[7], js->velocity[6], js->velocity[10],  js->velocity[9];

		
}

//posizione del contatto e misura della forza
void eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){

	if(eebl->states.empty()){ 
	}
	else{
		eef_bl<<eebl->states[0].total_wrench.force.x,
		eebl->states[0].total_wrench.force.y, eebl->states[0].total_wrench.force.z;

		coo_ee_bl<<eebl->states[0].contact_positions[0].x, eebl->states[0].contact_positions[0].y, eebl->states[0].contact_positions[0].z;
	}
}
//posizione del contatto e misura della forza
void eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){

	if(eebr->states.empty()){
	}
	else{
		eef_br<<eebr->states[0].total_wrench.force.x,
		eebr->states[0].total_wrench.force.y, eebr->states[0].total_wrench.force.z;

		coo_ee_br<<eebr->states[0].contact_positions[0].x, eebr->states[0].contact_positions[0].y,eebr->states[0].contact_positions[0].z;
	}
}
//posizione del contatto e misura della forza
void eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){
	if(eefl->states.empty()){
	}
	else{
		eef_fl<<eefl->states[0].total_wrench.force.x,
		eefl->states[0].total_wrench.force.y, eefl->states[0].total_wrench.force.z;

		coo_ee_fl<< eefl->states[0].contact_positions[0].x, eefl->states[0].contact_positions[0].y, eefl->states[0].contact_positions[0].z;
	}
}
//posizione del contatto e misura della forza
void eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){
	if(eefr->states.empty()){
	}
	else{
		eef_fr<<eefr->states[0].total_wrench.force.x,
		eefr->states[0].total_wrench.force.y, eefr->states[0].total_wrench.force.z;

		coo_ee_fr<<eefr->states[0].contact_positions[0].x,eefr->states[0].contact_positions[0].y, eefr->states[0].contact_positions[0].z;
	}
}

//posizione centro di massa
void Com_cb(geometry_msgs::PointStampedConstPtr pos){
	
	 com_pos << pos->point.x, pos->point.y, pos->point.z;
}

//velocità centro di massa
void VCom_cb(geometry_msgs::TwistStampedConstPtr data){
	
	com_vel << data->twist.linear.x, data->twist.linear.y, data->twist.linear.z;
	
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
	//tf::Matrix3x3(q).getEulerYPR(yaw_eu,pitch_eu,roll_eu);
	/*
	world_H_base << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll), floating_base_position[0],
					sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), floating_base_position[1],
					-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), floating_base_position[2],
					0, 0, 0, 1;
*/
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

int main(int argc, char **argv){
	string modelFile="/home/salvatore/ros_ws/src/DogBotV4/ROS/src/dogbot_description/urdf/dogbot.urdf";

	CIN_DIR  *leglength; 
	leglength = new CIN_DIR;

	QUADRUPED *doggo;
	doggo = new QUADRUPED(modelFile);

	PROB_QUAD *ottim;
	ottim = new PROB_QUAD;

	POLI_SUP *poligono_sup;
	poligono_sup = new POLI_SUP;  
	
	cout<<"prova0"<<endl;
	TrajPlanner *traiettoria;

	CAPTURE_POINT *cp;
	cp = new CAPTURE_POINT;
	
	
	
	//Start node
	ros::init(argc, argv, "ros_control_node");
	ros::NodeHandle _nh;
	cout<<"prova00"<<endl;
	// Gazebo node 
     gazebo::transport::NodePtr node(new gazebo::transport::Node());
     node->Init();

	// Rate
      ros::Rate loop_rate(1000);
      gazebo::client::setup(argc,argv);

    cout<<"prova01"<<endl;
	//Subscriber
	ros::Subscriber _jointstate_sub = _nh.subscribe ("/dogbot/joint_states", 1, Joint_cb);
	ros::Subscriber _eebl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",1, eebl_cb);
	ros::Subscriber _eebr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",1, eebr_cb);
	ros::Subscriber _eefl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",1, eefl_cb);
	ros::Subscriber _eefr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",1, eefr_cb);
	ros::Subscriber _modelState_sub = _nh.subscribe("/gazebo/model_states", 1, modelState_cb);
	ros::Subscriber _com_sub = _nh.subscribe("/dogbot/cog",1 ,Com_cb);
	ros::Subscriber _vcom_sub = _nh.subscribe("/dogbot/v_cog",1, VCom_cb);
	
	//Publisher
	ros::Publisher _tau_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command",1);
	ros::Publisher _errore_pub = _nh.advertise<std_msgs::Float64>("/errore",1);
	//Service 
	ros::ServiceClient pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
		 cout<<"prova02"<<endl;
	// Gazebo publisher in case the qp problem takes too long for the control loop
      gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
	  	cout<<"prova020"<<endl;
      pub->WaitForConnection();
	  cout<<"prova021"<<endl;
      gazebo::msgs::WorldControl stepper;
     // Set multi-step to requested iterations
       stepper.set_step(1);
	   cout<<"prova03"<<endl;

	//Segnale di controllo
	   std_msgs::Float64MultiArray msg_ctrl;

	   std_srvs::Empty pauseSrv;
		 cout<<"prova04"<<endl;
		 while(!joint_state_available && !joint_base_available )
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        ros::spinOnce();
    }
	
	//pause gazebo
	pauseGazebo.call(pauseSrv);

	//Update robot 
	doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
	
	//traiettoria
	double ti=0.0, tf=1.0, t=0.0;
	
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
	

		// initial simulation time 
     ros::Time begin = ros::Time::now();
     ROS_INFO_STREAM_ONCE("Starting control loop ...");

	  while ((ros::Time::now()-begin).toSec() < tf-0.001)
    { 
		//prendo punto della traiettorie nell'ista desi 
	
	cout<<"joint_state_available: "<<joint_state_available<<endl;
			if(joint_state_available==true && joint_base_available==true){
				/*
				cout<<"world_H_base"<<endl<<world_H_base<<endl;
				cout<<"basevel"<<endl<<basevel<<endl;
				cout<<"q_joints"<<endl<<q_joints<<endl;
				cout<<"dq_joints"<<endl<<dq_joints<<endl;
				cout<<"gravity1"<<endl<<gravity1<<endl;
				*/
				
			
				VectorXd b=doggo->getBiasMatrix();
				Matrix<double,18,18> M=doggo->getMassMatrix();
				Matrix<double,24,18> Jc=doggo->getJacobian();
				Matrix<double,24,1> Jcdqd=doggo->getBiasAcc();
				Matrix<double,18,18> T=doggo->getTransMatrix();
				Matrix<double,18,18> T_dot=doggo->getTdotMatrix();
			
				//cout<<"b:"<<endl<<b<<endl;

				// Time
         		t = (ros::Time::now()-begin).toSec();
         		int idx= std::round( t*1000);

				//Calcolo vettori desiderati prendo solo la posizione e non l'orientamento
				Matrix<double,3,1> composdes, comveldes, comaccdes;
				composdes<<traj.pos(0,idx), traj.pos(1,idx), traj.pos(2,idx);
				comveldes<<traj.vel(0,idx), traj.vel(1,idx), traj.vel(2,idx);
				
				comaccdes<<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx);

				cout<<"com posizione desiderata:"<<endl<<composdes<<endl;
				cout<<"com velocità desiderata:"<<endl<<comveldes<<endl;
				
				//Calcolo lunghezza gamba fisica
				leglength->calculate_leg_length_cb(hp, kp);
				VectorXd ll = leglength->getLegLength();
				/*
				cout<<"leglength: "<<endl<<ll<<endl;
				cout<<"hp: "<<endl<<hp<<endl;
				cout<<"he: "<<endl<<he<<endl;
				cout<<"coordinate ee bl: "<<endl<<coo_ee_bl<<endl;
				*/
				poligono_sup->calcoloPoligonoSupporto(ll, hp, he, kp, rp, eef_bl, eef_br, eef_fl, eef_fr, coo_ee_bl, coo_ee_br, coo_ee_fl, coo_ee_fr, rot_world_virtual_base);
				//coefficinete angolare
				float m = poligono_sup->getm();
				//intercetta verticale
				float q_positive = poligono_sup->getq_newp();
				float q_negative = poligono_sup->getq_newn();
				float q_s = poligono_sup->getqs();
				float q_r = poligono_sup->getqr();

				
			
				//stampo giunti che passo al controllo ottimo
				for (int i=0; i<18; i++){
					cout<<"joint "<<i<<": "<<q_joints_total[i]<<endl;
				}

				//ottim->CalcoloProbOttimo(b, M, Jc, Jcdqd, T, T_dot, q_joints_total, dq_joints_total, composdes, comveldes, com_pos, com_vel);
				//vector<double> tau = ottim->getTau();
				cout<<"wow"<<endl;
				cp->capture_point(b, M, Jc, Jcdqd, T, T_dot, q_joints_total, dq_joints_total, com_pos, com_vel, m, q_positive, q_negative, q_s, q_r);
				cout<<"wow1"<<endl;
				//--------------------pubblico le coppie calcolate --------------------
				
			cout<<"prova4"<<endl;
/*
				// set up dimensions
				msg_ctrl.layout.dim.push_back(std_msgs::MultiArrayDimension());
				msg_ctrl.layout.dim[0].size = tau.size();
				msg_ctrl.layout.dim[0].stride = 1;
				msg_ctrl.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

				// copy in the data
				msg_ctrl.data.clear();
				msg_ctrl.data.insert(msg_ctrl.data.end(), tau.begin(), tau.end());
				//stampa tau e controlla anche il topic command
				for(int i =0; i<12; i++){
					cout<<"tau: "<<tau[11-i]<<endl;
				}
				
				_tau_pub.publish(msg_ctrl);
		*/		
			

			}
			// One step in gazebo world ( to use if minqp problem takes too long for control loop)
        	pub->Publish(stepper);
			cout<<"stepper"<<endl;
			ros::spinOnce();
			cout<<"stepper2"<<endl;
			
		
		
		
	}
 //pase gazebo call
 pauseGazebo.call(pauseSrv);
 	
	
	
		return(0);	

}