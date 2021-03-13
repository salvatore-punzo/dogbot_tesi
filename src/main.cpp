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
float w=sqrt(9.81/com_zdes);
float  cpx, cpy;
ros::Time ts;
//Vector3d com_pos,com_vel;



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

//posizione centro di massa
/*void Com_cb(geometry_msgs::PointStampedConstPtr pos){
	
	 com_pos << pos->point.x, pos->point.y, pos->point.z;
}

//velocità centro di massa
void VCom_cb(geometry_msgs::TwistStampedConstPtr data){
	
	com_vel << data->twist.linear.x, data->twist.linear.y, data->twist.linear.z;
	
}*/



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
	std::ofstream com_file("com_file.txt");
	std::ofstream foothold_file("foothold.txt");
	std::ofstream foothold_des_file("foothold_des.txt");
	std::ofstream capture_point_file("capture_point.txt");
	std::ofstream tempo_simulazione_file("tempo_simulazione.txt");
	std::ofstream com_traiettoria_des_file("com_traiettoria_des.txt");
	std::ofstream tau_file("tau.txt");
	std::ofstream x_lim_file("x_lim_poligono.txt");

	CIN_DIR  *leglength; 
	leglength = new CIN_DIR;

	QUADRUPED *doggo;
	doggo = new QUADRUPED(modelFile);

	PROB_QUAD *ottim;
	ottim = new PROB_QUAD;

	PROB_QUAD_CP *ottim_cp;
	ottim_cp = new PROB_QUAD_CP;

	POLI_SUP *poligono_sup;
	poligono_sup = new POLI_SUP;  
	
	
	TrajPlanner *traiettoria;

	CAPTURE_POINT *cp;
	cp = new CAPTURE_POINT;
	
	// Set controller Viviana
    QUADRUPEDController *doggoControl; //controller_(doggo);
	doggoControl = new QUADRUPEDController(*doggo);

	//Step Control
	QUADRUPEDStep *doggoStep;
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
	//ros::Subscriber _com_sub = _nh.subscribe("/dogbot/cog",1 ,Com_cb);
	//ros::Subscriber _vcom_sub = _nh.subscribe("/dogbot/v_cog",1, VCom_cb);
	
	//Publisher
	ros::Publisher _tau_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command",1);
	//ros::Publisher _errore_pub = _nh.advertise<std_msgs::Float64>("/errore",1);
	ros::Publisher _foothold_pub = _nh.advertise<std_msgs::Float64MultiArray>("/foothold",1);
	ros::Publisher _cp_pub = _nh.advertise<geometry_msgs::Point>("/cp",1);
	ros::Publisher _x_lim_pub = _nh.advertise<std_msgs::Float64MultiArray>("/x_lim",1);
	
	//Service 
	ros::ServiceClient set_model_configuration_srv = _nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient set_model_state_srv = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	ros::ServiceClient pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
		 
	// Gazebo publisher in case the qp problem takes too long for the control loop
      gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
      pub->WaitForConnection();
	  
      gazebo::msgs::WorldControl stepper;
     // Set multi-step to requested iterations
       stepper.set_step(1);
	   

	//Segnale di controllo
		std_msgs::Float64MultiArray msg_ctrl;
		std_msgs::Float64MultiArray tau1_msg;
		std_msgs::Float64MultiArray tau_step_msg;
	//Topic per grafico
		std_msgs::Float64MultiArray foothold;
		geometry_msgs::Point capture_p;
		std_msgs::Float64MultiArray x_lim;
	
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
	pauseGazebo.call(pauseSrv);

	//Update robot 
	/*
	cout<<"world_H_base"<<endl<<world_H_base<<endl;
				cout<<"basevel"<<endl<<basevel<<endl;
				cout<<"q_joints"<<endl<<q_joints<<endl;
				cout<<"dq_joints"<<endl<<dq_joints<<endl;
				cout<<"gravity1"<<endl<<gravity1<<endl;
				*/

	doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
	
	//traiettoria
	double ti=0.0, tf=1.5, t=0.0;
	
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
	 
	 // Set Gain Matrix per controllo Viviana
     Eigen::MatrixXd Kcom=2500*Eigen::MatrixXd::Identity(6,6);
     Eigen::MatrixXd Dcom=50*Eigen::MatrixXd::Identity(6,6);

		// initial simulation time 
     ros::Time begin = ros::Time::now();
	 ts = ros::Time::now();
     ROS_INFO_STREAM_ONCE("Starting control loop ...");
	bool cpok = true; 
	  while ((ros::Time::now()-begin).toSec() < tf-0.001 && cpok)
    { 
		//prendo punto della traiettorie nell'ista desi 
	
		cout<<"joint_state_available: "<<joint_state_available<<endl;
			if(joint_state_available==true && joint_base_available==true)
			{
				/*
				cout<<"world_H_base"<<endl<<world_H_base<<endl;
				cout<<"basevel"<<endl<<basevel<<endl;
				cout<<"q_joints"<<endl<<q_joints<<endl;
				cout<<"dq_joints"<<endl<<dq_joints<<endl;
				cout<<"gravity1"<<endl<<gravity1<<endl;
				*/
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
/*
				cout<<"com posizione desiderata:"<<endl<<composdes<<endl;
				cout<<"com velocità desiderata:"<<endl<<comveldes<<endl;
				cout<<"com posizione ottenuta:"<<endl<<com_pos<<endl;
				cout<<"com velocità ottenuta:"<<endl<<com_vel<<endl;
*/
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
				float m_blfl = poligono_sup->getm_blfl();
				float m_flfr = poligono_sup->getm_flfr();
				float m_frbr = poligono_sup->getm_frbr();
				float m_brbl = poligono_sup->getm_brbl();

				float q_blfl = poligono_sup->getq_blfl();
				float q_flfr = poligono_sup->getq_flfr();
				float q_frbr = poligono_sup->getq_frbr();
				float q_brbl = poligono_sup->getq_brbl();

/*  //coefficienti angolari delle rette del poligono di supporto

				cout<<"m_blfl: "<<m_blfl<<endl;
				cout<<"m_flfr: "<<m_flfr<<endl;
				cout<<"m_frbr: "<<m_frbr<<endl;
				cout<<"m_brbl: "<<m_brbl<<endl;

	//intercette verticali delle rette del poligono di supporto

				cout<<"q_blfl: "<<q_blfl<<endl;
				cout<<"q_flfr: "<<q_flfr<<endl;
				cout<<"q_frbr: "<<q_frbr<<endl;
				cout<<"q_brbl: "<<q_brbl<<endl;
*/

				/* //parametri delle rette del poligono di supporto caso bipede
				//coefficinete angolare
				float m = poligono_sup->getm();
				//intercetta verticale
				float q_positive = poligono_sup->getq_newp();
				float q_negative = poligono_sup->getq_newn();
				float q_s = poligono_sup->getqs();
				float q_r = poligono_sup->getqr();
				*/
				/*
				cout<<"mm: "<<m<<endl;
				cout<<"qsm: "<<q_s<<endl;
				cout<<"qrm: "<<q_r<<endl;
				*/

			
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
    			y_sup = coo_ee_fl(1);
				//Controllo con capture point
				//ottim_cp->CalcoloProbOttimoCP(b, M, Jc, Jcdqd, T, T_dot, q_joints_total, dq_joints_total, composdes, comveldes, com_pos, com_vel, Jcom, Jcomdot,
				//m_blfl, m_flfr, m_frbr, m_brbl, q_blfl, q_flfr, q_frbr, q_brbl, x_inf, x_sup, y_inf, y_sup);
				//vector<double> tau = ottim_cp->getTau();

				//controllo sul capture point e traiettoria
				
				cpx = com_pos(0)+com_vel(0)/w;
				cpy = com_pos(1)+com_vel(1)/w;
				cout<<"cpx-main: "<<cpx<<endl;
				//supponendo la spinta con la stessa direzione dell'asse x
				capture_p.x=cpx;
				capture_p.y=cpy;
				_cp_pub.publish(capture_p);
				capture_point_file<<cpx<<" "<<cpy<<"\n";
				capture_point_file.flush();
				if(cpx > x_sup)
				{	
					cpok = false;
					
				}

				foothold_file<<coo_ee_bl(0)<<" "<<coo_ee_bl(1)<<" "<<coo_ee_bl(2)<<" "<<coo_ee_br(0)<<" "
				<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<coo_ee_fl(0)<<" "<<coo_ee_fl(1)<<" "<<coo_ee_fl(2)<<" "
				<<coo_ee_fr(0)<<" "<<coo_ee_fr(1)<<" "<<coo_ee_fr(2)<<" "<<"\n";
				foothold_file.flush();
				x_lim_file<<x_sup<<"\n";
				x_lim_file.flush();
				//pubblico i foothold
				foothold.data.clear();
				foothold.data.push_back(coo_ee_bl(0));
				foothold.data.push_back(coo_ee_bl(1));
				foothold.data.push_back(coo_ee_bl(2));
				foothold.data.push_back(coo_ee_br(0));
				foothold.data.push_back(coo_ee_br(1));
				foothold.data.push_back(coo_ee_br(2));
				foothold.data.push_back(coo_ee_fl(0));
				foothold.data.push_back(coo_ee_fl(1));
				foothold.data.push_back(coo_ee_fl(2));
				foothold.data.push_back(coo_ee_fr(0));
				foothold.data.push_back(coo_ee_fr(1));
				foothold.data.push_back(coo_ee_fr(2));

				_foothold_pub.publish(foothold);

				
				
				x_lim.data.push_back(x_sup);
				_x_lim_pub.publish(x_lim);

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
      			Eigen::VectorXd tau;
      			tau.resize(12);
       			tau = doggoControl->Cntr(composdes, comveldes, comaccdes, Kcom, Dcom, 
				   m_blfl, m_flfr, m_frbr, m_brbl, q_blfl, q_flfr, q_frbr, q_brbl,
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
				cout<<"stepper"<<endl;
				ros::spinOnce();
				cout<<"stepper2"<<endl;
				loop_rate.sleep();
		
		
		
	}
	
	
	if(cpok==false)//memo rimetti la condizione uguale a false 
	{
		//traiettoria 1
					doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
					MatrixXd com_pos=doggo->getCOMpos();
					MatrixXd com_vel=doggo->getCOMvel();	
					cpx = com_pos(0)+com_vel(0)/w;
					cpy = com_pos(1)+com_vel(1)/w;
					
					double t_in=0.0, t_fin=0.07, t1;
					ros::Time begin1 = ros::Time::now();
					
					//traiettoria del centro di massa
					Matrix<double,6,1> init_pos1, end_pos1, init_vel1, end_vel1, init_acc1, end_acc1;

					// Initial position
					init_pos1= doggo->getCOMpos();
					
					// Desired position
					end_pos1<<(cpx+0.002-coo_ee_br(0))/4, 0.0, 0.0, 0,0,0; //((cpx+0.01)/2-coo_ee_bl(0))/2
				
					// Initial velocity
					init_vel1= doggo->getCOMvel();
					
		 
					end_vel1 = Matrix<double,6,1>::Zero();
					end_vel1 = init_acc1 = end_acc1;
					//std::cout<<"acc"<<init_acc<<"acc2"<<end_acc<<std::endl;
					traiettoria = new TrajPlanner(t_in, t_fin, init_pos1, end_pos1, init_vel1, end_vel1, init_acc1, end_acc1);
					trajectory_point traj_com;
					traj_com = traiettoria->getTraj();
		 //TRAIETTORIA DELLE GAMBE 1
					
					
					Matrix<double,6,1> pos_ini, pos_fin, vel_ini, vel_fin, acc_ini, acc_fin;

					// Initial position
					pos_ini<< 0,0,0,0,0,0;
					
					// Desired position
					pos_fin<<(cpx+0.002-coo_ee_br(0))/2, 0, 0.2, 0,0,0; //(cpx+0.01-coo_ee_br(0))/2
				
					

					vel_fin = Matrix<double,6,1>::Zero();
					vel_fin = acc_ini = acc_fin =vel_ini;
					traiettoria = new TrajPlanner(t_in, t_fin, pos_ini, pos_fin, vel_ini, vel_fin, acc_ini, acc_fin); //prima spline
					trajectory_point traj1;
					traj1 = traiettoria->getTraj();

					
					
		 

		while((ros::Time::now()-begin1).toSec() < t_fin-0.001)
		{	
			t1 = (ros::Time::now()-begin1).toSec();
			int idx1= std::round( t1*1000);
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			//traiettoria per il centro di massa
			//Calcolo vettori desiderati prendo solo la posizione e non l'orientamento
					Matrix<double,6,1> composdes, comveldes, comaccdes;
					composdes<<traj.pos(0,idx1), traj.pos(1,idx1), traj.pos(2,idx1),MatrixXd::Zero(3,1);
					comveldes<<traj.vel(0,idx1), traj.vel(1,idx1), traj.vel(2,idx1),MatrixXd::Zero(3,1);
					comaccdes<<traj.acc(0,idx1), traj.acc(1,idx1), traj.acc(2,idx1),MatrixXd::Zero(3,1);
					//scrivo dati su file
					com_traiettoria_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
					com_traiettoria_des_file.flush();

					//traiettoria per le gambe
					Matrix<double,6,1> footposdes, footveldes, footaccdes;
					footposdes<<traj1.pos(0,idx1), traj1.pos(1,idx1), traj1.pos(2,idx1),MatrixXd::Zero(3,1);
					footveldes<<traj1.vel(0,idx1), traj1.vel(1,idx1), traj1.vel(2,idx1),MatrixXd::Zero(3,1);
					footaccdes<<traj1.acc(0,idx1), traj1.acc(1,idx1), traj1.acc(2,idx1),traj1.acc(0,idx1), traj1.acc(1,idx1), traj1.acc(2,idx1);
					
					com_pos = doggo->getCOMpos();
					com_vel = doggo->getCOMvel();
					//scrivo dati su file
					com_file<<com_pos(0)<<" "<<com_pos(1)<<" "<<com_pos(2)<<" "<<com_pos(3)<<" "<<com_pos(4)<<" "<<com_pos(5)<<"\n";
					com_file.flush();
					//cpx = com_pos(0)+com_vel(0)/w;
					//cpy = com_pos(1)+com_vel(1)/w;
					capture_point_file<<cpx<<" "<<cpy<<"\n";
					capture_point_file.flush();
					foothold_des_file<<footposdes(0)<<" "<<footposdes(1)<<" "<<footposdes(2)<<" "<<"\n";
					foothold_des_file.flush();
					foothold_file<<coo_ee_bl(0)<<" "<<coo_ee_bl(1)<<" "<<coo_ee_bl(2)<<" "<<coo_ee_br(0)<<" "
					<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<coo_ee_fl(0)<<" "<<coo_ee_fl(1)<<" "<<coo_ee_fl(2)<<" "
					<<coo_ee_fr(0)<<" "<<coo_ee_fr(1)<<" "<<coo_ee_fr(2)<<" "<<"\n";
					foothold_file.flush();
					ts = ros::Time::now();
					tempo_simulazione_file<<ts<<"\n";
					tempo_simulazione_file.flush();


					

					
					// control vector
					Eigen::VectorXd tau_step;
					tau_step.resize(12);
					tau_step = doggoStep->step(composdes,comveldes,comaccdes,Kcom,Dcom,footaccdes);
							
					
							
					//pubblico le tau
					std::cout<<"tau_step"<<tau_step<<std::endl;
					// Set command message
					tau_step_msg.data.clear();
					std::vector<double> ta(12,0.0);

					// torques in right order
					ta[11]=tau_step(7);
					ta[10]=tau_step(6);
					ta[9]=tau_step(2);
					ta[8]=tau_step(5);
					ta[7]=tau_step(4);
					ta[6]=tau_step(3);
					ta[5]=tau_step(9);
					ta[4]=tau_step(8);
					ta[3]=tau_step(1);
					ta[2]=tau_step(11);
					ta[1]=tau_step(10);
					ta[0]=tau_step(0);


					// Fill Command message
					for(int i=0; i<12; i++)
					{
						tau_file<<ta[11-i]<<" ";
						tau_step_msg.data.push_back(ta[i]);
					}
					tau_file<<"\n";
					tau_file.flush();
					//Sending command
						_tau_pub.publish(tau_step_msg);

						pub->Publish(stepper);
				
						ros::spinOnce();
		}
		
					//-------------------------traiettoria 2 del com--------------------------
					doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
					double t_in2=0.0, t_fin2=0.07, t2;
					ros::Time begin2 = ros::Time::now();
					com_pos=doggo->getCOMpos();
					com_vel=doggo->getCOMvel();
					//traiettoria del centro di massa
					Matrix<double,6,1> init_pos2, end_pos2, init_vel2, end_vel2, init_acc2, end_acc2;
		
					// Initial position
					init_pos2= doggo->getCOMpos();
					cpx = com_pos(0)+com_vel(0)/w;
					cpy = com_pos(1)+com_vel(1)/w;
					
					cout<<"com_pos t2: "<<endl<<com_pos<<endl;
					cout<<"com_vel t2: "<<endl<<com_vel<<endl;
					cout<<"w t2: "<<w<<endl;
					// Desired position
					end_pos2<<(cpx+0.002-coo_ee_br(0))/4, 0.0, 0.0, 0,0,0;//((0.01)/2-coo_ee_bl(0))/2 deve essere lo stesso cpx di prima
					
					cout<<"cpx t2: "<<cpx<<endl;
				
					// Initial velocity
					init_vel2= doggo->getCOMvel();
					

					end_vel2 = Matrix<double,6,1>::Zero();
					end_vel2 = init_acc2 = end_acc2;
					//std::cout<<"acc"<<init_acc<<"acc2"<<end_acc<<std::endl;
					traiettoria = new TrajPlanner(t_in2, t_fin2, init_pos2, end_pos2, init_vel2, end_vel2, init_acc2, end_acc2);
					trajectory_point traj_com2;
					traj_com2 = traiettoria->getTraj();
					//TRAIETTORIA DELLE GAMBE 2
					
					
					Matrix<double,6,1> pos_ini2, pos_fin2, vel_ini2, vel_fin2, acc_ini2, acc_fin2;

					// Initial position
					pos_ini2<< 0,0,0,0,0,0;
					
					// Desired position
					pos_fin2<<(cpx+0.002-coo_ee_br(0))/2, 0, -0.0, 0,0,0;//dopo rimetti 0.2 al terzo elemento mentre al I termine metti (cpx+0.01-coo_ee_br(0))/2
				
					

					vel_fin2 = Matrix<double,6,1>::Zero();
					vel_fin2 = acc_ini2 = acc_fin2 =vel_ini2;
					traiettoria = new TrajPlanner(t_in2, t_fin2, pos_ini2, pos_fin2, vel_ini2, vel_fin2, acc_ini2, acc_fin2); //prima spline
					trajectory_point traj2;
					traj2 = traiettoria->getTraj();


					
			while((ros::Time::now()-begin2).toSec() < t_fin2-0.001 && (!contact_fr && !contact_br))
		{	
			t2 = (ros::Time::now()-begin2).toSec();
			int idx2= std::round( t2*1000);
			
			doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
			//traiettoria per il centro di massa
			//Calcolo vettori desiderati prendo solo la posizione e non l'orientamento
					Matrix<double,6,1> composdes, comveldes, comaccdes;
					composdes<<traj_com2.pos(0,idx2), traj_com2.pos(1,idx2), traj_com2.pos(2,idx2),MatrixXd::Zero(3,1);
					comveldes<<traj_com2.vel(0,idx2), traj_com2.vel(1,idx2), traj_com2.vel(2,idx2),MatrixXd::Zero(3,1);
					comaccdes<<traj_com2.acc(0,idx2), traj_com2.acc(1,idx2), traj_com2.acc(2,idx2),MatrixXd::Zero(3,1);
					//scrivo dati su file
					com_traiettoria_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
					com_traiettoria_des_file.flush();


					//traiettoria per le gambe
					Matrix<double,6,1> footposdes, footveldes, footaccdes;
					footposdes<<traj2.pos(0,idx2), traj2.pos(1,idx2), traj2.pos(2,idx2),MatrixXd::Zero(3,1);
					footveldes<<traj2.vel(0,idx2), traj2.vel(1,idx2), traj2.vel(2,idx2),MatrixXd::Zero(3,1);
					footaccdes<<traj2.acc(0,idx2), traj2.acc(1,idx2), traj2.acc(2,idx2),traj2.acc(0,idx2), traj2.acc(1,idx2), traj2.acc(2,idx2);


					
					com_pos = doggo->getCOMpos();
					com_vel = doggo->getCOMvel();
					//scrivo dati su file
					com_file<<com_pos(0)<<" "<<com_pos(1)<<" "<<com_pos(2)<<" "<<com_pos(3)<<" "<<com_pos(4)<<" "<<com_pos(5)<<"\n";
					com_file.flush();
					//cpx = com_pos(0)+com_vel(0)/w;
					//cpy = com_pos(1)+com_vel(1)/w;
					capture_point_file<<cpx<<" "<<cpy<<"\n";
					capture_point_file.flush();
					foothold_des_file<<footposdes(0)<<" "<<footposdes(1)<<" "<<footposdes(2)<<" "<<"\n";
					foothold_des_file.flush();
					foothold_file<<coo_ee_bl(0)<<" "<<coo_ee_bl(1)<<" "<<coo_ee_bl(2)<<" "<<coo_ee_br(0)<<" "
					<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<coo_ee_fl(0)<<" "<<coo_ee_fl(1)<<" "<<coo_ee_fl(2)<<" "
					<<coo_ee_fr(0)<<" "<<coo_ee_fr(1)<<" "<<coo_ee_fr(2)<<" "<<"\n";
					foothold_file.flush();
					ts = ros::Time::now();
					tempo_simulazione_file<<ts<<"\n";
					tempo_simulazione_file.flush();

					
					// control vector
					Eigen::VectorXd tau_step;
					tau_step.resize(12);
					tau_step = doggoStep->step(composdes,comveldes,comaccdes,Kcom,Dcom,footaccdes);
								
					//pubblico le tau
					std::cout<<"tau_step"<<tau_step<<std::endl;
					// Set command message
					tau_step_msg.data.clear();
					std::vector<double> ta(12,0.0);

					// torques in right order
					ta[11]=tau_step(7);
					ta[10]=tau_step(6);
					ta[9]=tau_step(2);
					ta[8]=tau_step(5);
					ta[7]=tau_step(4);
					ta[6]=tau_step(3);
					ta[5]=tau_step(9);
					ta[4]=tau_step(8);
					ta[3]=tau_step(1);
					ta[2]=tau_step(11);
					ta[1]=tau_step(10);
					ta[0]=tau_step(0);


					// Fill Command message
					for(int i=0; i<12; i++)
					{
						tau_file<<ta[11-i]<<" ";
						tau_step_msg.data.push_back(ta[i]);
					}
					tau_file<<"\n";
					tau_file.flush();
					//Sending command
						_tau_pub.publish(tau_step_msg);
						
						pub->Publish(stepper);
				
						ros::spinOnce();
		}
	}
 
 //pase gazebo call
 pauseGazebo.call(pauseSrv);
 	
	
	
		return(0);	

}