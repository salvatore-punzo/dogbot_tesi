#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <stdlib.h>
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
#include <tf/tf.h>
#include "cin_diretta.h" 
#include "quadruped.h"
#include "prob_quadratico.h"
#include "poligono_supporto.h"


using namespace Eigen;
using namespace std;
using namespace alglib;

bool update = false;

VectorXd hp(4);
VectorXd kp(4);
VectorXd he(4);
VectorXd rp(4);

Vector3d eef_bl;
Vector3d eef_br;
Vector3d eef_fl;
Vector3d eef_fr;

Vector3d coo_ee_bl;
Vector3d coo_ee_br;
Vector3d coo_ee_fl;
Vector3d coo_ee_fr;

Matrix3d rot_world_virtual_base;

Vector3d gravity1(0,0, -9.81); //mi serve solo per inizializzare la classe quadruped->update vedi se va bene dichiararlo qui
Matrix4d world_H_base; //come sopra
VectorXd q_joints(12,1);
VectorXd dq_joints(12,1);
Matrix<double,6,1> basevel;
Vector3d floating_base_position;
Matrix<double, 18,1> q_joints_total;
Matrix<double, 18,1> dq_joints_total;
Matrix<double,6,1> floating_base_pose;
Matrix<double,6,1> floating_base_posed;
double com_zdes = 0.4;


void Joint_cb(sensor_msgs::JointStateConstPtr js){

	update = true;
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

	q_joints << js->position[11], js->position[8], js->position[2], js->position[5], hp[1],  kp[1], hp[0],  kp[0] , hp[2], kp[2], hp[3], kp[3];
	dq_joints << js->velocity[11], js->velocity[8], js->velocity[2], js->velocity[5], js->velocity[4],  js->velocity[3], js->velocity[1],  js->velocity[0] , js->velocity[7], js->velocity[6], js->velocity[10],  js->velocity[9];

		
}

void eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){
	
	eef_bl<<eebl->states[0].total_wrench.force.x,
	eebl->states[0].total_wrench.force.y, eebl->states[0].total_wrench.force.z;

	coo_ee_bl<<eebl->states[0].contact_positions[0].x, eebl->states[0].contact_positions[0].y, eebl->states[0].contact_positions[0].z;
}
void eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){
	
	eef_br<<eebr->states[0].total_wrench.force.x,
	eebr->states[0].total_wrench.force.y, eebr->states[0].total_wrench.force.z;

	coo_ee_br<<eebr->states[0].contact_positions[0].x, eebr->states[0].contact_positions[0].y,eebr->states[0].contact_positions[0].z;
}

void eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){

	eef_fl<<eefl->states[0].total_wrench.force.x,
	eefl->states[0].total_wrench.force.y, eefl->states[0].total_wrench.force.z;

	coo_ee_fl<< eefl->states[0].contact_positions[0].x, eefl->states[0].contact_positions[0].y, eefl->states[0].contact_positions[0].z;
}
void eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){
	
	eef_fr<<eefr->states[0].total_wrench.force.x,
	eefr->states[0].total_wrench.force.y, eefr->states[0].total_wrench.force.z;

	coo_ee_fr<<eefr->states[0].contact_positions[0].x,eefr->states[0].contact_positions[0].y, eefr->states[0].contact_positions[0].z;
}

void Com_cb(geometry_msgs::PointStampedConstPtr pos){
	
	float x_com = pos->point.x;
	float y_com = pos->point.y;
	float z_com = pos->point.z;
}

void VCom_cb(geometry_msgs::TwistStampedConstPtr data){
	
	float xdcom = data->twist.linear.x;
	float ydcom = data->twist.linear.y;
	float zdcom = data->twist.linear.z;

}

void vbody_cb(geometry_msgs::TwistStampedConstPtr vj){
	
	Vector3d vl_floating_base;
	Vector3d va_floating_base;
	

	vl_floating_base << vj->twist.linear.x, vj->twist.linear.y, vj->twist.linear.z;
	va_floating_base << vj->twist.angular.x, vj->twist.angular.y, vj->twist.angular.z;
	//cout<<"velocità floating base lungo x"<<vl_floating_base[0]<<endl;
	basevel << vj->twist.linear.x, vj->twist.linear.y, vj->twist.linear.z, vj->twist.angular.x, vj->twist.angular.y, vj->twist.angular.z;
	dq_joints_total<<basevel,dq_joints;
}

void modelState_cb(gazebo_msgs::ModelStatesConstPtr pt){

	double roll, pitch, yaw;
	double yaw_eu, pitch_eu, roll_eu;
	Matrix<double,3,1> floating_base_orientation;
	tf::Quaternion q(pt->pose[2].orientation.x, pt->pose[2].orientation.y, pt->pose[2].orientation.z, pt->pose[2].orientation.w);
	floating_base_position<<pt->pose[2].position.x, pt->pose[2].position.y, pt->pose[2].position.z;
	
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	tf::Matrix3x3(q).getEulerYPR(yaw_eu,pitch_eu,roll_eu);
	
	world_H_base << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll), floating_base_position[0],
					sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), floating_base_position[1],
					-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), floating_base_position[2],
					0, 0, 0, 1;

	rot_world_virtual_base << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll),
							sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
							-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll);

	floating_base_orientation<< roll_eu, pitch_eu, yaw_eu;
	q_joints_total<<floating_base_position, floating_base_orientation, q_joints;
	floating_base_pose<< floating_base_position, floating_base_orientation;
	
}

int main(int argc, char **argv){
	string modelFile="/home/salvatore/ros_ws/src/DogBotV4/ROS/src/dogbot_description/urdf/dogbot.urdf";
	
	CIN_DIR  *leglength; //va bene dichiarata in questa parte del codice?
	leglength = new CIN_DIR;

	QUADRUPED *doggo;
	doggo = new QUADRUPED(modelFile);

	PROB_QUAD *ottim;
	ottim = new PROB_QUAD;

	POLI_SUP *poligono_sup;
	poligono_sup = new POLI_SUP;
	
	ros::init(argc, argv, "ros_main");

    
    ros::NodeHandle _nh;
    
	ros::Subscriber _jointstate_sub = _nh.subscribe ("/dogbot/joint_states", 10, &Joint_cb);
	ros::Subscriber _eebl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",10, &eebl_cb);
	ros::Subscriber _eebr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",10, &eebr_cb);
	ros::Subscriber _eefl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",10, &eefl_cb);
	ros::Subscriber _eefr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",10, &eefr_cb);
	ros::Subscriber _modelState_sub = _nh.subscribe("/gazebo/model_states", 10, &modelState_cb);
	ros::Subscriber _com_sub = _nh.subscribe("/dogbot/cog",10 ,&Com_cb);
	ros::Subscriber _vcom_sub = _nh.subscribe("/dogbot/v_cog",10, &VCom_cb);
	ros::Subscriber _vbody_sub = _nh.subscribe("/dogbot/v_body",10, &vbody_cb);
	
	ros::Publisher _tau_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command",10);
	ros::Publisher _errore_pub = _nh.advertise<std_msgs::Float64>("/errore",10);
	
	//ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	
	
	ros::Rate rate(1000);
	int id;
		//ros::time::now() < 3 al posto di ros::ok
	while( ros::ok()){
		//metto ciclo for per traiettoria
		double dist = com_zdes - floating_base_position[2];
		double step = dist/1000;
		std_msgs::Float64 msg;
		
		
		for(id=0;id<1000;id++){
		ros::spinOnce();
		_errore_pub.publish(msg);

		if(update==true){

			dist = com_zdes - floating_base_position[2];
			msg.data=dist;
			
			leglength->calculate_leg_length_cb(hp, kp);
			VectorXd ll = leglength->getLegLength();
			/*
			cout<<"leglength: "<<endl<<ll<<endl;
			cout<<"hp: "<<endl<<hp<<endl;
			cout<<"he: "<<endl<<he<<endl;
			cout<<"coordinate ee bl: "<<endl<<coo_ee_bl<<endl;
			*/
			poligono_sup->calcoloPoligonoSupporto(ll, hp, he, kp, rp, eef_bl, eef_br, eef_fl, eef_fr, coo_ee_bl, coo_ee_br, coo_ee_fl, coo_ee_fr, rot_world_virtual_base);
			
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
			
			//cout<<"b:"<<endl<<b<<endl;
			floating_base_posed<<floating_base_pose[0], floating_base_pose[1],com_zdes, floating_base_pose[3],floating_base_pose[4],floating_base_pose[5];
			
		/*

			double t=0;
			while(t = (ros::Time::now()).toSec() < 3){
    		cout<<"tempo_simulazione:"<<t<<endl;
			}
         */
		
			ottim->CalcoloProbOttimo(b, M, Jc, Jcdqd, T, T_dot, q_joints_total, dq_joints_total,floating_base_position[2],basevel[2],floating_base_pose, floating_base_posed,id,step);
			vector<double> tau = ottim->getTau();
			//--------------------pubblico le coppie calcolate --------------------
			std_msgs::Float64MultiArray msg;
			

			// set up dimensions
			msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
			msg.layout.dim[0].size = tau.size();
			msg.layout.dim[0].stride = 1;
			msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

			// copy in the data
			msg.data.clear();
			msg.data.insert(msg.data.end(), tau.begin(), tau.end());
			
			_tau_pub.publish(msg);
			
			

		}//chiudo il ciclo for
		update=false;
		
		}
		//calcolo poligono(lunghezza gamba)
		
	}
 //pase gazebo call
 	
	
	
		return(0);	

}