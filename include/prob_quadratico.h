
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include "std_msgs/Int32.h"
#include <stdlib.h>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>

#include <std_msgs/Float64MultiArray.h>

#include "alglib/optimization.h"
#include "alglib/stdafx.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"

using namespace Eigen;
using namespace alglib;
using namespace std;

class PROB_QUAD{
    public:
        PROB_QUAD();
        void CalcoloProbOttimo(VectorXd &b, Matrix<double,18,18> &M, Matrix<double,24,18> &Jc, Matrix<double,24,1> &Jcdqd, Matrix<double,18,18> &T, Matrix<double,18,18> &T_dot,Matrix<double, 18,1> &q_joints_total, Matrix<double, 18,1> &dq_joints_total, Matrix<double,6,1> &composdes, Matrix<double,6,1> &comveldes,  MatrixXd &com_pos, MatrixXd &com_vel,  Eigen::Matrix<double,6,18> Jt1, Eigen::Matrix<double,6,18> Jcomdot);
        //get function
        vector<double> getTau();

    private:
    

    Matrix<double,18,12> Jc_T_B;
	Matrix<double,12,18> B_T_Jc;
	Matrix<double,4,12> Sn;
	
	Matrix<double,6,18> Jt1_T;
	Matrix<double,6,1> Jt1_Tdot_dq;
    Matrix<double,12,18> S;
	Matrix<double,18,12> S_T;
    Matrix<double,24,12> B;
	Matrix<double, 6, 1> q_base;
	
	int CT[40];
	Matrix<double,6,1> e;
	Matrix<double,6,1> e_dot;
	Matrix<double,6,6> kp = 2500*MatrixXd::Identity(6,6);
	Matrix<double,6,6> kd = 50*MatrixXd::Identity(6,6); //2*sqrt(kp);
    double com_zdes = 0.4;

    vector<double> tau;
/*
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
*/
};
