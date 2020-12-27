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

class CAPTURE_POINT{
    public:
        CAPTURE_POINT();
        void capture_point(VectorXd &b, Matrix<double,18,18> &M, Matrix<double,24,18> &Jc, Matrix<double,24,1> &Jcdqd, Matrix<double,18,18> &T, Matrix<double,18,18> &T_dot,Matrix<double, 18,1> &q_joints_total, Matrix<double, 18,1> &dq_joints_total, Vector3d &com_pos, Vector3d &com_vel, float &m, float &q_plus, float &q_minus, float &qs, float &qr);
    
        //get function
        vector<double> getTau();

    private:
        Matrix<double,18,12> Jc_T_B;
        Matrix<double,12,18> B_T_Jc;
        Matrix<double,4,12> Sn;
        Matrix<double,3,18> Jt1;
        Matrix<double,3,18> Jt1_T;
        Matrix<double,3,1> Jt1_Tdot_dq;
        Matrix<double,12,18> S;
        Matrix<double,18,12> S_T;
        Matrix<double,24,12> B;
        Matrix<double, 6, 1> q_base;
        double tnp,tnn, tnr, tns;
        
        int CT[34];
        Matrix3d kp = 25*MatrixXd::Identity(3,3);
        Matrix3d kd = 10*MatrixXd::Identity(3,3); //2*sqrt(kp);
        double dt=0.001;
	    double Dt=10*dt;
        double com_zdes = 0.4;//posizione desiderata centro di massa
        float w;//eigenfrequency
        vector<double> tau;
};