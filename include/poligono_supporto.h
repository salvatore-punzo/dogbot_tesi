
#ifndef POLIGONOSUPPORTO
#define POLIGONOSUPPORTO

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


class POLI_SUP{

    public:
        POLI_SUP();
        void calcoloPoligonoSupporto(VectorXd &ll, VectorXd &hp, VectorXd &he, VectorXd &kp, VectorXd &rp, Vector3d &eef_bl, Vector3d &eef_br,Vector3d &eef_fl, Vector3d &eef_fr, Vector3d &coo_ee_bl, Vector3d &coo_ee_br, Vector3d &coo_ee_fl, Vector3d &coo_ee_fr, Matrix3d &rot_world_virtual_base);
      //get function
        float getm();
        float getq_newp();
        float getq_newn();
        float getqr();
        float getqs();
    private:
        Matrix3d rot_roll_br, rot_pitch_br, rot_knee_br, rot_roll_bl, rot_pitch_bl, rot_knee_bl;
        Vector3d sele_z;
        Vector3d eef_bl_wc; 
        Vector3d eef_br_wc;
        float cop_x;
		float cop_y;

        float _len_leg_blfr;
        float _hip_j_blfr;
        float _hip_eff_blfr;
        Vector3d _ee_f_vb_rl;

        float _len_leg_brfl;
        float _hip_j_brfl;
        float _hip_eff_brfl;
        Vector3d _ee_f_vb_ll;

        float xc_vb_rl;
        float yc_vb_rl;
        float xc_vb_ll;
        float yc_vb_ll;

        float eef_vb_rlz;
        float eef_vb_llz;

        float m;
		float q;
		float q_newp;
		float q_newn;
		float m_pep;
		float qr;
		float qs;
		float xa,ya,xb,yb,xc,yc,xd,yd;


};


#endif