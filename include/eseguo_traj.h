#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
// Eigen headers 
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
//
#include"quadruped.h"
#include "traj_planner.h"
#include "prob_quadratico.h"

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

//gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>


using namespace Eigen;
using namespace std;

class ESEGUOtraj
{
    

    public:

        ESEGUOtraj();
        ESEGUOtraj(QUADRUPED &quadruped_);
        ESEGUOtraj (PROB_QUAD);
       
        
        vector<double> eseguo_traj(int idx3,
        Matrix<double, 18,1> q_joints_total, Matrix<double, 18,1> dq_joints_total, trajectory_point traj_com3);
    
    private:
        
        QUADRUPED*  doggo;
        PROB_QUAD* ottim;

       
	   

	    
};