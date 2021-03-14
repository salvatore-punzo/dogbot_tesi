#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
// Eigen headers 
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include"quadruped.h"

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

using namespace Eigen;
using namespace std;

class ESEGUOtraj
{

    public:

        ESEGUOtraj();
        ESEGUOtraj(QUADRUPED &quadruped_);
        
        vector<double> eseguo_traj(ros::Time begin3, double tf3, double t3);
    
    private:
        
        QUADRUPED*  doggo;

};