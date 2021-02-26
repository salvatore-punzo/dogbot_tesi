#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <stdlib.h>

#include <math.h>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include "quadruped.h"

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

using namespace Eigen;
using namespace std;

class QUADRUPEDStep
{
    public:
        QUADRUPEDStep();
        QUADRUPEDStep(QUADRUPED &quadruped_);

        VectorXd step(Eigen::Matrix<double,6,1> &CoMPosDes,
                 Eigen::Matrix<double,6,1> &CoMVelDes,
                 Eigen::Matrix<double,6,1> &CoMAccDes,
                 Eigen::MatrixXd &Kcom,
                 Eigen::MatrixXd &Dcom,
                 Eigen::Matrix<double,6,1> footaccdes);

    private:
        QUADRUPED* dogbot;

};