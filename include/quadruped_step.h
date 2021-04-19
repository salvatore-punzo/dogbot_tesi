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

         Eigen::VectorXd Cntr(iDynTree::Vector6 &CoMPosDes,
                 iDynTree::Vector6 &CoMVelDes,
                 iDynTree::Vector6 &CoMAccDes,
                 Eigen::MatrixXd &Kcom,
                 Eigen::MatrixXd &Dcom,
                 Eigen::Matrix<double,12,1> &fext);

        VectorXd step(Eigen::Matrix<double,6,1> &CoMPosDes,
                 Eigen::Matrix<double,6,1> &CoMVelDes,
                 Eigen::Matrix<double,6,1> &CoMAccDes,
                 Eigen::MatrixXd &Kcom,
                 Eigen::MatrixXd &Dcom,
                 Eigen::Matrix<double,6,1> footaccdes);

        Eigen::VectorXd CntrBr(iDynTree::Vector6 &CoMPosDes,
                 iDynTree::Vector6 &CoMVelDes,
                 iDynTree::Vector6 &CoMAccDes,
                 Eigen::MatrixXd &Kcom,
                 Eigen::MatrixXd &Dcom, Eigen::VectorXd vdotswdes, QUADRUPED::SWING_LEGS swinglegs, Eigen::Matrix<double,12,1> fext);       

        Eigen::VectorXd CntrOl(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::VectorXd vdotswdes,
                            QUADRUPED::SWING_LEG swingleg, Eigen::Matrix<double,12,1> fext
                            );
    private:
        QUADRUPED* dogbot;

};