#ifndef QUADRUPEDCONTROL
#define QUADRUPEDCONTROL

#include <cstdlib>
#include <iostream>
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

class QUADRUPEDController
{

 public:

     QUADRUPEDController();
   
     QUADRUPEDController(QUADRUPED &quadruped_);
     
     
     Eigen::VectorXd Cntr(Eigen::Matrix<double,6,1> &CoMPosDes,
                 Eigen::Matrix<double,6,1> &CoMVelDes,
                 Eigen::Matrix<double,6,1> &CoMAccDes,
                 Eigen::MatrixXd &Kcom,
                 Eigen::MatrixXd &Dcom,
                 float &m, float &q_plus, float &q_minus, float &qs, float &qr);



 private:

     QUADRUPED*  dogbot;

};

#endif