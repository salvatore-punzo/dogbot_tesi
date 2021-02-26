#include "quadruped_step.h"

using namespace Eigen;

QUADRUPEDStep::QUADRUPEDStep(){

}

QUADRUPEDStep::QUADRUPEDStep(QUADRUPED &quadruped_)
{
    dogbot = &quadruped_;
}

VectorXd QUADRUPEDStep::step(Eigen::Matrix<double,6,1> &CoMPosDes,
                 Eigen::Matrix<double,6,1> &CoMVelDes,
                 Eigen::Matrix<double,6,1> &CoMAccDes,
                 Eigen::MatrixXd &Kcom,
                 Eigen::MatrixXd &Dcom,
                 Eigen::Matrix<double,6,1> footaccdes)
{

  // Compute deltax, deltav (modificato)

    Eigen::Matrix<double,6,1> deltax= CoMPosDes-dogbot->getCOMpos();
    Eigen::Matrix<double,6,1> deltav= CoMVelDes-dogbot->getCOMvel();


  // Compute desired CoM Wrench
    double mass_robot=dogbot->getMass();

    Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
    
    g_acc(2,0)=9.81;
    
    Eigen::Matrix<double,18,1> deltag=dogbot->getBiasMatrixCOM();
    
    const int n=dogbot->getDoFsnumber();
    Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();

    Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*CoMAccDes;

  // Control vector
    Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);

  //solve quadratic problem

  tau=dogbot->qpproblembr( Wcom_des, footaccdes,  QUADRUPED::SWING_LEGS::L2, Eigen::Matrix<double,12,1>::Zero());
  return tau;
}