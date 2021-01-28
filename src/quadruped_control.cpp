#include "quadruped_control.h"


QUADRUPEDController::QUADRUPEDController()
{

}

QUADRUPEDController::QUADRUPEDController(QUADRUPED &quadruped_)

{
    dogbot = &quadruped_;
  
}


// Control (modificato)
Eigen::VectorXd QUADRUPEDController::Cntr(Eigen::Matrix<double,6,1> &CoMPosDes,
                            Eigen::Matrix<double,6,1> &CoMVelDes,
                            Eigen::Matrix<double,6,1> &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            float &m, float &q_plus, float &q_minus, float &qs, float &qr
                            )
{

// Compute deltax, deltav (modificato)

  Eigen::Matrix<double,6,1> deltax= CoMPosDes-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= CoMVelDes-dogbot->getCOMvel();
std::cout<<"c"<<std::endl;

// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();
std::cout<<"c001"<<std::endl;
  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  std::cout<<"c002"<<std::endl;
  g_acc(2,0)=9.81;
  std::cout<<"c003"<<std::endl;
  Eigen::Matrix<double,18,1> deltag=dogbot->getBiasMatrixCOM();
  std::cout<<"c01"<<std::endl;
  const int n=dogbot->getDoFsnumber();
  Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();
std::cout<<"c02"<<std::endl;
  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*CoMAccDes;
std::cout<<"c03"<<std::endl;
// Control vector
  Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);

// Solve quadratic problem
std::cout<<"c1"<<std::endl;
  tau=dogbot->qpproblem(Wcom_des,m, q_plus, q_minus, qs, qr);
  std::cout<<"c2"<<std::endl;
  return tau;


}
