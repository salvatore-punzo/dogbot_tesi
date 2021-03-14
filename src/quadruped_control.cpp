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
                            float &m_blfl, float &m_flfr, float &m_frbr, float &m_brbl, float &q_blfl, float &q_flfr, float &q_frbr, float &q_brbl,
                            float &x_inf, float &x_sup, float&y_inf, float &y_sup)
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

// Solve quadratic problem

  tau=dogbot->qpproblem(Wcom_des);
  
  //tau=dogbot->qpproblem_cp(Wcom_des,m_blfl, m_flfr, m_frbr, m_brbl, q_blfl, q_flfr, q_frbr, q_brbl,x_inf, x_sup, y_inf, y_sup);
  
  return tau;


}

// Control (modificato)
Eigen::VectorXd QUADRUPEDController::Cntr1(Eigen::Matrix<double,6,1> &CoMPosDes,
                            Eigen::Matrix<double,6,1> &CoMVelDes,
                            Eigen::Matrix<double,6,1> &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom
                            )
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

// Solve quadratic problem

  tau=dogbot->qpproblem(Wcom_des);
  
  //tau=dogbot->qpproblem_cp(Wcom_des,m_blfl, m_flfr, m_frbr, m_brbl, q_blfl, q_flfr, q_frbr, q_brbl,x_inf, x_sup, y_inf, y_sup);
  
  return tau;


}
