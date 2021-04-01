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
  std::cout<<"CoMAccDes: "<<endl<<CoMAccDes<<endl;
  //tau=dogbot->qpproblembr( Wcom_des, footaccdes,  QUADRUPED::SWING_LEGS::L2, Eigen::Matrix<double,12,1>::Zero());
  Eigen::Vector3d vdotaccdes;
  vdotaccdes<<footaccdes(0), footaccdes(1),footaccdes(2);
  std::cout<<"vdotaccdes"<<endl<<vdotaccdes<<endl;
  std::cout<<"footaccdes"<<endl<<footaccdes<<endl;
  tau=dogbot->qpproblemol(Wcom_des,vdotaccdes, QUADRUPED::SWING_LEG::BR);
  return tau;
}

Eigen::VectorXd QUADRUPEDStep::CntrBr(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::VectorXd vdotswdes,
                            QUADRUPED::SWING_LEGS swinglegs,
                            Eigen::Matrix<double,12,1> fext)
{
  int swl1, swl2, stl1, stl2;
    switch(swinglegs){
		case QUADRUPED::L1: swl1=0;
		swl2=0 ; 
		stl1=0;
		stl2=0 ; 
		break;
		case QUADRUPED::L2: swl1=0;
		swl2=6 ;
		stl1=3;
		stl2=9 ; 
		 break;
		case QUADRUPED::L3: swl1=3;
		swl2=9;
		stl1=0;
		stl2=6; 
		 break;
	}

// Compute deltax, deltav

  Eigen::Matrix<double,6,1> deltax= toEigen(CoMPosDes)-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= toEigen(CoMVelDes)-dogbot->getCOMvel();


// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();

  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  g_acc(2,0)=9.81;
  Eigen::Matrix<double,18,1> deltag=dogbot->getBiasMatrixCOM();
  
  const int n=dogbot->getDoFsnumber();
  Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();
   Eigen::Matrix<double,12,18> J=dogbot->getJacobianCOM_linear();
  
   Eigen::Matrix<double,6,6> Jcom;
   Jcom.block(0,0,3,6)=J.block(stl1,0,3,6);
   Jcom.block(3,0,3,6)=J.block(stl2,0,3,6);
   Eigen::Matrix<double,6,1> fext_st;
   fext_st.block(0,0,3,1)=fext.block(stl1,0,3,1);
   fext_st.block(3,0,3,1)=fext.block(stl2,0,3,1);
    Eigen::Matrix<double,6,1> fext_sw;
   fext_sw.block(0,0,3,1)=fext.block(swl1,0,3,1);
   fext_sw.block(3,0,3,1)=fext.block(swl2,0,3,1);
  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*toEigen(CoMAccDes);
 std::cout<<"Wcomdes"<<Wcom_des<<std::endl;
// Control vector
  Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);

// Solve quadratic problem
Eigen::Matrix<double,12,1> fext1= Eigen::Matrix<double,12,1>::Zero();
  tau=dogbot->qpproblembr(Wcom_des, vdotswdes, swinglegs, fext);
  
  return tau;


}

Eigen::VectorXd QUADRUPEDStep::CntrOl(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::VectorXd vdotswdes,
                            QUADRUPED::SWING_LEG swingleg,
                            Eigen::Matrix<double,12,1> fext)
{
  int swl1, stl1, stl2, stl3;
    switch(swingleg){
		case QUADRUPED::BR: swl1=0; 
		stl1=3;
		stl2=6 ; 
		stl3=9;
		break;
		case QUADRUPED::FL: swl1=6;
		stl1=0;
		stl2=3 ; 
		stl3=9;
		break;
		case QUADRUPED::BL: swl1=3;
		stl1=0;
		stl2=6; 
		stl3=9;
		break;
		case QUADRUPED::FR: swl1=9;
		stl1=0;
		stl2=3; 
		stl3=6;
		 break;
	}

// Compute deltax, deltav

  Eigen::Matrix<double,6,1> deltax= toEigen(CoMPosDes)-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= toEigen(CoMVelDes)-dogbot->getCOMvel();


// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();

  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  g_acc(2,0)=9.81;
  Eigen::Matrix<double,18,1> deltag=dogbot->getBiasMatrixCOM();
  
  const int n=dogbot->getDoFsnumber();
  Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();
  
  Eigen::Matrix<double,12,18> J=dogbot->getJacobianCOM_linear();
  Eigen::Matrix<double,9,6> Jcom;
   Jcom.block(0,0,3,6)=J.block(stl1,0,3,6);
   Jcom.block(3,0,3,6)=J.block(stl2,0,3,6);
   Jcom.block(6,0,3,6)=J.block(stl3,0,3,6);

  Eigen::Matrix<double,9,1> fext_st;
   fext_st.block(0,0,3,1)=fext.block(stl1,0,3,1);
   fext_st.block(3,0,3,1)=fext.block(stl2,0,3,1);
   fext_st.block(6,0,3,1)=fext.block(stl3,0,3,1);

  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*toEigen(CoMAccDes);
 std::cout<<"Wcomdes"<<Wcom_des<<std::endl;
// Control vector
  Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);

// Solve quadratic problem
  Eigen::Matrix<double,12,1> fext1=Eigen::Matrix<double,12,1>::Zero();
  tau=dogbot->qpproblemol(Wcom_des, vdotswdes, swingleg);
  
  return tau;


}
