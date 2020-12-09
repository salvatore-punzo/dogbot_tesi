#include "quadruped.h"
#include "alglib/optimization.h"

QUADRUPED::QUADRUPED()
{
};

QUADRUPED::QUADRUPED(std::string modelFile)
{   //std::string modelFile="/home/viviana/catkin_ws/src/DogBotV4/ROS/src/dogbot_description/urdf/dogbot.urdf";
	
	// Create the robot 
	createrobot(modelFile);
    model = kinDynComp.model();
	
	// Resize matrices of the class given the number of DOFs
    n=model.getNrOfDOFs();
    robot_mass=model.getTotalMass();
    jointPos=iDynTree::VectorDynSize(n);
    baseVel=iDynTree::Twist();
    jointVel=iDynTree::VectorDynSize(n);
	q=iDynTree::VectorDynSize(6+n);
	dq=iDynTree::VectorDynSize(6+n);
	qmin= iDynTree::VectorDynSize(n);
	qmax= iDynTree::VectorDynSize(n);
	Bias=iDynTree::VectorDynSize(n+6);
	GravMatrix=iDynTree::MatrixDynSize(n+6,1);
    MassMatrix=iDynTree::FreeFloatingMassMatrix(model) ;
    Jcom=iDynTree::MatrixDynSize(3,6+n);
	Jac=iDynTree::MatrixDynSize(24,6+n);	
	Jdqd=iDynTree::MatrixDynSize(24,1);
    T=iDynTree::MatrixDynSize(6+n,6+n);
	T_inv_dot=iDynTree::MatrixDynSize(6+n,6+n);
    MassMatrixCOM=iDynTree::FreeFloatingMassMatrix(model) ;
    BiasCOM=iDynTree::VectorDynSize(n+6);
	GravMatrixCOM=iDynTree::MatrixDynSize(n+6,1);
	JacCOM=iDynTree::MatrixDynSize(24,6+n);
	JacCOM_lin=iDynTree::MatrixDynSize(12,6+n);
	JdqdCOM=iDynTree::MatrixDynSize(24,1);
	JdqdCOM_lin=iDynTree::MatrixDynSize(12,1);
}



//Update elements of the class given the new state

void QUADRUPED::update (Eigen::Matrix4d &eigenWorld_H_base, Eigen::VectorXd &eigenJointPos, Eigen::VectorXd &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity)
{   

   // Update joints, base and gravity from inputs
	
	 iDynTree::fromEigen(world_H_base,eigenWorld_H_base);
     iDynTree::toEigen(jointPos) = eigenJointPos;
     iDynTree::fromEigen(baseVel,eigenBasevel);
     toEigen(jointVel) = eigenJointVel;
     toEigen(gravity)  = eigenGravity;

	
	//Set the state for the robot 
	
	 kinDynComp.setRobotState(world_H_base,jointPos,
                              baseVel,jointVel,gravity);
  
    // Compute Center of Mass
     iDynTree::Vector3 base_angle=world_H_base.getRotation().asRPY();
     toEigen(CoM)<<toEigen(kinDynComp.getCenterOfMassPosition()),
	               toEigen(base_angle);
				   
	//Compute velocity of the center of mass
    Eigen::Matrix<double,3,1> CoM_vel_lin=toEigen(kinDynComp.getCenterOfMassVelocity());
	Eigen::Matrix<double,3,1> CoM_vel_ang=eigenBasevel.block(3,0,3,1);

	toEigen(CoM_vel)<<CoM_vel_lin,
	                  CoM_vel_ang;

    // Compute position
	toEigen(q)<<toEigen(CoM),
	            eigenJointPos;
	// Compute velocity 

	toEigen(dq)<<toEigen(CoM_vel),
	             eigenJointVel;

	// Joint limits

    toEigen(qmin)<<-1.75 , -1.75,-1.75,-1.75,-3.15, -0.02, -1.58, -2.62,  -1.58, -2.62, -3.15, -0.02;
    toEigen(qmax)<<1.75, 1.75, 1.75, 1.75, 1.58, 2.62, 3.15, 0.02,  3.15, 0.02, 1.58, 2.62;
	




	// Get mass, bias (C(q,v)*v+g(q)) and gravity (g(q)) matrices
        //Initialize ausiliary vector
	
	     iDynTree::FreeFloatingGeneralizedTorques bias_force(model);
	     iDynTree::FreeFloatingGeneralizedTorques grav_force(model);
	
	  //Compute Mass Matrix
	
	     kinDynComp.getFreeFloatingMassMatrix(MassMatrix); 
	
	  //Compute Coriolis + gravitational terms (Bias)
	
	     kinDynComp.generalizedBiasForces(bias_force);
	    
	     toEigen(Bias)<<iDynTree::toEigen(bias_force.baseWrench()),
	                   iDynTree::toEigen(bias_force.jointTorques());

	  //Compute Gravitational term
	
	     kinDynComp.generalizedGravityForces(grav_force);

	     toEigen(GravMatrix)<<iDynTree::toEigen(grav_force.baseWrench()),
	                        iDynTree::toEigen(grav_force.jointTorques());

	  //Compute Jacobian term
	
	     computeJac();
	
	  // Compute Bias Acceleration -> J_dot*q_dot
	
	     computeJacDotQDot();
	
	  // Velocity vector (base+joints)
	     Eigen::Matrix<double, 18,1> q_dot;
	               
         q_dot<< eigenBasevel,
	             eigenJointVel;
	
	 
	  // Compute Matrix needed for transformation from floating base representation to CoM representation
	
	     computeTransformation(q_dot);
	    
	   Eigen::Matrix<double,18,1> dqtrans=toEigen(T)*q_dot;
	   /*
       std::cout<<"q"<<q_dot<<std::endl;
       std::cout<<"q"<<toEigen(dq)<<std::endl;
	   std::cout<<"qtrans"<<dqtrans<<std::endl;
	   */
	  // Compute Mass Matrix in CoM representation
	 
	     toEigen(MassMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T).inverse();


	  // Compute Coriolis+gravitational term in CoM representation
	
	     toEigen(BiasCOM)=toEigen(T).transpose().inverse()*toEigen(Bias)+toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T_inv_dot)*q_dot;

	  // Compute gravitational term in CoM representation
	
	     toEigen(GravMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(GravMatrix);
	
	  // Compute Jacobian term in CoM representation
	  
	     toEigen(JacCOM)=toEigen(Jac)*toEigen(T).inverse();
	
	  // Compute Bias Acceleration -> J_dot*q_dot  in CoM representation
	
	     toEigen(JdqdCOM)=toEigen(Jdqd)+toEigen(Jac)*toEigen(T_inv_dot)*q_dot;
	  
	
      
  
	
}

void QUADRUPED::createrobot(std::string modelFile)
{  bool ok = mdlLoader.loadModelFromFile(modelFile);

    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return ;
    }
   
    // Create a KinDynComputations class from the model
    
    ok = kinDynComp.loadRobotModel(mdlLoader.model());

    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return ;
    }
    
  

}


// Compute Jacobian
void  QUADRUPED::computeJac()
{    
    //Set ausiliary matrices
	  iDynTree::MatrixDynSize Jac1(6,6+n);
	  iDynTree::MatrixDynSize Jac2(6,6+n);
	  iDynTree::MatrixDynSize Jac3(6,6+n);
	  iDynTree::MatrixDynSize Jac4(6,6+n);
	
	// Compute Jacobian for each leg
	
	   // Jacobian for back right leg
       kinDynComp.getFrameFreeFloatingJacobian(8,Jac1);
	
	   // Jacobian for back left leg
	   kinDynComp.getFrameFreeFloatingJacobian(11,Jac2);
	
	  // Jacobian for front left leg
	  kinDynComp.getFrameFreeFloatingJacobian(14,Jac3);
	
	  // Jacobian for front right leg
	  kinDynComp.getFrameFreeFloatingJacobian(17,Jac4);
	
	 // Full Jacobian
	 toEigen(Jac)<<toEigen(Jac1),
	               toEigen(Jac2),
	               toEigen(Jac3),
	               toEigen(Jac4);
	
}



// Compute Bias acceleration: J_dot*q_dot
void  QUADRUPED::computeJacDotQDot()
	
{
	   
	  // Bias acceleration for back right leg
	  iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc(8); 
	  
	  // Bias acceleration for back left leg
	  iDynTree::Vector6 Jdqd2=kinDynComp.getFrameBiasAcc(11); 
	  
	  // Bias acceleration for front left leg
	  iDynTree::Vector6 Jdqd3=kinDynComp.getFrameBiasAcc(14); 
	 
	  // Bias acceleration for front right leg
	  iDynTree::Vector6 Jdqd4=kinDynComp.getFrameBiasAcc(17); 
	
	
	  // Compute Bias acceleration for all the legs (backright-backleft-frontleft-frontright)
	
      toEigen(Jdqd)<<toEigen(Jdqd1),
                     toEigen(Jdqd2),
	                 toEigen(Jdqd3),
                     toEigen(Jdqd4);
	
	
}



//Method to get directly Jacobian time derivative, slower

void  QUADRUPED::computeJacDot(Eigen::VectorXd Vel_)
{    
	
 
	 // set ausiliary matrices
	iDynTree::MatrixDynSize Jac;
	Jac=iDynTree::MatrixDynSize(6,6+n);
		
	Eigen::MatrixXd Jac_;
	Jac_=Eigen::MatrixXd::Zero(6,18);
	
	Eigen::VectorXd jac_dot_k_=Eigen::VectorXd::Zero(6);
	Eigen::MatrixXd jac_=Eigen::MatrixXd::Zero(6,18);
	Eigen::MatrixXd Jacdot=Eigen::MatrixXd::Zero(24,18);
	
	// Compute derivative for each leg
	int k=0;
	for (unsigned int k=0; k<4; ++k)
	
	{   
	// Getting Jacobian of the leg
		kinDynComp.getFrameFreeFloatingJacobian(8+3*k,Jac);
        Jac_= iDynTree::toEigen(Jac);
		
    for(unsigned int i=0;i<6+n;++i)
    {  
     // Getting column i derivative
		
            for(unsigned int j=0;j<6+n;++j)
            {
                // Column J is the sum of all partial derivatives  ref (41)
                    jac_dot_k_ += getPartialDerivativeJac(Jac_,j,i)*Vel_(j) ;
				//*
            }
		
        jac_.col(i)= jac_dot_k_ ;
        jac_dot_k_=Eigen::VectorXd::Zero(6);
		
        }
	 	
	 Jac_=Eigen::MatrixXd::Zero(6,18);
	 Jacdot.block(0+6*k,0,6,18)<<jac_;
	 jac_=Eigen::MatrixXd::Zero(6,18);
	

} 
	//Eigen::VectorXd jd=Eigen::VectorXd::Zero(6);
	
	
	
}

// Compute partial time derivative
Eigen::VectorXd QUADRUPED::getPartialDerivativeJac(const Eigen::MatrixXd Jacobian, const unsigned int& joint_idx,  const unsigned int& column_idx)
{   // column's indices
    int j=joint_idx;
    int i=column_idx;
	
	// get columns
	Eigen::VectorXd jac_j_ = Jacobian.col(j);
    Eigen::VectorXd jac_i_ = Jacobian.col(i);
	
	// Get linear and rotational parts
	Eigen::Vector3d jac_j_vel = jac_j_.head(3);
    Eigen::Vector3d jac_i_vel = jac_i_.head(3);
	
	Eigen::Vector3d jac_j_rot = jac_j_.tail(3);
    Eigen::Vector3d jac_i_rot = jac_i_.tail(3);
	
	// Ausiliary vector
	Eigen::Vector3d t_djdq_vel=Eigen::Vector3d::Zero();
	Eigen::Vector3d t_djdq_rot=Eigen::Vector3d::Zero();
	
	if(j < i)
    {
        // P_{\Delta}({}_{bs}J^{j})  ref (20)
        t_djdq_vel = jac_j_rot.cross(jac_i_vel);
        t_djdq_rot = jac_j_rot.cross(jac_i_rot);
		
    }else if(j > i)
    {
        // M_{\Delta}({}_{bs}J^{j})  ref (23)
        
        t_djdq_vel = -jac_j_vel.cross(jac_i_rot);
		
    }else if(j == i)
    {
         // ref (40)
         
         t_djdq_vel = jac_i_rot.cross(jac_i_vel);
		
    }
	
	
	Eigen::VectorXd t_djdq_=Eigen::VectorXd::Zero(6);
	t_djdq_<<t_djdq_vel,
	         t_djdq_rot;
	
    return t_djdq_;
	
	

}



// Compute matrix transformation T needed to recompute matrices/vector after the coordinate transform to the CoM
void QUADRUPED::computeTransformation(Eigen::VectorXd Vel_)
{   
	//Set ausiliary matrices
	iDynTree::MatrixDynSize Jb(6,6+n);
	iDynTree::MatrixDynSize Jbc(3,n);
	iDynTree::Vector3 xbc;
	iDynTree::MatrixDynSize xbc_hat(3,3);
	iDynTree::MatrixDynSize xbc_hat_dot(3,3);
	iDynTree::MatrixDynSize Jbc_dot(6,6+n);
	iDynTree::Vector3 xbc_dot;
	
	// Compute T matrix
	    // Get jacobians of the floating base and of the com
	    kinDynComp.getFrameFreeFloatingJacobian(0,Jb);
	    kinDynComp.getCenterOfMassJacobian(Jcom);
	
	    // Compute jacobian Jbc=d(xc-xb)/dq used in matrix T
	    toEigen(Jbc)<<toEigen(Jcom).block<3,12>(0,6)-toEigen(Jb).block<3,12>(0,6);
	
	    // Get xb (floating base position) and xc ( com position)
	    iDynTree::Position xb = world_H_base.getPosition();
	    iDynTree::Position xc= kinDynComp.getCenterOfMassPosition();
	
	    // Vector xcb=xc-xb
	    toEigen(xbc)=toEigen(xc)-toEigen(xb);
	
	    // Skew of xcb
	    toEigen(xbc_hat)<<0, -toEigen(xbc)[2], toEigen(xbc)[1],
	                      toEigen(xbc)[2], 0, -toEigen(xbc)[0],                          
	                     -toEigen(xbc)[1], toEigen(xbc)[0], 0;
	
	
     	// Matrix T for the transformation
	    toEigen(T)<<Eigen::MatrixXd::Identity(3,3), -toEigen(xbc_hat), toEigen(Jbc),
	                Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,12),
	                Eigen::MatrixXd::Zero(12,3),  Eigen::MatrixXd::Zero(12,3), Eigen::MatrixXd::Identity(12,12);

	
	
	
	//Compute time derivative of T 
       // Compute derivative of xbc
	   toEigen(xbc_dot)=toEigen(kinDynComp.getCenterOfMassVelocity())-toEigen(baseVel.getLinearVec3());
	
	   //Compute skew of xbc
   	   toEigen(xbc_hat_dot)<<0, -toEigen(xbc_dot)[2], toEigen(xbc_dot)[1],
	                      toEigen(xbc_dot)[2], 0, -toEigen(xbc_dot)[0],                          
	                      -toEigen(xbc_dot)[1], toEigen(xbc_dot)[0], 0;

	
	   // Time derivative of Jbc
	   kinDynComp.getCentroidalAverageVelocityJacobian(Jbc_dot);
	
	  Eigen::Matrix<double,6,18> Jbcdot;
	  Eigen::Matrix<double,6,18> jac_;
	  Eigen::VectorXd jac_dot_k_=Eigen::VectorXd::Zero(6);
	
	   for(unsigned int i=0;i<6+n;++i)
         {  
            // Getting column i derivative
		
            for(unsigned int j=0;j<6+n;++j)
            {
                // Column J is the sum of all partial derivatives  ref (41)
                    jac_dot_k_ += getPartialDerivativeJac(toEigen(Jbc_dot),j,i)*Vel_(j) ;
				//*
            }
		
        jac_.col(i)= jac_dot_k_ ;
        jac_dot_k_=Eigen::VectorXd::Zero(6);
		
        }
	   
	    Jbcdot.block(0,0,6,18)<<jac_;

	   // Tdot matrix
	   toEigen(T_inv_dot)<<Eigen::MatrixXd::Zero(3,3), -toEigen(xbc_hat_dot), -Jbcdot.block<3,12>(0,6),
                      Eigen::MatrixXd::Zero(15,18);
	
	
}


// return Mass Matrix
Eigen::MatrixXd QUADRUPED::getMassMatrix()
{
	Eigen::MatrixXd massMatrix = iDynTree::toEigen(MassMatrix);
	return massMatrix;

}


//return Bias Matrix
Eigen::VectorXd QUADRUPED::getBiasMatrix()
{
	Eigen::VectorXd coriolis = iDynTree::toEigen(Bias);
	return coriolis;

}

//return gravity term
Eigen::VectorXd QUADRUPED::getGravityMatrix()
{
	Eigen::VectorXd grav_term = iDynTree::toEigen(GravMatrix);
	return grav_term;

}


//return Jacobian
Eigen::MatrixXd QUADRUPED::getJacobian()
{
	Eigen::Matrix<double,24,18> Jacobian = iDynTree::toEigen(Jac);
	return Jacobian;

}


//return Bias Acceleration --> J_dot*q_dot
Eigen::MatrixXd QUADRUPED::getBiasAcc()
{
	Eigen::Matrix<double,24,1> BiasAcc = iDynTree::toEigen(Jdqd);
	return BiasAcc;

}

//return matrix T
Eigen::MatrixXd QUADRUPED::getTransMatrix()
{
	Eigen::Matrix<double,18,18> trans = iDynTree::toEigen(T);
	return trans;

}
//return matrix T_dot 
Eigen::MatrixXd QUADRUPED::getTdotMatrix()
{
	Eigen::Matrix<double,18,18> t_dot = iDynTree::toEigen(T_inv_dot);
	return t_dot;

}

// return Mass Matrix in COM representation
Eigen::MatrixXd QUADRUPED::getMassMatrixCOM()
{
	Eigen::MatrixXd massMatrix = iDynTree::toEigen(MassMatrixCOM);
	return massMatrix;

}


//return Bias Matrix in COM representation
Eigen::VectorXd QUADRUPED::getBiasMatrixCOM()
{
	Eigen::VectorXd coriolis = iDynTree::toEigen(BiasCOM);
	return coriolis;

}

//return gravity term in COM representation
Eigen::VectorXd QUADRUPED::getGravityMatrixCOM()
{
	Eigen::VectorXd grav_term = iDynTree::toEigen(GravMatrixCOM);
	return grav_term;

}


//return Jacobian in COM representation
Eigen::MatrixXd QUADRUPED::getJacobianCOM()
{
	Eigen::Matrix<double,24,18> Jacobian = iDynTree::toEigen(JacCOM);
	return Jacobian;

}

Eigen::MatrixXd QUADRUPED::getJacobianCOM_linear()
{

	for (int i=0; i<4; ++i)
	{
       toEigen(JacCOM_lin).block(0+3*i,0,3,18)=toEigen(JacCOM).block(0+6*i,0,3,18);
	}
    
	Eigen::Matrix<double,12,18> Jacobian_lin = iDynTree::toEigen(JacCOM_lin);
	return Jacobian_lin;

}


//return Bias Acceleration --> J_dot*q_dot in COM representation
Eigen::MatrixXd QUADRUPED::getBiasAccCOM()
{
	Eigen::Matrix<double,24,1> BiasAcc = iDynTree::toEigen(JdqdCOM);
	return BiasAcc;

}


Eigen::MatrixXd QUADRUPED::getBiasAccCOM_linear()
{

	for (int i=0; i<4; ++i)
	{
       toEigen(JdqdCOM_lin).block(0+3*i,0,3,1)=toEigen(JdqdCOM).block(0+6*i,0,3,1);
	}
    
	Eigen::Matrix<double,12,1> Jacobian_lin = iDynTree::toEigen(JdqdCOM_lin);
	return Jacobian_lin;

}


Eigen::MatrixXd QUADRUPED::getCOMpos()
{
 Eigen::Matrix<double,6,1> COMpos= iDynTree::toEigen(CoM);
	
 return COMpos;
}
  
  
Eigen::MatrixXd QUADRUPED::getCOMvel()
{
 Eigen::Matrix<double,6,1> COMvel= iDynTree::toEigen(CoM_vel);
 return COMvel;

}


double QUADRUPED::getMass()
{
	return robot_mass;
}


int QUADRUPED::getDoFsnumber()
{
	return n;
}


Eigen::MatrixXd QUADRUPED::getMassMatrixCOM_com()
{
	Eigen::MatrixXd massMatrix = iDynTree::toEigen(MassMatrixCOM).block(0,0,6,6);
	return massMatrix;
   
}


Eigen::MatrixXd QUADRUPED::getMassMatrixCOM_joints()
{
    Eigen::MatrixXd massMatrix = iDynTree::toEigen(MassMatrixCOM).block(6,6,n,n);
	return massMatrix;
}




