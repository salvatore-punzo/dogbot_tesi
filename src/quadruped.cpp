#include "quadruped.h"
#include "alglib/optimization.h"
#include "Td_matrix.cpp"

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
	qb=iDynTree::VectorDynSize(6+n);
	dqb=iDynTree::VectorDynSize(6+n);
	qmin= iDynTree::VectorDynSize(n);
	qmax= iDynTree::VectorDynSize(n);
	Bias=iDynTree::VectorDynSize(n+6);
	GravMatrix=iDynTree::MatrixDynSize(n+6,1);
    MassMatrix=iDynTree::FreeFloatingMassMatrix(model) ;
    Jcom=iDynTree::MatrixDynSize(3,6+n);
	Jb=iDynTree::MatrixDynSize(6,6+n);
	Jcomdot=iDynTree::MatrixDynSize(6,6+n);
	Jac=iDynTree::MatrixDynSize(24,6+n);	
	JacDot=iDynTree::MatrixDynSize(24,6+n);
	Jdqd=iDynTree::MatrixDynSize(24,1);
    T=iDynTree::MatrixDynSize(6+n,6+n);
	T_inv_dot=iDynTree::MatrixDynSize(6+n,6+n);
	T_dot=iDynTree::MatrixDynSize(6+n,6+n);//sp
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
		   
    // Compute position base +joints
	toEigen(qb)<<toEigen(world_H_base.getPosition()),
	            toEigen(base_angle),
	            eigenJointPos;

    // Compute position COM+joints
	toEigen(q)<<toEigen(CoM),
	            eigenJointPos;

	// Compute velocity COM+joints

	toEigen(dq)<<toEigen(CoM_vel),
	             eigenJointVel;

	// Compute velocity base+joints

	toEigen(dqb)<<eigenBasevel,
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
    
	  // Compute Jacobian derivative
	     computeJacDot(q_dot);


	  // Compute Matrix needed for transformation from floating base representation to CoM representation

	     computeTransformation(q_dot);

	  // Compute Mass Matrix in CoM representation 
	     toEigen(MassMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T).inverse();


	  // Compute Coriolis+gravitational term in CoM representation
	
	     toEigen(BiasCOM)=toEigen(T).transpose().inverse()*toEigen(Bias)+toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T_inv_dot)*toEigen(dq);

	  // Compute gravitational term in CoM representation
	
	     toEigen(GravMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(GravMatrix);
	
	  // Compute Jacobian term in CoM representation
	  
	     toEigen(JacCOM)=toEigen(Jac)*toEigen(T).inverse();
	
	     ComputeJaclinear();

	  // Compute Bias Acceleration -> J_dot*q_dot  in CoM representation
	
	     toEigen(JdqdCOM)=toEigen(JacDot)*toEigen(dq)+toEigen(Jac)*toEigen(T_inv_dot)*toEigen(dq);
	  
	     computeJdqdCOMlinear();
	

  
	
}

// create robot
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

void QUADRUPED::ComputeJaclinear()
{
  Eigen::Matrix<double,12,24> B;
  B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

  toEigen(JacCOM_lin)=B*toEigen(JacCOM);

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

void  QUADRUPED::computeJacDot(Eigen::Matrix<double,18,1> Vel_)
{    
	
 
	 // set ausiliary matrices
	iDynTree::MatrixDynSize Jac;
	Jac=iDynTree::MatrixDynSize(6,6+n);
    iDynTree::Twist vel_foot;

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
		vel_foot=kinDynComp.getFrameVel(8+3*k);

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
	 Jacdot.block(0+6*k,6,6,12)<<jac_.block(0,6,6,12);

    Eigen::Matrix<double,6,1> xic_dot=toEigen(vel_foot)-toEigen(baseVel);
    
	Eigen::Matrix<double,3,3> xic_hat_dot;
    xic_hat_dot<<0, -xic_dot[2], xic_dot[1],
	            xic_dot[2], 0, -xic_dot[0],                          
	            -xic_dot[1], xic_dot[0], 0;



     Jacdot.block(0+6*k,0,6,6)<<Eigen::MatrixXd::Zero(3,3), -xic_hat_dot,
	                            Eigen::MatrixXd::Zero(3,6) ;


	 jac_=Eigen::MatrixXd::Zero(6,18);
     
	
	

} 

	toEigen(JacDot)=Jacdot;
	
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


void QUADRUPED::computeJdqdCOMlinear()
{
	Eigen::Matrix<double,12,24> B;
    B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);


    toEigen(JdqdCOM_lin)= Eigen::MatrixXd::Zero(12,1);
    toEigen(JdqdCOM_lin)=B*toEigen(JdqdCOM);
	
}


// Compute matrix transformation T needed to recompute matrices/vector after the coordinate transform to the CoM
void QUADRUPED::computeTransformation(Eigen::VectorXd Vel_)
{   
	//Set ausiliary matrices
	
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
	  
	  Eigen::Matrix<double,6,12> jac_;
	  Eigen::VectorXd jac_dot_k_=Eigen::VectorXd::Zero(6);

	  TDMAT Tj(toEigen(qb),toEigen(dqb));
	  Eigen::Matrix<double,3,12> Jbcdot=Tj.getTdmatrix();

	   // Tdot matrix
	   toEigen(T_inv_dot)<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot), Jbcdot,
                      Eigen::MatrixXd::Zero(15,18);
	
	//Tdot matrix cambiano i segni
	   toEigen(T_dot)<<Eigen::MatrixXd::Zero(3,3), -toEigen(xbc_hat_dot), -Jbcdot,
                      Eigen::MatrixXd::Zero(15,18);
}


// return Mass Matrix
Eigen::MatrixXd QUADRUPED::getMassMatrix()
{
	
	return toEigen(MassMatrix);

}
//return Jcom Matrix
Eigen::MatrixXd QUADRUPED::getJCOMMatrix()
{
	Eigen::Matrix<double,6,18> J_com;
	J_com<< iDynTree::toEigen(Jcom),
	        toEigen(Jb).block(3,0,3,18);
	return J_com;

}

//return Jcom Matrix
Eigen::MatrixXd QUADRUPED::getJCOMDot()
{
	Eigen::MatrixXd J_com = iDynTree::toEigen(Jcomdot);
	return J_com;

}
//return Bias Matrix
Eigen::VectorXd QUADRUPED::getBiasMatrix()
{

	return toEigen(Bias);

}

//return gravity term
Eigen::VectorXd QUADRUPED::getGravityMatrix()
{
	
	return toEigen(GravMatrix);

}


//return Jacobian
Eigen::MatrixXd QUADRUPED::getJacobian()
{
	
	return toEigen(Jac);

}


//return Bias Acceleration --> J_dot*q_dot
Eigen::MatrixXd QUADRUPED::getBiasAcc()
{

	return toEigen(Jdqd);

}

//return matrix T
Eigen::MatrixXd QUADRUPED::getTransMatrix()
{

	return toEigen(T);

}

// return Mass Matrix in COM representation
Eigen::MatrixXd QUADRUPED::getMassMatrixCOM()
{
	
	return toEigen(MassMatrixCOM);

}
//return matrix T_dot 
Eigen::MatrixXd QUADRUPED::getTdotMatrix()
{
	Eigen::Matrix<double,18,18> t_dot = iDynTree::toEigen(T_dot);
	return t_dot;

}

//return Bias Matrix in COM representation
Eigen::VectorXd QUADRUPED::getBiasMatrixCOM()
{
	
	return toEigen(BiasCOM);

}

//return gravity term in COM representation
Eigen::VectorXd QUADRUPED::getGravityMatrixCOM()
{

	return toEigen(GravMatrixCOM);

}


//return Jacobian in COM representation
Eigen::MatrixXd QUADRUPED::getJacobianCOM()
{

	return toEigen(JacCOM);

}

Eigen::MatrixXd QUADRUPED::getJacobianCOM_linear()
{
    

	return toEigen(JacCOM_lin);

}


//return Bias Acceleration --> J_dot*q_dot in COM representation
Eigen::MatrixXd QUADRUPED::getBiasAccCOM()
{
	
	return toEigen(JdqdCOM);

}

// Return Bias accelration in CoM representation
Eigen::MatrixXd QUADRUPED::getBiasAccCOM_linear()
{
	return toEigen(JdqdCOM_lin);

}


Eigen::MatrixXd QUADRUPED::getCOMpos()
{

	
 return toEigen(CoM);
}
  
  
Eigen::MatrixXd QUADRUPED::getCOMvel()
{

 return toEigen(CoM_vel);

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

	return toEigen(MassMatrixCOM).block(0,0,6,6);
   
}


Eigen::MatrixXd QUADRUPED::getMassMatrixCOM_joints()
{
   
	return toEigen(MassMatrixCOM).block(6,6,n,n);

	
}
Eigen::MatrixXd QUADRUPED::getCtq()
{
	
	return Ctq;

}
Eigen::MatrixXd QUADRUPED::getsolution()
{
   
	return x_eigen;
}
//---------------nuove getfunction-----------------------
Eigen::Matrix<double,3,3> QUADRUPED::getBRworldtransform()
{    
	
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(8);
    return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> QUADRUPED::getBLworldtransform()
{   iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(11);

   return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> QUADRUPED::getFLworldtransform()
{   iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(14);

 return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> QUADRUPED::getFRworldtransform()
{   iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(17);

   return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,1> QUADRUPED::getbrlowerleg()
{   iDynTree::Transform  World_br;
    World_br=kinDynComp.getRelativeTransform(7,8);

   return toEigen(World_br.getPosition());
}


Eigen::MatrixXd QUADRUPED::getBRpos()
{
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(8);
	return toEigen(World_br.getPosition());

}

Eigen::MatrixXd QUADRUPED::getBLpos()
{
	iDynTree::Transform  World_bl;
    World_bl=kinDynComp.getWorldTransform(11);
	return toEigen(World_bl.getPosition());

}

Eigen::MatrixXd QUADRUPED::getFLpos()
{
	iDynTree::Transform  World_fl;
    World_fl=kinDynComp.getWorldTransform(14);
	return toEigen(World_fl.getPosition());

}

Eigen::MatrixXd QUADRUPED::getFRpos()
{
	iDynTree::Transform  World_fr;
    World_fr=kinDynComp.getWorldTransform(17);
	return toEigen(World_fr.getPosition());

}

Eigen::MatrixXd QUADRUPED::getBRvel()
{   iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel(8);

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getBLvel()
{
	iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel(11);

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getFLvel()
{
iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel(14);

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getFRvel()
{
	iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel(17);

	return toEigen(br_vel.getLinearVec3() );

}



// Quadratic problem
Eigen::VectorXd QUADRUPED::qpproblem( Eigen::Matrix<double,6,1> &Wcom_des)
{
	Eigen::Matrix<double,6,1> com_pos = iDynTree::toEigen(CoM);
	Eigen::Matrix<double,6,1> com_vel = iDynTree::toEigen(CoM_vel);
	

	// Set variables
   int variables=30;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(58,31);
   Lt.setlength(58);


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 12, 6> Jstcom= toEigen(JacCOM_lin).block(0,0,12,6);
   Eigen::Matrix<double, 12, 12> Jstj= toEigen(JacCOM_lin).block(0,6,12,12);

   // cost function quadratic matrix
   Eigen::Matrix<double,12,30> Sigma= Eigen::Matrix<double,12,30>::Zero();
   Sigma.block(0,18,12,12)= Eigen::Matrix<double,12,12>::Identity();
   
   Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;

   Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,30,30> eigenQ= eigenQ2+Eigen::Matrix<double,30,30>::Identity();
 

	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	Eigen::Matrix<double,30,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*Wcom_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }




    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(30,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	

	//Equality constraints
	Eigen::Matrix<double,18, 30> eigenA= Eigen::Matrix<double,18,30>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,18,6,12)=-Jstcom.transpose();

    eigenA.block(6,0,12,6)=Jstcom;

    eigenA.block(6,6,12,12)=Jstj;

    // Known term
    Eigen::Matrix<double,18, 1> eigenb= Eigen::Matrix<double,18,1>::Zero();

    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;
	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);;

	eigenb.block(6,0,12,1)=-toEigen(JdqdCOM_lin);


    
    

	//Inequality Constraints

	Eigen::Matrix<double,40, 30> eigenD= Eigen::Matrix<double,40,30>::Zero();
	
	 // Torque limits
	eigenD.block(16,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(16,18,12,12)=-Jstj.transpose();

	eigenD.block(28,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(28,18,12,12)=Jstj.transpose();

	

	//Friction
	   double mu=1;
	   Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
	   n<< 0,
	       0,
		   1;

	   Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
	   t1<< 1,
	       0,
		   0;

       Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
	   t2<<0,
	       1,
		   0;

	   Eigen::Matrix<double,4,3> cfr=Eigen::Matrix<double,4,3>::Zero();
  
	   cfr<<(-mu*n+t1).transpose(),
	        (-mu*n+t2).transpose(),
			-(mu*n+t1).transpose(),
			-(mu*n+t2).transpose();
     
	    Eigen::Matrix<double,16,12> Dfr=Eigen::Matrix<double,16,12>::Zero();

		for(int i=0; i<4; i++)
		{
			Dfr.block(0+4*i,0+3*i,4,3)=cfr;
		}
		

    eigenD.block(0,18,16,12)=Dfr;


    // Known terms for inequality
	Eigen::Matrix<double,40, 1> eigenC= Eigen::Matrix<double,40,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
	Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();

    Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);

	eigenC.block(16,0,12,1)=tau_max-eigenBiascom;
	eigenC.block(28,0,12,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

	
	
     //Linear constraints matrix
    Eigen::Matrix<double,58, 31> eigenL= Eigen::Matrix<double,58,31>::Zero();

	eigenL<< eigenA,eigenb,
	         eigenD, eigenC;
			

    
    
   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 18)
            {
				Lt(i) = 0.0; 
			}
        else
           {
            Lt(i) = -1.0; 
		   }
		   for ( int j = 0; j < eigenL.cols(); j++ )
             L(i,j) = eigenL(i,j);
    }
   
   

    //Bounded constraints
     Eigen::Matrix<double,30, 1> eigenBL= Eigen::Matrix<double,30,1>::Zero();
     
	 eigenBL.block(6,0,12,1)=ddqmin;

     Eigen::Matrix<double,30, 1> eigenBU= Eigen::Matrix<double,30,1>::Zero();
	
	 eigenBU.block(6,0,12,1)=ddqmax;

     for ( int i = 0; i < 30; i++ )
    {  
		if (i<6)
	  {
       bl(i) = alglib::fp_neginf;
       bu(i) = alglib::fp_posinf;
	   }
       else if (i>=6 && i<18)
	   { bl(i) = eigenBL(i,0);
	     bu(i) = eigenBU(i,0);
	    }
		else
		{
	    bl(i) = alglib::fp_neginf;
        bu(i) = alglib::fp_posinf;

		}
    };

    bl(20)=0;
    bl(23)=0;
    bl(26)=0;
	bl(29)=0;


    
    // Set qp
    alglib::minqpsetbc(state, bl, bu);
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	alglib::minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);


    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);

    Eigen::VectorXd x_eigen= Eigen::VectorXd::Zero(30) ;
	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);




    
	Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
	tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(18,0,12,1);
	return tau;

}

// Quadratic problem with capture point
Eigen::VectorXd QUADRUPED::qpproblem_cp( Eigen::Matrix<double,6,1> &Wcom_des, float &m_blfl, float &m_flfr, 
float &m_frbr, float &m_brbl, float &q_blfl, float &q_flfr, float &q_frbr, float &q_brbl,
float &x_inf, float &x_sup, float&y_inf, float &y_sup)
{
	Eigen::Matrix<double,6,1> com_pos = iDynTree::toEigen(CoM);
	Eigen::Matrix<double,6,1> com_vel = iDynTree::toEigen(CoM_vel);
	//eigenfrequency
	w=sqrt(9.81/com_zdes);
	/* //termini noti del problema quadratico
	tnp = 2*w/(pow(Dt,2)*w + 2 * Dt) * (q_frbr +m_frbr * com_pos(0) +m_frbr * com_vel(0) * Dt  +m_frbr *com_pos(0)/w - com_pos(1) - com_vel(1) * Dt - com_vel(1)/w);
	tnn = 2*w/(pow(Dt,2)*w + 2 * Dt) * (q_blfl +m_blfl * com_pos(0) +m_blfl * com_vel(0) * Dt  +m_blfl *com_pos(0)/w - com_pos(1) - com_vel(1) * Dt - com_vel(1)/w);
   	tnr = 2*w/(pow(Dt,2)*w + 2 * Dt) * (q_flfr +m_flfr * com_pos(0) +m_flfr * com_vel(0) * Dt  +m_flfr *com_pos(0)/w - com_pos(1) - com_vel(1) * Dt - com_vel(1)/w);
	tns = 2*w/(pow(Dt,2)*w + 2 * Dt) * (q_brbl +m_brbl * com_pos(0) +m_brbl * com_vel(0) * Dt  +m_brbl *com_pos(0)/w - com_pos(1) - com_vel(1) * Dt - com_vel(1)/w);
	*/
	//vincoli superiori e inferiori per il poligono di supporto 
	tnp = 2*w/(pow(Dt,2)*w + 2 * Dt) * (x_sup  - com_pos(0) - com_vel(0) * Dt  -com_pos(0)/w);
	tnn = 2*w/(pow(Dt,2)*w + 2 * Dt) * (x_inf  - com_pos(0) - com_vel(0) * Dt  -com_pos(0)/w);

	tnp = 2*w/(pow(Dt,2)*w + 2 * Dt) * (y_inf  - com_pos(1) - com_vel(1) * Dt  -com_pos(1)/w);
	tnp = 2*w/(pow(Dt,2)*w + 2 * Dt) * (y_sup  - com_pos(1) - com_vel(1) * Dt  -com_pos(1)/w);

	// Set variables
   int variables=30;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(62,31);
   Lt.setlength(62);


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 12, 6> Jstcom= toEigen(JacCOM_lin).block(0,0,12,6);
   Eigen::Matrix<double, 12, 12> Jstj= toEigen(JacCOM_lin).block(0,6,12,12);

   // cost function quadratic matrix
   Eigen::Matrix<double,12,30> Sigma= Eigen::Matrix<double,12,30>::Zero();
   Sigma.block(0,18,12,12)= Eigen::Matrix<double,12,12>::Identity();
   
   Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;

   Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,30,30> eigenQ= eigenQ2+Eigen::Matrix<double,30,30>::Identity();
 

	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	Eigen::Matrix<double,30,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*Wcom_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }




    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(30,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	

	//Equality constraints
	Eigen::Matrix<double,18, 30> eigenA= Eigen::Matrix<double,18,30>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,18,6,12)=-Jstcom.transpose();

    eigenA.block(6,0,12,6)=Jstcom;

    eigenA.block(6,6,12,12)=Jstj;

    // Known term
    Eigen::Matrix<double,18, 1> eigenb= Eigen::Matrix<double,18,1>::Zero();

    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;
	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);;

	eigenb.block(6,0,12,1)=-toEigen(JdqdCOM_lin);


    
    

	//Inequality Constraints

	Eigen::Matrix<double,40, 30> eigenD= Eigen::Matrix<double,40,30>::Zero();
	
	 // Torque limits
	eigenD.block(16,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(16,18,12,12)=-Jstj.transpose();

	eigenD.block(28,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(28,18,12,12)=Jstj.transpose();

	

	//Friction
	   double mu=1;
	   Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
	   n<< 0,
	       0,
		   1;

	   Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
	   t1<< 1,
	       0,
		   0;

       Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
	   t2<<0,
	       1,
		   0;

	   Eigen::Matrix<double,4,3> cfr=Eigen::Matrix<double,4,3>::Zero();
  
	   cfr<<(-mu*n+t1).transpose(),
	        (-mu*n+t2).transpose(),
			-(mu*n+t1).transpose(),
			-(mu*n+t2).transpose();
     
	    Eigen::Matrix<double,16,12> Dfr=Eigen::Matrix<double,16,12>::Zero();

		for(int i=0; i<4; i++)
		{
			Dfr.block(0+4*i,0+3*i,4,3)=cfr;
		}
		

    eigenD.block(0,18,16,12)=Dfr;


    // Known terms for inequality
	Eigen::Matrix<double,40, 1> eigenC= Eigen::Matrix<double,40,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
	Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();

    Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);

	eigenC.block(16,0,12,1)=tau_max-eigenBiascom;
	eigenC.block(28,0,12,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

   /* 
   // aggiungi ad eigenL questi vincoli
   Matrix<double,1,42>::Zero(), -m, 1, tnp,
		Matrix<double,1,42>::Zero(), -m, 1, tnn,
		Matrix<double,1,42>::Zero(), 1/m, 1, tnr,
		Matrix<double,1,42>::Zero(), 1/m, 1, tns;
	*/
	
	
     //Linear constraints matrix
    Eigen::Matrix<double,62, 31> eigenL= Eigen::Matrix<double,62,31>::Zero();

	eigenL<< eigenA,eigenb,
	         eigenD, eigenC,
			 1,0,Eigen::Matrix<double,1,28>::Zero(),tnp,
			 1,0,Eigen::Matrix<double,1,28>::Zero(),tnn,
			 0,1,Eigen::Matrix<double,1,28>::Zero(),tnr,
			 0,1,Eigen::Matrix<double,1,28>::Zero(),tns;
			 /* vincoli nel caso in cui i lati del poligono non siano paralleli agli assi del sistema mondo
			 -m_frbr,1,Eigen::Matrix<double,1,28>::Zero(),tnp,
			 -m_blfl,1,Eigen::Matrix<double,1,28>::Zero(),tnn,
			 -m_flfr,1,Eigen::Matrix<double,1,28>::Zero(),tnr,
			 -m_brbl,1,Eigen::Matrix<double,1,28>::Zero(),tns;*/

    
    
   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 18)
            {
				Lt(i) = 0.0; 
			}
        else
           {
            Lt(i) = -1.0; 
		   }
		   for ( int j = 0; j < eigenL.cols(); j++ )
             L(i,j) = eigenL(i,j);
    }
   
    //Vincoli aggiuntivi per capture point
	Lt(58)= -1;
	Lt(59)= 1;
	Lt(60)= -1;
	Lt(61)= 1;

    //Bounded constraints
     Eigen::Matrix<double,30, 1> eigenBL= Eigen::Matrix<double,30,1>::Zero();
     
	 eigenBL.block(6,0,12,1)=ddqmin;

     Eigen::Matrix<double,30, 1> eigenBU= Eigen::Matrix<double,30,1>::Zero();
	
	 eigenBU.block(6,0,12,1)=ddqmax;

     for ( int i = 0; i < 30; i++ )
    {  
		if (i<6)
	  {
       bl(i) = alglib::fp_neginf;
       bu(i) = alglib::fp_posinf;
	   }
       else if (i>=6 && i<18)
	   { bl(i) = eigenBL(i,0);
	     bu(i) = eigenBU(i,0);
	    }
		else
		{
	    bl(i) = alglib::fp_neginf;
        bu(i) = alglib::fp_posinf;

		}
    };

    bl(20)=0;
    bl(23)=0;
    bl(26)=0;
	bl(29)=0;


    
    // Set qp
    alglib::minqpsetbc(state, bl, bu);
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	alglib::minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);


    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);

    Eigen::VectorXd x_eigen= Eigen::VectorXd::Zero(30) ;
	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);




    
	Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
	tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(18,0,12,1);
	return tau;

}

// Quadratic problem back right leg
Eigen::VectorXd QUADRUPED::qpproblemol( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::Vector3d vdotswdes,  SWING_LEG swingleg)
{
	int swl1, stl1, stl2, stl3;
    switch(swingleg){
		case BR: swl1=0; 
		stl1=3;
		stl2=6 ; 
		stl3=9;
		break;
		case FL: swl1=6;
		stl1=0;
		stl2=3 ; 
		stl3=9;
		break;
		case BL: swl1=3;
		stl1=0;
		stl2=6; 
		stl3=9;
		break;
		case FR: swl1=9;
		stl1=0;
		stl2=3; 
		stl3=6;
		 break;
	}

	// Set variables
   int variables=30;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(84,31);
   Lt.setlength(84);


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 9, 6> Jstcom= Eigen::Matrix<double,9,6>::Zero();
    Jstcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(stl1,0,3,6);
	Jstcom.block(3,0,3,6)= toEigen(JacCOM_lin).block(stl2,0,3,6);
    Jstcom.block(6,0,3,6)= toEigen(JacCOM_lin).block(stl3,0,3,6);

   Eigen::Matrix<double, 9, 12> Jstj= Eigen::Matrix<double,9,12>::Zero();
    Jstj.block(0,0,3,12)=toEigen(JacCOM_lin).block(stl1,6,3,12);
    Jstj.block(3,0,3,12)=toEigen(JacCOM_lin).block(stl2,6,3,12);
    Jstj.block(6,0,3,12)=toEigen(JacCOM_lin).block(stl3,6,3,12);

    Eigen::Matrix<double, 9, 18> Jst= Eigen::Matrix<double,9,18>::Zero();
    Jst.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,0,3,18);
    Jst.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,0,3,18);
	Jst.block(6,0,3,18)=toEigen(JacCOM_lin).block(stl3,0,3,18);


   Eigen::Matrix<double, 3, 6> Jswcom= Eigen::Matrix<double,3,6>::Zero();
    Jswcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(swl1,0,3,6);
	
   Eigen::Matrix<double, 3, 12> Jswj=  Eigen::Matrix<double,3,12>::Zero();
   Jswj.block(0,0,3,12)=toEigen(JacCOM_lin).block(swl1,6,3,12);


   // cost function quadratic matrix
   Eigen::Matrix<double,9,30> Sigma= Eigen::Matrix<double,9,30>::Zero();
   Sigma.block(0,18,9,9)= Eigen::Matrix<double,9,9>::Identity();
   
   Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;

   Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
   eigenR.block(27,27,3,3)=10000000*Eigen::Matrix<double,3,3>::Identity();
   
   Eigen::Matrix<double,30,30> eigenQ= eigenQ2+eigenR;
 

	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	Eigen::Matrix<double,30,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*Wcom_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }




    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(30,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	

	//Equality constraints
	Eigen::Matrix<double,15, 30> eigenA= Eigen::Matrix<double,15,30>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,18,6,9)=-Jstcom.transpose();

    eigenA.block(6,0,9,6)=Jstcom;

    eigenA.block(6,6,9,12)=Jstj;

	//std::cout<<"eigenA"<<eigenA<<std::endl;

    // Known term
    Eigen::Matrix<double,15, 1> eigenb= Eigen::Matrix<double,15,1>::Zero();

    Eigen::Matrix<double,9,1> Jdqdst= Eigen::Matrix<double,9,1>::Zero();
	 Jdqdst<<toEigen(JdqdCOM_lin).block(stl1,0,3,1),
	         toEigen(JdqdCOM_lin).block(stl2,0,3,1),
			 toEigen(JdqdCOM_lin).block(stl3,0,3,1);
			 
    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;



	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);

	eigenb.block(6,0,9,1)=-Jdqdst;

    //std::cout<<"eigenB"<<eigenb<<std::endl;
    

	//Inequality Constraints

	Eigen::Matrix<double,69,30> eigenD= Eigen::Matrix<double,69,30>::Zero();
	
	 // Torque limits
	eigenD.block(15,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(15,18,12,9)=-Jstj.transpose();

	eigenD.block(27,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(27,18,12,9)=Jstj.transpose();
    
    eigenD.block(39,0,3,6)=Jswcom;

    eigenD.block(39,6,3,12)=Jswj;

	eigenD.block(39,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(42,0,3,6)=-Jswcom;

    eigenD.block(42,6,3,12)=-Jswj;

    eigenD.block(42,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(45,6,12,12)=Eigen::Matrix<double,12,12>::Identity();

    eigenD.block(57,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
    
	//Friction
	   double mu=1;
	   Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
	   n<< 0,
	       0,
		   1;

	   Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
	   t1<< 1,
	       0,
		   0;

       Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
	   t2<<0,
	       1,
		   0;

	   Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
  
	   cfr<<(-mu*n+t1).transpose(),
	        (-mu*n+t2).transpose(),
			-(mu*n+t1).transpose(),
			-(mu*n+t2).transpose(),
			-n.transpose();
     
	    Eigen::Matrix<double,15,9> Dfr=Eigen::Matrix<double,15,9>::Zero();

		for(int i=0; i<3; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
		

    eigenD.block(0,18,15,9)=Dfr;

    //std::cout<<"eigenD"<<eigenD<<std::endl;
    // Known terms for inequality
	Eigen::Matrix<double,69, 1> eigenC= Eigen::Matrix<double,69,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
	Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();

    Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);
	for (int i=0; i<3; i++)
	{ eigenC.block(4+i*5,0,1,1)<<-2;}
	eigenC.block(15,0,12,1)=tau_max-eigenBiascom;
	eigenC.block(27,0,12,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

     eigenC.block(45,0,12,1)=ddqmax;
	 eigenC.block(57,0,12,1)=-ddqmin;
	 
	 Eigen::Matrix<double,3,1> Jdqdsw= Eigen::Matrix<double,3,1>::Zero();
	 Jdqdsw<<toEigen(JdqdCOM_lin).block(swl1,0,3,1);

     
	





	iDynTree::MatrixDynSize Jac1(6,18);
	iDynTree::MatrixDynSize Jac2(6,18);
    kinDynComp.getFrameFreeFloatingJacobian(7,Jac2);
	toEigen(Jac1)=toEigen(Jac2)*toEigen(T).inverse();

 
	 eigenC.block(39,0,3,1)= vdotswdes-Jdqdsw;
	 eigenC.block(42,0,3,1)= -vdotswdes+Jdqdsw;
	
	  // std::cout<<"eigenc"<<eigenC<<std::endl;
     //Linear constraints matrix
    Eigen::Matrix<double,84, 31> eigenL= Eigen::Matrix<double,84,31>::Zero();

   
	eigenL<< eigenA,eigenb,
	         eigenD, eigenC;

    
    
   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 15)
            {
				Lt(i) = 0.0; 
			}
        else
           {
            Lt(i) = -1.0; 
		   }
		   for ( int j = 0; j < eigenL.cols(); j++ )
             L(i,j) = eigenL(i,j);
    }
    

     
    //Bounded constraints
     /*Eigen::Matrix<double,30, 1> eigenBL= Eigen::Matrix<double,30,1>::Zero();
     
	 eigenBL.block(6,0,12,1)=ddqmin;

     Eigen::Matrix<double,30, 1> eigenBU= Eigen::Matrix<double,30,1>::Zero();
	
	 eigenBU.block(6,0,12,1)=ddqmax;

     for ( int i = 0; i < 30; i++ )
    {  
		if (i<6)
	  {
       bl(i) = alglib::fp_neginf;
       bu(i) = alglib::fp_posinf;
	   }
       else if (i>=6 && i<18)
	   { bl(i) = eigenBL(i,0);
	     bu(i) = eigenBU(i,0);
	    }
		else
		{
	    bl(i) = alglib::fp_neginf;
        bu(i) = alglib::fp_posinf;

		}
    };

    bl(20)=0;
    bl(23)=0;
    bl(26)=0;
	bl(29)=0;*/

   
    // Set qp
   // alglib::minqpsetbc(state, bl, bu);
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	alglib::minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);
    

    alglib::minqpoptimize(state);

	// Solve qp
	
    alglib::minqpresults(state, x_, rep);

    
	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);


    std::cout<<"solution"<<x_eigen<<std::endl;

    
	Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
	tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(18,0,9,1);
	std::cout<<"acc"<<toEigen(JacCOM_lin).block(0,0,3,18)*x_eigen.block(0,0,18,1)+toEigen(JdqdCOM_lin).block(0,0,3,1)<<std::endl;
	std::cout<<"vel"<<toEigen(JacCOM_lin).block(0,0,3,18)*toEigen(dq)<<std::endl;
	return tau;

}

// Quadratic problem back right leg
Eigen::VectorXd QUADRUPED::qpproblembr( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::VectorXd vdotswdes,  SWING_LEGS swinglegs, Eigen::Matrix<double,12,1> fext)
{
	int swl1, swl2, stl1, stl2;
	/*
    switch(swinglegs){
		case L1: swl1=0;
		swl2=0; 
		stl1=0;//0 indica br; 3 bl; 6->fl; 9->fr
		stl2=0; 
		break;
		case L2: swl1=0;
		swl2=9 ;
		stl1=3;
		stl2=6; 
		 break;
		case L3: swl1=3;
		swl2=6;
		stl1=0;
		stl2=9; 
		 break;
	}
	*/
	  switch(swinglegs){
		case L1: swl1=0;
		swl2=0; 
		stl1=0;//0 indica br; 3 bl; 6->fl; 9->fr
		stl2=0; 
		break;
		case L2: swl1=0;
		swl2=6 ;
		stl1=3;
		stl2=9; 
		 break;
		case L3: swl1=3;
		swl2=9;
		stl1=0;
		stl2=6; 
		 break;
	}

	// Set variables
   int variables=30;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(82,31);
   Lt.setlength(82);


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 6, 6> Jstcom= Eigen::Matrix<double,6,6>::Zero();
    Jstcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(stl1,0,3,6);
	Jstcom.block(3,0,3,6)= toEigen(JacCOM_lin).block(stl2,0,3,6);

   Eigen::Matrix<double, 6, 12> Jstj= Eigen::Matrix<double,6,12>::Zero();
    Jstj.block(0,0,3,12)=toEigen(JacCOM_lin).block(stl1,6,3,12);
    Jstj.block(3,0,3,12)=toEigen(JacCOM_lin).block(stl2,6,3,12);

  Eigen::Matrix<double, 6, 18> Jst= Eigen::Matrix<double,6,18>::Zero();
 Jst.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,0,3,18);
    Jst.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,0,3,18);

   Eigen::Matrix<double, 6, 6> Jswcom= Eigen::Matrix<double,6,6>::Zero();
    Jswcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(swl1,0,3,6);
	 Jswcom.block(3,0,3,6)= toEigen(JacCOM_lin).block(swl2,0,3,6);
   Eigen::Matrix<double, 6, 12> Jswj=  Eigen::Matrix<double,6,12>::Zero();
   Jswj.block(0,0,3,12)=toEigen(JacCOM_lin).block(swl1,6,3,12);
   Jswj.block(3,0,3,12)=toEigen(JacCOM_lin).block(swl2,6,3,12);
std::cout<<"a1"<<endl;

   // cost function quadratic matrix
   Eigen::Matrix<double,6,30> Sigma= Eigen::Matrix<double,6,30>::Zero();
   Sigma.block(0,18,6,6)= Eigen::Matrix<double,6,6>::Identity();
   
   Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;

   Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
   eigenR.block(24,24,6,6)=1000000000000*Eigen::Matrix<double,6,6>::Identity();
   
   Eigen::Matrix<double,30,30> eigenQ= eigenQ2+eigenR;
 
std::cout<<"a2"<<endl;

	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	Eigen::Matrix<double,30,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*Wcom_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }

/*std::cout<<"a21"<<endl;
std::cout<<"T_s: "<<endl<<T_s<<endl;
std::cout<<"Wcom_des: "<<endl<<Wcom_des<<endl;
std::cout<<"eigenc: "<<endl<<eigenc<<endl;

*/

    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(30,state);
	std::cout<<"a22"<<endl;

	alglib::minqpsetquadraticterm( state,Q);
	std::cout<<"a23"<<endl;

    alglib::minqpsetlinearterm(state,c); //
	
	std::cout<<"a3"<<endl;


	//Equality constraints
	Eigen::Matrix<double,12, 30> eigenA= Eigen::Matrix<double,12,30>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,18,6,6)=-Jstcom.transpose();

    eigenA.block(6,0,6,6)=Jstcom;

    eigenA.block(6,6,6,12)=Jstj;

	//std::cout<<"eigenA"<<eigenA<<std::endl;

    // Known term
    Eigen::Matrix<double,12, 1> eigenb= Eigen::Matrix<double,12,1>::Zero();

    Eigen::Matrix<double,6,1> Jdqdst= Eigen::Matrix<double,6,1>::Zero();
	 Jdqdst<<toEigen(JdqdCOM_lin).block(stl1,0,3,1),
	         toEigen(JdqdCOM_lin).block(stl2,0,3,1);
			 
    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;

       Eigen::Matrix<double,6,1> fext_st;
   fext_st.block(0,0,3,1)=fext.block(stl1,0,3,1);
   fext_st.block(3,0,3,1)=fext.block(stl2,0,3,1);
	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1)+Jstcom.transpose()*fext_st;

	eigenb.block(6,0,6,1)=-Jdqdst;

    //std::cout<<"eigenB"<<eigenb<<std::endl;
    
    
    Eigen::Matrix<double,18,18> P=Eigen::Matrix<double,18,18>::Identity()-Jst.transpose()*(Jst*toEigen(MassMatrixCOM).inverse()*Jst.transpose()).inverse()*Jst*toEigen(MassMatrixCOM).inverse();
   
   
	//Inequality Constraints

	Eigen::Matrix<double,70,30> eigenD= Eigen::Matrix<double,70,30>::Zero();
	
	 // Torque limits
	eigenD.block(10,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(10,18,12,6)=-Jstj.transpose();

	eigenD.block(22,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(22,18,12,6)=Jstj.transpose();
    
    eigenD.block(34,0,3,6)=Jswcom.block(0,0,3,6);

    eigenD.block(34,6,3,12)=Jswj.block(0,0,3,12);

	eigenD.block(37,0,3,6)=Jswcom.block(3,0,3,6);

    eigenD.block(37,6,3,12)=Jswj.block(3,0,3,12);

	eigenD.block(34,24,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(37,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(40,0,3,6)=-Jswcom.block(0,0,3,6);

    eigenD.block(40,6,3,12)=-Jswj.block(0,0,3,12);

	eigenD.block(43,0,3,6)=-Jswcom.block(3,0,3,6);

    eigenD.block(43,6,3,12)=-Jswj.block(3,0,3,12);

    eigenD.block(40,24,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(43,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(46,6,12,12)=Eigen::Matrix<double,12,12>::Identity();

    eigenD.block(58,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
    
	//Friction
	   double mu=1;
	   Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
	   n<< 0,
	       0,
		   1;

	   Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
	   t1<< 1,
	       0,
		   0;

       Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
	   t2<<0,
	       1,
		   0;

	   Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
  
	   cfr<<(-mu*n+t1).transpose(),
	        (-mu*n+t2).transpose(),
			-(mu*n+t1).transpose(),
			-(mu*n+t2).transpose(),
			-n.transpose();
     
	    Eigen::Matrix<double,10,6> Dfr=Eigen::Matrix<double,10,6>::Zero();

		for(int i=0; i<2; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
		

    eigenD.block(0,18,10,6)=Dfr;

    //std::cout<<"eigenD"<<eigenD<<std::endl;
    // Known terms for inequality
	Eigen::Matrix<double,70, 1> eigenC= Eigen::Matrix<double,70,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
	Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();

    Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);

    
	eigenC.block(10,0,12,1)=tau_max-eigenBiascom;
	eigenC.block(22,0,12,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

     eigenC.block(46,0,12,1)=ddqmax;
	 eigenC.block(58,0,12,1)=-ddqmin;


	 Eigen::Matrix<double,6,1> Jdqdsw= Eigen::Matrix<double,6,1>::Zero();
	 Jdqdsw<<toEigen(JdqdCOM_lin).block(swl1,0,3,1),
	         toEigen(JdqdCOM_lin).block(swl2,0,3,1);
 
	  Eigen::Matrix<double,6,1> fext_lambda= Eigen::Matrix<double,6,1>::Zero();

	  Eigen::Matrix<double,18,18> Si;
	  Si<<Eigen::Matrix<double,6,18>::Zero(),
	      Eigen::Matrix<double,12,6>::Zero(),Eigen::Matrix<double,12,12>::Identity();
    
    
	fext_lambda<<toEigen(JacCOM_lin).block(swl1,0,3,18)*(P*toEigen(MassMatrixCOM)+Eigen::Matrix<double,18,18>::Identity()-P).inverse()*P*toEigen(JacCOM_lin).transpose()*fext,
	            toEigen(JacCOM_lin).block(swl2,0,3,18)*(P*toEigen(MassMatrixCOM)+Eigen::Matrix<double,18,18>::Identity()-P).inverse()*P*toEigen(JacCOM_lin).transpose()*fext;

	  //std::cout<<"fextl"<<fext_lambda<<std::endl;

	iDynTree::MatrixDynSize Jac1(6,18);
	iDynTree::MatrixDynSize Jac2(6,18);
    kinDynComp.getFrameFreeFloatingJacobian(7,Jac2);
	toEigen(Jac1)=toEigen(Jac2)*toEigen(T).inverse();

	
	
    //std::cout<<"fextl"<<toEigen(JacCOM_lin).block(0,0,12,6).transpose()*fext<<std::endl;

	//std::cout<<"fextl"<<(toEigen(Jac1).block(0,0,3,18)*toEigen(MassMatrixCOM).inverse()*P*toEigen(Jac1).block(0,0,3,18).transpose()).inverse()*toEigen(Jac1).block(0,0,3,18)*toEigen(MassMatrixCOM).inverse()*Si.transpose()*P*toEigen(JacCOM_lin).transpose()*fext<<std::endl;
	//Eigen::Matrix<double,6,6> Lambdasw1=(toEigen(JacCOM_lin).block(0,0,3,18)*Mc.inverse()*P*toEigen(JacCOM_lin).block(0,0,3,18).transpose()).inverse();
	Eigen::Matrix<double,6,1> vdotsw_lambda;
	vdotsw_lambda<<vdotswdes.block(0,0,3,1),
	               vdotswdes.block(3,0,3,1);

	 eigenC.block(34,0,6,1)= vdotsw_lambda-Jdqdsw-fext_lambda;
	 eigenC.block(40,0,6,1)= -vdotsw_lambda+Jdqdsw+fext_lambda;
	
	  // std::cout<<"eigenc"<<eigenC<<std::endl;
     //Linear constraints matrix
    Eigen::Matrix<double,82, 31> eigenL= Eigen::Matrix<double,82,31>::Zero();

   
	eigenL<< eigenA,eigenb,
	         eigenD, eigenC;

    
    
   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 12)
            {
				Lt(i) = 0.0; 
			}
        else
           {
            Lt(i) = -1.0; 
		   }
		   for ( int j = 0; j < eigenL.cols(); j++ )
             L(i,j) = eigenL(i,j);
    }
    

     
    //Bounded constraints
     /*Eigen::Matrix<double,30, 1> eigenBL= Eigen::Matrix<double,30,1>::Zero();
     
	 eigenBL.block(6,0,12,1)=ddqmin;

     Eigen::Matrix<double,30, 1> eigenBU= Eigen::Matrix<double,30,1>::Zero();
	
	 eigenBU.block(6,0,12,1)=ddqmax;

     for ( int i = 0; i < 30; i++ )
    {  
		if (i<6)
	  {
       bl(i) = alglib::fp_neginf;
       bu(i) = alglib::fp_posinf;
	   }
       else if (i>=6 && i<18)
	   { bl(i) = eigenBL(i,0);
	     bu(i) = eigenBU(i,0);
	    }
		else
		{
	    bl(i) = alglib::fp_neginf;
        bu(i) = alglib::fp_posinf;

		}
    };

    bl(20)=0;
    bl(23)=0;
    bl(26)=0;
	bl(29)=0;*/


    
    // Set qp
   // alglib::minqpsetbc(state, bl, bu);
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    std::cout<<"a4"<<endl;

    alglib::minqpreport rep;
	alglib::minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);
    std::cout<<"a5"<<endl;


    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);


   
	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);


    std::cout<<"solution"<<x_eigen.block(18,0,6,1)<<std::endl;

    
	Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
	tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(18,0,6,1);
	//std::cout<<"acc"<<toEigen(JacCOM_lin).block(swl1,0,3,18)*x_eigen.block(0,0,18,1)+toEigen(JdqdCOM_lin).block(swl1,0,3,1)<<std::endl;
	//std::cout<<"vel"<<toEigen(JacCOM_lin).block(swl1,0,3,18)*toEigen(dq)<<std::endl;
	return tau;


}