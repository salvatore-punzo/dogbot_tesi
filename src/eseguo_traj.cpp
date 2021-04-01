#include "eseguo_traj.h"

ESEGUOtraj::ESEGUOtraj()
{
	
}

ESEGUOtraj::ESEGUOtraj(QUADRUPED &quadruped_)

{
    doggo = &quadruped_;
  
}

ESEGUOtraj::ESEGUOtraj(PROB_QUAD){
	
}


vector<double> ESEGUOtraj::eseguo_traj(int idx3,Matrix<double, 18,1> q_joints_total, 
Matrix<double, 18,1> dq_joints_total, trajectory_point traj_com3)
{	
	
		
		cout<<"wow1"<<endl;	
		//doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
		cout<<"wow1.1.1"<<endl;
		VectorXd b=doggo->getBiasMatrix();
		Matrix<double,18,18> M=doggo->getMassMatrix();
		cout<<"wow1.1"<<endl;
		Matrix<double,24,18> Jc=doggo->getJacobian();
		Matrix<double,24,1> Jcdqd=doggo->getBiasAcc();
		Matrix<double,18,18> T=doggo->getTransMatrix();
		Matrix<double,18,18> T_dot=doggo->getTdotMatrix();
		Matrix<double,6,18> Jcom=doggo->getJCOMMatrix();				
        Matrix<double,6,18> Jcomdot=doggo->getJCOMDot();				
		MatrixXd com_pos=doggo->getCOMpos();				
        MatrixXd com_vel=doggo->getCOMvel();
		cout<<"wow1.2"<<endl;
		//traiettoria per il centro di massa
		//Calcolo vettori desiderati prendo solo la posizione e non l'orientamento
		Matrix<double,6,1> composdes, comveldes, comaccdes;
		composdes<<traj_com3.pos(0,idx3), traj_com3.pos(1,idx3), traj_com3.pos(2,idx3),MatrixXd::Zero(3,1);
		comveldes<<traj_com3.vel(0,idx3), traj_com3.vel(1,idx3), traj_com3.vel(2,idx3),MatrixXd::Zero(3,1);
		comaccdes<<traj_com3.acc(0,idx3), traj_com3.acc(1,idx3), traj_com3.acc(2,idx3),MatrixXd::Zero(3,1);
					
		com_pos = doggo->getCOMpos();
		com_vel = doggo->getCOMvel();
		cout<<"wow2"<<endl;
		/*//scrivo dati su file
		com_file<<com_pos(0)<<" "<<com_pos(1)<<" "<<com_pos(2)<<" "<<com_pos(3)<<" "<<com_pos(4)<<" "<<com_pos(5)<<"\n";
		com_file.flush();
		cpx = com_pos(0)+com_vel(0)/w;
		cpy = com_pos(1)+com_vel(1)/w;
		capture_point_file<<cpx<<" "<<cpy<<"\n";
		capture_point_file.flush();
		com_traiettoria_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
		com_traiettoria_des_file.flush();
		foothold_file<<coo_ee_bl(0)<<" "<<coo_ee_bl(1)<<" "<<coo_ee_bl(2)<<" "<<coo_ee_br(0)<<" "
					<<coo_ee_br(1)<<" "<<coo_ee_br(2)<<" "<<coo_ee_fl(0)<<" "<<coo_ee_fl(1)<<" "<<coo_ee_fl(2)<<" "
					<<coo_ee_fr(0)<<" "<<coo_ee_fr(1)<<" "<<coo_ee_fr(2)<<" "<<"\n";
		foothold_file.flush();
		ts = ros::Time::now();
		tempo_simulazione_file<<ts<<"\n";
		tempo_simulazione_file.flush();
		*/
        //controllo senza capture point
			ottim->CalcoloProbOttimo(b, M, Jc, Jcdqd, T, T_dot, q_joints_total, dq_joints_total, composdes, comveldes, com_pos, com_vel, Jcom, Jcomdot);
			cout<<"wow21"<<endl;
			vector<double> tau = ottim->getTau();
			cout<<"wow3"<<endl;
			return tau;
			
		
    
}
