#include "prob_quadratico.h"
//#include "traj_planner.h"

using namespace std;
PROB_QUAD::PROB_QUAD(){
   
};

void PROB_QUAD::CalcoloProbOttimo(VectorXd &b, Matrix<double,18,18> &M, Matrix<double,24,18> &Jc, Matrix<double,24,1> &Jcdqd, Matrix<double,18,18> &T, Matrix<double,18,18> &T_dot,Matrix<double, 18,1> &q_joints_total, Matrix<double, 18,1> &dq_joints_total, Matrix<double,3,1> &composdes, Matrix<double,3,1> &comveldes, Vector3d &com_pos, Vector3d &com_vel){
//cout<<"What a wonderful day!"<<endl;

//Jacobian task 1
	Jt1<<1,Matrix<double,1,17>::Zero(),
		0,1, Matrix<double,1,16>::Zero(),
		0,0,1,Matrix<double,1,15>::Zero();

	Jt1_T<<Jt1 * T;

	Jt1_Tdot_dq<<Jt1 * T_dot * dq_joints_total;

	//Sn è la matrice che seleziona le componenti sull'asse z delle forze di contatto
	Sn<<0,0,1,Matrix<double,1,9>::Zero(),
		Matrix<double,1,5>::Zero(),1,Matrix<double,1,6>::Zero(),
		Matrix<double,1,8>::Zero(),1,Matrix<double,1,3>::Zero(),
		Matrix<double,1,11>::Zero(),1;

    Matrix<double,3,3> I = Matrix<double,3,3>::Identity(3,3);
	Matrix<double,3,3> zero = Matrix<double,3,3>::Zero(3,3);

	B<< I,zero,zero,zero,
		zero,zero,zero,zero,
		zero,I,zero,zero,
		zero,zero,zero,zero,
		zero,zero,I,zero,
		zero,zero,zero,zero,
		zero,zero,zero,I,
		zero,zero,zero,zero;

    S << MatrixXd::Zero(12,6), MatrixXd::Identity(12,12);
	S_T = S.transpose();
    Jc_T_B = Jc.transpose() * B;
	B_T_Jc = B.transpose() * Jc;
    //cout<<"wow"<<endl;

    //imposto il problema quadratico
	Matrix<double,43,43> aa;
	aa<<Matrix<double,43,43>::Zero();
	aa(42,42)=1;
	real_2d_array a;
	a.setlength(43,43);
	a.setcontent(43,43,&aa(0,0));
	
  
    real_1d_array s;
	s.setlength(43);
	for(int i = 0; i< 43; i++){
		
		s(0)=1;
	} 
	


	//vincoli del problema quadratico 


	real_2d_array c; 
	
	
	Matrix<double,40,44> A;

	
     
		e<<com_pos - composdes;//posizione corrente - posizione desiderata
		
		e_dot<<com_vel - comveldes; //velocità corrente asse z - velocità desiderata
	//controlla i segni di kp e kd, controlla i segni della matrice A, verifica Jt1
	
	A<<M,Jc_T_B,-S_T,Matrix<double,18,1>::Zero(),-b,
		B_T_Jc, Matrix<double,12,25>::Zero(),-B.transpose()*Jcdqd,
		Matrix<double,4,18>::Zero(), Sn, Matrix<double,4,13>::Zero(),Matrix<double,4,1>::Zero(),
		Jt1_T, Matrix<double,3,24>::Zero(),Matrix<double,3,1>::Ones(), -Jt1_Tdot_dq -kd*e_dot - kp * e,
		Jt1_T, Matrix<double,3,24>::Zero(),-Matrix<double,3,1>::Ones(), -Jt1_Tdot_dq -kd*e_dot - kp * e;
	//cout<<"A:"<<endl;
	//cout<<A<<endl;
	c.setlength(40,44);
	c.setcontent(40,44, &A(0,0));

	
	

	//parametro che imposta la tipologia del vincolo 
	integer_1d_array ct;
	ct.setlength(40);
	for(int i = 0; i <30; i++){
		ct(i) = 0;
	}
	for(int i = 30; i<37; i++){
		ct(i) = 1;
	}
	//metti dis
	for(int i = 37; i<40; i++){
		ct(i) = -1;
	}
	//box constrain
	real_1d_array bndl;
	bndl.setlength(43);
	real_1d_array bndu;
	bndu.setlength(43);
	VectorXd qmin(12);
	VectorXd qmax(12);
	double dt=0.001;
	double Dt=10*dt;

    //limiti della posizione dei giunti
	qmin<<-1.7453,-1.7453,-1.7453,-1.7453,-3.14159265359,-0.02,-1.57079632679,-2.61795,-1.57079632679,-2.61795,-3.14159265359,-0.02;
	qmax<<1.7453, 1.7453, 1.7453, 1.7453, 1.57079632679, 2.61795, 3.14159265359, 0.02, 3.14159265359, 0.02, 1.57079632679, 2.61795;
	
	//Vincoli sulle accelerazioni ai giunti virtuali
	for(int i = 0; i<6; i++){
		bndl(i)=fp_neginf;//-INFINITY
		bndu(i)=fp_posinf;//+INFINITY
	}
	//Vincoli sulle accelerazioni
	for(int i = 6; i<18; i++){
		bndl(i)=(2/pow(Dt,2)) * (qmin(i-6)-q_joints_total(i)- Dt * dq_joints_total(i));// fp_neginf;//-INFINITY
		bndu(i)=(2/pow(Dt,2)) * (qmax(i-6)-q_joints_total(i)- Dt * dq_joints_total(i));// fp_posinf;//+INFINITY
	}
	//Vincoli sulle forze di contatto
	for(int i = 18; i<30; i++){
		bndl(i)=fp_neginf;//-INFINITY
		bndu(i)=fp_posinf;//+INFINITY
	}
	//Vincoli sulle coppie
	for(int i=30; i<42; i++){
		bndl(i)= -60;
		bndu(i)= 60;
	}

	bndl(42)= 0;//-Infinity
	bndu(42)= fp_posinf;//+Infinity
	/*
for(int i = 6;i<18;i++){
	cout<<"limite superiore sulle accelerazioni: "<<bndu(i)<<endl;
	cout<<"limite inferiore sulle accelerazioni: "<<bndl(i)<<endl;
	cout<<"q joints corrente "<<i<<": "<<q_joints_total(i)<<endl;
	cout<<"qmax - qcorrente: "<<qmax(i-6)-q_joints_total(i)<<endl;
	cout<<"qmin - qcorrente: "<<qmin(i-6)-q_joints_total(i)<<endl;
}
	*/
	real_1d_array x;
	minqpstate state;
    minqpreport rep;

	 // NOTE: for convex problems you may try using minqpsetscaleautodiag()
    //       which automatically determines variable scales.
	minqpsetscale(state, s);
	// create solver, set quadratic/linear terms
	
    minqpcreate(43, state);
    minqpsetquadraticterm(state, a);
	//minqpsetlinearterm(state, l); non mi servono perchè di default sono già impostati a zero
    minqpsetlc(state, c, ct);
	minqpsetbc(state, bndl, bndu);
	
	minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
    minqpoptimize(state);

	minqpresults(state, x, rep);

	cout<<"Variabili ottimizzate: ";
    printf("%s\n", x.tostring(1).c_str());
	cout<<endl;

    tau = { x(30),x(40),x(41),x(31),x(38),x(39),x(33),x(34),x(35),x(32),x(36),x(37)};
	/*
	for(int i =0; i<12; i++){
		cout<<"tau: "<<tau[11-i]<<endl;
	}
*/

}
//get function

vector<double> PROB_QUAD::getTau(){

    return tau;
}