#include "capture_point.h"

using namespace std;

CAPTURE_POINT::CAPTURE_POINT(){

};

void CAPTURE_POINT::capture_point(VectorXd &b, Matrix<double,18,18> &M, Matrix<double,24,18> &Jc, Matrix<double,24,1> &Jcdqd, Matrix<double,18,18> &T, Matrix<double,18,18> &T_dot,Matrix<double, 18,1> &q_joints_total, Matrix<double, 18,1> &dq_joints_total, Vector3d &com_pos, Vector3d &com_vel){
	//eigenfrequency
	w=sqrt(9.81/com_zdes);
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
	//aa(42,42)=1;
	real_2d_array a;
	a.setlength(42,42);
	a.setcontent(42,42,&aa(0,0));
	
  
    real_1d_array s;
	s.setlength(42);
	for(int i = 0; i< 42; i++){
		
		s(0)=1;
	} 
	


	//vincoli del problema quadratico 


	real_2d_array c; 
	
	
	Matrix<double,34,43> A;
    A<<M,Jc_T_B,-S_T,-b,
		B_T_Jc, Matrix<double,12,24>::Zero(),-B.transpose()*Jcdqd,
		Matrix<double,4,18>::Zero(), Sn, Matrix<double,4,13>::Zero();
	
	//cout<<"A:"<<endl;
	//cout<<A<<endl;
	c.setlength(34,43);
    c.setcontent(34,43, &A(0,0));

	
	

	//parametro che imposta la tipologia del vincolo 
	integer_1d_array ct;
	ct.setlength(34);
	for(int i = 0; i <30; i++){
		ct(i) = 0;
	}
	for(int i = 30; i<34; i++){
		ct(i) = 1;
	}

	//box constrain
	real_1d_array bndl;
	bndl.setlength(42);
	real_1d_array bndu;
	bndu.setlength(42);
	VectorXd qmin(12);
	VectorXd qmax(12);
	double dt=0.001;
	double Dt=10*dt;

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