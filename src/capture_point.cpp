#include "capture_point.h"

using namespace std;

CAPTURE_POINT::CAPTURE_POINT(){

};

void CAPTURE_POINT::capture_point(VectorXd &b, Matrix<double,18,18> &M, Matrix<double,24,18> &Jc, Matrix<double,24,1> &Jcdqd, Matrix<double,18,18> &T, Matrix<double,18,18> &T_dot,Matrix<double, 18,1> &q_joints_total, Matrix<double, 18,1> &dq_joints_total, Vector3d &com_pos, Vector3d &com_vel,  float &m, float &q_plus, float &q_minus, float &qs, float &qr){
	//eigenfrequency
	w=sqrt(9.81/com_zdes);
	/*
	cout<<"m: "<<m<<endl;
	cout<<"qs: "<<qs<<endl;
	cout<<"qr: "<<qr<<endl;
	*/
	
	tnp = 2*w/(pow(Dt,2)*w + 2 * Dt) * (q_plus +m * com_pos[0] +m * com_vel[0] * Dt  +m *com_pos[0]/w - com_pos[1] - com_vel[1] * Dt - com_vel[1]/w);
	tnn = 2*w/(pow(Dt,2)*w + 2 * Dt) * (q_minus +m * com_pos[0] +m * com_vel[0] * Dt  +m *com_pos[0]/w - com_pos[1] - com_vel[1] * Dt - com_vel[1]/w);
   	tnr = 2*w/(pow(Dt,2)*w + 2 * Dt) * (qr + (-1/m) * com_pos[0] +(-1/m) * com_vel[0] * Dt  +(-1/m) *com_pos[0]/w - com_pos[1] - com_vel[1] * Dt - com_vel[1]/w);
	tns = 2*w/(pow(Dt,2)*w + 2 * Dt) * (qs + (-1/m) * com_pos[0] +(-1/m) * com_vel[0] * Dt  +(-1/m) *com_pos[0]/w - com_pos[1] - com_vel[1] * Dt - com_vel[1]/w);

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
	//termini quadratici della funzione da minimizzare
	Matrix<double,44,44> aa;
	aa<<Matrix<double,44,44>::Zero();
	aa(41,41)=1;
	real_2d_array a;
	a.setlength(44,44);
	a.setcontent(44,44,&aa(0,0));
	
  
    real_1d_array s;
	s.setlength(44);
	for(int i = 0; i< 44; i++){
		
		s(0)=1;
	} 
	


	//vincoli del problema quadratico 


	real_2d_array c; 
	
	
	Matrix<double,38,45> A;
	
    A<< -m, 1, Matrix<double,1,42>::Zero(), tnp,
		-m, 1, Matrix<double,1,42>::Zero(), tnn,
		1/m, 1, Matrix<double,1,42>::Zero(), tnr,
		1/m, 1, Matrix<double,1,42>::Zero(), tns,
		Matrix<double,18,2>::Zero(), M,Jc_T_B,-S_T,-b,
		Matrix<double,12,2>::Zero(), B_T_Jc, Matrix<double,12,24>::Zero(),-B.transpose()*Jcdqd,
		Matrix<double,4,2>::Zero(), Matrix<double,4,18>::Zero(), Sn, Matrix<double,4,12>::Zero(),Matrix<double,4,1>::Zero();
	/*
	cout<<"A:"<<endl;
	cout<<A<<endl;
	*/
	
	c.setlength(38,45);
    c.setcontent(38,45, &A(0,0));

	


	//parametro che imposta la tipologia del vincolo 
	integer_1d_array ct;
	ct.setlength(38);
	ct(0)= -1;
	ct(1)= 1;
	ct(2)= -1;
	ct(3)= 1;

	for(int i = 4; i <34; i++){
		ct(i) = 0;
	}
	for(int i = 34; i<38; i++){
		ct(i) = 1;
	}

	//box constrain
	real_1d_array bndl;
	bndl.setlength(44);
	real_1d_array bndu;
	bndu.setlength(44);
	VectorXd qmin(12);
	VectorXd qmax(12);
	

	qmin<<-1.7453,-1.7453,-1.7453,-1.7453,-3.14159265359,-0.02,-1.57079632679,-2.61795,-1.57079632679,-2.61795,-3.14159265359,-0.02;
	qmax<<1.7453, 1.7453, 1.7453, 1.7453, 1.57079632679, 2.61795, 3.14159265359, 0.02, 3.14159265359, 0.02, 1.57079632679, 2.61795;
	
	
	//Vincoli sulle accelerazioni ai giunti virtuali
	for(int i = 0; i<8; i++){
		bndl(i)=fp_neginf;//-INFINITY
		bndu(i)=fp_posinf;//+INFINITY
	}
	
	//Vincoli sulle accelerazioni
	for(int i = 8; i<20; i++){
		bndl(i)=(2/pow(Dt,2)) * (qmin(i-8)-q_joints_total(i-2)- Dt * dq_joints_total(i-2));// fp_neginf;//-INFINITY
		bndu(i)=(2/pow(Dt,2)) * (qmax(i-8)-q_joints_total(i-2)- Dt * dq_joints_total(i-2));// fp_posinf;//+INFINITY
	}
	
	//Vincoli sulle forze di contatto
	for(int i = 20; i<32; i++){
		bndl(i)=fp_neginf;//-INFINITY
		bndu(i)=fp_posinf;//+INFINITY
	}
	
	//Vincoli sulle coppie
	for(int i=32; i<44; i++){
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
	
    minqpcreate(44, state);
	
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
//i valori delle coppie sono scalati di 2 perchè ho aggiunto 2 variabili di controllo iniziali
    tau = { x(32),x(42),x(43),x(33),x(40),x(41),x(35),x(36),x(37),x(34),x(38),x(39)};
	/*
	for(int i =0; i<12; i++){
		cout<<"tau: "<<tau[11-i]<<endl;
	}
*/

}


//get function

vector<double> CAPTURE_POINT::getTau(){

    return tau;
}