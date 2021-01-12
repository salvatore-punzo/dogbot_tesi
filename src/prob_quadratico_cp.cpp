#include "prob_quadratico_cp.h"
//#include "traj_planner.h"

using namespace std;
PROB_QUAD_CP::PROB_QUAD_CP(){
   
};

void PROB_QUAD_CP::CalcoloProbOttimoCP(VectorXd &b, Matrix<double,18,18> &M, Matrix<double,24,18> &Jc, Matrix<double,24,1> &Jcdqd, Matrix<double,18,18> &T, Matrix<double,18,18> &T_dot,Matrix<double, 18,1> &q_joints_total, Matrix<double, 18,1> &dq_joints_total, 
Matrix<double,6,1> &composdes, Matrix<double,6,1> &comveldes,  MatrixXd &com_pos, MatrixXd &com_vel,  Eigen::Matrix<double,6,18> Jt1, Eigen::Matrix<double,6,18> Jcomdot,  float &m, float &q_plus, float &q_minus, float &qs, float &qr){
	//eigenfrequency
	w=sqrt(9.81/com_zdes);
	
	/*
	cout<<"m: "<<m<<endl;
	cout<<"qs: "<<qs<<endl;
	cout<<"qr: "<<qr<<endl;
	*/
	
	tnp = 2*w/(pow(Dt,2)*w + 2 * Dt) * (q_plus +m * com_pos(0) +m * com_vel(0) * Dt  +m *com_pos(0)/w - com_pos(1) - com_vel(1) * Dt - com_vel(1)/w);
	tnn = 2*w/(pow(Dt,2)*w + 2 * Dt) * (q_minus +m * com_pos(0) +m * com_vel(0) * Dt  +m *com_pos(0)/w - com_pos(1) - com_vel(1) * Dt - com_vel(1)/w);
   	tnr = 2*w/(pow(Dt,2)*w + 2 * Dt) * (qr + (-1/m) * com_pos(0) +(-1/m) * com_vel(0) * Dt  +(-1/m) *com_pos(0)/w - com_pos(1) - com_vel(1) * Dt - com_vel(1)/w);
	tns = 2*w/(pow(Dt,2)*w + 2 * Dt) * (qs + (-1/m) * com_pos(0) +(-1/m) * com_vel(0) * Dt  +(-1/m) *com_pos(0)/w - com_pos(1) - com_vel(1) * Dt - com_vel(1)/w);


		
	// Matrici Jacobiane

	

	Jt1_dot_dq<<Jcomdot * dq_joints_total;

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

    
	// errore
    e<<com_pos - composdes;//posizione corrente - posizione desiderata
		
	e_dot<<com_vel - comveldes; //velocità corrente asse z - velocità desiderata
	
	Eigen::Matrix<double,6,1> matrixb= -Jt1_dot_dq -kd*e_dot - kp * e;

    // Termine quadratico, minimizza l'equazione relativa all'errore, motion task
	Eigen::Matrix<double,18,44> Sigma= Eigen::Matrix<double,18,44>::Zero();
    Sigma.block(0,0,18,18)= Eigen::Matrix<double,18,18>::Identity();
    Eigen::Matrix<double,6,44> matrixA=Jt1*Sigma;
	Eigen::Matrix<double,6,6> eigenQ= Eigen::Matrix<double,6,6>::Identity();    
    Eigen::Matrix<double,44,44> eigenQ1=  matrixA.transpose()*eigenQ* matrixA;

    
    //imposto il problema quadratico
	/*Matrix<double,43,43> aa;
	aa<<Matrix<double,43,43>::Identity();
	aa(42,42)=1;*/
	real_2d_array a;
	a.setlength(44,44);
	//a.setcontent(42,42,&eigenQ1(0,0));

	 for ( int i = 0; i < eigenQ1.rows(); i++ ){
        for ( int j = 0; j < eigenQ1.cols(); j++ )
             a(i,j) = eigenQ1(i,j);
    } 

	
	// Termine lineare
	real_1d_array lineartermalglib;
	lineartermalglib.setlength(44);
	Eigen::Matrix<double,44,1> linearterm= -matrixA.transpose()*eigenQ.transpose()*matrixb; 
	 for ( int i = 0; i < linearterm.rows(); i++ ){
       for ( int j = 0; j < linearterm.cols(); j++ )
             lineartermalglib(i) = linearterm(i,j);
    }

    real_1d_array s;
	s.setlength(44);
	for(int i = 0; i< 44; i++){
		
		s(0)=1;
	} 
	

   //Friction cones
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
     
	    Eigen::Matrix<double,20,12> Dfr=Eigen::Matrix<double,20,12>::Zero();

		for(int i=0; i<4; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
		

	
	//limiti della posizione dei giunti
     VectorXd qmin(12);
	VectorXd qmax(12);
	double dt=0.001;
	double Dt=10*dt;

    
	qmin<<-1.7453,-1.7453,-1.7453,-1.7453,-3.14159265359,-0.02,-1.57079632679,-2.61795,-1.57079632679,-2.61795,-3.14159265359,-0.02;
	qmax<<1.7453, 1.7453, 1.7453, 1.7453, 1.57079632679, 2.61795, 3.14159265359, 0.02, 3.14159265359, 0.02, 1.57079632679, 2.61795;

	Eigen::Matrix<double,12, 1> ddqmin=(2/pow(Dt,2)) * (qmin-q_joints_total.block(6,0,12,1)- Dt * dq_joints_total.block(6,0,12,1));
    Eigen::Matrix<double,12, 1> ddqmax=(2/pow(Dt,2)) * (qmax-q_joints_total.block(6,0,12,1)- Dt * dq_joints_total.block(6,0,12,1));

    Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
	Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();


	real_2d_array c; 
	
	//vincoli del problema quadratico 
	Matrix<double,102,45> A;
     
	//controlla i segni di kp e kd, controlla i segni della matrice A, verifica Jt1
	
	A<< M,-Jc_T_B,-S_T,Matrix<double,18,2>::Zero(), -b,
		B_T_Jc, Matrix<double,12,24>::Zero(),Matrix<double,12,2>::Zero(),-B.transpose()*Jcdqd,
		Matrix<double,20,18>::Zero(), Dfr, Matrix<double,20,15>::Zero(),
		Matrix<double,12,6>::Zero(), Eigen::Matrix<double,12,12>::Identity(), Matrix<double,12,26>::Zero(), ddqmax,
		Matrix<double,12,6>::Zero(), -Eigen::Matrix<double,12,12>::Identity(), Matrix<double,12,26>::Zero(), -ddqmin,
		Matrix<double,12,30>::Zero(), Eigen::Matrix<double,12,12>::Identity(), Matrix<double,12,2>::Zero(),tau_max,
		Matrix<double,12,30>::Zero(), -Eigen::Matrix<double,12,12>::Identity(), Matrix<double,12,2>::Zero(),-tau_min,
		Matrix<double,1,42>::Zero(), -m, 1, tnp,
		Matrix<double,1,42>::Zero(), -m, 1, tnn,
		Matrix<double,1,42>::Zero(), 1/m, 1, tnr,
		Matrix<double,1,42>::Zero(), 1/m, 1, tns;
    
	//cout<<"A:"<<endl;
	//cout<<A<<endl;

	c.setlength(102,45);
	//c.setcontent(98,43, &A(0,0));
	 for ( int i = 0; i < A.rows(); i++ ){
		   for ( int j = 0; j < A.cols(); j++ )
            c(i,j) = A(i,j);
       
    }
    
	
	

	//parametro che imposta la tipologia del vincolo 
	integer_1d_array ct;
	ct.setlength(102);
	for(int i = 0; i <30; i++){
		ct(i) = 0;
	}
	/*for(int i = 30; i<33; i++){
		ct(i) = 1;
	}*/
	//metti dis
	for(int i = 30; i<98; i++){
		ct(i) = -1;
	}
	ct(98)= -1;
	ct(99)= 1;
	ct(100)= -1;
	ct(101)= 1;
	//box constrain
	real_1d_array bndl;
	bndl.setlength(45);
	real_1d_array bndu;
	bndu.setlength(45);
	/*VectorXd qmin(12);
	VectorXd qmax(12);
	double dt=0.001;
	double Dt=10*dt;

    //limiti della posizione dei giunti
	qmin<<-1.7453,-1.7453,-1.7453,-1.7453,-3.14159265359,-0.02,-1.57079632679,-2.61795,-1.57079632679,-2.61795,-3.14159265359,-0.02;
	qmax<<1.7453, 1.7453, 1.7453, 1.7453, 1.57079632679, 2.61795, 3.14159265359, 0.02, 3.14159265359, 0.02, 1.57079632679, 2.61795;*/
	
	//Vincoli sulle accelerazioni ai giunti virtuali
	/*for(int i = 0; i<6; i++){
		bndl(i)=fp_neginf;//-INFINITY
		bndu(i)=fp_posinf;//+INFINITY
	}
	//Vincoli sulle accelerazioni
	for(int i = 6; i<18; i++){
		bndl(i)=(2/pow(Dt,2)) * (qmin(i-6)-q_joints_total(i)- Dt * dq_joints_total(i));// fp_neginf;//-INFINITY
		bndu(i)=(2/pow(Dt,2)) * (qmax(i-6)-q_joints_total(i)- Dt * dq_joints_total(i));// fp_posinf;//+INFINITY
	}
	//Vincoli sulle forze di contatto
	for(int i = 0; i<4; i++){
		// Componente x
		bndl(3*i+18)=fp_neginf;//-INFINITY
		bndu(3*i+18)=fp_posinf;//+INFINITY

		//Componente y
		bndl(3*i+19)=fp_neginf;//-INFINITY
		bndu(3*i+19)=fp_posinf;//+INFINITY

		//Componente z
		bndu(3*i+20)=fp_posinf;//+INFINITY
	}
	//Vincoli sulle coppie
	for(int i=30; i<42; i++){
		bndl(i)= -60;
		bndu(i)= 60;
	}
printf("%s\n", bndl.tostring(1).c_str());
	bndl(42)= 0;//-Infinity
	bndu(42)= fp_posinf;//+Infinity*/
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
	
    minqpcreate(44, state);
	minqpsetquadraticterm(state, a);
	minqpsetlinearterm(state, lineartermalglib); //non mi servono perchè di default sono già impostati a zero
	minqpsetlc(state, c, ct);
	//minqpsetbc(state, bndl, bndu);
	minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);
    minqpoptimize(state);

	minqpresults(state, x, rep);
    printf("%d\n", int(rep.terminationtype));
	cout<<"Variabili ottimizzate: ";
    printf("%s\n", x.tostring(1).c_str());
	cout<<endl;

    tau = { x(30),x(40),x(41),x(31),x(38),x(39),x(33),x(34),x(35),x(32),x(36),x(37)};
    
}



//get function

vector<double> PROB_QUAD_CP::getTau(){

    return tau;
}