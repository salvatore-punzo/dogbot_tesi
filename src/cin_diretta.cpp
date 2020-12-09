#include "cin_diretta.h"

CIN_DIR::CIN_DIR(){

};


void CIN_DIR::calculate_leg_length_cb(VectorXd &hp, VectorXd &kp){
	
	
    
    float fx;
    float fz;
    string l[]={"bl","br","fl","fr"};
    
    for (int i = 0;i<4;i++){
        //hp[i] = js->position[1 +3*i];
        //kp[i] = js->position[3*i];
        
        fx = a1 * cos (hp[i]) + a2 * cos(hp[i] + kp[i]);
	    fz = a1 * sin (hp[i]) + a2 * sin(hp[i] + kp[i]);

        rl(i) = sqrt(pow(fx,2)+pow(fz,2));
	    //cout<<"lunghezza leg "<<l[i]<<":"<<rl(i) <<endl;    


    }
}

// return leg length
    
    VectorXd CIN_DIR::getLegLength(){

	VectorXd legLength = rl;
	return legLength;
    }
    

