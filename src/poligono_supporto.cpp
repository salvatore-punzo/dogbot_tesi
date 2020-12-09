#include "poligono_supporto.h"

using namespace std;

POLI_SUP::POLI_SUP(){

};

void POLI_SUP::calcoloPoligonoSupporto(VectorXd &ll, VectorXd &hp, VectorXd &he, VectorXd &kp, VectorXd &rp, Vector3d &eef_bl, Vector3d &eef_br,Vector3d &eef_fl, Vector3d &eef_fr, Vector3d &coo_ee_bl, Vector3d &coo_ee_br, Vector3d &coo_ee_fl, Vector3d &coo_ee_fr, Matrix3d &rot_world_virtual_base){

    //cout<<"prova"<<endl;
    sele_z<<0,0,1;
    //tutti i vettori: ll, hp ,he, kp.. contengono i dati nel seguente ordine: bl,br,fl,fr
    //Vettori rotazioni che mi servono per portare il vettore forza dal sistema di riferimento dell'ee a quello della floating base
    rot_knee_br << 1, 0, 0,
                0, cos(- kp(1)), -sin(- kp(1)),
                0, sin( kp(1)), cos(- kp(1));

    rot_pitch_br << 1, 0, 0,
                0, cos(hp(1)), -sin(hp(1)),
                0, sin(hp(1)), cos(hp(1));
        

        
    rot_roll_br << cos(- rp(1)), 0, sin(- rp(1)),
                0, 1, 0,
                - sin(- rp(1)), 0, cos(-rp(1));

                   rot_knee_br << 1, 0, 0,
                0, cos(- kp(1)), -sin(- kp(1)),
                0, sin( kp(1)), cos(- kp(1));

    rot_knee_bl << 1, 0, 0,
                0, cos(- kp(0)), -sin(- kp(0)),
                0, sin( kp(0)), cos(- kp(0));

    rot_pitch_bl << 1, 0, 0,
                0, cos(hp(0)), -sin(hp(0)),
                0, sin(hp(0)), cos(hp(0));
        

        
    rot_roll_bl << cos(- rp(0)), 0, sin(- rp(0)),
                0, 1, 0,
                - sin(- rp(0)), 0, cos(-rp(0));

//----------------------------------------------------------------
    eef_bl_wc = rot_world_virtual_base * rot_roll_bl * rot_pitch_bl * rot_knee_bl * eef_bl;
    eef_br_wc = rot_world_virtual_base * rot_roll_br * rot_pitch_br * rot_knee_br * eef_br;

//rot_world_virtual_base invece mi serve per portare il vettore dal sistema di riferimento della floating base a quello del sistema mondo
	
//sono i parametri della gamba destra del bipede

    _len_leg_blfr = ( ll(0) + ll(3) )/2;
     _hip_j_blfr = ( hp(0) + hp(3) )/2;
     _hip_eff_blfr = he(0) + he(3); //he è la coppia applicata all'hip joint
     _ee_f_vb_rl = 2 * eef_bl_wc; 

//sono i parametri della gamba sinistra del bipede

    _len_leg_brfl = ( ll(1) + ll(2) )/2;
    _hip_j_brfl = ( hp(1) + hp(2) )/2;
    _hip_eff_brfl = he(1) + he(2);
	_ee_f_vb_ll = 2 * eef_br_wc; 
	
//calcolo posizioni dei piedi del virtual biped
/*
	xc_vb_rl = (coo_ee_bl(0) + coo_ee_fr(0))/2; 
	yc_vb_rl = (coo_ee_bl(1) + coo_ee_fr(1))/2;

	xc_vb_ll = (coo_ee_br(0) + coo_ee_fl(0))/2;
	yc_vb_ll = (coo_ee_br(1) + coo_ee_fl(1))/2;

	cout<<"coordinata x del piede destro del virtual biped: "<<xc_vb_rl<<endl;
	cout<<"coordinata y del piede destro del virtual biped: "<<yc_vb_rl<<endl;
	cout<<"coordinata x del piede sinistro del virtual biped: "<<xc_vb_ll<<endl;
	cout<<"coordinata y del piede sinistro del virtual biped: "<<yc_vb_ll<<endl;
*/
	
//Calcolo componente z della forza 
	eef_vb_rlz =_ee_f_vb_rl.transpose() * sele_z;
	eef_vb_llz =_ee_f_vb_ll.transpose() * sele_z;

	cop_x = (eef_vb_rlz * xc_vb_rl + eef_vb_llz * xc_vb_ll)/(eef_vb_rlz+ eef_vb_llz);
	cop_y = (eef_vb_rlz * yc_vb_rl + eef_vb_llz * yc_vb_ll)/(eef_vb_rlz+ eef_vb_llz);
/*
	cout<<"copx: "<<cop_x<<endl; //TO DO: verifica se i COP cosí calcolati vanno bene o se li devi calcolare con l'altra formula vedi paper
	cout<<"copy: "<<cop_y<<endl;
*/

}

//GET FUNCTION