#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdlib.h>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>


#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"

using namespace Eigen;
using namespace std;

class CIN_DIR{
    public:
        CIN_DIR();
        void calculate_leg_length_cb(VectorXd &hp, VectorXd &kp);

        // get function
        VectorXd getLegLength();
    
    private:
    //dimensione dei link
		float a1=0.315;
		float a2=0.315;
        Matrix<double,4,1> rl;
        
	
};