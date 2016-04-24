#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h> 

#define PI 3.14
#define g 9.81


//  global parameters



ros::Publisher joint_state_pub;






typedef struct Task {
	float x;
	float z;
	float psi;
	float dx;
	float dz;
	float dpsi;
};
/////// avishai's code//////
// Links lengths
const float L1 = 0.264; //420e-3; //Second and third links
const float L2 = 0.258; //400e-3; //Fourth and fifth links
const float L3 = 0.3; //?????235.9e-3; //Sixth link and gripper

void InverseKinematics3R(Task G, float* q, float* dq) {
	//Inverse kinematics function
	float xt, zt, gamma, R;
	float L1n, L2n, b = 0, cosb2, sinb2;
	int i, sigma;
	const float a = 0.03;
	
	// Position
	for (i=1; i<=4; i++) {
		L1n = L1 + b;
		L2n = L2 + b;
		
		xt = G.x + L3*sin(G.psi);
		zt = G.z - L3*cos(G.psi);
		R = sqrt(xt*xt + zt*zt);
		sigma = -1;
		gamma = atan2(zt, -xt);
		q[0] = PI/2 - gamma + sigma*acos((R*R+L1n*L1n-L2n*L2n)/(2*L1n*R));
		cosb2 = (-R*R + L1n*L1n + L2n*L2n)/(2*L1n*L2n);
		sinb2 = sqrt(1 - cosb2*cosb2);
		q[1] = PI - PI/4 - atan2(sinb2, cosb2);
		
		b = a / tan(PI/2 - (q[1] + PI/4)/2);		
	}
	
	q[2] = G.psi - (q[0] + q[1] + PI/4);
	
	// Velocity
	dq[0] = (G.dx*(L2*cos(q[0] + q[1]) + L2*sin(q[0] + q[1]) - a*cos(q[0] + q[1]) + a*sin(q[0] + q[1])))/(a*a*cos(q[0] + q[1])*cos(q[0]) - a*a*cos(q[0] + q[1])*sin(q[0]) + a*a*sin(q[0] + q[1])*cos(q[0]) + a*a*sin(q[0] + q[1])*sin(q[0]) - L1*L2*cos(q[0] + q[1])*cos(q[0]) + L1*L2*cos(q[0] + q[1])*sin(q[0]) - L1*L2*sin(q[0] + q[1])*cos(q[0]) + L1*a*cos(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*cos(q[0]) - L1*L2*sin(q[0] + q[1])*sin(q[0]) + L1*a*cos(q[0] + q[1])*sin(q[0]) - L1*a*sin(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*sin(q[0]) - L2*a*sin(q[0] + q[1])*cos(q[0]) + L1*a*sin(q[0] + q[1])*sin(q[0]) + L2*a*sin(q[0] + q[1])*sin(q[0])) - (G.dz*(L2*cos(q[0] + q[1]) - L2*sin(q[0] + q[1]) + a*cos(q[0] + q[1]) + a*sin(q[0] + q[1])))/(a*a*cos(q[0] + q[1])*cos(q[0]) - a*a*cos(q[0] + q[1])*sin(q[0]) + a*a*sin(q[0] + q[1])*cos(q[0]) + a*a*sin(q[0] + q[1])*sin(q[0]) - L1*L2*cos(q[0] + q[1])*cos(q[0]) + L1*L2*cos(q[0] + q[1])*sin(q[0]) - L1*L2*sin(q[0] + q[1])*cos(q[0]) + L1*a*cos(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*cos(q[0]) - L1*L2*sin(q[0] + q[1])*sin(q[0]) + L1*a*cos(q[0] + q[1])*sin(q[0]) - L1*a*sin(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*sin(q[0]) - L2*a*sin(q[0] + q[1])*cos(q[0]) + L1*a*sin(q[0] + q[1])*sin(q[0]) + L2*a*sin(q[0] + q[1])*sin(q[0])) - (G.dpsi*(sqrt(2)*L2*L3*sin(q[0] + q[1] + q[2])*cos(q[0] + q[1]) - sqrt(2)*L2*L3*cos(q[0] + q[1] + q[2])*sin(q[0] + q[1]) + sqrt(2)*L3*a*cos(q[0] + q[1] + q[2])*cos(q[0] + q[1]) + sqrt(2)*L3*a*sin(q[0] + q[1] + q[2])*sin(q[0] + q[1])))/(a*a*cos(q[0] + q[1])*cos(q[0]) - a*a*cos(q[0] + q[1])*sin(q[0]) + a*a*sin(q[0] + q[1])*cos(q[0]) + a*a*sin(q[0] + q[1])*sin(q[0]) - L1*L2*cos(q[0] + q[1])*cos(q[0]) + L1*L2*cos(q[0] + q[1])*sin(q[0]) - L1*L2*sin(q[0] + q[1])*cos(q[0]) + L1*a*cos(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*cos(q[0]) - L1*L2*sin(q[0] + q[1])*sin(q[0]) + L1*a*cos(q[0] + q[1])*sin(q[0]) - L1*a*sin(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*sin(q[0]) - L2*a*sin(q[0] + q[1])*cos(q[0]) + L1*a*sin(q[0] + q[1])*sin(q[0]) + L2*a*sin(q[0] + q[1])*sin(q[0]));
	dq[1] = (G.dz*(2*L1*cos(q[0]) - 2*a*sin(q[0]) + sqrt(2)*L2*cos(q[0] + q[1]) - sqrt(2)*L2*sin(q[0] + q[1]) + sqrt(2)*a*cos(q[0] + q[1]) + sqrt(2)*a*sin(q[0] + q[1])))/(sqrt(2)*a*a*sin(q[0] + q[1])*cos(q[0]) - sqrt(2)*a*a*cos(q[0] + q[1])*sin(q[0]) + sqrt(2)*a*a*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*a*a*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*a*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*L2*a*sin(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*L2*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*L2*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*L2*sin(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*a*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L2*a*cos(q[0] + q[1])*cos(q[0]) - sqrt(2)*L1*L2*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*L1*a*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*a*sin(q[0] + q[1])*cos(q[0]) + sqrt(2)*L2*a*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L2*a*sin(q[0] + q[1])*cos(q[0])) - (G.dx*(2*L1*sin(q[0]) + 2*a*cos(q[0]) + sqrt(2)*L2*cos(q[0] + q[1]) + sqrt(2)*L2*sin(q[0] + q[1]) - sqrt(2)*a*cos(q[0] + q[1]) + sqrt(2)*a*sin(q[0] + q[1])))/(sqrt(2)*a*a*sin(q[0] + q[1])*cos(q[0]) - sqrt(2)*a*a*cos(q[0] + q[1])*sin(q[0]) + sqrt(2)*a*a*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*a*a*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*a*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*L2*a*sin(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*L2*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*L2*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*L2*sin(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*a*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L2*a*cos(q[0] + q[1])*cos(q[0]) - sqrt(2)*L1*L2*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*L1*a*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*a*sin(q[0] + q[1])*cos(q[0]) + sqrt(2)*L2*a*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L2*a*sin(q[0] + q[1])*cos(q[0])) + (G.dpsi*(L1*L3*sin(q[0] + q[1] + q[2])*sin(q[0]) - L3*a*cos(q[0] + q[1] + q[2])*sin(q[0]) + L3*a*sin(q[0] + q[1] + q[2])*cos(q[0]) - L3*a*sin(q[0] + q[1] + q[2])*sin(q[0]) + L1*L3*cos(q[0] + q[1] + q[2])*cos(q[0]) - L1*L3*cos(q[0] + q[1] + q[2])*sin(q[0]) + L1*L3*sin(q[0] + q[1] + q[2])*cos(q[0]) - L3*a*cos(q[0] + q[1] + q[2])*cos(q[0]) - sqrt(2)*L2*L3*cos(q[0] + q[1] + q[2])*sin(q[0] + q[1]) + sqrt(2)*L2*L3*sin(q[0] + q[1] + q[2])*cos(q[0] + q[1]) + sqrt(2)*L3*a*cos(q[0] + q[1] + q[2])*cos(q[0] + q[1]) + sqrt(2)*L3*a*sin(q[0] + q[1] + q[2])*sin(q[0] + q[1])))/(a*a*cos(q[0] + q[1])*cos(q[0]) - a*a*cos(q[0] + q[1])*sin(q[0]) + a*a*sin(q[0] + q[1])*cos(q[0]) + a*a*sin(q[0] + q[1])*sin(q[0]) - L1*L2*cos(q[0] + q[1])*cos(q[0]) + L1*L2*cos(q[0] + q[1])*sin(q[0]) - L1*L2*sin(q[0] + q[1])*cos(q[0]) + L1*a*cos(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*cos(q[0]) - L1*L2*sin(q[0] + q[1])*sin(q[0]) + L1*a*cos(q[0] + q[1])*sin(q[0]) - L1*a*sin(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*sin(q[0]) - L2*a*sin(q[0] + q[1])*cos(q[0]) + L1*a*sin(q[0] + q[1])*sin(q[0]) + L2*a*sin(q[0] + q[1])*sin(q[0]));
	dq[2] = (G.dx*(2*L1*sin(q[0]) + 2*a*cos(q[0])))/(sqrt(2)*a*a*sin(q[0] + q[1])*cos(q[0]) - sqrt(2)*a*a*cos(q[0] + q[1])*sin(q[0]) + sqrt(2)*a*a*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*a*a*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*a*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*L2*a*sin(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*L2*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*L2*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*L2*sin(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*a*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L2*a*cos(q[0] + q[1])*cos(q[0]) - sqrt(2)*L1*L2*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*L1*a*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*a*sin(q[0] + q[1])*cos(q[0]) + sqrt(2)*L2*a*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L2*a*sin(q[0] + q[1])*cos(q[0])) - (G.dz*(2*L1*cos(q[0]) - 2*a*sin(q[0])))/(sqrt(2)*a*a*sin(q[0] + q[1])*cos(q[0]) - sqrt(2)*a*a*cos(q[0] + q[1])*sin(q[0]) + sqrt(2)*a*a*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*a*a*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*a*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*L2*a*sin(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*L2*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*L2*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*L2*sin(q[0] + q[1])*cos(q[0]) + sqrt(2)*L1*a*cos(q[0] + q[1])*cos(q[0]) + sqrt(2)*L2*a*cos(q[0] + q[1])*cos(q[0]) - sqrt(2)*L1*L2*sin(q[0] + q[1])*sin(q[0]) + sqrt(2)*L1*a*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L1*a*sin(q[0] + q[1])*cos(q[0]) + sqrt(2)*L2*a*cos(q[0] + q[1])*sin(q[0]) - sqrt(2)*L2*a*sin(q[0] + q[1])*cos(q[0])) + (G.dpsi*(a*a*cos(q[0] + q[1])*cos(q[0]) - a*a*cos(q[0] + q[1])*sin(q[0]) + a*a*sin(q[0] + q[1])*cos(q[0]) + a*a*sin(q[0] + q[1])*sin(q[0]) - L1*L3*sin(q[0] + q[1] + q[2])*sin(q[0]) + L3*a*cos(q[0] + q[1] + q[2])*sin(q[0]) - L3*a*sin(q[0] + q[1] + q[2])*cos(q[0]) + L3*a*sin(q[0] + q[1] + q[2])*sin(q[0]) - L1*L2*cos(q[0] + q[1])*cos(q[0]) + L1*L2*cos(q[0] + q[1])*sin(q[0]) - L1*L2*sin(q[0] + q[1])*cos(q[0]) + L1*a*cos(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*cos(q[0]) - L1*L2*sin(q[0] + q[1])*sin(q[0]) + L1*a*cos(q[0] + q[1])*sin(q[0]) - L1*a*sin(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*sin(q[0]) - L2*a*sin(q[0] + q[1])*cos(q[0]) + L1*a*sin(q[0] + q[1])*sin(q[0]) + L2*a*sin(q[0] + q[1])*sin(q[0]) - L1*L3*cos(q[0] + q[1] + q[2])*cos(q[0]) + L1*L3*cos(q[0] + q[1] + q[2])*sin(q[0]) - L1*L3*sin(q[0] + q[1] + q[2])*cos(q[0]) + L3*a*cos(q[0] + q[1] + q[2])*cos(q[0])))/(a*a*cos(q[0] + q[1])*cos(q[0]) - a*a*cos(q[0] + q[1])*sin(q[0]) + a*a*sin(q[0] + q[1])*cos(q[0]) + a*a*sin(q[0] + q[1])*sin(q[0]) - L1*L2*cos(q[0] + q[1])*cos(q[0]) + L1*L2*cos(q[0] + q[1])*sin(q[0]) - L1*L2*sin(q[0] + q[1])*cos(q[0]) + L1*a*cos(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*cos(q[0]) - L1*L2*sin(q[0] + q[1])*sin(q[0]) + L1*a*cos(q[0] + q[1])*sin(q[0]) - L1*a*sin(q[0] + q[1])*cos(q[0]) + L2*a*cos(q[0] + q[1])*sin(q[0]) - L2*a*sin(q[0] + q[1])*cos(q[0]) + L1*a*sin(q[0] + q[1])*sin(q[0]) + L2*a*sin(q[0] + q[1])*sin(q[0]));
	/*xt = G.x - L3*sin(G.psi);
	zt = G.z - L3*cos(G.psi);
	gamma = atan2(zt, xt);
	R = sqrt(xt*xt + zt*zt);
	
	q[0] = PI/2 - (gamma + acos((R*R+L1*L1-L2*L2)/(2*L1*R)));
	q[1] = PI/2 - (atan2(zt-L1*cos(q[0]), xt-L1*sin(q[0])) + q[0]);
	q[2] = G.psi - q[0] - q[1];

	dq[0] = (G.dx*sin(q[0] + q[1]) - G.dz*cos(q[0] + q[1]) + L3*G.dpsi*sin(q[2]))/(L1*sin(q[1]));
	dq[1] = -(L2*G.dx*sin(q[0] + q[1]) - L2*G.dz*cos(q[0] + q[1]) - L1*G.dz*cos(q[0]) + L1*G.dx*sin(q[0]) + L1*L3*G.dpsi*sin(q[1] + q[2]) + L2*L3*G.dpsi*sin(q[2]))/(L1*L2*sin(q[1]));
	dq[2] = (G.dx*sin(q[0]) - G.dz*cos(q[0]) + L3*G.dpsi*sin(q[1] + q[2]) + L2*G.dpsi*sin(q[1]))/(L2*sin(q[1]));
	*/
}

float** TrajectoryGenerator(int N, float T, float xc, float z0, float vd) {
	// N - number of trajectory points
	// T - duratiohight_pubn of motion
	// xc - constant x coordinate to move along
	// z0 - initial z coordinate
	// vd - desired velocity after T 
	// motion along: z(t) = vd/(2T)*t^2 + z0

	int i, j;
	float dt = T/N, t = 0;
	Task G;
	float q[3];
	float dq[3];

	// Allocate trajectory matrix
	float** angles = (float**)malloc(N*sizeof(float*));
	for (i=0; i<N; i++)
		angles[i] = (float*)malloc(6*sizeof(float));

	G.x = xc; G.dx = 0;
	G.psi = PI/2; G.dpsi = 0;
	for (i=0; i<N; i++) {
		t += dt;
		G.z = vd/(2*T)*t*t + z0;
		G.dz = vd/T*t;
		InverseKinematics3R(G, q, dq);
		for (j=0; j<3; j++) {
			angles[i][j] = q[j];
			angles[i][j+3] = dq[j];
		}
	}
	return angles;
}




void move_arm(double x,double z,double psi,double dx,double dz,double dpsi)
{
	float q_inv[3];
	float dq_inv[3];
	Task target_position;
	
	target_position.x=x;
	target_position.z=z;
	target_position.psi=psi;
	target_position.dx=dx;
	target_position.dz=dz;
	target_position.dpsi=dpsi;
	
	sensor_msgs::JointState joint_state_msg;
	joint_state_msg.name.resize(6); 
	joint_state_msg.position.resize(6); 
	joint_state_msg.velocity.resize(6); 
	  
	InverseKinematics3R(target_position, q_inv,dq_inv);
	
	joint_state_msg.name[0]="arm_rotation1_joint";
	joint_state_msg.name[1]="arm_shoulder1_joint";
	joint_state_msg.name[2]="arm_shoulder2_joint";
	joint_state_msg.name[3]="arm_rotation2_joint";
	joint_state_msg.name[4]="arm_shoulder3_joint";
	joint_state_msg.name[5]="arm_wrist_joint";
	
	joint_state_msg.position[0]=0;
	joint_state_msg.position[1]=q_inv[0];
	joint_state_msg.position[2]=q_inv[1];
	joint_state_msg.position[3]=0;
	joint_state_msg.position[4]=q_inv[2];
	joint_state_msg.position[5]=0;
	
    joint_state_msg.velocity[0]=0;
    joint_state_msg.velocity[1]=dq_inv[0];
    joint_state_msg.velocity[2]=dq_inv[1];
    joint_state_msg.velocity[3]=0;
    joint_state_msg.velocity[4]=dq_inv[2];
    joint_state_msg.velocity[5]=0;
    
    joint_state_pub.publish(joint_state_msg);
    
}


void grip(double rate)
{
	
	
	
}





int main(int argc, char **argv)
{

  ros::init(argc, argv, "h_manipulator");
  ros::NodeHandle n;
  
  float** angles;

  
  joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_commands", 10000);

	

  ros::Rate loop_rate(30);







while(ros::ok()) {

  
		move_arm(0,0.3,PI/2,0.1,0.1,0.1);
		ros::spinOnce();
	}


  return 0;
}


