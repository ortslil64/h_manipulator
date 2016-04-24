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
ros::Publisher right_gripper_pub;
ros::Publisher left_gripper_pub;




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

	// ROS_INFO("q1=%lf  q2=%lf   q3=%lf   z=%lf",q[0],q[1],q[2],G.z);
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

float** TrajectoryGenerator_xyz(int N, float T, float xc, float z0, float vd) {
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
		angles[i] = (float*)malloc(2*sizeof(float));

	G.x = xc; G.dx = 0;
	G.psi = PI/2; G.dpsi = 0;
	for (i=0; i<N; i++) {
		t += dt;
		
G.z = vd/(2*T)*t*t + z0;
		G.dz = vd/T*t;
	
		angles[i][0] = G.z;
		angles[i][1] = G.dz;
		
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
    ros::spinOnce();
}


void move_angles(double q1,double q2,double q3,double dq1,double dq2,double dq3)
{
	
	
	
	sensor_msgs::JointState joint_state_msg;
	joint_state_msg.name.resize(6); 
	joint_state_msg.position.resize(6); 
	joint_state_msg.velocity.resize(6); 
	  
	
	
	joint_state_msg.name[0]="arm_rotation1_joint";
	joint_state_msg.name[1]="arm_shoulder1_joint";
	joint_state_msg.name[2]="arm_shoulder2_joint";
	joint_state_msg.name[3]="arm_rotation2_joint";
	joint_state_msg.name[4]="arm_shoulder3_joint";
	joint_state_msg.name[5]="arm_wrist_joint";
	
	joint_state_msg.position[0]=0;
	joint_state_msg.position[1]=q1;
	joint_state_msg.position[2]=q2;
	joint_state_msg.position[3]=0;
	joint_state_msg.position[4]=q3;
	joint_state_msg.position[5]=0;
	
    joint_state_msg.velocity[0]=0;
    joint_state_msg.velocity[1]=dq1;
    joint_state_msg.velocity[2]=dq2;
    joint_state_msg.velocity[3]=0;
    joint_state_msg.velocity[4]=dq3;
    joint_state_msg.velocity[5]=0;
    
    joint_state_pub.publish(joint_state_msg);
    ros::spinOnce();
  //  ROS_INFO("q1=%lf  q2=%lf   q3=%lf",q1,q2,q3);
}

void grip(double rate)
{
	std_msgs::Float64 right_gripper_msg;
	std_msgs::Float64 left_gripper_msg;
	left_gripper_msg.data=-rate;
	right_gripper_msg.data=rate;
	right_gripper_pub.publish(right_gripper_msg);
	left_gripper_pub.publish(left_gripper_msg);
	ros::spinOnce();
}





int main(int argc, char **argv)
{

  ros::init(argc, argv, "h_manipulator");
  ros::NodeHandle n;
  int i;
  float N=65;
  
  double dt = 0.05;
  
  float T=dt*N;
  float** angles;
  float** angles_xyz;
  
  joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_commands", 100);
  right_gripper_pub = n.advertise<std_msgs::Float64>("/right_controller/command", 100);
  left_gripper_pub = n.advertise<std_msgs::Float64>("/left_controller/command", 100);

  ros::Rate loop_rate(150);
	ros::Duration pause_loop(0.01);
	ros::Duration pause_loop2(dt);

//angles_xyz=TrajectoryGenerator_xyz((int)N, T, -0.5, 0,1);
angles=TrajectoryGenerator((int)N, T, -0.4, 0.13,35	);

angles[(int)N-1][3]=0;
angles[(int)N-1][4]=0;
angles[(int)N-1][5]=0;
	



for(i=0;i<500;i++)
{
	move_arm(-0.4,0.13,PI/2,0.3,0.3,0.2);
	pause_loop.sleep();
	grip(-0.05);
	
}
i=0;
	

while(ros::ok()) {
for(i;i<N-1;i++)
{

move_angles(angles[i][0],angles[i][1],angles[i][2],0,0,0);

if(i<6) grip(-0.05);
if(i>6	 && i<10) grip(0.02);
if(i>10) grip(-0.05);
pause_loop2.sleep();
}
move_angles(angles[i][0],angles[i][1],angles[i][2],angles[i][3],angles[i][4],angles[i][5]);
loop_rate.sleep();
}		
	
	


  return 0;
}


