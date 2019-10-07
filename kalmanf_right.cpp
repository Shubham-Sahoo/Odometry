#include <iostream>
#include<fstream>
#include<stdlib.h>
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <sstream>
using Eigen::MatrixXf;
using Eigen::VectorXf;
using namespace std;


MatrixXf G(3,1);    //(3,1)
MatrixXf G_meas(3,1);   //3,1
MatrixXf b(3,1);   //3,1
MatrixXf H(1,3);   //1,3
MatrixXf R(3,3);
MatrixXf P(3,3);
MatrixXf sigma(3,3);
MatrixXf Z(3,1);   //3,1
MatrixXf Y(3,1);   //3,1
MatrixXf S(3,3);  	
MatrixXf K(3,3);
MatrixXf I(3,3);
MatrixXf Gnew(3,3);
MatrixXf Hnew(3,3);

struct data_imu{
	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;

};
data_imu gval;

std_msgs::Float32 bn;


void kalman(data_imu data_gyro)
{
	float gx,gy,gz;
	int i,j;
  	gx = data_gyro.angular_velocity_x;//rand()%100;
	gy = data_gyro.angular_velocity_y;//rand()%100;
	gz = data_gyro.angular_velocity_z;//rand()%100;
	cout<<"gx :"<<gx<<"\n";
 	
  	//v and epsilon are to be added...


  	G <<gx, gy, gz;
	b<< 0,
		0,
		0;
	P<< 0.0282,0,0,
		0,0.0247,0,
		0,0,0.0328;
	R<< 0.0282,0,0,
		0,0.0247,0,
		0,0,0.0328;
	sigma<< 0.0282,0,0,
			0,0.0247,0,
			0,0,0.0328;
	I<< 1,0,0,
		0,1,0,
		0,0,1;		


	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			Hnew(i,j)=H(0,j);
		}
	}	
	//Hnew<<1,1,1,1,1,1,1,1,1;	
	G_meas = G-b; // G = G_true + epsilon, where epsilon is the gyroscope noise			

	H = 2.0*(G_meas-b).transpose();
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			Gnew(i,j)=G_meas(i,0)-b(i,0);
		}
	}
	//Gnew<<0,0,0,0,0,0,0,0,0;

	R = 4.0*(((Gnew).cwiseProduct(sigma.cwiseProduct(Gnew.transpose()))) + 2.0*(sigma.cwiseProduct(sigma)));
	Z = G_meas.cwiseProduct(G_meas);  //np.linalg.det
	Y = (Z.array() - (H*b)(0,0)).matrix();
	S = Hnew.cwiseProduct(P.cwiseProduct(Hnew.transpose())) + R;
	K = P.cwiseProduct(Hnew.transpose().cwiseProduct(S.inverse()));
	b = b + K*Y; // try dot K.dot(Y)
	P = (I - K.cwiseProduct(Hnew)).cwiseProduct(P);
	cout<<"Bias : "<<b<<"\n";
	bn.data=b(2,0);

}


void right(const geometry_msgs::Twist& msg)
{
   gval.angular_velocity_x=msg.angular.x;
   gval.angular_velocity_y=msg.angular.y;
   gval.angular_velocity_z=msg.angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kalman_left");
	ros::NodeHandle n;
	
	ros::Subscriber sub1 = n.subscribe("/enc_right",1000,&right);
	ros::Publisher pub1 = n.advertise<std_msgs::Float32>("bias_rz", 1);
	while(ros::ok())
	{	
		pub1.publish(bn);
		ros::spinOnce();
	}
	 
	ros::spin();
	return 0;


}

