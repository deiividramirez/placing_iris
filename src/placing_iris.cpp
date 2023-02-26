/*
Intel (Zapopan, Jal), Robotics Lab (CIMAT, Gto), Patricia Tavares & Gerardo Rodriguez.
November 20th, 2017

Modified by: David Leonardo Ramírez 2023 -> david.parada@cimat.mx
*/

/******************************************************* ROS libraries*/
#include <ros/ros.h>
#include <mav_msgs/conversions.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

/****************************************************** c++ libraries */
#include <random>
#include <vector>
#include <string>
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <opencv2/core.hpp>

using namespace std;

string workspace = WORKSPACE;

void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void writeFile(vector<float> &vec, const string &name);

geometry_msgs::PointStamped pos_msg;
double oYaw;

ros::Publisher pos_pub;
ros::Subscriber pos_sub;

vector<float> error_vec;
vector<float> x;
vector<float> y;
vector<float> z;
vector<float> yaw;

Eigen::VectorXd desired, actual_pos;
float X, Y, Z, Yaw, error;
int conteo = 0;

/* Main function */
int main(int argc, char **argv)
{

	/***************************************************************************************** INIT */
	ros::init(argc, argv, "placing_iris");
	ros::NodeHandle nh;

	string slash("/");
	string name(argv[1]);
	string publish("/command/trajectory");
	// string subscribe("/ground_truth/desired");
	string subscribe("/ground_truth/pose");
	string topic_pub = slash + name + publish;
	string topic_sub = slash + name + subscribe;

	std::cout << "Publishing to: " << topic_pub << std::endl;
	std::cout << "Subscribing to: " << topic_sub << std::endl;

	/************************************************************* CREATING PUBLISHER AND SUBSCRIBER*/
	pos_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topic_pub, 1);
	pos_sub = nh.subscribe<geometry_msgs::Pose>(topic_sub, 1, poseCallback, ros::TransportHints().tcpNoDelay());
	/* ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PointStamped>(topic_sub, 1, positionCallback); */
	ros::Rate rate(20);

	/************************************************************* DEFINING THE POSE */
	std::random_device rd;														 // obtain a random number from hardware
	std::mt19937 gen(rd());														 // seed the generator
	std::uniform_real_distribution<> disXY(-3.0, 3.0); // define the range for X and Y
	std::uniform_real_distribution<> disZ(0.0, 3.0);	 // define the range for Z
	std::uniform_real_distribution<> disYaw(-2, 2);		 // define the range yaw

	X = argc > 2 ? atof(argv[2]) : disXY(gen);		// if x coordinate has been given
	Y = argc > 3 ? atof(argv[3]) : disXY(gen);		// if y coordinate has been given
	Z = argc > 4 ? atof(argv[4]) : disZ(gen);			// if z coordinate has been given
	Yaw = argc > 5 ? atof(argv[5]) : disYaw(gen); // if yaw has been given

	vector<float> params = {X, Y, Z, Yaw};
	writeFile(params, workspace + "/src/placing_iris/placing_iris_params.txt");

	desired.resize(3);
	// desired(0) = X;
	desired(0) = (X - 0.127794);
	desired(1) = Y;
	desired(2) = Z;

	/******************************************************************************* BUCLE START*/
	while (ros::ok())
	{
		// get a msg
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

/*
	function: positionCallback
	description: gets the position of the drone and assigns it to the variable point_msg.
	params: ptr to msg.
*/
void poseCallback(const geometry_msgs::Pose::ConstPtr &UAV)
{
	pos_msg.point.x = UAV->position.x;
	pos_msg.point.y = UAV->position.y;
	pos_msg.point.z = UAV->position.z;

	tf::Quaternion q(UAV->orientation.x, UAV->orientation.y, UAV->orientation.z, UAV->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double actual_roll, actual_pitch, actual_yaw;
	mat.getEulerYPR(actual_yaw, actual_pitch, actual_roll);

	oYaw = actual_yaw;

	actual_pos = Eigen::Vector3d(pos_msg.point.x, pos_msg.point.y, pos_msg.point.z);

	// create message for the pose
	trajectory_msgs::MultiDOFJointTrajectory msg;
	// prepare msg
	msg.header.stamp = ros::Time::now();

	Eigen::Vector3d error_pos = desired - actual_pos;
	error = error_pos.norm() - 0.127794;

	cout << "error: " << error << endl;
	cout << "error_pos: " << error_pos << endl;

	if (error > 2)
	{
		int i = (int) ((error + 1)/2);
		// for (int index = 1; index <= i; index++)
		// {
		Eigen::Vector3d position_temp = (1.0-1.0/error)*actual_pos + (1.0/error)*desired;
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position_temp, Yaw, &msg);
		// }
	}
	else
	{
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired, Yaw, &msg);
	}
	// publish
	pos_pub.publish(msg);
	// verify if it's over

	/* std::cout << "Error: " << error << std::endl;
	std::cout << "=> X: " << pos_msg.point.x << " Y: " << pos_msg.point.y << " Z: " << pos_msg.point.z << std::endl;
	std::cout << ">> X: " << X << " Y: " << Y << " Z: " << Z << std::endl
						<< std::endl; */

	std::cout << "COORDINATES || DESIRED -> REAL -> ERROR" << std::endl;
	std::cout << "X || " << X << " -> " << pos_msg.point.x << " -> " << X - pos_msg.point.x << std::endl;
	std::cout << "Y || " << Y << " -> " << pos_msg.point.y << " -> " << Y - pos_msg.point.y << std::endl;
	std::cout << "Z || " << Z << " -> " << pos_msg.point.z << " -> " << Z - pos_msg.point.z << std::endl;
	std::cout << "Yaw || " << Yaw << " -> " << oYaw << " -> " << Yaw - oYaw << std::endl;
	std::cout << "Error global: " << error << std::endl
						<< std::endl;

	error_vec.push_back(error);
	x.push_back(pos_msg.point.x);
	y.push_back(pos_msg.point.y);
	z.push_back(pos_msg.point.z);
	yaw.push_back(oYaw);

	if (error < 0.01 || conteo++ > 250)
	{
		writeFile(error_vec, workspace + "/src/placing_iris/placing_iris_error.txt");
		writeFile(x, workspace + "/src/placing_iris/placing_iris_x.txt");
		writeFile(y, workspace + "/src/placing_iris/placing_iris_y.txt");
		writeFile(z, workspace + "/src/placing_iris/placing_iris_z.txt");
		writeFile(yaw, workspace + "/src/placing_iris/placing_iris_yaw.txt");

		/* std::cout << "Pose desired of the drone ==> " << endl
							<< "X: " << X << ", Y: " << Y << ", Z: " << Z << ", Yaw: " << Yaw << endl;

		std::cout << "Real pose of the drone <<== " << endl
							<< "X: " << pos_msg.point.x << ", Y: " << pos_msg.point.y << ", Z: " << pos_msg.point.z << ", Yaw: " << Yaw << endl; */

		std::cout << "COORDINATES || DESIRED -> REAL -> ERROR" << std::endl;
		std::cout << "X || " << X << " -> " << pos_msg.point.x << " -> " << X - pos_msg.point.x << std::endl;
		std::cout << "Y || " << Y << " -> " << pos_msg.point.y << " -> " << Y - pos_msg.point.y << std::endl;
		std::cout << "Z || " << Z << " -> " << pos_msg.point.z << " -> " << Z - pos_msg.point.z << std::endl;
		std::cout << "Yaw || " << Yaw << " -> " << oYaw << " -> " << Yaw - oYaw << std::endl;
		std::cout << "Error global: " << error << std::endl
							<< std::endl;
		ros::shutdown();
	}

	/* std::cout << "P: " << msg->point.x << " - " << msg->point.y << " - " << msg->point.z << std::endl; */
}

// void writeMatrix(cv::Mat &mat, const string &name)
// {
// 	ofstream myfile;
// 	myfile.open(name);
// 	for (int i = 0; i < mat.rows; i++)
// 	{
// 		for (int j = 0; j < mat.cols; j++)
// 		{
// 			myfile << mat.at<float>(i, j) << " ";
// 		}
// 		myfile << endl;
// 	}
// 	myfile.close();
// }

void writeFile(vector<float> &vec, const string &name)
{
	ofstream myfile;
	myfile.open(name);
	for (int i = 0; i < vec.size(); i++)
		myfile << vec[i] << endl;
	myfile.close();
}