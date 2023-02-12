/*
Intel (Zapopan, Jal), Robotics Lab (CIMAT, Gto), Patricia Tavares & Gerardo Rodriguez.
November 20th, 2017
This ROS code is used to connect rotors_simulator hummingbird's data and
move it to a desired pose.
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

void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void writeFile(vector<float> &vec, const string &name);

geometry_msgs::PointStamped pos_msg;
double oYaw;

/* Main function */
int main(int argc, char **argv)
{

	/***************************************************************************************** INIT */
	ros::init(argc, argv, "placing_iris");
	ros::NodeHandle nh;

	// if (argc < 2)
	// return 0; // if no name was given, nothing to do here-
	string slash("/");
	string name(argv[1]);
	// string name("iris");
	string publish("/command/trajectory");
	// string subscribe("/ground_truth/position");
	string subscribe("/ground_truth/pose");
	string topic_pub = slash + name + publish;
	string topic_sub = slash + name + subscribe;

	std::cout << "Publishing to: " << topic_pub << std::endl;
	std::cout << "Subscribing to: " << topic_sub << std::endl;

	/************************************************************* CREATING PUBLISHER AND SUBSCRIBER*/
	ros::Publisher pos_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topic_pub, 1);
	ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::Pose>(topic_sub, 1, poseCallback, ros::TransportHints().tcpNoDelay());
	/* ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PointStamped>(topic_sub, 1, positionCallback); */
	ros::Rate rate(20);

	/************************************************************* DEFINING THE POSE */
	std::random_device rd;														 // obtain a random number from hardware
	std::mt19937 gen(rd());														 // seed the generator
	std::uniform_real_distribution<> disXY(-3.0, 3.0); // define the range for X and Y
	std::uniform_real_distribution<> disZ(0.0, 3.0);	 // define the range for Z
	std::uniform_real_distribution<> disYaw(-2, 2);		 // define the range yaw

	float X, Y, Z, Yaw, error;
	X = argc > 2 ? atof(argv[2]) : disXY(gen);		// if x coordinate has been given
	Y = argc > 3 ? atof(argv[3]) : disXY(gen);		// if y coordinate has been given
	Z = argc > 4 ? atof(argv[4]) : disZ(gen);			// if z coordinate has been given
	Yaw = argc > 5 ? atof(argv[5]) : disYaw(gen); // if yaw has been given

	vector<float> error_vec;
	vector<float> x;
	vector<float> y;
	vector<float> z;
	vector<float> yaw;
	vector<float> params = {X, Y, Z, Yaw};

	Eigen::VectorXd position;
	position.resize(3);
	// position(0) = X;
	position(0) = X - 0.124;
	position(1) = Y;
	position(2) = Z;

	int conteo = 0;

	/******************************************************************************* BUCLE START*/
	while (ros::ok())
	{
		// get a msg
		ros::spinOnce();
		// create message for the pose
		trajectory_msgs::MultiDOFJointTrajectory msg;

		// prepare msg
		msg.header.stamp = ros::Time::now();
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position, Yaw, &msg);
		// publish
		pos_pub.publish(msg);
		// verify if it's over
		error = (pos_msg.point.x - X) * (pos_msg.point.x - X) + (pos_msg.point.y - Y) * (pos_msg.point.y - Y) + (pos_msg.point.z - Z) * (pos_msg.point.z - Z);

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
		yaw.push_back(Yaw);

		// cout << pos_msg.orientation.x << endl;

		if (error < 0.01 || conteo++ > 250)
		{
			break;
		}

		rate.sleep();
	}

	writeFile(error_vec, "placing_iris_error.txt");
	writeFile(x, "placing_iris_x.txt");
	writeFile(y, "placing_iris_y.txt");
	writeFile(z, "placing_iris_z.txt");
	writeFile(yaw, "placing_iris_yaw.txt");
	writeFile(params, "placing_iris_params.txt");

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

	return 0;
}

/*
	function: positionCallback
	description: gets the position of the drone and assigns it to the variable point_msg.
	params: ptr to msg.
*/
void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	pos_msg.point.x = msg->position.x;
	pos_msg.point.y = msg->position.y;
	pos_msg.point.z = msg->position.z;

	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);

	oYaw = yaw;

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