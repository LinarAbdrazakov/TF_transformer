#include <iostream>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

//#include <boost/bind.hpp>


tf2_ros::Buffer tfBuffer;

ros::Publisher pub;


void transform_to_matrix(const geometry_msgs::TransformStamped& transformStamped, Eigen::Matrix4d& matrix) 
{
	matrix.setIdentity();

	Eigen::Vector3d t;
	t.x() = transformStamped.transform.translation.x;
	t.y() = transformStamped.transform.translation.y;
	t.z() = transformStamped.transform.translation.z;

	Eigen::Quaterniond q;
	q.x() = transformStamped.transform.rotation.x;
	q.y() = transformStamped.transform.rotation.y;
	q.z() = transformStamped.transform.rotation.z;
	q.w() = transformStamped.transform.rotation.w;

	matrix.block(0,0,3,3) = q.normalized().toRotationMatrix();
	matrix.block(0,3,3,1) = t;
}


void pose_to_matrix(const geometry_msgs::Pose& pose, Eigen::Matrix4d& matrix) 
{
	matrix.setIdentity();

	Eigen::Vector3d t;
	t.x() = pose.position.x;
	t.y() = pose.position.y;
	t.z() = pose.position.z;

	Eigen::Quaterniond q;
	q.x() = pose.orientation.x;
	q.y() = pose.orientation.y;
	q.z() = pose.orientation.z;
	q.w() = pose.orientation.w;

	matrix.block(0,0,3,3) = q.normalized().toRotationMatrix();
	matrix.block(0,3,3,1) = t;
}


void matrix_to_pose(const Eigen::Matrix4d& matrix, geometry_msgs::Pose& pose) 
{
	pose.position.x = matrix(0, 3);
	pose.position.y = matrix(1, 3);
	pose.position.z = matrix(2, 3);

	Eigen::Matrix3d rot;
	rot = matrix.block(0, 0, 3, 3);
	Eigen::Quaterniond q(rot);

	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();
}


void transfrom_to_twist_matrix(const geometry_msgs::TransformStamped& transformStamped, Eigen::MatrixXd& matrix) 
{
	matrix.setIdentity();

	Eigen::Vector3d t;
	t.x() = transformStamped.transform.translation.x;
	t.y() = transformStamped.transform.translation.y;
	t.z() = transformStamped.transform.translation.z;

	Eigen::Matrix3d t_hat;
	t_hat << 0, -t(2), t(1),
			 t(2), 0, -t(0),
    		-t(1), t(0), 0;

	Eigen::Quaterniond q;
	q.x() = transformStamped.transform.rotation.x;
	q.y() = transformStamped.transform.rotation.y;
	q.z() = transformStamped.transform.rotation.z;
	q.w() = transformStamped.transform.rotation.w;

	Eigen::Matrix3d rot = q.normalized().toRotationMatrix().transpose();
	matrix.block(0,0,3,3) = rot;
	matrix.block(3,3,3,3) = rot;
	matrix.block(0,3,3,3) = t_hat * rot;
}


void twist_to_vector(const geometry_msgs::Twist& twist, Eigen::VectorXd& twist_vector) 
{
	twist_vector(0) = twist.linear.x;
	twist_vector(1) = twist.linear.y;
	twist_vector(2) = twist.linear.z;

	twist_vector(3) = twist.angular.x;
	twist_vector(4) = twist.angular.y;
	twist_vector(5) = twist.angular.z;
}


void vector_to_twist(const Eigen::VectorXd& twist_vector, geometry_msgs::Twist& twist) 
{
	twist.linear.x = twist_vector(0);
	twist.linear.y = twist_vector(1);
	twist.linear.z = twist_vector(2);

	twist.angular.x = twist_vector(3);
	twist.angular.y = twist_vector(4);
	twist.angular.z = twist_vector(5);
}


void odomCallback(const nav_msgs::Odometry& msg) 
{
	nav_msgs::Odometry msg_result;

	std::string from_frame, to_frame;
	ros::param::get("from_frame", from_frame);
	ros::param::get("to_frame", to_frame);

	geometry_msgs::TransformStamped transformStamped;
	try
 	{
    	transformStamped = tfBuffer.lookupTransform(from_frame, to_frame, ros::Time(0));

    	// convert transform to matrix 
    	Eigen::Matrix4d transfrom_matrix;
    	transform_to_matrix(transformStamped, transfrom_matrix);

    	// convert pose to matrix
    	Eigen::Matrix4d pose_matrix;
    	pose_to_matrix(msg.pose.pose, pose_matrix);

    	// apply transfrom on pose
    	Eigen::Matrix4d pose_matrix_result;
    	pose_matrix_result = transfrom_matrix.transpose() * pose_matrix * transfrom_matrix;
    	matrix_to_pose(pose_matrix_result, msg_result.pose.pose);

    	// convert transform to twist_transfrom
    	Eigen::MatrixXd twist_transfrom(6, 6);
    	transfrom_to_twist_matrix(transformStamped, twist_transfrom);

    	// convert twist to vector
    	Eigen::VectorXd twist_vector(6);
    	twist_to_vector(msg.twist.twist, twist_vector);

    	// apply transfrom on twist
    	Eigen::VectorXd twist_vector_result(6);
    	twist_vector_result = twist_transfrom * twist_vector;
    	vector_to_twist(twist_vector_result, msg_result.twist.twist);

    	// copy header
    	msg_result.header = msg.header;
    	msg_result.header.frame_id = "odom";
    	msg_result.child_frame_id = to_frame;

    	// publish odometry
    	pub.publish(msg_result);
 	}
 	catch (tf2::TransformException &ex) 
 	{
   		ROS_WARN("%s",ex.what());
 	}
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "odometry_transformer");
	ros::NodeHandle node;

	ros::param::set("from_frame", "zed_left_camera_optical_frame");
	ros::param::set("to_frame", "base_link");

	tf2_ros::TransformListener tfListener(tfBuffer);

 	pub = node.advertise<nav_msgs::Odometry>("/visual_odom", 10);
 	ros::Subscriber sub = node.subscribe("/OpenVSLAM/odom", 10, odomCallback);

 	ros::spin();

	return 0;
}
