#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/bind.hpp>


// void odomCallback(const nav_msgs::Odometry& msg) 
// {
// 	nav_msgs::Odometry msg_result;

// 	geometry_msgs::TransformStamped transformStamped;
// 	try
//  	{
//     	transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));

//     	// convert pose
//     	geometry_msgs::PoseStamped pose_in;
//     	pose_in.pose.position = msg.pose.pose.position;
//     	pose_in.pose.orientation = msg.pose.pose.orientation;

//     	geometry_msgs::PoseStamped pose_out;
//     	tf2::doTransform(pose_in, pose_out, transformStamped);

//     	msg_result.pose.pose.position = pose_out.pose.position;
//     	msg_result.pose.pose.orientation = pose_out.pose.orientation;

//     	// convert twist

//     	// publish odometry
//     	//pub.publish(msg_result);
//  	}
//  	catch (tf2::TransformException &ex) 
//  	{
//    		ROS_WARN("%s",ex.what());
//  	}
// }

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "odometry_transformer");
	ros::NodeHandle node;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

 	ros::Publisher pub = node.advertise<nav_msgs::Odometry>("odom", 10);
 	//ros::Subscriber sub = node.subscribe("source_odom", 10, boost::bind(&odomCallback, tfBuffer, pub, _1));

 	ros::spin();

	return 0;
}