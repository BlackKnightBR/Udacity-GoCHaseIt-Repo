#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

//ROS::Publisher motor commands
ros::Publisher motor_command_publisher;

//Callback function para drive_robot
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){

	ROS_INFO("Definindo nova posição - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

	geometry_msgs::Twist motor_commands;
	motor_commands.linear.x = req.linear_x;
	motor_commands.angular.z = req.angular_z;

	motor_command_publisher.publish(motor_commands);

	res.msg_feedback = "Robot move - linear,x: " + std::to_string(motor_commands.linear.x) + ", angular,z: " + std::to_string(motor_commands.angular.z);
	ROS_INFO_STREAM(res.msg_feedback);

	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "drive_bot");
	ros::NodeHandle n;
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
	ROS_INFO("Ready to send motor commands");

	ros::spin();

	return 0;

}
