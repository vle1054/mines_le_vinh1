#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <std_srvs/Empty.h>
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double PI = 3.141592653589793238462643383279;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool cloclwise);
double degrees2radians(double angle_in_degrees);
double setDesiredOrientation(double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
double getDistance(double x1, double y1, double x2, double y2);
void moveto(double xcord, double ycord, double distance_tolerance);

int main(int argc, char **argv)
{

								ros::init(argc, argv, "mines_le_vinh");
								ros::NodeHandle n;
								double speed, angular_speed;
								double distance, angle;
								bool isForward, clockwise;

								velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
								pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback); //call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
								ros::Rate loop_rate(0.5);
								ros::ServiceClient clear = n.serviceClient<std_srvs::Empty>("clear");
								std_srvs::Empty empty;

								ROS_INFO("\n\n\n ********MOVING TO STARTING POINT*********\n");

								while (ros::ok()) {

																double scale = .8;
																turtlesim::Pose goal_pose;
																goal_pose.x = 1+1;
																goal_pose.y = 1+1;
																goal_pose.theta = 0;
																moveGoal(goal_pose, 0.01);
																loop_rate.sleep();
																moveto(1*scale,1*scale,.01);
																clear.call(empty);

																ROS_INFO("\n\n\n ********DRAWING M*********\n");

																moveto(1*scale, 3*scale, 0.01);
																moveto(2*scale, 3*scale, 0.01);
																moveto(2*scale, 8*scale, 0.01);
																moveto(1*scale, 8*scale, 0.01);
																moveto(1*scale, 10*scale, 0.01);
																moveto(4*scale, 10*scale, 0.01);
																moveto(7*scale, 7*scale, 0.01);
																moveto(10*scale, 10*scale, 0.01);
																moveto(13*scale, 10*scale, 0.01);
																moveto(13*scale, 8*scale, 0.01);
																moveto(12*scale, 8*scale, 0.01);
																moveto(12*scale, 3*scale, 0.01);
																moveto(13*scale, 3*scale, 0.01);
																moveto(13*scale, 1*scale, 0.01);
																moveto(9*scale, 1*scale, 0.01);
																moveto(9*scale, 3*scale, 0.01);
																moveto(10*scale, 3*scale, 0.01);
																moveto(10*scale, 7*scale, 0.01);
																moveto(7*scale, 3*scale, 0.01);
																moveto(4*scale, 7*scale, 0.01);
																moveto(4*scale, 3*scale, 0.01);
																moveto(5*scale, 3*scale, 0.01);
																moveto(5*scale, 1*scale, 0.01);
																moveto(1*scale, 1*scale, 0.01);
																ros::spin();
								}
								return 0;
}

void move(double speed, double distance, bool isForward){
								geometry_msgs::Twist vel_msg;

								if (isForward)
																vel_msg.linear.x =abs(speed);
								else
																vel_msg.linear.x =-abs(speed);
								vel_msg.linear.y =0;
								vel_msg.linear.z =0;
								vel_msg.angular.x = 0;
								vel_msg.angular.y = 0;
								vel_msg.angular.z =0;

								double t0 = ros::Time::now().toSec();
								double current_distance = 0.0;
								ros::Rate loop_rate(100);
								do {
																velocity_publisher.publish(vel_msg);
																double t1 = ros::Time::now().toSec();
																current_distance = speed * (t1-t0);
																ros::spinOnce();
																loop_rate.sleep();
								} while(current_distance<distance);
								vel_msg.linear.x =0;
								velocity_publisher.publish(vel_msg);

}

void rotate (double angular_speed, double relative_angle, bool clockwise){
								geometry_msgs::Twist vel_msg;

								vel_msg.linear.x =0;
								vel_msg.linear.y =0;
								vel_msg.linear.z =0;
								vel_msg.angular.x = 0;
								vel_msg.angular.y = 0;

								if (clockwise)
																vel_msg.angular.z =-abs(angular_speed);
								else
																vel_msg.angular.z =abs(angular_speed);

								double current_angle = 0.0;
								ros::Rate loop_rate(1000);
								double t0 = ros::Time::now().toSec();
								do {
																velocity_publisher.publish(vel_msg);
																double t1 = ros::Time::now().toSec();
																current_angle = angular_speed * (t1-t0);
																ros::spinOnce();
																loop_rate.sleep();
								} while(current_angle<relative_angle);
								vel_msg.angular.z =0;
								velocity_publisher.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees){
								return angle_in_degrees *PI /180.0;
}

double setDesiredOrientation(double desired_angle_radians)
{
								double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
								bool clockwise = ((relative_angle_radians<0) ? true : false);
								rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
								turtlesim_pose.x=pose_message->x;
								turtlesim_pose.y=pose_message->y;
								turtlesim_pose.theta=pose_message->theta;
}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
								geometry_msgs::Twist vel_msg;

								ros::Rate loop_rate(10);
								do {
																vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
																vel_msg.linear.y = 0;
																vel_msg.linear.z = 0;

																vel_msg.angular.x = 0;
																vel_msg.angular.y = 0;
																vel_msg.angular.z = 4*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);

																velocity_publisher.publish(vel_msg);
																ros::spinOnce();
																loop_rate.sleep();
								} while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
								vel_msg.linear.x = 0;
								vel_msg.angular.z = 0;
								velocity_publisher.publish(vel_msg);
}

void moveto(double xcord, double ycord, double distance_tolerance){
								geometry_msgs::Twist vel_msg;
								ros::Rate loop_rate(.8);
								setDesiredOrientation(atan2(ycord - turtlesim_pose.y, xcord - turtlesim_pose.x));
								loop_rate.sleep();
								move(3*getDistance(turtlesim_pose.x, turtlesim_pose.y, xcord, ycord), getDistance(turtlesim_pose.x, turtlesim_pose.y, xcord, ycord), distance_tolerance);
								loop_rate.sleep();
}

double getDistance(double x1, double y1, double x2, double y2){
								return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}
