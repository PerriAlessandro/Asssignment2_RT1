#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "second_assignment/ChangeVel.h"

ros::Publisher pub;
float currentVelocity = 1.0;
float fr_threshold=1.5;

float min(float arr[], int imin, int imax)
{
	float dist = 100;
	for (int i = imin; i < imax; i++)
	{

		if (arr[i] < dist)
		{
			dist = arr[i];
		}
	}
return dist;
}

bool changeVelocity(second_assignment::ChangeVel::Request &req, second_assignment::ChangeVel::Response &res)
{

	ROS_INFO("Input Received: [%c]", req.input);

	if (req.input == 'a')
	{
		res.multiplier = 2.0 * currentVelocity;
	}
	if (req.input == 'd')
	{
		res.multiplier = 0.5 * currentVelocity;
	}
	if (req.input == 'r')
	{
		res.multiplier = 1;
	}
	currentVelocity = res.multiplier;
	ROS_INFO("Speed= [%f]", currentVelocity); 

	return true;
}

void driveCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

	geometry_msgs::Twist my_vel;

	float array[msg->ranges.size()];
	for (int i = 0; i < msg->ranges.size(); i++)	array[i] = msg->ranges[i];

	float dist_left = min(array, 600, msg->ranges.size()-1);
	float dist_right = min(array, 0, 120);
	float dist_front = min(array, 330, 390);

	//ROS_INFO("LEFT: [%f], RIGHT:[%f], FRONT: [%f]", dist_left, dist_right, dist_front); //printf

	if (dist_front < fr_threshold)
	{ //check if the frontal distance is lower than a_th_gld

		if (dist_left <= dist_right)//checks if the distance of the left golden token is lower than the one of the right token
		{
			if (2 * dist_left < dist_right) //in this case the the left distance (dist_left) is at least 1.5 times smaller than the right distance (dist_right), so i only need to turn to the right
				my_vel.angular.z = -1.0;

			else
			{ //the two lateral distances are too similar, better to go forward while turning
				my_vel.linear.x = 1.0;
				my_vel.angular.z = -1.0;
			}
		}
		else if (2 * dist_right < dist_left) //if the cycle arrives here, it means that dist_right<dist_left

			my_vel.angular.z = 1.0;
		else
		{
			my_vel.linear.x = 1.0;
			my_vel.angular.z = 1.0;
		}
	}
	else
	{ //if none of the previous conditions occured, then go forward
		my_vel.linear.x = currentVelocity;
	}
	pub.publish(my_vel);
}

int main(int argc, char **argv)
{

	
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS //system
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/base_scan", 1000, driveCallback);

	
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 

	ros::ServiceServer service = nh.advertiseService("/changeVel", changeVelocity);
	ros::spin();
	return 0;
}
/**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
   
   //ros::Subscriber sub = nh.subscribe("rt1_turtle/pose", 1,turtleCallback);  //This method connects to the master to register interest in a given topic. The node will automatically be connected with publishers on this topic. On each message receipt, fp(turtleCallback) is invoked and passed a shared pointer to the message received. This message should not be changed in place, as it is shared with any other subscriptions to this topic.
