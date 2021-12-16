#include "ros/ros.h"
#include "second_assignment/ChangeVel.h"
#include "std_srvs/Empty.h"

// ANSI colors
#define RESET "\033[0m"
#define BHRED "\e[1;91m"
#define BHGRN "\e[1;92m"
#define BHYEL "\e[1;93m"
#define BHBLU "\e[1;94m"
#define BHCYN "\e[1;96m"

int main(int argc, char **argv)
{
	// initialization of the node
	ros::init(argc, argv, "UI_node");
	ros::NodeHandle nh;
	// declare ServiveClient object
	ros::ServiceClient client = nh.serviceClient<second_assignment::ChangeVel>("/changeVel");
	// declare the object that contains the request and the response of the service used to change the velocity
	second_assignment::ChangeVel srv1;
	// declare the empty service used to reset the position
	std_srvs::Empty reset_srv;

	std::cout << BHCYN "\n ################################## USER INTERFACE ################################## \n\n " RESET;
	std::cout << BHCYN "Press 'a' to accelerate, press 'd' to decelerate, 'r' to reset position, 'q' to quit \n " RESET;
	char c;

	// while loop used to receive keyboard input by user
	while (c != 'q')
	{
		// waiting for a keyboard input
		std::cin.clear();
		std::cin >> c;

		switch (c)
		{
			// if user presses 'a', a request is sent to the server 'srv1' and the robot will accelerate
		case 'a':
		{
			std::cout << BHBLU "The robot is accelerating!" RESET "\n";
			srv1.request.input = 'a';
			client.waitForExistence();
			client.call(srv1);
			std::cout << BHCYN "Current speed: " << srv1.response.multiplier << "\n " RESET;
			break;
		}
			// if user presses 'd', a request is sent to the server 'srv1' and the robot will decelerate
		case 'd':
		{
			std::cout << BHYEL "The robot is decelerating!" RESET "\n";
			srv1.request.input = 'd';
			client.waitForExistence();
			client.call(srv1);
			std::cout << BHCYN "Current speed: " << srv1.response.multiplier << "\n " RESET;
			break;
		}
			// if user presses 'r', a request is sent to the server 'srv1' and the robot will reset the speed to 1.0,
			//  moreover, another service  that resets the position is called
		case 'r':
		{
			std::cout << BHGRN "RESET POSITION (AND SPEED)" RESET "\n";
			srv1.request.input = 'r';
			client.waitForExistence();
			client.call(srv1);
			ros::service::call("/reset_positions", reset_srv);
			std::cout << BHCYN "Current speed: " << srv1.response.multiplier << "\n " RESET;
			break;
		}

		// if user presses 'q', nothing happens and the program exits from the while loop
		case 'q':
			break;
		default:
			std::cout << BHRED "No existing command! Retry.." RESET "\n";
			std::cout << BHCYN "Press 'a' to accelerate, press 'd' to decelerate, 'r' to reset position, 'q' to quit \n " RESET;
		}
	}

	std::cout << BHRED "QUIT" RESET "\n";
	return 0;
}
