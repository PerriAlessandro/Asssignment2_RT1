#include "ros/ros.h"
#include "second_assignment/ChangeVel.h"
#include "std_srvs/Empty.h" //std_srvs contains two service types called Empty and Trigger, which are common
//service patterns for sending a signal to a ROS node. For the Empty service, no actual data is exchanged
//between the service and the client. The Trigger service adds the possibility to check if triggering was successful or not.

#define RESET "\033[0m"

#define BHBLK "\e[1;90m"
#define BHRED "\e[1;91m"
#define BHGRN "\e[1;92m"
#define BHYEL "\e[1;93m"
#define BHBLU "\e[1;94m"
#define BHMAG "\e[1;95m"
#define BHCYN "\e[1;96m"
#define BHWHT "\e[1;97m"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "UI_node"); //initialization of the node
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<second_assignment::ChangeVel>("/changeVel");
	second_assignment::ChangeVel srv1;
	std_srvs::Empty reset_srv;

	std::cout << BHCYN "\n ################################## USER INTERFACE ################################## \n\n " RESET;
	std::cout << BHCYN "Press 'a' to accelerate, press 'd' to decelerate, 'r' to reset position, 'q' to quit \n " RESET;
	char c;

	while (c != 'q')
	{
		std::cin.clear();
		std::cin >> c;

		switch (c)
		{
		case 'a':
		{
			std::cout << BHBLU "The robot is accelerating!" RESET "\n";
			srv1.request.input = 'a';
			client.waitForExistence();
			client.call(srv1);
			std::cout << BHCYN "Current speed: " << srv1.response.multiplier << "\n " RESET;
			break;
		}

		case 'd':
		{
			std::cout << BHYEL "The robot is decelerating!" RESET "\n";
			srv1.request.input = 'd';
			client.waitForExistence();
			client.call(srv1);
			std::cout << BHCYN "Current speed: " << srv1.response.multiplier << "\n " RESET;
			break;
		}

		case 'r':
		{
			std::cout << BHGRN "RESET POSITION (AND SPEED)" RESET "\n";
			srv1.request.input = 'r';
			client.waitForExistence();
			client.call(srv1);
			ros::service::call("/reset_positions",reset_srv);
			std::cout << BHCYN "Current speed: " << srv1.response.multiplier << "\n " RESET;
			break;
		}
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

/*ServiceServer ros::NodeHandle::advertiseService ( const std::string & service,
bool(T::*)(MReq &, MRes &) srv_func,
T * obj
)

\param service Service name to advertise on
 \param srv_func Member function pointer to call when a message has arrived
 \param obj Object to call srv_func on
 \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.*/
