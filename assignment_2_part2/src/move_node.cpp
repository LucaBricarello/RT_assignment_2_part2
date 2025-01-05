#include <iostream>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char * argv[])
{
  	rclcpp::init(argc, argv);
  	auto node = rclcpp::Node::make_shared("publisher");
  	auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  	
  	geometry_msgs::msg::Twist my_vel;
  	
  	my_vel.linear.x = 11;
	my_vel.angular.z = 11;

  	while (rclcpp::ok()) 
  	{
  		while (my_vel.linear.x < -10 || my_vel.linear.x > 10)
  		{
			printf("Choose velocity on X axis, range -10 to 10 \n");
			std::cin >> my_vel.linear.x;
			// Check if the input failed
			printf("\n");
			if (my_vel.linear.x < -10 && my_vel.linear.x > 10)
			{
				std::cout << "Invalid input. Out of range.\n";
			}
			if (std::cin.fail()) 
        		{
            			std::cin.clear();
            			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            			std::cout << "Invalid input. Enter a numeric value.\n";
            			my_vel.linear.x = 11;
        		}
		}
	
		while (my_vel.angular.z < -10 || my_vel.angular.z > 10)
  		{
			printf("Choose angular velocity aroud Z axis, range -10 to 10 \n");
			std::cin >> my_vel.angular.z;
			printf("\n");
			if (my_vel.angular.z < -10 && my_vel.angular.z > 10)
			{
				std::cout << "Invalid input. Out of range.\n";
			}
			if (std::cin.fail()) 
        		{
            			std::cin.clear();
            			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            			std::cout << "Invalid input. Enter a numeric value.\n";
				my_vel.angular.z = 11;
        		}
		}

		publisher->publish(my_vel);

		std::this_thread::sleep_for(std::chrono::seconds(1));
		
		my_vel.linear.x = 0;
		my_vel.angular.z = 0;

		publisher->publish(my_vel);
		
		my_vel.linear.x = 11;
		my_vel.angular.z = 11;
  	}
  	
  	rclcpp::shutdown();
  	return 0;
}
