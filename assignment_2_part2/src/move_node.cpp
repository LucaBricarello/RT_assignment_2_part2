#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "interfaces/srv/lin_vel.hpp"

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

using std::placeholders::_1;

class RobotNode : public rclcpp::Node
{
	public:
  		RobotNode() : Node("robot_node")
  		{
    			subscription_ = this->create_subscription<nav_msgs::msg::Odometry>( "/odom", 10, std::bind(&RobotNode::robot_callback, this, _1));
    			
    			// Publishers
        		publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        		pos_pub = this->create_publisher<geometry_msgs::msg::Point>("position_feet", 10);

        		// Service
        		lin_vel_service_ = this->create_service<interfaces::srv::LinVel>(
            		 "set_lin_vel", std::bind(&RobotNode::lin_vel_service_callback, this, std::placeholders::_1, std::placeholders::_2));
  		}

	private:
  		void robot_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  		{
    			geometry_msgs::msg::Twist my_vel;
    			
    			RCLCPP_INFO(this->get_logger(), "\nx: %f\ny: %f\n\n", msg->pose.pose.position.x, msg->pose.pose.position.y);
	
			if (msg->pose.pose.position.x < 9.0 && msg->pose.pose.position.x >= 2.0)
			{
				my_vel.linear.x = 1.0;
			}
			else
			{
				
				if (msg->pose.pose.position.x >= 9.0)
				{
					my_vel.linear.x = 1.0;
					my_vel.angular.z = 1.0;
				}
				else
				{
					my_vel.linear.x = 1.0;
					my_vel.angular.z = -1.0;
				}
							
			}
	
			publisher->publish(my_vel);
			
			geometry_msgs::msg::Point my_pos;
			
			my_pos.x = msg->pose.pose.position.x * 3.28;
			my_pos.y = msg->pose.pose.position.y * 3.28;
			
			pos_pub->publish(my_pos);
  		}
  		
  		// Callback for the service
    		void lin_vel_service_callback(const std::shared_ptr<interfaces::srv::LinVel::Request> request,
        					std::shared_ptr<interfaces::srv::LinVel::Response> response)
    		{
    			if (request->lin_vel == 0)
    			{
        			lin_vel = 0;
        			RCLCPP_INFO(this->get_logger(), "Linear velocity updated to: 0");
        		}
        		else
        		{
        			lin_vel = 1;
        			RCLCPP_INFO(this->get_logger(), "Linear velocity updated to: 0");
        		}
    		}
    		
    		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    		// Publishers
    		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pos_pub;

    		// Service
    		rclcpp::Service<interfaces::srv::LinVel>::SharedPtr lin_vel_service_;

    		// Internal variables
    		float lin_vel;
};

int main(int argc, char * argv[])
{
  	rclcpp::init(argc, argv);
    	rclcpp::spin(std::make_shared<RobotNode>());
    	rclcpp::shutdown();
    	return 0;
}
