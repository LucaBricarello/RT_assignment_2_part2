#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

using std::placeholders::_1;

class RobotSubscriber : public rclcpp::Node
{
	public:
  		RobotSubscriber() : Node("robot_subscriber")
  		{
    			subscription_ = this->create_subscription<nav_msgs::msg::Odometry>( "/odom", 10, std::bind(&RobotSubscriber::robot_callback, this, _1));
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
  		}
  		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  	rclcpp::init(argc, argv);
  	auto node = rclcpp::Node::make_shared("move_node");
  	
  	publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  	
  	rclcpp::spin(std::make_shared<RobotSubscriber>());
  	
  	rclcpp::shutdown();
  	return 0;
}
