// Description: A node that sets a GPIO output pin to HIGH if
// it receives a "true" message. If so, it outputs an unlocking
// command to the terminal. It also sets the output pin to LOW
// after 10 seconds 
// Date: April 30, 2025 

#include <JetsonGPIO.h>
#include <memory>
#include <functional>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class MaglockController : public rclcpp::Node
{
	private:
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
		int pin_;		// Stores the pin number to be set to HIGH

		void command_callback(const std_msgs::msg::Bool::SharedPtr msg)
		{
			// Sets the output pin to HIGH if the message says "true"
			if (msg->data)
			{
				RCLCPP_INFO(this->get_logger(), "Sending unlock command...");
				GPIO::output(pin_, GPIO::HIGH);

				// Used the prevent the node from receiving any new messages
				sleep(10);

				RCLCPP_INFO(this->get_logger(), "Locking...");
				GPIO::output(pin_, GPIO::LOW);
			}
		}

	public:
		MaglockController() : Node("maglock_controller")
		{
			subscription_ = this->create_subscription<std_msgs::msg::Bool>("maglock_cmd", 10, std::bind(&MaglockController::command_callback, this, std::placeholders::_1));
			GPIO::setmode(GPIO::BOARD);
			pin_ = 15;
			GPIO::setup(pin_, GPIO::OUT);
		}

		~MaglockController()
		{
			GPIO::cleanup(pin_);
		}

};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node{ std::make_shared<MaglockController>() };
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
