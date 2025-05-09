// Date: April 30, 2025 

#include <JetsonGPIO.h>
#include <memory>
#include <functional>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "gpio_control/gpio_control.h"

class MaglockControl : public rclcpp::Node
{
	public:
		MaglockControl() : Node("maglock_control_server"), gpio(GPIOControl::BOARD)
		{
			service_ = this->create_service<example_interfaces::srv::SetBool>(
				"maglock", std::bind(&MaglockControl::handle_request, this, std::placeholders::_1, std::placeholders::_2));
			RCLCPP_INFO(this->get_logger(), "Maglock service ready...");
			gpio.setupPin(10, GPIOControl::Out);		// Assuming pin 10 is connected to the maglock	
		}

	private:
		void handle_request(const example_interfaces::srv::SetBool::Request::SharedPtr request,
			example_interfaces::srv::SetBool::Response::SharedPtr response)
		{
			if (request->data)
			{
				gpio.setLow(10);
				response->success = true;
				return;
			}
			gpio.setHigh(10);
			response->success = true;
		}

		rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
		GPIOControl gpio;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node{ std::make_shared<MaglockControl>() };
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
