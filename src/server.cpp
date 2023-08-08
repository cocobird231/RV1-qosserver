#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/safety.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("safetyserver_params_node");
    auto server = std::make_shared<vehicle_interfaces::SafetyServer>(params->nodeName, params->safetyService);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
