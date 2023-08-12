#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/qos.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("qosserver_params_node");
    auto server = std::make_shared<vehicle_interfaces::QoSServer>(params->nodeName, params->qosService);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
