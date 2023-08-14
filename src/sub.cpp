#include <random>

#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define NODE_NAME "qossubtest_0_node"
#define TOPIC_NAME "topic"

using namespace std::chrono_literals;

class SampleSubscriber : public vehicle_interfaces::VehicleServiceNode
{
private:
    rclcpp::Subscription<vehicle_interfaces::msg::WheelState>::SharedPtr subscription_;
    std::string nodeName_;

private:
    void _topicCallback(const vehicle_interfaces::msg::WheelState::SharedPtr msg)
    {
        static int64_t cnt = 0;
        if (cnt++ % 100 == 1)
            RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_timerCallback] Heard!");
    }

    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Get qmap size: %d", qmap.size());
        for (const auto& [k, v] : qmap)
        {
            RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Get qmap[%s]", k.c_str());

            if (k == TOPIC_NAME || k == (std::string)this->get_namespace() + "/" + TOPIC_NAME)
            {

                this->subscription_.reset();
                RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Subscription reset [%p:%d]", this->subscription_, this->subscription_.use_count());
                this->subscription_ = this->create_subscription<vehicle_interfaces::msg::WheelState>(TOPIC_NAME, 
                    *v, std::bind(&SampleSubscriber::_topicCallback, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Subscription resume [%p:%d]", this->subscription_, this->subscription_.use_count());
            }
        }
    }

public:
    SampleSubscriber(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::VehicleServiceNode(gParams), 
        rclcpp::Node(NODE_NAME)
    {
        this->nodeName_ = NODE_NAME;
        this->subscription_ = this->create_subscription<vehicle_interfaces::msg::WheelState>(TOPIC_NAME, 
            10, std::bind(&SampleSubscriber::_topicCallback, this, std::placeholders::_1));

        this->addQoSTracking(this->subscription_->get_topic_name());
        this->addQoSCallbackFunc(std::bind(&SampleSubscriber::_qosCallback, this, std::placeholders::_1));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("qossubtest_params_node");
    params->timesyncService = "";
    params->safetyService = "";
    auto timeSyncSub = std::make_shared<SampleSubscriber>(params);
    rclcpp::spin(timeSyncSub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}