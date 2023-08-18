#include <random>

#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"
#include "vehicle_interfaces/msg/distance.hpp"

#define NODE_NAME "qossubtest_0_node"
#define TOPIC_NAME_0 "topic0"
#define TOPIC_NAME_1 "topic1"

using namespace std::chrono_literals;

class SampleSubscriber : public vehicle_interfaces::VehicleServiceNode
{
private:
    rclcpp::Subscription<vehicle_interfaces::msg::WheelState>::SharedPtr sub0_;
    rclcpp::Subscription<vehicle_interfaces::msg::Distance>::SharedPtr sub1_;
    std::string nodeName_;

private:
    void _topic0Callback(const vehicle_interfaces::msg::WheelState::SharedPtr msg)
    {
        static int64_t cnt = 0;
        if (cnt++ % 100 == 1)
            RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_topic0Callback] Heard!");
    }

    void _topic1Callback(const vehicle_interfaces::msg::Distance::SharedPtr msg)
    {
        static int64_t cnt = 0;
        if (cnt++ % 100 == 1)
            RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_topic1Callback] Heard!");
    }

    /*
     * Edit _qosCallback() for QoS update strategies
     */
    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Get qmap size: %d", qmap.size());
        for (const auto& [k, v] : qmap)
        {
            RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Get qmap[%s]", k.c_str());

            if (k == TOPIC_NAME_0 || k == (std::string)this->get_namespace() + "/" + TOPIC_NAME_0)
            {

                this->sub0_.reset();
                RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Subscription0 reset. [%p:%d]", this->sub0_, this->sub0_.use_count());
                this->sub0_ = this->create_subscription<vehicle_interfaces::msg::WheelState>(TOPIC_NAME_0, 
                    *v, std::bind(&SampleSubscriber::_topic0Callback, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Subscription0 resume. [%p:%d]", this->sub0_, this->sub0_.use_count());
            }
            else if (k == TOPIC_NAME_1 || k == (std::string)this->get_namespace() + "/" + TOPIC_NAME_1)
            {

                this->sub1_.reset();
                RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Subscription1 reset. [%p:%d]", this->sub1_, this->sub1_.use_count());
                this->sub1_ = this->create_subscription<vehicle_interfaces::msg::Distance>(TOPIC_NAME_1, 
                    *v, std::bind(&SampleSubscriber::_topic1Callback, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_qosCallback] Subscription1 resume. [%p:%d]", this->sub1_, this->sub1_.use_count());
            }
        }
    }

public:
    SampleSubscriber(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::VehicleServiceNode(gParams), 
        rclcpp::Node(NODE_NAME)
    {
        this->nodeName_ = NODE_NAME;

        /*
         * Add following codes for QoSUpdateNode support
         */
        this->addQoSCallbackFunc(std::bind(&SampleSubscriber::_qosCallback, this, std::placeholders::_1));

        rclcpp::QoS* qos[2];
        qos[0] = new rclcpp::QoS(10);
        this->addQoSTracking(TOPIC_NAME_0, qos[0]);
        this->sub0_ = this->create_subscription<vehicle_interfaces::msg::WheelState>(TOPIC_NAME_0, 
            *qos[0], std::bind(&SampleSubscriber::_topic0Callback, this, std::placeholders::_1));
        
        qos[1] = new rclcpp::QoS(10);
        this->addQoSTracking(TOPIC_NAME_1, qos[1]);
        this->sub1_ = this->create_subscription<vehicle_interfaces::msg::Distance>(TOPIC_NAME_1, 
            *qos[1], std::bind(&SampleSubscriber::_topic1Callback, this, std::placeholders::_1));
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