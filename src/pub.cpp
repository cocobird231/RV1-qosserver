#include <random>

#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define NODE_NAME "qospubtest_0_node"
#define TOPIC_NAME "topic"

using namespace std::chrono_literals;

class SamplePublisher : public vehicle_interfaces::VehicleServiceNode
{
private:
    rclcpp::Publisher<vehicle_interfaces::msg::WheelState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string nodeName_;

private:
    void _timerCallback()
    {
        static uint64_t cnt = 0;
        auto msg = vehicle_interfaces::msg::WheelState();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = cnt;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = 20;

        msg.gear = vehicle_interfaces::msg::WheelState::GEAR_NEUTRAL;
        msg.steering = cnt % 512;
        msg.pedal_throttle = cnt % 256;
        msg.pedal_brake = cnt % 128;
        msg.pedal_clutch = cnt % 64;
        msg.button = cnt % 32;
        msg.func = cnt % 16;

        if (cnt++ % 100 == 1)
            RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_timerCallback] Publishing");
        this->publisher_->publish(msg);
    }

    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Get qmap size: %d", qmap.size());
        for (const auto& [k, v] : qmap)
        {
            RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Get qmap[%s]", k.c_str());

            if (k == TOPIC_NAME || k == (std::string)this->get_namespace() + "/" + TOPIC_NAME)
            {
                this->timer_->cancel();
                while (!this->timer_->is_canceled())
                    std::this_thread::sleep_for(100ms);
                this->timer_.reset();// Call destructor
                RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Timer reset. [%p:%d]", this->timer_, this->timer_.use_count());

                this->publisher_.reset();
                RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Publisher reset [%p:%d]", this->publisher_, this->publisher_.use_count());
                this->publisher_ = this->create_publisher<vehicle_interfaces::msg::WheelState>(TOPIC_NAME, *v);
                RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Publisher resume [%p:%d]", this->publisher_, this->publisher_.use_count());

                this->timer_ = this->create_wall_timer(20ms, std::bind(&SamplePublisher::_timerCallback, this));
                RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Timer resume [%p:%d]", this->timer_, this->timer_.use_count());
            }
        }
    }

public:
    SamplePublisher(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::VehicleServiceNode(gParams), 
        rclcpp::Node(NODE_NAME)
    {
        this->nodeName_ = NODE_NAME;
        this->publisher_ = this->create_publisher<vehicle_interfaces::msg::WheelState>(TOPIC_NAME, 10);
        this->timer_ = this->create_wall_timer(20ms, std::bind(&SamplePublisher::_timerCallback, this));

        this->addQoSTracking(this->publisher_->get_topic_name());
        this->addQoSCallbackFunc(std::bind(&SamplePublisher::_qosCallback, this, std::placeholders::_1));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("qospubtest_params_node");
    params->timesyncService = "";
    params->safetyService = "";
    auto timeSyncPub = std::make_shared<SamplePublisher>(params);
    rclcpp::spin(timeSyncPub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}