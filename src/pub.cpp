#include <random>

#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define NODE_NAME "safetytest_0_node"
#define TOPIC_NAME "topic"

using namespace std::chrono_literals;

class SamplePublisher : public vehicle_interfaces::TimeSyncNode, public vehicle_interfaces::SafetyNode
{
private:
    rclcpp::Publisher<vehicle_interfaces::msg::WheelState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string nodeName_;
    int cnt_;
    float emP_;

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

private:
    void timer_callback()
    {
        auto msg = vehicle_interfaces::msg::WheelState();
        msg.header.device_id = this->nodeName_;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();

        msg.gear = vehicle_interfaces::msg::WheelState::GEAR_NEUTRAL;
        msg.steering = cnt_ % 512;
        msg.pedal_throttle = cnt_ % 256;
        msg.pedal_brake = cnt_ % 128;
        msg.pedal_clutch = cnt_ % 64;
        msg.button = cnt_ % 32;
        msg.func = cnt_ % 16;

        RCLCPP_INFO(this->get_logger(), "Publishing: %d | %d %d %d %d | %d %d", msg.gear, 
            msg.steering, msg.pedal_throttle, msg.pedal_brake, msg.pedal_clutch, msg.button, msg.func);
        this->publisher_->publish(msg);
        this->cnt_++;

        static std::uniform_real_distribution<> uniDistrib{0.0, 1.0};
        this->setEmergency(this->nodeName_, uniDistrib(this->gen_));
    }

public:
    SamplePublisher(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::TimeSyncNode(NODE_NAME, gParams->timesyncService, gParams->timesyncInterval_ms, gParams->timesyncAccuracy_ms), 
        vehicle_interfaces::SafetyNode(NODE_NAME, gParams->safetyService), 
        rclcpp::Node(NODE_NAME)
    {
        this->nodeName_ = NODE_NAME;
        this->publisher_ = this->create_publisher<vehicle_interfaces::msg::WheelState>(TOPIC_NAME, 10);
        this->timer_ = this->create_wall_timer(20ms, std::bind(&SamplePublisher::timer_callback, this));
        this->cnt_ = 0;
        this->emP_ = 0.0;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("safetytest_params_node");
    auto timeSyncPub = std::make_shared<SamplePublisher>(params);
    rclcpp::spin(timeSyncPub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}