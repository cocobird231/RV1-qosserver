#include <random>

#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define NODE_NAME "qostest_0_node"
#define TOPIC_NAME "topic"

using namespace std::chrono_literals;

class SamplePublisher : public vehicle_interfaces::VehicleServiceNode
{
private:
    rclcpp::Publisher<vehicle_interfaces::msg::WheelState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string nodeName_;
    int cnt_;
    float emP_;

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

    std::map<std::string, rclcpp::QoS*> qmap_;

private:
    void _timerCallback()
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

        if (cnt_ % 100 == 1)
            RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_timerCallback] Publishing");
        this->publisher_->publish(msg);
        this->cnt_++;

        static std::uniform_real_distribution<> uniDistrib{0.0, 1.0};
        // this->setEmergency(this->nodeName_, uniDistrib(this->gen_));
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

        this->cnt_ = 0;
        this->emP_ = 0.0;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("qostest_params_node");
    params->timesyncService = "";
    params->safetyService = "";
    auto timeSyncPub = std::make_shared<SamplePublisher>(params);
    rclcpp::spin(timeSyncPub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}