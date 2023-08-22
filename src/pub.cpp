#include <random>

#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"
#include "vehicle_interfaces/msg/distance.hpp"

#define NODE_NAME "qospubtest_0_node"
#define TOPIC_NAME_0 "topic0"
#define TOPIC_NAME_1 "topic1"

using namespace std::chrono_literals;

class SamplePublisher : public vehicle_interfaces::VehicleServiceNode
{
private:
    rclcpp::Publisher<vehicle_interfaces::msg::WheelState>::SharedPtr pub0_;
    rclcpp::Publisher<vehicle_interfaces::msg::Distance>::SharedPtr pub1_;
    rclcpp::TimerBase::SharedPtr timer0_;
    rclcpp::TimerBase::SharedPtr timer1_;
    std::string nodeName_;

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

private:
    void _timer0Callback()
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
            RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_timer0Callback] Publishing");
        this->pub0_->publish(msg);
    }

    void _timer1Callback()
    {
        static uint64_t cnt = 0;
        auto msg = vehicle_interfaces::msg::Distance();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_ULTRASONIC;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = cnt;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = 50;

        msg.unit_type = vehicle_interfaces::msg::Distance::UNIT_METER;
        msg.min = 0;
        msg.max = 10;

        static std::uniform_real_distribution<> uniDistrib{0.0, 1.0};
        msg.distance = uniDistrib(gen_) * 10.0;

        if (cnt++ % 100 == 1)
            RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_timer1Callback] Publishing");
        this->pub1_->publish(msg);
    }

    /*
     * Edit _qosCallback() for QoS update strategies
     */
    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Get qmap size: %d", qmap.size());
        for (const auto& [k, v] : qmap)
        {
            RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Get qmap[%s]", k.c_str());

            if (k == TOPIC_NAME_0 || k == (std::string)this->get_namespace() + "/" + TOPIC_NAME_0)
            {
                this->timer0_->cancel();
                while (!this->timer0_->is_canceled())
                    std::this_thread::sleep_for(50ms);
                
                this->timer0_.reset();// Call destructor
                // std::cout << "[SamplePublisher::_qosCallback] Timer0 reset. [" << this->timer0_ << ":" << this->timer0_.use_count() << "]\n";
                this->pub0_.reset();// Call destructor
                // std::cout << "[SamplePublisher::_qosCallback] Publisher0 reset. [" << this->pub0_ << ":" << this->pub0_.use_count() << "]\n";
                this->pub0_ = this->create_publisher<vehicle_interfaces::msg::WheelState>(TOPIC_NAME_0, *v);
                // std::cout << "[SamplePublisher::_qosCallback] Publisher0 resume. [" << this->pub0_ << ":" << this->pub0_.use_count() << "]\n";
                this->timer0_ = this->create_wall_timer(20ms, std::bind(&SamplePublisher::_timer0Callback, this));
                // std::cout << "[SamplePublisher::_qosCallback] Timer0 resume. [" << this->timer0_ << ":" << this->timer0_.use_count() << "]\n";
            }
            else if (k == TOPIC_NAME_1 || k == (std::string)this->get_namespace() + "/" + TOPIC_NAME_1)
            {
                this->timer1_->cancel();
                while (!this->timer1_->is_canceled())
                    std::this_thread::sleep_for(50ms);
                
                this->timer1_.reset();// Call destructor
                // std::cout << "[SamplePublisher::_qosCallback] Timer1 reset. [" << this->timer1_ << ":" << this->timer1_.use_count() << "]\n";
                this->pub1_.reset();// Call destructor
                // std::cout << "[SamplePublisher::_qosCallback] Publisher1 reset. [" << this->pub1_ << ":" << this->pub1_.use_count() << "]\n";
                this->pub1_ = this->create_publisher<vehicle_interfaces::msg::Distance>(TOPIC_NAME_1, *v);
                // std::cout << "[SamplePublisher::_qosCallback] Publisher1 resume. [" << this->pub1_ << ":" << this->pub1_.use_count() << "]\n";
                this->timer1_ = this->create_wall_timer(50ms, std::bind(&SamplePublisher::_timer1Callback, this));
                // std::cout << "[SamplePublisher::_qosCallback] Timer1 resume. [" << this->timer1_ << ":" << this->timer1_.use_count() << "]\n";
            }
        }
    }

public:
    SamplePublisher(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::VehicleServiceNode(gParams), 
        rclcpp::Node(NODE_NAME)
    {
        this->nodeName_ = NODE_NAME;

        /*
         * Add following codes for QoSUpdateNode support
         */
        this->addQoSCallbackFunc(std::bind(&SamplePublisher::_qosCallback, this, std::placeholders::_1));

        {
            vehicle_interfaces::QoSPair qpair = this->addQoSTracking(TOPIC_NAME_0);
            if (qpair.first == "")
                RCLCPP_ERROR(this->get_logger(), "[SamplePublisher] Failed to add topic to track list: %s", TOPIC_NAME_0);
            else
            {
                RCLCPP_INFO(this->get_logger(), "[SamplePublisher] QoS profile [%s]:\nDepth: %d\nReliability: %d", 
                    qpair.first.c_str(), qpair.second->depth(), qpair.second->reliability());
            }
            this->pub0_ = this->create_publisher<vehicle_interfaces::msg::WheelState>(TOPIC_NAME_0, *qpair.second);
            this->timer0_ = this->create_wall_timer(20ms, std::bind(&SamplePublisher::_timer0Callback, this));
        }

        {
            vehicle_interfaces::QoSPair qpair = this->addQoSTracking(TOPIC_NAME_1);
            if (qpair.first == "")
                RCLCPP_ERROR(this->get_logger(), "[SamplePublisher] Failed to add topic to track list: %s", TOPIC_NAME_1);
            else
            {
                RCLCPP_INFO(this->get_logger(), "[SamplePublisher] QoS profile [%s]:\nDepth: %d\nReliability: %d", 
                    qpair.first.c_str(), qpair.second->depth(), qpair.second->reliability());
            }
            this->pub1_ = this->create_publisher<vehicle_interfaces::msg::Distance>(TOPIC_NAME_1, *qpair.second);
            this->timer1_ = this->create_wall_timer(50ms, std::bind(&SamplePublisher::_timer1Callback, this));
        }
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