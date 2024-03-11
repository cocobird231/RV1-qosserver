#include <random>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/qos.h"
#include "vehicle_interfaces/utils.h"

class QoSControlNode : public rclcpp::Node
{
private:
    std::shared_ptr<rclcpp::Node> regClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::QosReg>::SharedPtr regClient_;

    std::shared_ptr<rclcpp::Node> reqClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::QosReq>::SharedPtr reqClient_;

    rclcpp::Node::SharedPtr paramNode_;
	rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr paramClient_;

public:
    QoSControlNode(const std::string& nodeName, const std::string& qosServiceName) : rclcpp::Node(nodeName)
    {
        this->regClientNode_ = rclcpp::Node::make_shared(nodeName + "_qosreg_client");
        this->regClient_ = this->regClientNode_->create_client<vehicle_interfaces::srv::QosReg>(qosServiceName + "_Reg");

        this->reqClientNode_ = rclcpp::Node::make_shared(nodeName + "_qosreq_client");
        this->reqClient_ = this->reqClientNode_->create_client<vehicle_interfaces::srv::QosReq>(qosServiceName + "_Req");

        this->paramNode_ = rclcpp::Node::make_shared(nodeName + "_qosparam_client");
        this->paramClient_ = this->paramNode_->create_client<rcl_interfaces::srv::SetParametersAtomically>("qosserver_0_node/set_parameters_atomically");
        RCLCPP_INFO(this->get_logger(), "[QoSControlNode] Constructed");

        bool stopF = false;
        vehicle_interfaces::ConnToService(this->regClient_, stopF);
        vehicle_interfaces::ConnToService(this->reqClient_, stopF);
        vehicle_interfaces::ConnToService(this->paramClient_, stopF);
    }

    bool requestQosReg(const std::shared_ptr<vehicle_interfaces::srv::QosReg::Request>& req)
    {
        RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReg]");
        auto result = this->regClient_->async_send_request(req);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReg] Request: %d, qid: %ld", res->response, res->qid);
            return res->response;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReg] Request failed.");
        return false;
    }

    bool requestQosReq(const std::string& topicName, const std::string& qosType, std::map<std::string, std::map<std::string, rmw_qos_profile_t> >& outQoSProfile)
    {
        auto request = std::make_shared<vehicle_interfaces::srv::QosReq::Request>();
        request->topic_name = topicName;
        request->qos_type = qosType;
        auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReq] Response: %d, qid: %ld", res->response, res->qid);
            if (res->response)
            {
                outQoSProfile.clear();
                for (int i = 0; i < res->qos_profile_vec.size(); i++)
                {
                    outQoSProfile[res->topic_name_vec[i]][res->qos_type_vec[i]] = vehicle_interfaces::CvtMsgToRMWQoS(res->qos_profile_vec[i]);
                }
            }
            return res->response;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReq] Request failed.");
        return false;
    }

    bool setParam(const rcl_interfaces::msg::Parameter& param)
	{
		auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
		request->parameters.push_back(param);

		auto result = this->paramClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->paramNode_, result, 10s) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->paramNode_, result, 10s) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "[QoSControlNode::setParam] Request: %d, reason: %s", 
                response->result.successful, response->result.reason.c_str());
			return response->result.successful;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSControlNode::setParam] Request failed.");
		return false;
	}
};

void SpinExecutor(rclcpp::executors::SingleThreadedExecutor* exec, bool& stopF)
{
    std::this_thread::sleep_for(1s);
    printf("[SpinExecutor] Spin start.\n");
    stopF = false;
    exec->spin();
    printf("[SpinExecutor] Spin ended.\n");
    stopF = true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("qoscontrol_params_node");
    auto control = std::make_shared<QoSControlNode>("qoscontrol_0_node", "qos_0");
    rclcpp::executors::SingleThreadedExecutor* exec = new rclcpp::executors::SingleThreadedExecutor();
    exec->add_node(control);

    bool stopF = true;
    auto th = std::thread(SpinExecutor, exec, std::ref(stopF));

    while (stopF)
        std::this_thread::sleep_for(500ms);

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

    printf(
"/**\n\
 * Profile Registration\n\
 * - Profile add (determine (p)ublisher, (s)ubscription or (b)oth):\n\
 *   - pr a <topic_name> <p/s/b>\n\
 *   - pr a <topic_name> <p/s/b> <depth>\n\
 *   - pr a <topic_name> <p/s/b> <depth> <reliability_enum>\n\
 *   - pr a <topic_name> <p/s/b> <depth> <reliability_enum> <durability_enum>\n\
 * - Profile remove:\n\
 *   - pr r <topic_name>\n\
 * - Profile clear:\n\
 *   - pr c\n\
 * - Profile save:\n\
 *   - pr s\n\
 * \n\
 * Set Parameters\n\
 * - Enable/Disable publish:\n\
 *   - pa p <0/1>\n\
 * - Set publish interval (ms):\n\
 *   - pa i <interval_ms>\n\
 * \n\
 * Profile Request\n\
 * - Profile request (determine (p)ublish or (s)ubscription):\n\
 *   - prq <topic_name> <p/s>\n\
 *   - prq all\n\
 * Quit\n\
 * - q\n\
 */\n"
    );

    while (!stopF)
    {
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);
        
        if (inputStr.size() <= 0)
            continue;
        auto inputStrVec = vehicle_interfaces::split(inputStr, ", ");
        if (inputStrVec[0] == "q")
        {
            stopF = true;
            break;
        }
        else if (inputStrVec[0] == "pr" && inputStrVec.size() >= 2)
        {
            vehicle_interfaces::srv::QosReg::Request req;

            if (inputStrVec[1] == "c")
            {
                req.clear_profiles = true;
            }
            else if (inputStrVec[1] == "s")
            {
                req.save_qmap = true;
            }
            else if (inputStrVec[1] == "a" && inputStrVec.size() >= 4)
            {
                req.topic_name = inputStrVec[2];
                req.qos_type = inputStrVec[3] == "p" ? "publisher" : 
                                (inputStrVec[3] == "s" ? "subscription" : 
                                (inputStrVec[3] == "b" ? "both" : ""));
                try
                {
                    if (inputStrVec.size() == 4)
                    {
                        static std::uniform_real_distribution<> uniDistrib{0.0, 1.0};
                        int depth = uniDistrib(gen_) * 10;
                        req.qos_profile.depth = depth;
                    }
                    else
                    {
                        if (inputStrVec.size() >= 5)
                            req.qos_profile.depth = std::stoi(inputStrVec[4]);
                        if (inputStrVec.size() >= 6)
                            req.qos_profile.reliability = std::stoi(inputStrVec[5]);
                        if (inputStrVec.size() >= 7)
                            req.qos_profile.durability = std::stoi(inputStrVec[6]);
                    }
                }
                catch (...)
                {
                    continue;
                }
            }
            else if (inputStrVec[1] == "r" && inputStrVec.size() == 3)
            {
                req.topic_name = inputStrVec[2];
                req.remove_profile = true;
            }
            else
                continue;

            control->requestQosReg(std::make_shared<vehicle_interfaces::srv::QosReg::Request>(req));
        }
        else if (inputStrVec[0] == "pa" && inputStrVec.size() == 3)
        {
            auto param = rcl_interfaces::msg::Parameter();

            if (inputStrVec[1] == "p")
            {
                param.name = "enabled_publish";
                param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
                param.value.bool_value = inputStrVec[2] == "true" || inputStrVec[2] == "1";
            }
            else if (inputStrVec[1] == "i")
            {
                param.name = "publish_interval_ms";
                param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                param.value.double_value = std::stod(inputStrVec[2]);
            }
            else
                continue;

            control->setParam(param);
        }
        else if (inputStrVec[0] == "prq" && inputStrVec.size() >= 3)
        {
            std::map<std::string, std::map<std::string, rmw_qos_profile_t>> topicQoS;
            if (inputStrVec[2] == "p")
                control->requestQosReq(inputStrVec[1], "publisher", topicQoS);
            else if (inputStrVec[2] == "s")
                control->requestQosReq(inputStrVec[1], "subscription", topicQoS);
            else if (inputStrVec[2] == "b")
                control->requestQosReq(inputStrVec[1], "both", topicQoS);
            else
                continue;
            vehicle_interfaces::HierarchicalPrint hprint;
            for (const auto& [topicName, tQoS] : topicQoS)
            {
                for (const auto& [type, prof] : tQoS)
                {
                    hprint.push(0, "[Topic: %s]", topicName.c_str());
                    hprint.push(1, "[%s]", type.c_str());
                    hprint.push(2, "[History: %d]", prof.history);
                    hprint.push(2, "[Depth: %ld]", prof.depth);
                    hprint.push(2, "[Reliability: %d]", prof.reliability);
                    hprint.push(2, "[Durability: %d]", prof.durability);
                }
            }
            hprint.print();
            hprint.clear();
        }
        std::this_thread::sleep_for(100ms);
    }
    rclcpp::shutdown();
}
