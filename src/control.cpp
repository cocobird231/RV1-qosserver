#include <random>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/qos.h"

std::vector<std::string> split(std::string str, std::string delimiter)
{
    std::vector<std::string> splitStrings;
    int encodingStep = 0;
    for (size_t i = 0; i < str.length(); i++)
    {
        bool isDelimiter = false;
        for (auto& j : delimiter)
            if (str[i] == j)
            {
                isDelimiter = true;
                break;
            }
        if (!isDelimiter)// Is the spliting character
        {
            encodingStep++;
            if (i == str.length() - 1)
                splitStrings.push_back(str.substr(str.length() - encodingStep, encodingStep));
        }
        else// Is delimiter
        {
            if (encodingStep > 0)// Have characters need to split
                splitStrings.push_back(str.substr(i - encodingStep, encodingStep));
            encodingStep = 0;
        }
    }
    return splitStrings;
}

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
    }

    bool requestQosReg(const std::shared_ptr<vehicle_interfaces::srv::QosReg::Request>& req)
    {
        RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReg]");
        auto result = this->regClient_->async_send_request(req);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 100ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 100ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            RCLCPP_INFO(this->get_logger(), "[QoSControlNode::regQoSRequest] Request: %d, qid: %ld", res->response, res->qid);
            return res->response;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReg] Request failed.");
        return false;
    }

    bool requestQosReq(const std::string& topicName, rmw_qos_profile_t& outQoSProfile)
    {
        auto request = std::make_shared<vehicle_interfaces::srv::QosReq::Request>();
        request->topic_name = topicName;
        auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 100ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 100ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReq] Response: %d, qid: %ld", res->response, res->qid);
            if (res->response)
            {
                outQoSProfile = vehicle_interfaces::CvtMsgToRMWQoS(res->qos_profile);
                RCLCPP_INFO(this->get_logger(), "[QoSControlNode::requestQosReq] Profile get: %s, %d", 
                    vehicle_interfaces::getQoSProfEnumName(outQoSProfile.reliability).c_str(), outQoSProfile.depth);
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
    auto control = std::make_shared<QoSControlNode>("qoscontrol_0_node", params->qosService);
    rclcpp::executors::SingleThreadedExecutor* exec = new rclcpp::executors::SingleThreadedExecutor();
    exec->add_node(control);

    bool stopF = true;
    auto th = std::thread(SpinExecutor, exec, std::ref(stopF));

    while (stopF)
        std::this_thread::sleep_for(500ms);

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

    while (!stopF)
    {
        printf(">");
        // profile add topic_name
        // profile add topic_name 10
        // profile remove topic_name
        // profile clear
        // profile save
        // param enabled_publish true
        // param enabled_publish false
        // param publish_interval_ms 10000

        // pr a topic_name
        // pr a topic_name 10
        // pr r topic_name
        // pr c
        // pr s
        // pa p 1
        // pa i 10000
        std::string inputStr;
        std::getline(std::cin, inputStr);
        
        if (inputStr.size() <= 0)
            continue;
        auto inputStrVec = split(inputStr, ", ");

        if (inputStrVec[0] == "pr" && inputStrVec.size() >= 2)
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
            else if (inputStrVec[1] == "a" && inputStrVec.size() >= 3)
            {
                req.topic_name = inputStrVec[2];
                try
                {
                    if (inputStrVec.size() == 3)
                    {
                        static std::uniform_real_distribution<> uniDistrib{0.0, 1.0};
                        int depth = uniDistrib(gen_) * 100 + 1;
                        req.qos_profile.depth = depth;
                    }
                    else if (inputStrVec.size() == 4)
                    {
                        req.qos_profile.depth = std::stoi(inputStrVec[3]);
                    }
                    else if (inputStrVec.size() == 5)
                    {
                        req.qos_profile.depth = std::stoi(inputStrVec[3]);
                        req.qos_profile.reliability = std::stoi(inputStrVec[4]);
                    }
                    else
                        continue;
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
        std::this_thread::sleep_for(10ms);
    }
    rclcpp::shutdown();
}
