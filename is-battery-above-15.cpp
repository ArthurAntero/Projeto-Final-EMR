#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <iostream>

class IsBatteryAbove15 : public BT::ConditionNode
{
public:
    // Construtor que agora usa o nó ROS para criar o Subscriber
    explicit IsBatteryAbove15(const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr)
        : BT::ConditionNode(name, config), node_ptr_(node_ptr)
    {
        // Assinar o tópico de bateria (exemplo: /battery_level)
        str_subscriber_ptr_ = node_ptr_->create_subscription<std_msgs::msg::String>(
            "/battery_level", 10, std::bind(&IsBatteryAbove15::update_msg, this, std::placeholders::_1));
    }

    // Implementação do método tick
    BT::NodeStatus tick() override
    {
        double battery_level;
        
        // Acessar o valor da bateria
        if (!getInput("battery_level", battery_level))
        {
            std::cerr << "[ERROR] IsBatteryAbove15: Failed to get battery level!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Lógica para verificar o nível da bateria
        if (battery_level > 15.0)
        {
            std::cout << "[INFO] IsBatteryAbove15: Battery level is above 15%" << std::endl;
            std::string msg_ = "Battery level above 15%";
            setOutput("msg", msg_);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "[WARNING] IsBatteryAbove15: Battery level is below or equal to 15%" << std::endl;
            std::string msg_ = "Battery level less than 15%";
            setOutput("msg", msg_);
            return BT::NodeStatus::FAILURE;
        }
    }

    // Método de callback para atualizar a mensagem recebida do tópico
    void update_msg(const std_msgs::msg::String::SharedPtr new_msg)
    {
        // Apenas um exemplo de como atualizar a mensagem
        msg_ = new_msg->data;
        std::cout << "Received battery message: " << msg_ << std::endl;
    }

    // Lista de portas fornecidas para este nó
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("battery_level"), 
            BT::OutputPort<std::string>("msg")
        };
    }

private:
    rclcpp::Node::SharedPtr node_ptr_;  // Ponteiro para o nó ROS 2
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr str_subscriber_ptr_;
    std::string msg_;
};
