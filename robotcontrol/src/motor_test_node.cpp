#include <rclcpp/rclcpp.hpp>
#include "interface/msg/motor.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MotorTestNode : public rclcpp::Node
{
public:
    MotorTestNode(int motor_id, int current)
    : Node("motor_test_node"), motor_id_(motor_id), current_(current)
    {
        publisher_ = this->create_publisher<interface::msg::Motor>("motor_cmd", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&MotorTestNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "motor_test_node已启动，周期性给电机%d发送电流%d", motor_id_, current_);
    }

private:
    void timer_callback()
    {
        interface::msg::Motor msg;
        msg.mode = msg.MODE_CURRENT;
        msg.motor_id = motor_id_;
        msg.current = current_;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "已发送电流命令: 电机%d, 电流=%d", motor_id_, current_);
    }

    rclcpp::Publisher<interface::msg::Motor>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int motor_id_;
    int current_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 创建临时node用于参数声明和获取
    auto temp_node = std::make_shared<rclcpp::Node>("motor_test_param_helper");
    temp_node->declare_parameter<int>("motor_id", 1);
    temp_node->declare_parameter<int>("current", 500);
    int motor_id = temp_node->get_parameter("motor_id").as_int();
    int current = temp_node->get_parameter("current").as_int();

    // 检查参数合法性
    if (motor_id < 1 || motor_id > 4) {
        printf("电机ID必须在1-4之间 (当前: %d)\n", motor_id);
        return 1;
    }
    if (current < -16384 || current > 16384) {
        printf("电流值必须在-16384~16384之间 (当前: %d)\n", current);
        return 1;
    }

    auto node = std::make_shared<MotorTestNode>(motor_id, current);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
