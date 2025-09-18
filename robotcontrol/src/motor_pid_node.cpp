#include <rclcpp/rclcpp.hpp>
#include "interface/msg/motor.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MotorPIDNode : public rclcpp::Node
{
public:
    MotorPIDNode(int motor_id, int target_value)
    : Node("motor_pid_node"), motor_id_(motor_id), target_value_(target_value)
    {
        motor_pub_ = this->create_publisher<interface::msg::Motor>("motor_cmd", 10);
        motor_sub_ = this->create_subscription<interface::msg::Motor>(
            "motor_status", 10,
            std::bind(&MotorPIDNode::motor_status_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(20ms, std::bind(&MotorPIDNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "PID节点已启动，目标值: %d", target_value_);
    }

private:
    
    // 速度环PID参数，可根据实际调整
    double kp_ = 0.5, ki_ = 0.01, kd_ = 0.0;
    double error_sum_ = 0.0, last_error_ = 0.0;
    int speed_feedback_ = 0;

    int motor_id_;
    int target_value_; 

    rclcpp::Publisher<interface::msg::Motor>::SharedPtr motor_pub_;
    rclcpp::Subscription<interface::msg::Motor>::SharedPtr motor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 订阅电机反馈
    void motor_status_callback(const interface::msg::Motor::SharedPtr msg)
    {
        if (msg->motor_id == motor_id_) {
            speed_feedback_ = msg->speed; // 速度反馈 (RPM)
        }
    }

    // PID控制主循环
    void timer_callback()
    {
    // 1. 计算误差（目标速度 - 当前速度）
    double error = target_value_ - speed_feedback_;
    error_sum_ += error;
    double d_error = error - last_error_;
    last_error_ = error;

    // 2. PID输出（电流）
    double output = kp_ * error + ki_ * error_sum_ + kd_ * d_error;
    // 输出限幅 ±16000
    if (output > 16000) output = 16000;
    if (output < -16000) output = -16000;

    // 3. 生成并发布电流控制命令
    interface::msg::Motor msg;
    msg.mode = msg.MODE_CURRENT;
    msg.motor_id = motor_id_;
    msg.current = static_cast<int>(output);

    motor_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "目标速度:%d RPM 当前速度:%d RPM 输出电流:%d", target_value_, speed_feedback_, msg.current);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 创建临时node用于参数声明和获取
    auto temp_node = std::make_shared<rclcpp::Node>("motor_pid_param_helper");
    temp_node->declare_parameter<int>("motor_id", 1);
    temp_node->declare_parameter<int>("target_value", 0);
    int motor_id = temp_node->get_parameter("motor_id").as_int();
    int target_value = temp_node->get_parameter("target_value").as_int();

    if (motor_id < 1 || motor_id > 4) {
        printf("电机ID必须在1-4之间 (当前: %d)\n", motor_id);
        return 1;
    }

    auto node = std::make_shared<MotorPIDNode>(motor_id, target_value);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
