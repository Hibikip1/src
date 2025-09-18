#include <rclcpp/rclcpp.hpp>
#include "robotcontrol/bmcan_bus.hpp"
#include "interface/msg/motor.hpp"
#include <chrono>
#include <vector>

// M3508电机数据结构
struct Motor3508Data {
    uint16_t ecd;           // 机械转角 (0-8191)
    int16_t speed_rpm;      // 电机转速
    int16_t given_current;  // 扭矩电流
    uint8_t temperate;      // 温度
    int16_t last_ecd;       // 上一次机械转角
};

class MotorSend : public rclcpp::Node
{ 
public:
    MotorSend() : Node("motor_send_node")
    {
        
        motors_.resize(4); 
        
        RCLCPP_INFO(this->get_logger(), "%s节点已启动.", this->get_name());
        
        // 打开CAN通道
        BM_NotificationHandle result = canbus.open(channelhandle, "BM-CANFD-X1(5850)"); // 6059 5850
        if (result == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "CAN设备打开失败!");
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN设备已打开");
        }
        
        // 创建发布者和订阅者
        motor_pub_ = this->create_publisher<interface::msg::Motor>("motor_status", 10);
        motor_sub_ = this->create_subscription<interface::msg::Motor>(
            "motor_cmd", 10, 
            std::bind(&MotorSend::motor_callback, this, std::placeholders::_1));
        
        // 创建定时器，用于定时发送心跳或状态查询
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&MotorSend::timer_callback, this));
    }
    
    ~MotorSend() {
        // 关闭CAN通道
        canbus.close(channelhandle);
        RCLCPP_INFO(this->get_logger(), "CAN设备已关闭");
    }

private:
    BMCANTool canbus;
    BM_ChannelHandle channelhandle;
    
    // CAN通信相关ID
    const int cantx_id = 0x200; // 发送给电机的ID，C620电调使用0x200
    const int canrx_id = 0x201; // 接收电机反馈的ID，M3508电机1的反馈ID为0x201
    
    // ROS2发布者和订阅者
    rclcpp::Publisher<interface::msg::Motor>::SharedPtr motor_pub_;
    rclcpp::Subscription<interface::msg::Motor>::SharedPtr motor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 电机数据存储
    std::vector<Motor3508Data> motors_;
    
    // 定时器回调函数：每次循环多读几帧，实现类似中断效果
    void timer_callback() {
        int max_read = 10; // 每次最多读取10帧，防止死循环
        for (int i = 0; i < max_read; ++i) {
            uint8_t rxdata[8] = {0};
            BM_StatusTypeDef recv_ret = canbus.can_receive(channelhandle, canrx_id, rxdata, 1); // 1ms超时，快速轮询
            if (recv_ret == BM_ERROR_OK) {
                RCLCPP_INFO(this->get_logger(), "can_receive: ret=%d, canrx_id=0x%X, rxdata[0]=%02X", recv_ret, canrx_id, rxdata[0]);
                process_motor_feedback(canrx_id, rxdata);
            } else {
                // 没有更多新数据，提前退出
                break;
            }
        }
    }
    
    // 处理电机反馈数据
    void process_motor_feedback(int id, uint8_t* data) {
        // 打印原始CAN反馈数据，便于调试协议
        RCLCPP_INFO(this->get_logger(), "CAN反馈原始data: %02X %02X %02X %02X %02X %02X %02X %02X",
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        int motor_id = id - 0x200;
        if (motor_id >= 1 && motor_id <= 4) {
            int index = motor_id - 1;
            
            // 保存上一次角度
            motors_[index].last_ecd = motors_[index].ecd;
            
            // 解析电机反馈数据
            motors_[index].ecd = (uint16_t)(data[0] << 8 | data[1]);
            motors_[index].speed_rpm = (int16_t)(data[2] << 8 | data[3]);
            motors_[index].given_current = (int16_t)(data[4] << 8 | data[5]);
            motors_[index].temperate = data[6];
            
            // 使用角度差分计算速度（如果CAN协议速度为0）
            if (motors_[index].speed_rpm == 0) {
                // 计算角度差值，考虑8191->0的圈数变化
                int diff = (int)motors_[index].ecd - (int)motors_[index].last_ecd;
                if (diff < -4096) diff += 8192;      // 当过零点时，如0->8191
                else if (diff > 4096) diff -= 8192;  // 当过零点时，如8191->0
                
                // 使用角度差值和时间间隔计算速度（单位：RPM）
                // 假设定时器回调间隔为100ms
                // 计算公式：差值 * (1000ms/100ms) * (60s/1min) / 8192（一圈的角度）
                motors_[index].speed_rpm = (int16_t)(diff * 10.0f * 60.0f / 8192.0f);
            }
            
            // 发布电机状态
            auto msg = interface::msg::Motor();
            msg.motor_id = motor_id;
            msg.angle = motors_[index].ecd;
            msg.speed = motors_[index].speed_rpm;
            msg.current = motors_[index].given_current;
            msg.temperature = motors_[index].temperate;
            
            // 将数据复制到原始数据字段
            for (int i = 0; i < 8; i++) {
                msg.data[i] = data[i];
            }
            
            motor_pub_->publish(msg);
            
            RCLCPP_INFO(this->get_logger(), 
                "电机%d反馈: 角度=%u (0-8191), 速度=%d RPM, 电流=%d, 温度=%u°C",
                motor_id, motors_[index].ecd, motors_[index].speed_rpm, 
                motors_[index].given_current, motors_[index].temperate);
        }
    }
    
    // 发送电机控制命令
    void send_motor_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
        uint8_t chassis_can_send_data[8] = {0};
        chassis_can_send_data[0] = (motor1 >> 8) & 0xFF;
        chassis_can_send_data[1] = motor1 & 0xFF;
        chassis_can_send_data[2] = (motor2 >> 8) & 0xFF;
        chassis_can_send_data[3] = motor2 & 0xFF;
        chassis_can_send_data[4] = (motor3 >> 8) & 0xFF;
        chassis_can_send_data[5] = motor3 & 0xFF;
        chassis_can_send_data[6] = (motor4 >> 8) & 0xFF;
        chassis_can_send_data[7] = motor4 & 0xFF;
        
        RCLCPP_INFO(this->get_logger(), "发送电机控制命令: 电机1=%d, 电机2=%d, 电机3=%d, 电机4=%d", 
                  motor1, motor2, motor3, motor4);
        
        try {
            canbus.can_send(channelhandle, cantx_id, chassis_can_send_data, 1000);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送电机控制命令失败: %s", e.what());
        }
    }
    
    // 订阅者回调函数，处理接收到的电机控制命令
    void motor_callback(const interface::msg::Motor::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到电机控制指令: %d", msg->cmd);
        
        if (msg->mode == msg->MODE_CURRENT) {
            // 电流控制模式
            int16_t current_values[4] = {0, 0, 0, 0};
            
            if (msg->motor_id >= 1 && msg->motor_id <= 4) {
                // 单电机控制
                current_values[msg->motor_id - 1] = msg->current;
            } else if (msg->motor_id == 0) {
                // 直接使用data字段的原始数据
                send_motor_cmd(
                    (int16_t)((msg->data[0] << 8) | msg->data[1]),
                    (int16_t)((msg->data[2] << 8) | msg->data[3]),
                    (int16_t)((msg->data[4] << 8) | msg->data[5]),
                    (int16_t)((msg->data[6] << 8) | msg->data[7])
                );
                return;
            }
            
            send_motor_cmd(
                current_values[0], 
                current_values[1],
                current_values[2],
                current_values[3]
            );
        } else if (msg->mode == msg->MODE_SPEED) {
            // 速度控制模式 - 需要先进行转换再发送
            // 大疆M3508电机转速和输出电流值的转换需要具体参考文档
            
            // 这里简单转换，实际应用中可能需要PID控制
            int16_t current_values[4] = {0, 0, 0, 0};
            
            if (msg->motor_id >= 1 && msg->motor_id <= 4) {
                // 简单的速度到电流的转换（示例）
                // 实际使用中需要调整系数或使用PID控制
                current_values[msg->motor_id - 1] = msg->speed / 10;
            }
            
            send_motor_cmd(
                current_values[0], 
                current_values[1],
                current_values[2],
                current_values[3]
            );
        } else {
            // 未知模式，直接发送通用命令
            uint8_t candata[8] = {0};
            candata[0] = msg->cmd & 0xFF;
            
            try {
                canbus.can_send(channelhandle, cantx_id, candata, 1000);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "发送通用命令失败: %s", e.what());
            }
        }
    }
};

int main(int argc, char **argv)
{ 
    rclcpp::init(argc, argv);
    auto motor_send_node = std::make_shared<MotorSend>();

    rclcpp::spin(motor_send_node);
    rclcpp::shutdown();
    return 0;
}