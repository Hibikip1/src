#include "rclcpp/rclcpp.hpp"
#include "bmcan_bus.hpp"


class RobotControl : public rclcpp::Node
{
public:
    RobotControl(std::string name) : Node(name)
    {
        // rclcpp::Qos qos_profile(10);
        // qos_profile.reliable();
        // rclcpp::QoS qos_profile(1000); // 定义队列深度为 10
        // qos_profile.keep_last(100);   // 保留最近 20 条消息
        // qos_profile.reliable();      // 使用可靠传输
        RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());
        canbus.open(channelhandle, "BM-CANFD-X1(5852)");//6059  5852
        // cmd_vel_chassis_sub_ = this->create_subscription<interface::msg::Armor>(
        //     "auto_aid/Armor",100,
        //     std::bind(&RobotControl::cmd_vel_chassis_callback, this, std::placeholders::_1));
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&RobotControl::timer_callback, this));
    }
    ~RobotControl(){
        canbus.close(channelhandle);
    }

private:
    /*创建bmcan 相关参数*/
    BMCANTool canbus;
    BM_ChannelHandle channelhandle;
    const int cantx_id = 0x200;
    const int canrx_id = 0x201;
    /*创建cmd_vel_chassis话题订阅者*/
    // rclcpp::Subscription<interface::msg::Armor>::SharedPtr cmd_vel_chassis_sub_;
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;

    /*创建cmd_vel_chassis订阅回调函数*/
    // void cmd_vel_chassis_callback(const interface::msg::Armor msg)
    // {   
    //     std::cout<<"test"<<std::endl;
    //     //将cmd_chassis_vel数据通过bmcan 发送到机器人
        int16_t linger_x = msg.x*10 ;//* 65535/1000;
        int16_t linger_y = msg.y*10 ;//* 65535/1000;
        // int16_t linger_x = 1;//* 65535/1000;
        // int16_t linger_y = 1;//* 65535/1000;
        int16_t distance = msg.distance*10;//* 65535/1000;
        int16_t num_ = msg.armor_num;
        uint8_t candata[8] = {0};
        if(linger_x == 0 && linger_y == 0)
        {
            candata[0] = 10;//没有识别到装甲板
        }
        else if(num_ == 10)
        {
            candata[0] = 10;//没有识别到装甲板上的数字
        }
        else if(num_ == 1)
        {
            candata[0] = 1;//识别到的数字是1
        }
        else if(num_ == 2)
        {
            candata[0] = 2;//识别到的数字是2
        }
        else if(num_ == 3)
        {
            candata[0] = 3;//识别到的数字是3
        }
        else if(num_ == 4)
        {
            candata[0] = 4;//识别到的数字是4
        }
        else if(num_ == 9)
        {
            candata[0] = 9;//识别到的装甲板是哨兵
        }
        else
        {
            candata[0] = 0;//没有识别到的装甲板
        }
        if(linger_y == 0 && linger_x == 0)
        {
            candata[1] = 0;//是否发射
        }
        else if(linger_x <= 800 && linger_x >= -800 && linger_y <= 9800 && linger_y >= -9800)
        {
            candata[1] = 1;//是否发射
        }
        else 
        {
            candata[1] = 0;//是否发射
        }
        candata[2]= linger_x & 0xFF;
        candata[3]= (linger_x >> 8) & 0xFF;
        candata[4]= linger_y & 0xFF;
        candata[5]= (linger_y >> 8) & 0xFF;
        candata[6]= distance & 0xFF;
        candata[7]= (distance >> 8) & 0xFF; 
        canbus.can_send(channelhandle, cantx_id, candata, 1000);
    }

    void timer_callback(){
        uint8_t candata[8] = {1};
        // candata[0] = 1;//是否识别到*****改
        // candata[1] = 2;//是否发射*****改
        // candata[2]= 3;
        // candata[3]= 3;
        // candata[4]= 3;
        // candata[5]= 3;
        // candata[6]= 3;
        // candata[7]= 3;
        canbus.can_send(channelhandle, cantx_id, candata, 1000);  
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControl>("robot_control_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

