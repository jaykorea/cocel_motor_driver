#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "cocel_driver/msg/cocel_driver_cmd.hpp"
#include "cocel_driver/msg/motor_driver_cmd.hpp"

int main(int argc, char* argv[])
{
    // ROS 2 노드 초기화
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("motor_command_publisher");
    
    // 퍼블리셔 생성
    auto publisher = node->create_publisher<cocel_driver::msg::CocelDriverCmd>("/cmd_topic", 10);
    cocel_driver::msg::CocelDriverCmd cmd_msg;
    
    // 모터 ID 설정
    std::vector<int> motor_ids = {2, 9, 11, 7, 5, 6};
    cmd_msg.motor_id = motor_ids;

    // 명령줄 인수를 통해 받은 값으로 모터 명령 구성
    for (int i = 0; i < motor_ids.size(); ++i) {
        cocel_driver::msg::MotorDriverCmd motor_cmd;
        motor_cmd.pos = std::stof(argv[1]); // 첫 번째 인자로 모든 모터의 pos 설정
        motor_cmd.vel = std::stof(argv[2]); // 두 번째 인자로 모든 모터의 vel 설정
        motor_cmd.kp = std::stof(argv[3]);  // 세 번째 인자로 모든 모터의 kp 설정
        motor_cmd.kd = std::stof(argv[4]);  // 네 번째 인자로 모든 모터의 kd 설정
        motor_cmd.tau_ff = std::stof(argv[5]); // 다섯 번째 인자로 모든 모터의 tau_ff 설정
        cmd_msg.motor_cmd.push_back(motor_cmd);
    }

    // 메시지 발행
    publisher->publish(cmd_msg);
    RCLCPP_INFO(node->get_logger(), "Published motor command");

    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
