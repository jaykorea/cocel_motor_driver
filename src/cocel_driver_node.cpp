#include <cstdio>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <thread>
#include <vector>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"

// custom msgs
#include "cocel_driver/msg/motor_driver_cmd.hpp"
#include "cocel_driver/msg/cocel_driver_cmd.hpp"


#include "motor_driver/MotorDriver.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


uint64_t get_time_in_microseconds() {
  uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  return us;
}

void printMotorStates(const std::map<int, motor_driver::motorState>& motor_states) {
  for (const auto& state : motor_states) {
    std::cout << "Motor ID: " << state.first 
              << " Position: " << state.second.position
              << " Velocity: " << state.second.velocity
              << " Torque: " << state.second.torque << std::endl;
  }
}

class MotorConfigurator : public rclcpp::Node {
public:
    MotorConfigurator() : Node("motor_configurator") {
        declare_parameters();
        load_parameters();
    }

    std::string getCanInterface() const {
        return can_interface_;
    }

    std::vector<int> getMotorIds() const {
        return motor_ids_;
    }

    motor_driver::MotorType getMotorType() const {
        return motor_type_;
    }

private:
    std::string can_interface_;
    std::vector<int> motor_ids_;
    motor_driver::MotorType motor_type_;
    std::string motor_type_str;

    void declare_parameters() {
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<int>("left_hip_motor_id", 1);
        this->declare_parameter<int>("left_thigh_motor_id", 2);
        this->declare_parameter<int>("left_leg_motor_id", 3);
        this->declare_parameter<int>("right_hip_motor_id", 4);
        this->declare_parameter<int>("right_thigh_motor_id", 5);
        this->declare_parameter<int>("right_leg_motor_id", 6);
        this->declare_parameter<std::string>("motor_type", "AK70_10V1p1");
        this->declare_parameter<float>("POS_MIN", 0);
        this->declare_parameter<float>("POS_MAX", 0);
        this->declare_parameter<float>("VEL_MIN", 0);
        this->declare_parameter<float>("VEL_MAX", 0);
        this->declare_parameter<float>("KP_MIN", 0);
        this->declare_parameter<float>("KP_MAX", 0);
        this->declare_parameter<float>("KD_MIN", 0);
        this->declare_parameter<float>("KD_MAX", 0);
        this->declare_parameter<float>("T_MIN", 0);
        this->declare_parameter<float>("T_MAX", 0);
    }

    void load_parameters() {
        RCLCPP_INFO(this->get_logger(), "Loading Parameters...");
        std::vector<int> motor_ids;
        int motor_id;
        float pos_min;
        float pos_max;
        float vel_min;
        float vel_max;
        float kp_min;
        float kp_max;
        float kd_min;
        float kd_max;
        float t_min;
        float t_max;

        if(!this->get_parameter("can_interface", can_interface_)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get can_interface parameter.");
        }
        if(!this->get_parameter("left_hip_motor_id", motor_id)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get left_hip_motor_id parameter.");
        } else motor_ids.push_back(motor_id);
        if(!this->get_parameter("left_thigh_motor_id", motor_id)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get left_thigh_motor_id parameter.");
        } else motor_ids.push_back(motor_id);
        if(!this->get_parameter("left_leg_motor_id", motor_id)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get left_leg_motor_id parameter.");
        } else motor_ids.push_back(motor_id);
        if(!this->get_parameter("right_hip_motor_id", motor_id)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get right_hip_motor_id parameter.");
        } else motor_ids.push_back(motor_id);
        if(!this->get_parameter("right_thigh_motor_id", motor_id)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get right_thigh_motor_id parameter.");
        } else motor_ids.push_back(motor_id);
        if(!this->get_parameter("right_leg_motor_id", motor_id)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get right_leg_motor_id parameter.");
        } else motor_ids.push_back(motor_id);
        if(!this->get_parameter("motor_type", motor_type_str)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get motor_type parameter.");
        }
        if(!this->get_parameter("POS_MIN", pos_min)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get pos_min parameter.");
        }
        if(!this->get_parameter("POS_MAX", pos_max)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get pos_max parameter.");
        }
        if(!this->get_parameter("VEL_MIN", vel_min)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get vel_min parameter.");
        }
        if(!this->get_parameter("VEL_MAX", vel_max)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get vel_max parameter.");
        }
        if(!this->get_parameter("KP_MIN", kp_min)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get kp_min parameter.");
        }      
        if(!this->get_parameter("KP_MAX", kp_max)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get kp_max parameter.");
        }      
        if(!this->get_parameter("KD_MIN", kd_min)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get kd_min parameter.");
        }
        if(!this->get_parameter("KD_MAX", kd_max)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get kd_max parameter.");
        }      
        if(!this->get_parameter("T_MIN", t_min)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get t_min parameter.");
        }                
        if(!this->get_parameter("T_MAX", t_max)){
          RCLCPP_ERROR(this->get_logger(), "Failed to get t_max parameter.");
        }      
        
        motor_ids_ = motor_ids;
        motor_type_ = stringToMotorType(motor_type_str);
    } 

    motor_driver::MotorType stringToMotorType(const std::string& type) {
        static const std::unordered_map<std::string, motor_driver::MotorType> typeMap = {
            {"AK70_10V1p1", motor_driver::MotorType::AK70_10V1p1},
            {"AK80_6_V1", motor_driver::MotorType::AK80_6_V1},
            {"AK80_6_V1p1", motor_driver::MotorType::AK80_6_V1p1},
            {"AK80_6_V2", motor_driver::MotorType::AK80_6_V2},
            {"AK80_9_V1p1", motor_driver::MotorType::AK80_9_V1p1},
            {"AK80_9_V2", motor_driver::MotorType::AK80_9_V2},
            {"AK70_10V1p1", motor_driver::MotorType::AK70_10V1p1},
            {"AK10_9_V1p1", motor_driver::MotorType::AK10_9_V1p1}
        };

        auto it = typeMap.find(type);
        if (it != typeMap.end()) {
            return it->second;
        } else {
            throw std::runtime_error("Unknown motor type: " + type);
        }
    }
};

class MotorStatusPublisher : public rclcpp::Node {
public:
    MotorStatusPublisher(motor_driver::MotorDriver* motor_controller, std::vector<int>* motor_ids)
    : Node("motor_status_publisher") {
        this->motor_controller = motor_controller;
        this->motor_ids = motor_ids;
        motor_status_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_status", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MotorStatusPublisher::publishMotorStatus, this));
    }

private:
    void publishMotorStatus() {
        auto motor_states = this->motor_controller->getMotorState(*this->motor_ids);
        std_msgs::msg::Float32MultiArray msg;

        for (const auto& state : motor_states) {
            msg.data.push_back(static_cast<float>(state.first)); // Motor ID
            msg.data.push_back(state.second.position); // Position
            msg.data.push_back(state.second.velocity); // Velocity
            msg.data.push_back(state.second.torque); // Torque
        }

        motor_status_publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    motor_driver::MotorDriver* motor_controller;
    std::vector<int>* motor_ids;
};

class CommandSubscriber : public rclcpp::Node {
public:
  CommandSubscriber(motor_driver::MotorDriver* motor_controller, std::vector<int>* motor_ids)
  : Node("command_subscriber_node") {
    this->motor_controller = motor_controller;
    this->motor_ids = motor_ids;

    subscription_cmd = this->create_subscription<cocel_driver::msg::CocelDriverCmd>(
      "cmd_topic", 10, std::bind(&CommandSubscriber::cmd_callback, this, _1));

    subscription_stop = this->create_subscription<std_msgs::msg::Empty>(
      "stop_topic", 10, std::bind(&CommandSubscriber::stop_callback, this, _1));
  }

private:
  void cmd_callback(const cocel_driver::msg::CocelDriverCmd::SharedPtr msg) {
      //RCLCPP_INFO(this->get_logger(), "Sub Called...");
      controlMotors(msg);
  }
  void stop_callback(const std_msgs::msg::Empty::SharedPtr msg) {
      //RCLCPP_INFO(this->get_logger(), "Sub Called...");
      stopMotors();
  }
  void controlMotors(const cocel_driver::msg::CocelDriverCmd::SharedPtr msg) {

    std::map<int, motor_driver::motorCommand> commandMap;
    
    for (size_t i = 0; i < msg->motor_cmd.size(); ++i) {
        const auto& cmd = msg->motor_cmd[i];
        int motorId = msg->motor_id[i]; // motor_id와 motor_cmd의 인덱스를 매칭

        motor_driver::motorCommand moveCommandStruct = {cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau_ff};

        commandMap[motorId] = moveCommandStruct;
    }

    RCLCPP_INFO(this->get_logger(), "Moving motors...");

    auto commandState = this->motor_controller->sendRadCommand(commandMap);
    //printMotorStates(commandState);

    //std::this_thread::sleep_for(std::chrono::milliseconds(10));  // How much time should I wait here??
  }

  void stopMotors() {
    motor_driver::motorCommand stopCommandStruct = {0, 0, 0, 0, 0};
    std::map<int, motor_driver::motorCommand> stopCommandMap;
    for (int id : *this->motor_ids) {
      stopCommandMap[id] = stopCommandStruct;
    }
    auto commandState = this->motor_controller->sendRadCommand(stopCommandMap);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  motor_driver::MotorDriver* motor_controller;
  std::vector<int>* motor_ids;
  rclcpp::Subscription<cocel_driver::msg::CocelDriverCmd>::SharedPtr subscription_cmd;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_stop;
};

void InitMotors(motor_driver::MotorDriver& motor_controller, std::vector<int>& motor_ids) {
  std::cout << "Initializing Motors..." << std::endl;

  // for (int id : motor_ids) {
  //   std::cout << "ID: " << id << std::endl;
  // }

  auto start_state = motor_controller.enableMotor(motor_ids);
  printMotorStates(start_state);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::cout << "Setting Zero Position..." << std::endl;
  auto stateZero = motor_controller.setZeroPosition(motor_ids);
  printMotorStates(stateZero);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void DisableMotors(motor_driver::MotorDriver& motor_controller, std::vector<int>& motor_ids) {

  std::cout << "Disabling Motor..." << std::endl;
  auto end_state = motor_controller.disableMotor(motor_ids);
  printMotorStates(end_state);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto motor_configurator_node = std::make_shared<MotorConfigurator>();

  std::vector<int> motor_ids = motor_configurator_node->getMotorIds();
  std::string can_interface = motor_configurator_node->getCanInterface();
  motor_driver::MotorType motor_type = motor_configurator_node->getMotorType();

  motor_driver::MotorDriver motor_controller(motor_ids, can_interface.c_str(), motor_type);

  InitMotors(motor_controller, motor_ids);

  auto subscriber_node = std::make_shared<CommandSubscriber>(&motor_controller, &motor_ids);
  auto publisher_node = std::make_shared<MotorStatusPublisher>(&motor_controller, &motor_ids);

  std::thread subscriber_thread([&]() { rclcpp::spin(subscriber_node); });
  std::thread publisher_thread([&]() { rclcpp::spin(publisher_node); });

  subscriber_thread.join();
  publisher_thread.join();

  rclcpp::shutdown();
  DisableMotors(motor_controller, motor_ids);
  
  return 0;
}