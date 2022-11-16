#ifndef MOTOR_CONTROLLER_NODE_HPP
#define MOTOR_CONTROLLER_NODE_HPP

#include "RoboClaw.h"
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"



constexpr uint32_t ROBOCLAW_TIMEOUT_MS =  30;
constexpr uint8_t ROBOCLAW_ADDRESS = 0x80;


#define TARGET_ANGULAR_VELOCITY_TOPIC "target_angular_velocity"
#define ANGULAR_VELOCITY_TOPIC  "angular_velocity"
#define CURRENT_TOPIC   "current"

#define FREQUENCY_HZ 100

class motor_controller_node: public rclcpp :: Node
{
    public:
    motor_controller_node(std::string motor_id);

    
    


    private:
    std::vector <std::string> param_names_;
    std::string motor_id_;
    double ticks_per_rotation;
    bool motor_direction_inverse;
    bool encoder_direction_inverse;
    
    SERIAL_OBJ serialPort;
    RoboClaw roboclaw = RoboClaw(&serialPort,ROBOCLAW_TIMEOUT_MS);
    

    
    rclcpp::CallbackGroup::SharedPtr motor_callback_group;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_angular_velocity_subscriber;
    rclcpp::TimerBase::SharedPtr publisher_timer;


    void publish_angular_velocity(float &angular_velocity);
    void publish_current_mA(float &current);
    void publish_timer_callback(void);
    void target_angular_velocity_callback(std_msgs::msg::Float64::SharedPtr msg);
    float ticks_to_rads(int32_t &velocity_ticks);
    int32_t rads_to_ticks(float &rads);
    float get_angular_velocity(bool &valid_speed1);
    int16_t get_current_mA(void);
   
};


#endif //MOTOR_CONTROLLER_NODE_HPP