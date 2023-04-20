
#include "motor_controller_node.hpp"
#include <unistd.h>
#include <memory>
#include <chrono>

#define _USE_MATH_DEFINES
#include <cmath>
#define FE_TONEAREST 

motor_controller_node::motor_controller_node(std::string motor_id,bool publish_main_battery_voltage) : Node (motor_id+"_motor_control")
{
   this->motor_id_ = motor_id;
   this->publish_main_battery_voltage_ = publish_main_battery_voltage;
   
   param_names_ = {  motor_id + "_dev_name",
                    motor_id + "_baudrate",
                    motor_id + "_ticks_per_rotation",
                    motor_id + "_motor_direction_inverse",
                    motor_id + "_encoder_direction_inverse"

   };

    
    //Declare Parameters 
   for(auto &param : param_names_)
   {
        this->declare_parameter(param,nullptr);
        

   }

    //Setup variables holding parameters
   
    this->ticks_per_rotation = this->get_parameter(this->motor_id_ +"_ticks_per_rotation").as_int();
    this->motor_direction_inverse = this ->get_parameter(this->motor_id_ + "_motor_direction_inverse").as_bool();


    //Start Roboclaw 
    int baudrate = this->get_parameter(this->motor_id_+"_baudrate").as_int();
    std::string dev_name = this->get_parameter(this->motor_id_+"_dev_name").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(),"Port_Name: "<< dev_name<<" Baudrate: " << baudrate);
    roboclaw.begin(baudrate,dev_name.c_str());

    //Setup Callback group
    this->motor_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


    //Setup Topic
    std::string angular_velocity_topic = this->motor_id_ + "/" + ANGULAR_VELOCITY_TOPIC;
    std::string target_angular_velocity_topic = this->motor_id_ + "/" + TARGET_ANGULAR_VELOCITY_TOPIC;
    std:: string current_topic = this->motor_id_ + "/" + CURRENT_TOPIC;
    std::string main_battery_voltage_topic = this->motor_id_ + "/main_battery_voltage" ;

    //Setup Publishers
    angular_velocity_publisher = this->create_publisher<std_msgs::msg::Float64>(angular_velocity_topic,10);
    current_publisher = this->create_publisher<std_msgs::msg::Float64>(current_topic,10);
    if(publish_main_battery_voltage_)
        main_battery_voltage_publisher = this->create_publisher<std_msgs::msg::Float32>(main_battery_voltage_topic,10);
    
    //Setup Subscribers
    target_angular_velocity_subscriber = this->create_subscription<std_msgs::msg::Float64>(target_angular_velocity_topic,10,
    std::bind(&motor_controller_node::target_angular_velocity_callback,this,std::placeholders::_1));

    //Setup Timers
    publisher_timer = this->create_wall_timer(std::chrono::microseconds(10000),
    std::bind(&motor_controller_node::publish_timer_callback,this));
    
}

float motor_controller_node:: get_angular_velocity(bool &valid_speed1)
{
    
    uint8_t status1;
    int32_t ticks_1 = (uint32_t)roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status1, &valid_speed1);
    if(valid_speed1)
    {
        float rads_1 = ticks_to_rads(ticks_1);
        if(this->motor_direction_inverse)
        {
            rads_1 *= -1;
        }
        //RCLCPP_INFO_STREAM(this->get_logger(),"Ticks/s Published: "<<ticks_1 << " Rad/s: "<<rads_1);
        return rads_1;
        
    }

    else
    {
        return NULL;
    }
}

int16_t motor_controller_node::get_current_mA()
{
    int16_t current1;
    bool valid  = roboclaw.ReadCurrents(ROBOCLAW_ADDRESS,current1);
    if(valid)
    {
        current1 *= 10;
        return current1;
    }
    else
    {
        return NULL;
    }
}

void motor_controller_node:: publish_angular_velocity(float &angular_velocity)
{
    auto msg = std_msgs::msg::Float64(); 
    msg.data = angular_velocity;
    angular_velocity_publisher->publish(msg);
    
}

void motor_controller_node:: publish_current_mA(float &current)
{
    auto msg = std_msgs::msg::Float64(); 
    msg.data = current;
    current_publisher->publish(msg);
}

void motor_controller_node:: target_angular_velocity_callback(std_msgs::msg::Float64::SharedPtr msg)
{
    float angular_velocity = msg->data;
    int32_t ticks = rads_to_ticks(angular_velocity);
    if(this->motor_direction_inverse)
    {
        ticks *= -1;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),"Msg Received: "<<motor_id_<< ": Angular Vel: "<<angular_velocity << " Ticks/sec: "<< ticks);
    roboclaw.SpeedM1(ROBOCLAW_ADDRESS,(uint32_t)ticks);
}


float motor_controller_node::ticks_to_rads(int32_t &velocity_ticks)
{
    float rads =  (2* M_PI * velocity_ticks)/((float)this->ticks_per_rotation);
    return rads;
}

int32_t motor_controller_node::rads_to_ticks(float &rads)
{
    int ticks = std::lrint((rads * this->ticks_per_rotation)/(float)( 2* M_PI)) ;

    return ticks;
}


float motor_controller_node::get_main_battery_voltage()
{
    bool valid  = false;
    float battery_voltage = roboclaw.ReadMainBatteryVoltage(ROBOCLAW_ADDRESS,&valid);   
    if(valid)
    {
        return battery_voltage/(float)10;
    }
    
    return NULL;
}

void motor_controller_node::publish_main_battery_voltage(float &battery_voltage)
{
    auto msg = std_msgs::msg::Float32();
    msg.data = battery_voltage;
    main_battery_voltage_publisher->publish(msg);
}

void motor_controller_node:: publish_timer_callback()
{
    bool valid_speed = false;
    float angular_velocity = get_angular_velocity(valid_speed);
    if (valid_speed)
    {
        publish_angular_velocity(angular_velocity);
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),motor_id_<<": Error Getting Angular Speed");
    }

    float current1 = (float) get_current_mA();
    if(current1 != NULL)
    {
        publish_current_mA(current1);
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),motor_id_<<": Error Getting Current Reading");
    }

    if(publish_main_battery_voltage_)
    {
        float main_battery_voltage = get_main_battery_voltage();
        if(main_battery_voltage != NULL)
        {
            publish_main_battery_voltage(main_battery_voltage);
        }

        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(),motor_id_<<": Error Getting Main Battery Voltage Reading");
        }
    }
}



int main(int argc,char** argv)
{
    
    rclcpp::init(argc, argv);
    auto motor_front_right = std::make_shared<motor_controller_node>("front_right",true);
    auto motor_front_left = std::make_shared<motor_controller_node>("front_left",false);
    auto motor_rear_right = std::make_shared<motor_controller_node>("rear_right",false);
    auto motor_rear_left = std::make_shared<motor_controller_node>("rear_left",false);
    rclcpp::executors::MultiThreadedExecutor m_executor;
    m_executor.add_node(motor_front_right);
    m_executor.add_node(motor_front_left);
    m_executor.add_node(motor_rear_right);
    m_executor.add_node(motor_rear_left);
    m_executor.spin();
    
    
    rclcpp::shutdown();
    return 0;
}