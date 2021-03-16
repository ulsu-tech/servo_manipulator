#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <servo_robothw/servo_hw.hpp>
#include <controller_manager/controller_manager.h>

#define THIS_NODE_NAME "servo_control_node"

int main(int argc, char** argv){
    ros::init(argc, argv, THIS_NODE_NAME);
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    servo_hardware_interface::ServoHW hw;
    bool init_success = hw.init(nh,nh);
    if (not init_success) {
        ROS_FATAL("Unknown how to proceed with failure on initialization");
        exit(EXIT_FAILURE);
    }

    controller_manager::ControllerManager cm(&hw,nh);

    // EtherCAT network is served at rate 1 exchange / 5 ms ( 200 Hz)
    // TODO switch to Interpolate Positioning mode for Festo
    // as each update of target position takes at least 3 exchanges
    // reducing frequency from 1/200 to 1 / 100
    ros::Duration period(1.0/100); // 100Hz update rate

    ros::Subscriber sub = nh.subscribe("commands", 1000,
        &servo_hardware_interface::ServoHW::commandsCallback, &hw);
    
    ros::Subscriber subWrist = nh.subscribe("wrist", 1000,
        &servo_hardware_interface::ServoHW::wristCallback, &hw);

    ros::Subscriber subPneumatic = nh.subscribe("pneumatic", 1000,
        &servo_hardware_interface::ServoHW::pneumaticCallback, &hw);



    ROS_INFO("servo_control_node started");
    while(ros::ok() && hw.isActive()){
        hw.read(ros::Time::now(), period);
        cm.update(ros::Time::now(), period);
        hw.write(ros::Time::now(), period);
        period.sleep();
    }

    spinner.stop();
  return 0;
}

