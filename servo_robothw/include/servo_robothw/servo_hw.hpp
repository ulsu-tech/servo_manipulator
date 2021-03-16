#pragma once

#include <string>
#include <vector>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/controller_info.h>

#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

#define TOTAL_ROTATE_JOINTS_COUNT 6
#define ZERO_STREAM_FILENAME "zero_positions.zrs"

using std::string;
namespace servo_hardware_interface {

class ServoHW : public hardware_interface::RobotHW 
{
    //PositionJointInterface positionJointInterface;
    //PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
    double loop_hz_;
    //boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    double p_error_, v_error_, e_error_;
  private:
    ros::NodeHandle nh_;
    //interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_pos_interface;

    int num_joints;
    std::vector<string> joint_name;
    std::string iface_name;

    //actual states
    std::vector<double> joint_position_state;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;

    //std::vector<int> JOINT_SIGN;

    //given setpoints
    std::vector<double> joint_position_command;

    void updateThread();
    bool keepUpdates;

    pthread_t pth_pid;
    bool enableReadWrite;
    void loadPositionsFromFile();
    void storeHomePositions();

    bool isRunning;

  public:
    static void *threadFunc(void *);
    ServoHW();
    virtual ~ServoHW();
    static void CallbackParser(const std_msgs::String::ConstPtr&, std::string&, int&);

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;
    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) override;
    void doSwitch(const std::list<hardware_interface::ControllerInfo>& /*start_list*/,
                        const std::list<hardware_interface::ControllerInfo>& /*stop_list*/) override;
    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;

    //callback interface for commands
    void commandsCallback(const std_msgs::String::ConstPtr&);
    //callback interface for wrist
    void wristCallback(const std_msgs::String::ConstPtr&);
    //callback interface for pneumatic
    void pneumaticCallback(const std_msgs::String::ConstPtr&);
    // this particular implementation specific call
    bool setEnabled();

    void batteryHandlerCallback(const std_msgs::String::ConstPtr&);
    bool isActive() const { return isRunning; };

    static const uint16_t PRE_DEFINED_ERROR_FIELD_SLOTNUM;
    static const int errorFieldsCount;
    bool inited;

  private:
        double Ax_motor_link_reduction[TOTAL_ROTATE_JOINTS_COUNT];
        const double FestoSmallReductorMulti = 1./3.;
        const double Cilindric_A6_transmission = 56. / 35.;
        const double Wave_A6_transmission = 1. / 121.;
        const double A6_motor_link_reduction = FestoSmallReductorMulti / Cilindric_A6_transmission * Wave_A6_transmission;

        const double Cilindric_A5_transmission = 56. / 35.;
        const double Wave_A5_transmission = 1. / 160.;
        const double A5_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A5_transmission * Wave_A5_transmission;

        
        const double Cilindric_A4_transmission = 56. / 35.;
        const double Wave_A4_transmission = 1. / 161.;
        const double A4_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A4_transmission * Wave_A4_transmission;

        const double Cilindric_A3_transmission = 74. / 35.;
        const double Wave_A3_transmission = 1. / 161.;
        const double A3_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A3_transmission * Wave_A3_transmission;

        const double Cilindric_A2_transmission = 74. / 35.;
        const double Wave_A2_transmission = 1. / 161.;
        const double A2_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A2_transmission * Wave_A2_transmission;

        const double Cilindric_A1_transmission = 74. / 35.;
        const double Wave_A1_transmission = 1. / 161.;
        const double A1_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A1_transmission * Wave_A1_transmission;


    // influencing on 1-st index from 2nd index
        const double jointInfluence[TOTAL_ROTATE_JOINTS_COUNT - 1][TOTAL_ROTATE_JOINTS_COUNT - 1] =
    {   {   Wave_A2_transmission,        1e6, 1e6, 1e6, 1e6 }, // influence on joint 2 from previous ( 1 )
        {   Wave_A3_transmission,   Wave_A3_transmission, 1e6, 1e6, 1e6},  //influence on joint 3 from previous (1,2)
        { Wave_A4_transmission, Wave_A4_transmission, Wave_A4_transmission, 1e6, 1e6}, // on joint 4
        { Wave_A5_transmission, Wave_A5_transmission, Wave_A5_transmission, Wave_A5_transmission, 1e6}, // on joint 5
        { Wave_A6_transmission, Wave_A6_transmission, Wave_A6_transmission, Wave_A6_transmission, -Wave_A6_transmission} // on joint 6
    };
};
};
