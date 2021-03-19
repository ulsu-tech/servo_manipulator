#include "servo_robothw/servo_hw.hpp"

#include <pluginlib/class_list_macros.hpp>


#include <iostream>
#include <ios>
#include <fstream>
#include <unistd.h>

#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sstream>
#include <string>

namespace servo_hardware_interface {

ServoHW::ServoHW() :
      enableReadWrite(false)
    , isRunning(true)
    , inited(false)
    , hasCMDOut(false)
    , cmdLock(PTHREAD_MUTEX_INITIALIZER)
{
    Ax_motor_link_reduction[0] = A1_motor_link_reduction;
    Ax_motor_link_reduction[1] = A2_motor_link_reduction;
    Ax_motor_link_reduction[2] = A3_motor_link_reduction;
    Ax_motor_link_reduction[3] = A4_motor_link_reduction;
    Ax_motor_link_reduction[4] = A5_motor_link_reduction;
    Ax_motor_link_reduction[5] = A6_motor_link_reduction;
}

ServoHW::~ServoHW()
{
    enableReadWrite = false;
    if (keepUpdates) {
        keepUpdates = false;
        pthread_join(pth_pid, nullptr);
    }
    storeHomePositions();

    std::cout<<"ServoHW destructor done"<<std::endl;
}

void * ServoHW::threadFunc(void *arg)
{
    ServoHW * self = reinterpret_cast<ServoHW *>(arg);
    self->updateThread();
    return nullptr;
}

void ServoHW::updateThread()
{
  while(keepUpdates) {
//TODO implement this method
    const char getJointsCMD[] = "x\n";
    ::write(fd_COM, getJointsCMD, 2);
    char buff_input[300];
    int rq = ::read(fd_COM, buff_input, 300);
    // TODO strtok_r will save russian's democracy father
    char *svptr;
    char *line = strtok_r(buff_input, " \t", &svptr);
    if(line == nullptr || strncmp(line, "Angle", 5) != 0)
    {
        usleep(1000);
        continue;
    }
    int i=0;
    while(line = strtok_r(nullptr, " \t", &svptr))
    {
        joint_pos_last_com[i] = atoi(line); // in 0,01 of deg
        i++;
    }

    pthread_mutex_lock(&cmdLock);
    if(hasCMDOut)
    {
        const char bufferRq[]="F\n";
        char rqBuffer[300];
        ::write(fd_COM, bufferRq, 2);
        int rq = ::read(fd_COM, rqBuffer, 300);
        if(rq < 5 || strncmp(rqBuffer, "READY", 5) != 0)
        {
            ROS_INFO_STREAM("confirmation of buffer readiness returned "<<rq
                <<" bytes, with answer "<<rqBuffer);
        }
        else {
            ::write(fd_COM, tgtCMD, strlen(tgtCMD));
ROS_INFO_STREAM("Sending command\n"<<tgtCMD);
            hasCMDOut = false;
        }
    }
    pthread_mutex_unlock(&cmdLock);
    usleep(5000);
#if 0
    std::unique_lock<std::mutex> dataLock { ecn.getReadyMutex()};
    //ecn.getReadyDataCond().wait(dataLock);

    for(auto i = 0; i < festo_controllers_found; ++i) {
        f_cnt[i].updateOutputs();
    }
    ek1828->reflectOutput();
    el2819->reflectOutput();
    el4102->reflectOutput();
    ecn.getReadyDataCond().wait(dataLock);

    if (inited)
        earlyReactOnNewData();
#endif
  }
}

bool ServoHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
    ROS_DEBUG_STREAM("initing. Root NH namespace: "<<root_nh.getNamespace()
        <<"  and robot_hw_nh is "<<robot_hw_nh.getNamespace());
    robot_hw_nh.getParam("joints", joint_name);

    joint_name = std::vector<std::string> { "a1_joint", "a2_joint", "a3_joint",
        "a4_joint", "a5_joint", "a6_joint" };
    /*joint_name.push_back("a1_joint");
    joint_name.push_back("a2_joint");
    joint_name.push_back("a3_joint");
    joint_name.push_back("a4_joint");
    joint_name.push_back("a5_joint");
    joint_name.push_back("a6_joint");
    */
    num_joints = joint_name.size();
    //resize vectors
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    joint_position_command.resize(num_joints);

    joint_pos_last_com.resize(num_joints);

    jsh.resize(num_joints);
    jph.resize(num_joints);

    //Register handles
    for(int i=0; i<num_joints; i++){
        //State
        jsh[i] = hardware_interface::JointStateHandle(joint_name[i], &joint_position_state[i],
            &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jsh[i]);

        //Position
        jph[i] =  hardware_interface::JointHandle(jsh[i], &joint_position_command[i]);
        joint_pos_interface.registerHandle(jph[i]);
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&joint_pos_interface);

    robot_hw_nh.param("iface", iface_name, std::string("ttyACM0"));
//TODO rework from here and so on
    std::string fullPath = std::string("/dev/") + iface_name;
    fd_COM = open(fullPath.c_str(), O_NOCTTY | O_RDWR);
    if (fd_COM < 0)
    {
        ROS_ERROR_STREAM("Failed to open device "<<fullPath<<"  thus, exiting");
        return false;
    }
    
#if 0
    bool ecnStarted = ecn.startNetwork(iface_name.c_str());
    if (! ecnStarted) {
        ROS_ERROR("EtherCAT manager was not started. Are you root?");
        return false;
    } else {
        ROS_INFO("EtherCAT has started");
    }
    auto ecnInSafeOperational = ecn.SwitchSafeOperational();
    if (!ecnInSafeOperational) {
        ROS_ERROR("EtherCAT manager failed to switch to safe operational");
        return false;
    } else {
        ROS_INFO("EtherCAT is in Safe Op mode");
    }

    //TODO
    // iterate over ec_slave[i] and select only those with proper name
    festo_controllers_found = TOTAL_ROTATE_JOINTS_COUNT;
    f_cnt = reinterpret_cast<FestoController*>(std::calloc(
                        festo_controllers_found, sizeof(FestoController)));
    for(int i = 0; i < festo_controllers_found; ++i) {
        new (&f_cnt[i]) FestoController(
                    *(reinterpret_cast<Master2Slave*>(ecn.getOutputPointer(i+1))),
                    *(reinterpret_cast<Slave2Master*>(ecn.getInputPointer(i+1))),
                     ecn, i+1);

        f_cnt[i].m2s.modes_of_operation=1;
    }


    for(auto ec_i = 1; ec_i <= ecn.getDiscoveredSlavesCount(); ++ec_i)
    {
        //if (std::string("EK1828") == ecn.getSlaveName(ec_i))
        if( strcmp("EK1828", ecn.getSlaveName(ec_i)) == 0 )
            if(ek1828==nullptr)
                ek1828 = new EKXX(
          *(reinterpret_cast<Ekxx_Region_m2s*> (ecn.getOutputPointer(ec_i))),
          *(reinterpret_cast<Ekxx_Region_s2m*> (ecn.getInputPointer(ec_i))),
                            ecn, ec_i);

        if( strcmp("EL2819", ecn.getSlaveName(ec_i)) == 0 )
            if(el2819==nullptr)
                el2819 = new ELXX(
          *(reinterpret_cast<Elxx_Region_m2s*> (ecn.getOutputPointer(ec_i))),
          *(reinterpret_cast<Elxx_Region_s2m*> (ecn.getInputPointer(ec_i))),
                            ecn, ec_i);
        if( strcmp("EL4102", ecn.getSlaveName(ec_i)) == 0 )
            if(el4102==nullptr)
                el4102 = new EL41XX(
          *(reinterpret_cast<El41xx_Region_m2s*> (ecn.getOutputPointer(ec_i))),
          *(reinterpret_cast<El41xx_Region_s2m*> (ecn.getInputPointer(ec_i))),
                            ecn, ec_i);
    }
    if (el2819 == nullptr)
    {
        ROS_FATAL("EL2819 was not found online. Can not proceed");
        return false;
    }
    if (ek1828 == nullptr)
    {
        ROS_FATAL("EK1828 was not found online. Can not proceed");
        return false;
    }
    if (el4102==nullptr)
    {
        ROS_FATAL("EL4102 was not found online. Can not proceed");
          return false;
    }
#endif
    loadPositionsFromFile();

    keepUpdates = true;
    pthread_attr_t high_prio_attrs;
    pthread_attr_init(&high_prio_attrs);
    struct sched_param prio; prio.sched_priority = 10;
    pthread_attr_setschedparam(&high_prio_attrs, &prio);
    pthread_attr_setschedpolicy(&high_prio_attrs, SCHED_FIFO);
    if ( pthread_create(&pth_pid, &high_prio_attrs, threadFunc, this))
    {
        ROS_FATAL_STREAM("Failed to call create with high priority settings. Error is "<<(errno==EAGAIN?"no resources":"")
            <<(errno==EINVAL?"invalid settings in attr":"")<<(errno==EPERM?"no permission to set policy":""));
        ROS_INFO("Starting normal priority");
        pthread_create(&pth_pid, nullptr, threadFunc, this);
    }

    ROS_INFO("done");
    auto counter = 0;
    //TODO load zero states of joints
    enableReadWrite = true;
    inited = true;
    return true;
}

bool ServoHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list)
{
ROS_INFO_STREAM("Called ServoHW::prepareSwitch ");
    return hardware_interface::RobotHW::prepareSwitch(start_list, stop_list);
}

void ServoHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    //TODO
    ROS_INFO_STREAM("Called ServoHW::doSwitch");
    return;
}

void ServoHW::read(const ros::Time& time, const ros::Duration& period)
{
    if (not enableReadWrite)
    {
        ROS_DEBUG(" ServoHW::read not enableReadWrite");
        return;
    }
    const double popugai_to_rads = 1. /18000. * M_PI;

    //TODO lock me!!!
    for(auto i = 0; i < joint_position_state.size(); ++i)
    {
        joint_position_state[i] = popugai_to_rads * joint_pos_last_com[i];
        joint_velocity_state[i] = popugai_to_rads  * 0;
        joint_effort_state[i] =  0;
    }

    //TODO implement this method for this particular robot
#if 0

    int32_t *knownPosition = new int32_t[festo_controllers_found];
    int32_t *knownVelocities = new int32_t[festo_controllers_found];

    ROS_DEBUG_STREAM("called READ with time="<<time<<"    and period="<<period);

    for(auto i = 0; i < festo_controllers_found; ++i) {
        knownPosition[i] = f_cnt[i].getPositionValue();
        knownVelocities[i] = f_cnt[i].getVelocityValue();
        joint_position_state[i] = popugai_to_rads * Ax_motor_link_reduction[i] * knownPosition[i];
        joint_velocity_state[i] = popugai_to_rads * Ax_motor_link_reduction[i] * knownVelocities[i];
        for(auto j =0; j < i; ++j) {
            joint_position_state[i] -= jointInfluence[i-1][j] * joint_position_state[j];
            joint_velocity_state[i] -= jointInfluence[i-1][j] * joint_velocity_state[j];
        }
        joint_effort_state[i] =  f_cnt[i].getCurrentValue();

        ROS_DEBUG_STREAM("reporting for joint #"<<i
            <<" position = "<<joint_position_state[i]
            <<" velocity = "<<joint_velocity_state[i]
            <<" current = "<<joint_effort_state[i]);
    }
    delete[] knownPosition;
    delete[] knownVelocities;
#endif
    return;
}

void ServoHW::write(const ros::Time& time, const ros::Duration& period)
{
    //TODO before actual write into devices of target positions + required velocities,
    // check if at least 1 motor will require velocity truncation
    // and recalculate velocities, reducing by maximal reduction coefficient

    if (not enableReadWrite)
    {
        ROS_DEBUG("  ServoHW::write   not enableReadWrite");
        return;
    }
    const int MAX_ALLOWED_VELOCITY = 180000; //4000 rpm
    const double popugai_to_rads = 1. /18000. * M_PI;
    // ATT sequence matters!!! each position of next controller is dependent on
    ROS_DEBUG_STREAM("called WRITE with time="<<time<<"    and period="<<period);
    // calculated previous position
    // we'll check if command queue is empty, and only in that case - prepare and
    // send next command
    char rqBuffer[300];
    sprintf(rqBuffer, "A1 ");
    for(int i=0; i < num_joints; ++i)
    {
        double nextValue = joint_position_command[i]/popugai_to_rads;
        char tmp[20];
        sprintf(tmp, "%d ", static_cast<int>(nextValue));
        strcat(rqBuffer, tmp);
    }
    strcat(rqBuffer, "S 10\n");
    pthread_mutex_lock(&cmdLock);
        strcpy(tgtCMD, rqBuffer);
        hasCMDOut = true;
    pthread_mutex_unlock(&cmdLock);
//ROS_INFO_STREAM("Prepared command to servos:"<<tgtCMD);
//TODO implement this method
#if 0
    for(auto i=0; i < festo_controllers_found; ++i)
    {
        double motorPosition = joint_position_command[i];
        for(auto j=0; j < i; ++j) {
            motorPosition += jointInfluence[i-1][j] * joint_position_command[j];
        }
        motorPosition /= popugai_to_rads * Ax_motor_link_reduction[i];
        
        ROS_DEBUG_STREAM("setting for joint #"<<i
            <<"  new target position "<<motorPosition<<
            "   from request "<<joint_position_command[i]);
        //assuming that <current_position> to <desired_position> should be achieved
        // in same "period" as it was previously
        int lastKnownPosition = f_cnt[i].getPositionValue();
        f_cnt[i].setTargetPosition(static_cast<int>(motorPosition));

        int desiredVitess = std::abs((motorPosition - lastKnownPosition) / period.toSec());
        if (desiredVitess > MAX_ALLOWED_VELOCITY)
            desiredVitess = MAX_ALLOWED_VELOCITY;
        f_cnt[i].setPositioningVelocity(desiredVitess);
        f_cnt[i].startPositionChange();
    }
#endif
    return;
}

/**
    method allows to trigger each Festo's controller
    to Enabled state in case controller (and motor)
    went to disabled state (for example when Emergency button
    was pushed 
*/
bool ServoHW::setEnabled()
{
    enableReadWrite = false;
#if 0
    for( auto i=0; i < festo_controllers_found; ++i)
    {
        f_cnt[i].clearEIP();
        f_cnt[i].startEnabling();
    }
#endif
    enableReadWrite = true;
    return true;
}

void ServoHW::commandsCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Got command callback with arg: "<<msg->data );
    if (msg->data == "enable")
    {
        setEnabled();
    }
}

void ServoHW::CallbackParser(const std_msgs::String::ConstPtr& msg, std::string& str0, int& key)
{
	if(msg->data == " ")
	{
		key = -1;
		str0 = " ";
		return;
	}
	std::size_t pos = msg->data.find(" ");
	if (pos == std::string::npos)
	{
		key = -1;
		str0 = msg->data;
		return;
	}
  try {
	key = std::stoi( msg->data.substr(pos) );
  } catch (...) {
    key = -1;
  }
	str0 = msg->data.substr(0, pos);
}

void ServoHW::wristCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Got wrist callback with arg: "<<msg->data );
    std::string str0;
    int key;
	CallbackParser(msg, str0, key);

    if (str0 == "open")
    {
        //ek1828->powerOnChannelTimedOut(EK1828_OPEN_CHANNEL_NUMBER, EK1828_POWER_ON_DELAY_NS);
    }
    if (str0 == "close")
    {
        //ek1828->powerOnChannelTimedOut(EK1828_CLOSE_CHANNEL_NUMBER, EK1828_POWER_ON_DELAY_NS);
    }
    return;
}

void ServoHW::pneumaticCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Got pneumatic callback with arg: "<<msg->data );
    std::string str0;
    int key;
	CallbackParser(msg, str0, key);

#if 0
        if (key >= 0 && key < EL2819_NUMBER_OF_OUTPUTS_CHANNELS)
        {
            if (str0 == "open")
            {
                el2819->powerOnChannel(key);
            }
            if (str0 == "close")
            {
                el2819->powerOffChannel(key);
            }
        }
#endif
}

void ServoHW::loadPositionsFromFile()
{
    // TODO implement this method for ServoHW
#if 0
    std::list <int> zeroValues { 0, 0, 0, 0, 0, 522720 };
    std::ifstream zeroFile(ZERO_STREAM_FILENAME);
    if (!zeroFile.is_open())
    {
        int idx = 0;
        for( auto const & i: zeroValues) {
            f_cnt[idx++].setHomeOffset(i);
        }
        return;
    }
    int32_t val;
    for(auto i=0; i < festo_controllers_found; ++i) {
        zeroFile>>val;
        f_cnt[i].setHomeOffset(val);
    }
    zeroFile.close();
#endif
}

void ServoHW::storeHomePositions()
{
    // TODO implement this method for ServoHW
#if 0
    std::ofstream zeroFile(ZERO_STREAM_FILENAME);
    if (!zeroFile.is_open()) {
        // TODO notify on the problem with file opening
        return;
    }
    for( int idx =0; idx < festo_controllers_found; ++idx)
    {
        zeroFile<<festoContr[idx].getPositionValue()<<std::endl;
    }
    zeroFile.close();
#endif
}

const uint16_t ServoHW::PRE_DEFINED_ERROR_FIELD_SLOTNUM = 0x1003;
const int ServoHW::errorFieldsCount  = 4;

}; //namespace servo_hardware_interface

PLUGINLIB_EXPORT_CLASS(servo_hardware_interface::ServoHW, hardware_interface::RobotHW);
