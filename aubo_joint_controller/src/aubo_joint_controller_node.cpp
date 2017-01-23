/*! \class AuboController
 *  \file aubo_joint_controller_node.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 0.0.1
 *	\date 2016
 *  \brief
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <robotnik_msgs/State.h>
#include <sensor_msgs/JointState.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include "robotconnect.h"
#include "jointcmd.h"
#include "canbus.h"

//! Size of string for logging
#define DEFAULT_THREAD_DESIRED_HZ           50.0
#define OUR_ARM_JOINTS                      6

#define OUR_ARM_CONTROL_POSITION			1
#define OUR_ARM_CONTROL_VELOCITY			2
#define OUR_COMMAND_WATCHDOG				1
#define JOINT_MIN_RPM						1
#define JOINT_MAX_RPM						1000
#define JOINT_DEF_RPM						500

using namespace std;

struct simple_joint_state{
    //! desired joint position
    double position;
    //! desired joint velocity
    double velocity;
    //! desired joint effor
    double effort;
};

struct our_joint_state_struct{
    //! joint number
    uint8_t joint_number;

    //! joint type: MODEL_TYPE_J80 or MODEL_TYPE_J60
    uint8_t joint_type;

    //! send_command_
    uint8_t write_command_mode;
    uint8_t read_command_mode;
    
    //! lower_limit_
    double lower_limit;
    
    //! upper_limit_
    double upper_limit;

    //! desired values
    struct simple_joint_state desired_joint_state;
    //! real values
    struct simple_joint_state current_joint_state;


};

//! Defines return values for methods and functions
enum ReturnValue{
    OK = 0,
    INITIALIZED,
    THREAD_RUNNING,
    ERROR = -1,
    NOT_INITIALIZED = -2,
    THREAD_NOT_RUNNING = -3,
    COM_ERROR = -4,
    NOT_ERROR = -5
};


//! Class Rcomponent
class OurArmController{
    protected:
        //! Controls if has been initialized succesfully
        bool initialized, ros_initialized;
        //! Controls the execution of the OurArmController's thread
        bool running;

        //! State of the OurArmController
        int state;
        //! State before
        int previous_state;
        //!	Saves the name of the component
        string component_name;
        //! ROS node handle
        ros::NodeHandle nh_;
        //! Private ROS node handle
        ros::NodeHandle pnh_;
        //! Desired loop frequency
        double desired_freq_, real_freq;
        
        //! Save the values from the param
        double our_joint1_ll, our_joint2_ll, our_joint3_ll, our_joint4_ll, our_joint5_ll, our_joint6_ll;
        double our_joint1_ul, our_joint2_ul, our_joint3_ul, our_joint4_ul, our_joint5_ul, our_joint6_ul;

        //! Publish the component state
        ros::Publisher component_state_pub_;
        //! Publish the joints state
        ros::Publisher joints_state_pub_;
        //! Subscribes to read commands to set the joints position
        ros::Subscriber joints_command_sub_;

        //! Joint values
        sensor_msgs::JointState joints;


        // Examples:
        // ros::Subscriber sub_; // topic subscriber
        // ros::ServiceServer service_server_; // service server
        // ros::ServiceClient service_client_; // service client

        //! General status diagnostic updater
        diagnostic_updater::Updater *diagnostic_;

        //! Saves the last time it receives a command
        ros::Time last_command_time;

        //! VELOCITY, POSITION
        int control_mode_;
        //! Saves the current joint in action
        string current_joint;

        std::string can_device_;
        bool can_connected_;

        //! sets the desired joints rpms
        int joint_rpm_;


        std::map<std::string, our_joint_state_struct> joint2our;
    public:
        //! Public constructor
        OurArmController(ros::NodeHandle h);
        //! Public destructor
        ~OurArmController();

        //! Starts the control loop of the component and its subcomponents
        //! @return OK
        //! @return ERROR starting the thread
        //! @return RUNNING if it's already running
        //! @return NOT_INITIALIZED if it's not initialized
        virtual int start();
        //! Stops the main control loop of the component and its subcomponents
        //! @return OK
        //! @return ERROR if any error has been produced
        //! @return NOT_RUNNING if the main thread isn't running
        virtual int stop();
        //! Returns the general state of the OurArmController
        int getState();
        //! Returns the general state of the OurArmController as string
        char *getStateString();
        //! Returns the general state as string
        char *getStateString(int state);
        //! Method to get current update rate of the thread
        //! @return pthread_hz
        double getUpdateRate();

    protected:
        //! Configures and initializes the component
        //! @return OK
        //! @return INITIALIZED if the component is already intialized
        //! @return ERROR
        int setup();
        //! Closes and frees the reserved resources
        //! @return OK
        //! @return ERROR if fails when closes the devices
        //! @return RUNNING if the component is running
        //! @return NOT_INITIALIZED if the component is not initialized
        int shutdown();
        //! All core component functionality is contained in this thread.
        //!	All of the OurArmController component state machine code can be found here.
        void controlLoop();
        //! Actions performed on initial state
        void initState();
        //! Actions performed on standby state
        void standbyState();
        //! Actions performed on ready state
        void readyState();
        //! Actions performed on the emergency state
        void emergencyState();
        //! Actions performed on Failure state
        void failureState();
        //! Actions performed on Shudown state
        void shutdownState();
        //! Actions performed in all states
        void allState();
        //! Switches between states
        void switchToState(int new_state);
        //! Setups all the ROS' stuff
        int rosSetup();
        //! Shutdowns all the ROS' stuff
        int rosShutdown();
        //! Reads data a publish several info into different topics
        void rosPublish();
        //! Reads params from params server
        void rosReadParams();
        //! Disables all the arm brakes
        int disableArmBrakes();
        //! Sets the rpm limits for all the joints
        int setArmRPMLimits();
        // Examples
        // void topicCallback(const std_msgs::StringConstPtr& message); // Callback for a subscriptor
        // bool serviceServerCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); // Callback for a service server

        //! Diagnostic updater callback
        void diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat);

        void commandJointCallback(const sensor_msgs::JointState::ConstPtr& msg);
        //! Sets the velocity of the desired joint
        int setJointVelocity(string joint, double velocity);
        //! Sets the position of the joints
        int sendCommandJointsPositions();
        //!
        int stopJoints();
        
        int readJointsPositions();
        //! Sets the max rpms of a joint
        int setJointMaxRPM(string joint, unsigned int rpm);

};


/*! \fn OurArmController::OurArmController()
 *  \brief Constructor by default
 *	\param hz as double, sets the desired frequency of the controlthread
 *	\param h as ros::NodeHandle, ROS node handle
*/
OurArmController::OurArmController(ros::NodeHandle h):nh_(h), pnh_("~"){
    // Set main flags to false
    ros_initialized = initialized = running = false;
    // reads params from server
    rosReadParams();

    if(desired_freq_ <= 0.0)
        desired_freq_ = DEFAULT_THREAD_DESIRED_HZ;

    state = robotnik_msgs::State::INIT_STATE;
    can_connected_ = false;
    char j[3]= "\0";

    current_joint = "";
    // Inits the number of joints //TODO:
    for(int i = 0; i < OUR_ARM_JOINTS; i++){
        sprintf(j,"j%d",i+1); // TODO: revisar si es joint0, j0 o a saber que...
        joints.name.push_back(j);
        joints.position.push_back(0.0);
        joints.velocity.push_back(0.0);
        joints.effort.push_back(0.0);
    }

    //can_bus_ = new JointControlCanBus();

    // Realizar para cada una de las clases derivadas
    component_name.assign("OurArmController");

}

/*! \fn OurArmController::~OurArmController()
 * Destructor by default
*/
OurArmController::~OurArmController(){

}

/*! \fn int OurArmController::setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
int OurArmController::setup(){
    // Checks if has been initialized
    if(initialized){
        ROS_INFO("%s::Setup: Already initialized",component_name.c_str());

        return INITIALIZED;
    }

    //
    ///////////////////////////////////////////////////
    // Setups the component or another subcomponents if it's necessary //
    ///////////////////////////////////////////////////

    //bool was_can_initialized = can_bus_->JointControlInit(can_device_.c_str());
    bool was_can_initialized = robotconnect_init(can_device_.c_str(),JOINT_CANBUS);
    ROS_INFO("%s",can_device_.c_str());
    if (!was_can_initialized) {
        can_connected_ = false;
        ROS_ERROR("%s::Setup: Cannot initialize can device: %s",component_name.c_str(), can_device_.c_str());
        switchToState(robotnik_msgs::State::EMERGENCY_STATE); //TOOD: seguro que es a emergency state? o error?
        return NOT_INITIALIZED; // seguro?? o error?
    }
    initialized = true;
	can_connected_ = true;
    return OK;
}

/*! \fn int OurArmController::shutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return RUNNING if the component is running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int OurArmController::shutdown(){

    /*if(running){
        ROS_INFO("%s::Shutdown: Impossible while thread running, first must be stopped",component_name.c_str());
        return THREAD_RUNNING;
    }*/
    if(!initialized){
        ROS_INFO("%s::Shutdown: Impossible because of it's not initialized", component_name.c_str());
        return NOT_INITIALIZED;
    }

    //
    ///////////////////////////////////////////////////////
    // ShutDowns another subcomponents if it's necessary //
    ///////////////////////////////////////////////////////
    ROS_INFO("%s::Shutdown", component_name.c_str());

    //TODO cerrar can
    if (can_connected_) {
        //bool was_can_shutdown = can_bus_->JointControlUninit();
        bool was_can_shutdown = robotconnect_destroy(JOINT_CANBUS);
        if (!was_can_shutdown) {
            ROS_ERROR("%s::Shutdown: Cannot shutdown can device: %s",component_name.c_str(), can_device_.c_str());
            switchToState(robotnik_msgs::State::EMERGENCY_STATE); //TOOD: seguro que es a emergency state? o error?

        }
        can_connected_ = false;
    }
    initialized = false;

    return OK;
}


/*! \fn int OurArmController::start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int OurArmController::start(){
    // Performs ROS setup
    rosSetup();

    if(running){
        ROS_INFO("%s::start: the component's thread is already running", component_name.c_str());
        return THREAD_RUNNING;
    }

    ROS_INFO("%s started", component_name.c_str());

    running = true;

    // Executes the control loop
    controlLoop();

    return OK;

}

/*! \fn int OurArmController::stop()
 * Stops the control thread of the Motors
 * \return OK
 * \return ERROR if it can't be stopped
 * \return THREAD_NOT_RUNNING if the thread is not running
*/
int OurArmController::stop(){

    if(!running){
        ROS_INFO("%s::stop: Thread not running", component_name.c_str());

        return THREAD_NOT_RUNNING;
    }
    //
    ///////////////////////////////////////////////////
    // Stops another subcomponents, if it's necessary //
    ///////////////////////////////////////////////////
    //
    ROS_INFO("%s::Stop: Stopping the component", component_name.c_str());

    running = false;

    usleep(100000);

    return OK;
}

/*!	\fn void OurArmController::controlLoop()
 *	\brief All core component functionality is contained in this thread.
*/
void OurArmController::controlLoop(){
    ROS_INFO("%s::controlLoop(): Init", component_name.c_str());
    ros::Rate r(desired_freq_);
    ros::Time t1,t2;
    while(running && ros::ok()) {

        t1 = ros::Time::now();

        switch(state){

            case robotnik_msgs::State::INIT_STATE:
                initState();
            break;

            case robotnik_msgs::State::STANDBY_STATE:
                standbyState();
            break;

            case robotnik_msgs::State::READY_STATE:
                readyState();
            break;

            case robotnik_msgs::State::SHUTDOWN_STATE:
                shutdownState();
            break;

            case robotnik_msgs::State::EMERGENCY_STATE:
                emergencyState();
            break;

            case robotnik_msgs::State::FAILURE_STATE:
                failureState();
            break;

        }

        allState();

        ros::spinOnce();
        r.sleep();

        t2 = ros::Time::now();

        real_freq = 1.0/(t2 - t1).toSec();

    }

    shutdownState();
    // Performs ROS Shutdown
    rosShutdown();

    ROS_INFO("%s::controlLoop(): End", component_name.c_str());

}

/*!	\fn void OurArmController::initState()
 *	\brief Actions performed on initial
 * 	Setups the component
*/
void OurArmController::initState(){
    // If component setup is successful goes to STANDBY (or READY) state
    if(setup() != ERROR){

        /*
         for(int i = 0; i< 5; i++){
            readJointsPositions();
            usleep(100000);
        }

        stopJoints();*/
        if(setArmRPMLimits()!=0){
            switchToState(robotnik_msgs::State::EMERGENCY_STATE);
            return;
        }

        if(disableArmBrakes()!=0){
            switchToState(robotnik_msgs::State::EMERGENCY_STATE);
            return;
        }

        switchToState(robotnik_msgs::State::STANDBY_STATE);
    }
}

/*!	\fn int OurArmController::disableArmBrakes()
 *	\brief Disables the brakes of all the joints
 * \return 0 if OK, -1 if error
*/
int OurArmController::disableArmBrakes(){
    int ret = 0;
    for (auto &joint: joint2our) {
        ROS_WARN("OurArmController::disableArmBrakes: Disabling brake of joint %d", joint.second.joint_number);
        ret = joint_brake_enable(joint.second.joint_number);
        if(ret != 1){
            ROS_ERROR("OurArmController::disableArmBrakes: Error disabling brake %d", joint.second.joint_number);
            return -1;
        }
   }

    return 0;
}

/*!	\fn int OurArmController::setArmRPMLimits()
 *	\brief sets the max rpm for every joint
 * \return 0 if OK, -1 if error
*/
int OurArmController::setArmRPMLimits(){
    int ret = 0;
    for (auto &joint: joint2our) {
        ROS_WARN("OurArmController::setArmRPMLimits: setting %d rpms for joint %d", joint_rpm_, joint.second.joint_number);
        ret = setJointMaxRPM(joint.first, (unsigned int)joint_rpm_);
        if(ret != 1){
            ROS_ERROR("OurArmController::setArmRPMLimits: Error setting rpm for joint %d", joint.second.joint_number);
            return -1;
        }
   }

    return 0;
}


/*!	\fn void OurArmController::shutdownState()
 *	\brief Actions performed on Shutdown state
*/
void OurArmController::shutdownState(){

    if(shutdown() == OK){
        switchToState(robotnik_msgs::State::INIT_STATE);
    }
}

/*!	\fn void OurArmController::standbyState()
 *	\brief Actions performed on Standby state
*/
void OurArmController::standbyState(){
    switchToState(robotnik_msgs::State::READY_STATE);
}

/*!	\fn void OurArmController::readyState()
 *	\brief Actions performed on ready state
*/
void OurArmController::readyState(){

    //TODO revisar
    // State transitions - TODO - Include connection check !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if(can_connected_ == false){
        ROS_ERROR("%s::ReadyState: Can device %s is disconnected",component_name.c_str(), can_device_.c_str());
        // No data from subcomponents
        switchToState(robotnik_msgs::State::EMERGENCY_STATE);
        return;
    }
    
    readJointsPositions();

    double seconds_from_last_command = (ros::Time::now() - last_command_time).toSec();

    if( seconds_from_last_command  > OUR_COMMAND_WATCHDOG) {
		//ROS_INFO_THROTTLE(3, "Watchdog");
        int success = stopJoints();
        if (success != OK) {
            switchToState(robotnik_msgs::State::EMERGENCY_STATE);
        }
        return;
    }


    if (control_mode_ == OUR_ARM_CONTROL_POSITION) {
        /*int success = sendCommandJointsPositions();

        if (success != OK) {
            switchToState(robotnik_msgs::State::EMERGENCY_STATE);
        }*/
        return;
    }
}

/*!	\fn void OurArmController::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void OurArmController::emergencyState(){
//TODO revisar
    //TODO desconectar a can?
//    sleep(2);
//    //TODO conectar can?

//    if( mb_connected_){
//    // Data received
//        switchToState(robotnik_msgs::State::READY_STATE);
//    }
}

/*!	\fn void OurArmController::FailureState()
 *	\brief Actions performed on failure state
*/
void OurArmController::failureState(){

}

/*!	\fn void OurArmController::AllState()
 *	\brief Actions performed on all states
*/
void OurArmController::allState(){

    diagnostic_->update();

    rosPublish();
}

/*!	\fn double OurArmController::getUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double OurArmController::getUpdateRate(){
    return desired_freq_;
}

/*!	\fn int OurArmController::getState()
 * 	\brief returns the state of the component
*/
int OurArmController::getState(){
    return state;
}

/*!	\fn char *OurArmController::getStateString()
 *	\brief Gets the state of the component as string
*/
char *OurArmController::getStateString(){
    return getStateString(state);
}

/*!	\fn char *OurArmController::getStateString(int state)
 *	\brief Gets the state as a string
*/
char *OurArmController::getStateString(int state){
    switch(state){
        case robotnik_msgs::State::INIT_STATE:
            return (char *)"INIT";
        break;
        case robotnik_msgs::State::STANDBY_STATE:
            return (char *)"STANDBY";
        break;
        case robotnik_msgs::State::READY_STATE:
            return (char *)"READY";
        break;
        case robotnik_msgs::State::EMERGENCY_STATE:
            return (char *)"EMERGENCY";
        break;
        case robotnik_msgs::State::FAILURE_STATE:
            return (char *)"FAILURE";
        break;
        case robotnik_msgs::State::SHUTDOWN_STATE:
            return (char *)"SHUTDOWN";
        break;
        default:
            return (char *)"UNKNOWN";
        break;
    }
}


/*!	\fn void OurArmController::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void OurArmController::switchToState(int new_state){

    if(new_state == state)
        return;

    // saves the previous state
    previous_state = state;
    ROS_INFO("%s::SwitchToState: %s -> %s", component_name.c_str(), getStateString(state), getStateString(new_state));
    state = new_state;

}

/*!	\fn void OurArmController::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int OurArmController::rosSetup(){
    unsigned char pos[2];
    pos[0]=1;
    pos[1]=0;
    // Checks if has been initialized
    if(ros_initialized){
        ROS_INFO("%s::rosSetup: Already initialized",component_name.c_str());

        return INITIALIZED;
    }

    // Names of the joints
    pnh_.param<std::string>("j1", joints.name[0], "our_joint1");
    pnh_.param<std::string>("j2", joints.name[1], "our_joint2");
    pnh_.param<std::string>("j3", joints.name[2], "our_joint3");
    pnh_.param<std::string>("j4", joints.name[3], "our_joint4");
    pnh_.param<std::string>("j5", joints.name[4], "our_joint5");
    pnh_.param<std::string>("j6", joints.name[5], "our_joint6");

    our_joint_state_struct our_joint_1, our_joint_2, our_joint_3, our_joint_4, our_joint_5, our_joint_6;

    // Populate map of jointstate <> real joints
    our_joint_1.joint_number = 0x1;
    our_joint_1.joint_type = MODEL_TYPE_J80;
    our_joint_1.write_command_mode = CMDTYPE_WR_NR;
    our_joint_1.read_command_mode = CMDTYPE_RD;
    our_joint_1.lower_limit = our_joint1_ll;
    our_joint_1.upper_limit = our_joint1_ul;
    joint2our[joints.name[0]] = our_joint_1;

    our_joint_2.joint_number = 0x2;
    our_joint_2.joint_type = MODEL_TYPE_J80;
    our_joint_2.write_command_mode = CMDTYPE_WR_NR;
    our_joint_2.read_command_mode = CMDTYPE_RD;
    our_joint_2.lower_limit = our_joint2_ll;
    our_joint_2.upper_limit = our_joint2_ul;
    joint2our[joints.name[1]] = our_joint_2;

    our_joint_3.joint_number = 0x3;
    our_joint_3.joint_type = MODEL_TYPE_J80;
    our_joint_3.write_command_mode = CMDTYPE_WR_NR;
    our_joint_3.read_command_mode = CMDTYPE_RD;
    our_joint_3.lower_limit = our_joint3_ll;
    our_joint_3.upper_limit = our_joint3_ul;
    joint2our[joints.name[2]] = our_joint_3;

    our_joint_4.joint_number = 0x4;
    our_joint_4.joint_type = MODEL_TYPE_J60;
    our_joint_4.write_command_mode = CMDTYPE_WR_NR;
    our_joint_4.read_command_mode = CMDTYPE_RD;
    our_joint_4.lower_limit = our_joint4_ll;
    our_joint_4.upper_limit = our_joint4_ul;
    joint2our[joints.name[3]] = our_joint_4;

    our_joint_5.joint_number = 0x5;
    our_joint_5.joint_type = MODEL_TYPE_J60;
    our_joint_5.write_command_mode = CMDTYPE_WR_NR;
    our_joint_5.read_command_mode = CMDTYPE_RD;
    our_joint_5.lower_limit = our_joint5_ll;
    our_joint_5.upper_limit = our_joint5_ul;
    joint2our[joints.name[4]] = our_joint_5;
    //JointSendMsg(0x5, CMDTYPE_WR_NR, 0x61, pos, 2);
 
    our_joint_6.joint_number = 0x6;
    our_joint_6.joint_type = MODEL_TYPE_J60;
    our_joint_6.write_command_mode = CMDTYPE_WR_NR;
    our_joint_6.read_command_mode = CMDTYPE_RD;
    our_joint_6.lower_limit = our_joint6_ll;
    our_joint_6.upper_limit = our_joint6_ul;
    joint2our[joints.name[5]] = our_joint_6;
    

    // Publishers
    component_state_pub_ = pnh_.advertise<robotnik_msgs::State>("state", 1);
    // Subscribers
    joints_command_sub_ = pnh_.subscribe<sensor_msgs::JointState>("/joint_command", 10, &OurArmController::commandJointCallback, this);
    joints_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

    /*
    // EXAMPLES
    // Subscribers
    // topic, queue, callback
    sub_ = nh_.subscribe("topic name", 10,  &OurArmController::topicCallback, this);

    // Services server
    service_server_ = pnh_.advertiseService("service name", &OurArmController::serviceServerCb, this);
    // Services client
    service_client_ = nh_.serviceClient<std_srvs::Empty>("service client name");

    */

    // Sets up the diagnostic updater
    diagnostic_ = new diagnostic_updater::Updater();

    diagnostic_->setHardwareID("OurArmController");
    diagnostic_->add("State", this, &OurArmController::diagnosticUpdate);
    diagnostic_->broadcast(0, "Doing important initialization stuff.");

    ros_initialized = true;

    return OK;

}


/*!	\fn void OurArmController::rosReadParams
 * 	\brief Reads the params set in ros param server
*/
void OurArmController::rosReadParams(){

    pnh_.param("desired_freq", desired_freq_, DEFAULT_THREAD_DESIRED_HZ);
    pnh_.param("control_mode", control_mode_, OUR_ARM_CONTROL_POSITION);

    pnh_.param<std::string>("can_device", can_device_, "/dev/pcanusb1"); //TODO: cambiar nombre por defecto, añadir parametro al launcher
    
    pnh_.param<double>("our_joint1_ll", our_joint1_ll, -1.58);
    pnh_.param<double>("our_joint1_ul", our_joint1_ul, 1.58);
    pnh_.param<double>("our_joint2_ll", our_joint2_ll, -1.58);
    pnh_.param<double>("our_joint2_ul", our_joint2_ul, 1.58);
    pnh_.param<double>("our_joint3_ll", our_joint3_ll, -1.58);
    pnh_.param<double>("our_joint3_ul", our_joint3_ul, 1.58);
    pnh_.param<double>("our_joint4_ll", our_joint4_ll, -1.58);
    pnh_.param<double>("our_joint4_ul", our_joint4_ul, 1.58);
    pnh_.param<double>("our_joint5_ll", our_joint5_ll, -1.58);
    pnh_.param<double>("our_joint5_ul", our_joint5_ul, 1.58);
    pnh_.param<double>("our_joint6_ll", our_joint6_ll, -1.58);
    pnh_.param<double>("our_joint6_ul", our_joint6_ul, 1.58);

    pnh_.param<int>("joint_rpm", joint_rpm_, JOINT_DEF_RPM);

    

    /* Example
    pnh_.param<std::string>("port", port_, DEFAULT_DSPIC_PORT);
    pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "/odom_diff");
    pnh_.param<std::string>("base_frame_id", base_frame_id_, "/base_link");
    pnh_.param("publish_tf", publish_tf_, false);
    pnh_.param("desired_freq", desired_freq_, desired_freq_);*/
}

/*!	\fn int OurArmController::rosShutdown()
 * 	\brief Closes all ros stuff
*/
int OurArmController::rosShutdown(){
    if(running){
        ROS_INFO("%s::rosShutdown: Impossible while thread running, first must be stopped",component_name.c_str());
        return THREAD_RUNNING;
    }
    if(!ros_initialized){
        ROS_INFO("%s::rosShutdown: Impossible because of it's not initialized", component_name.c_str());
        return NOT_INITIALIZED;
    }


    ros_initialized = false;

    return OK;
}

/*!	\fn void OurArmController::rosPublish()
 * 	\brief Reads data a publish several info into different topics
*/
void OurArmController::rosPublish(){
    robotnik_msgs::State state_msg;

    // COMPONENT STATE
    state_msg.state = this->state;
    state_msg.desired_freq = this->desired_freq_;
    state_msg.real_freq = this->real_freq;
    state_msg.state_description = getStateString();

    component_state_pub_.publish(state_msg);


    // JOINT STATES
    sensor_msgs::JointState joints_msg = joints;
	joints_msg.header.frame_id =  "base"; //TODO es el baselink? u otro?
	joints_msg.header.stamp = ros::Time::now();
	
    for (int i = 0; i < joints_msg.name.size(); i++) {
        joints_msg.position[i] = joint2our[joints.name[i]].current_joint_state.position;
        joints_msg.velocity[i] = joint2our[joints.name[i]].current_joint_state.velocity;
        joints_msg.effort[i] = joint2our[joints.name[i]].current_joint_state.effort;
    }

    joints_state_pub_.publish(joints_msg);

}

/*!	\fn void OurArmController::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat)
 * 	\brief Callback to update the component diagnostic
*/
void OurArmController::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat){

    if(state == robotnik_msgs::State::READY_STATE || state == robotnik_msgs::State::INIT_STATE || state == robotnik_msgs::State::STANDBY_STATE)
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything OK!");
    else if (state == robotnik_msgs::State::EMERGENCY_STATE)
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Watch out!");
    else
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Error!");

    stat.add("State", getStateString());
}


/*!	\fn OurArmController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
 * 	\brief Topic callback to move all the joints through joint state interface
*/
void OurArmController::commandJointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    bool match_key = false;
    if(state == robotnik_msgs::State::READY_STATE){


        for(int i = 0; i<msg->name.size(); i++){
            try{
                if(msg->position[i]>=joint2our.at(msg->name[i]).lower_limit and msg->position[i]<=joint2our.at(msg->name[i]).upper_limit){
                    joint2our.at(msg->name[i]).desired_joint_state.position = msg->position[i];
                    joint2our.at(msg->name[i]).desired_joint_state.velocity = msg->velocity[i];
                    joint2our.at(msg->name[i]).desired_joint_state.effort = msg->effort[i];
                    match_key = true;

                    //if(joint2our.at(msg->name[i]).joint_number == 5){
                    int success = joint_setpos(joint2our.at(msg->name[i]).joint_number, joint2our.at(msg->name[i]).desired_joint_state.position,
                                             JOINT_RADIAN, joint2our.at(msg->name[i]).joint_type, joint2our.at(msg->name[i]).write_command_mode);
                    if (!success)
                            ROS_ERROR("%s::commandJointCallback: Error send command to joint %s", component_name.c_str(), msg->name[i].c_str());
                    //}
                }else{
                    ROS_WARN( "%s::jointStateCallback: joint %s, Out of range limit. Value is: %f",component_name.c_str(), msg->name[i].c_str(), msg->position[i]);
                }
            }catch (const std::out_of_range& oor) {
                ROS_ERROR("%s::jointStateCallback: joint %s, Out of Range error: %s",component_name.c_str(), msg->name[i].c_str(), oor.what());
            }
        }


        if(match_key)
            last_command_time = ros::Time::now();
    }

}


/*!	\fn int OurController::setJointVelocity(string joint, , double velocity)
 * 	\brief sets the joint velocity of the desired joint
*/
int OurArmController::setJointVelocity(string joint, double velocity){
    // It doesn't work in position mode
    int success = joint_setvel(joint2our.at(joint).joint_number, velocity,
                                        JOINT_RADIAN, joint2our.at(joint).joint_type, joint2our.at(joint).write_command_mode);



    return success;

}

/*!	\fn int OurArmController::setJointMaxRPM(string joint, unsigned int rpm)
 * 	\brief sets the max joint rpm
*/
int OurArmController::setJointMaxRPM(string joint, unsigned int rpm){
    if(rpm < JOINT_MIN_RPM)
        rpm = JOINT_MIN_RPM;
    else if(rpm > JOINT_MAX_RPM)
        rpm = JOINT_MAX_RPM;


    int success = joint_setmaxrpm(joint2our.at(joint).joint_number, rpm, joint2our.at(joint).write_command_mode);

    return success;
}

int OurArmController::readJointsPositions()
{    
	for (auto &joint: joint2our) {
        //joint.second.current_joint_state.position = can_bus_->readPosJ(joint.second.joint_number, joint.second.joint_type);
        //if (joint.second.joint_number == 1)
        //ROS_INFO("joint number: %d",joint.second.joint_number);
        //ROS_INFO("joint type: %d",joint.second.joint_type);
        joint.second.current_joint_state.position = joint_readpos(joint.second.joint_number, 0x02, 0x36, joint.second.joint_type);
		//	ROS_INFO_THROTTLE(1, "state joint %d valor %g\n", joint.second.joint_number, joint.second.current_joint_state.position);
    }
    return OK;
	
}

int OurArmController::sendCommandJointsPositions()
{
    //TODO revisar que funciona
    for (auto const &joint: joint2our) {
        bool success;
        //success = can_bus_->setTagPosRadio(joint.second.joint_number, joint.second.desired_joint_state.position, joint.second.joint_type, joint.second.write_command_mode);
        success = joint_setpos(joint.second.joint_number, joint.second.desired_joint_state.position, 0x02, joint.second.joint_type, joint.second.write_command_mode);
        //if (joint.second.joint_number == 1)
		//	ROS_INFO_THROTTLE(1, "command joint %d valor %g\n", joint.second.joint_number, joint.second.desired_joint_state.position);
		//ROS_INFO("command joint %d valor %g\n", joint.second.joint_number, joint.second.desired_joint_state.position);
        if (!success) {
            ROS_ERROR("%s::sendCommandJointsPositions: Error send command to joint %s", component_name.c_str(), joint.first.c_str());
            return ERROR;
        }
    }
    return OK;
}

/*!	\fn int OurArmController::stopJoints()
 * 	\brief Stops all the joints
*/
int OurArmController::stopJoints(){
    for (auto &joint: joint2our) {
        joint.second.desired_joint_state.position = joint.second.current_joint_state.position;
        joint.second.desired_joint_state.velocity = 0;
        joint.second.desired_joint_state.effort = joint.second.current_joint_state.effort;
    }

    return OK;
}



// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "our_arm_controller");

    ros::NodeHandle n;
  
	OurArmController controller(n);

    controller.start();

    return (0);
}
