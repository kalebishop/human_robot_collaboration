#include "robot_interface/gripper.h"
#include <locale>
#include <codecvt>

using namespace              std;
using namespace intera_core_msgs;

Gripper::Gripper(std::string _limb, bool _use_robot) :
                 gnh(_limb), limb(_limb), ee_name(""), ee_type(""), node_time(ros::Time(0, 0)), use_robot(_use_robot),
                 first_run(true), prop_set(false), g_print_level(0),
                 spinner(4), cmd_sequence(0), cmd_sender(ros::this_node::getName())
{
    if (not use_robot) return;

    gnh.param<int> ("/print_level", g_print_level, 0);

    pub_end_effector_cmd = gnh.advertise<IOComponentCommand>("/io/end_effector/command", 10);
    sub_end_effector_state = gnh.subscribe("/io/end_effector/state", SUBSCRIBER_BUFFER, &Gripper::gripperInitCb, this);

    spinner.start();

    // set the gripper parameters to their defaults
    // setParameters("", true);
}

void Gripper::setGripperState(const intera_core_msgs::IODeviceStatus& _state)
{
    std::lock_guard<std::mutex> lock(mutex_state);
    state = _state;
}

intera_core_msgs::IODeviceStatus Gripper::getGripperState()
{
    std::lock_guard<std::mutex> lock(mutex_state);
    return state;
}

void Gripper::setGripperProperties(const intera_core_msgs::IODeviceConfiguration& _props)
{
    std::lock_guard<std::mutex> lock(mutex_props);
    props = _props;
}

intera_core_msgs::IODeviceConfiguration Gripper::getGripperProperties()
{
    std::lock_guard<std::mutex> lock(mutex_props);
    return props;
}

void Gripper::gripperInitCb(const IONodeStatus &msg)
{
    if (ee_name == "" || (node_time.sec != msg.time.sec || node_time.nsec != msg.time.nsec))
    {
        if (msg.devices.size() > 0)
        {
            if (ee_name != msg.devices[0].name)
            {
                ee_name = msg.devices[0].name;
                ROS_INFO("Received EE Name: %s", ee_name.c_str());
                // create a publisher for the gripper's commands
                pub_cmd = gnh.advertise<IOComponentCommand>( "/io/end_effector/" +
                                                            ee_name + "/command", 10);

                // create a subscriber to the gripper's properties
                sub_prop  = gnh.subscribe("/io/end_effector/" + ee_name + "/config",
                                            SUBSCRIBER_BUFFER, &Gripper::gripperPropCb, this);

                // create a subscriber to the gripper's state
                sub_state = gnh.subscribe("/io/end_effector/" + ee_name + "/state",
                                            SUBSCRIBER_BUFFER, &Gripper::gripperCb, this);

                //Initially all the interesting properties of the state are unknown
                IODeviceStatus init_state;
                init_state.device.name = ee_name;

                setGripperState(init_state);
            }
            if (ee_name != "right_gripper")
            {
                ee_type = "clicksmart";
            }
            else
            {
                ee_type = "electric";
                return;
            }
            node_time = msg.time;
            if (msg.devices[0].status.tag == "down" || msg.devices[0].status.tag == "unready")
            {
                ROS_INFO("Activating ClickSmart...");
                initialize();
            }
        }
    }
}

void Gripper::initialize(double _timeout)
{
    IOComponentCommand cmd;
    cmd.time = ros::Time::now();
    cmd.op = "activate";
    cmd.args = "{\"devices\": [\"" + ee_name + "\"]}";
    pub_end_effector_cmd.publish(cmd);
    ros::Duration(0.5).sleep();
}

void Gripper::gripperCb(const IODeviceStatus &msg)
{
    ROS_DEBUG("[%s_gripper][%s] Received new state", getGripperLimb().c_str(), type().c_str());
    setGripperState(msg);

    if (first_run)
    {
        // wait for the properties to be set before issuing calibration command
        while(not prop_set)
        {
            ros::Duration(0.05).sleep();
        }

        if (!is_calibrated())
        {
            ROS_INFO("[%s_gripper][%s] Calibrating the gripper..",
                        getGripperLimb().c_str(), type().c_str());
            calibrate();
        }

        first_run=false;
    }
}

void Gripper::gripperPropCb(const IODeviceConfiguration &msg)
{
    ROS_DEBUG("[%s_gripper][%s] Received gripper properties",
                   getGripperLimb().c_str(), type().c_str());
    setGripperProperties(msg);
    prop_set = true;

    // shut down the subscriber after the properties are set once
    // because these properties do not change
    sub_prop.shutdown();
}

void Gripper::setParameters(std::string _parameters, bool _defaults)
{
    if(_defaults)
    {
        parameters = validParameters();
    }
    else
    {
        if(_parameters != "")
        {
            // some error checking needed here to prevent invalid parameters being set
            // this would involve converting between strings and dict-type objects
            // and comparing to valid_parameters' keys
            parameters = _parameters;
        }
    }
    // send the parameters to the gripper
    // std::string param_cmd = IOComponentCommands::CMD_CONFIGURE;
    // command(param_cmd, false, 0.0, parameters);
}

std::string Gripper::validParameters()
{
    std::string valid;

    if(type() == "electric")
    {
        valid = "{\"velocity\" : 50.0, \"moving_force\" : 40.0, "
                "\"holding_force\" : 30.0, \"dead_zone\" : 5.0}";
    }
    else if(type() == "suction")
    {
        valid = "{\"vacuum_sensor_threshold\" : 18.0, \"blow_off_seconds\" : 0.4}";
    }
    else
    {
        valid = "";
    }

    return valid;
}

void Gripper::calibrate(bool _block, double _timeout)
{
    if(type() != "electric") { capabilityWarning("calibrate"); }

    command("set", _block, _timeout, "{signals: {calibrate: {data: [True]}}}");
}

void Gripper::clearCalibration()
{
    if(type() != "electric") { capabilityWarning("clearCalibration"); }

    command("set", false, 0.0, "{signals: {calibrate: {data: [False]}}}");
}

bool Gripper::reboot()
{
    // give a warning if not capable and return
    if(type() != "electric")
    {
        capabilityWarning("reboot");
        return false;
    }

    ROS_INFO("[%s_gripper][%s] Rebooting. Please wait...",
                getGripperLimb().c_str(), type().c_str());

    command("set", true, 5.0, "{signals: {reboot: {data: [True]}}}");

    ROS_INFO("Reboot complete");
    return true;
}

std::string Gripper::get_name()
{
    return getGripperState().device.name;
}

bool Gripper::is_enabled()
{
    //return getGripperState().enabled==IODeviceStatus::STATE_TRUE;
    return true;
}

bool Gripper::is_calibrated()
{
    vector<IODataStatus> signals = getGripperState().signals;
    for (auto sig : signals)
    {
        if (sig.name == "is_calibrated")
        {
            if (sig.data == "[true]")
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}

bool Gripper::is_ready_to_grip()
{
    //return getGripperState().ready==IODeviceStatus::STATE_TRUE;
    return true;
}

bool Gripper::has_error()
{
    if (type() != "electric")
    {
        capabilityWarning("has_error");
        return false;
    }

    vector<IODataStatus> signals = getGripperState().signals;
    for (auto sig : signals)
    {
        if (sig.name == "has_error")
        {
            if (sig.data == "[true]")
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}

bool Gripper::is_sucking()
{
    if(type() != "suction") { capabilityWarning("is_sucking"); }

    // ROS_INFO("force is: %g\n",getGripperState().force);
    // return getGripperState().position<80;
    return false;
}

bool Gripper::is_gripping()
{
    if (type() != "electric")
    {
        capabilityWarning("is_gripping");
        return false;
    }
    vector<IODataStatus> signals = getGripperState().signals;
    for (auto sig : signals)
    {
        if (sig.name == "is_gripping")
        {
            if (sig.data == "[true]")
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}

/*bool Gripper::hasForce()
{
    return getGripperProperties().controls_force == true;
}

bool Gripper::hasPosition()
{
    return getGripperProperties().controls_position == true;
}*/

bool Gripper::open(bool _block, double _timeout)
{
    ROS_INFO_COND(g_print_level>=2, "[%s_gripper][%s] opening",
                   getGripperLimb().c_str(), type().c_str());

    if(type() == "electric")
    {
        return commandPosition(100.0, _block, _timeout);
    }
    else if (type() == "suction")
    {
        // check if the gripper is already not sucking
        if(not is_sucking())
        {
            ROS_WARN_COND(g_print_level>=1,"[%s_gripper][%s] requested open"
                                           " but gripper is already open",
                                           getGripperLimb().c_str(), type().c_str());

            return false;
        }

        return stop(_block, _timeout);
    }
    else if (type() == "clicksmart")
    {
        return command("set", false, 0, "{\"signals\": {\"grip_BJech7Hky4\": {\"data\": [true], \"format\": {\"type\": \"bool\"}}}}");
    }
    else
    {
        // give a warning if not capable and return
        capabilityWarning("open");
        return false;
    }
}

bool Gripper::close(bool _block, double _timeout)
{
    ROS_INFO_COND(g_print_level>=2, "[%s_gripper][%s] closing",
                   getGripperLimb().c_str(), type().c_str());

    if(type() == "electric")
    {
        return commandPosition(0.0, _block, _timeout);
    }
    else if (type() == "suction")
    {
        // no checks here for is_sucking() so that
        // the suction time may be extended as necessary
        //return commandSuction(_block, _timeout);
        return false;
    }
    else if (type() == "clicksmart")
    {
        return command("set", false, 0, "{\"signals\": {\"grip_BJech7Hky4\": {\"data\": [false], \"format\": {\"type\": \"bool\"}}}}");
    }
    else
    {
        // give a warning if not capable and return
        capabilityWarning("close");
        return false;
    }
}

bool Gripper::commandPosition(double _position, bool _block, double _timeout)
{
    // give a warning if not capable and return
    if(type() != "electric")
    {
        capabilityWarning("commandPosition");
        return false;
    }

    // calibrate the electric gripper if needed
    // do it in a blocking manner so that the gripper does not move before calibration
    if(not is_calibrated())
    {
        ROS_INFO("Calibrating gripper. Please wait...");
        calibrate(true, 3.0);
        ROS_INFO("Calibration complete");
    }

    // ensures that the gripper is positioned within physical limits
    if(_position >= 0.0 && _position <= 100.0)
    {
        ROS_DEBUG("Commanding position %g", _position);
        double position_m = ((double)_position)/100.0*0.041667;
        std::string position_args = "{signals: {position_m: {data: [" + std::to_string(position_m) + "]}}}";

        return command("set", _block, _timeout, position_args);
    }
    else
    {
        ROS_WARN("[%s_gripper][%s] position must be between 0.0 and 100.0",
                                 getGripperLimb().c_str(), type().c_str());
        return false;
    }
}

/*bool Gripper::commandSuction(bool _block, double _timeout)
{
    // give a warning if not capable and return
    if(type() != "suction")
    {
        capabilityWarning("commandSuction");
        return false;
    }

    std::string suction_cmd = IOComponentCommands::CMD_GO;
    std::string suction_args = "{\"grip_attempt_seconds\": " +
                                std::to_string(_timeout) + "}";

    return command(suction_cmd, _block, _timeout, suction_args);
}*/

bool Gripper::stop(bool _block, double _timeout)
{
    /*std::string stop_cmd;

    if(type() == "electric")
    {
        stop_cmd = IOComponentCommands::CMD_STOP;
    }
    else if(type() == "suction")
    {
        stop_cmd = IOComponentCommands::CMD_RELEASE;
    }
    else
    {
        // give a warning if not capable and return
        capabilityWarning("stop");
        return false;
    }*/

    return command("set", _block, _timeout, "{signals: {go: {data: [False]}}}");
}

bool Gripper::command(std::string _cmd, bool _block,
                      double _timeout, std::string _args)
{
    IOComponentCommand ee_cmd;
    ee_cmd.time = ros::Time::now();
    ee_cmd.op = _cmd;
    ee_cmd.args = "";
    if(_args != "")
    {
        ee_cmd.args = _args;
    }

    ROS_DEBUG("[%s_gripper][%s] Publishing: %s", getGripperLimb().c_str(),
                                            type().c_str(), _cmd.c_str());
    pub_cmd.publish(ee_cmd);

    if(_block)
    {
        ros::Duration timeout(_timeout);
        return wait(timeout);
    }

    return true;
}

void Gripper::capabilityWarning(std::string _function)
{
    ROS_WARN("[%s_gripper][%s] not capable of %s",
              getGripperLimb().c_str(), type().c_str(), _function.c_str());
}

int Gripper::incCmdSeq()
{
    // prevents cmd_sequence from overflowing the integer limit
    cmd_sequence = (cmd_sequence % std::numeric_limits<int>::max()) + 1;
    return cmd_sequence;
}

std::string Gripper::type()
{
    return ee_type;
}

bool Gripper::wait(ros::Duration _timeout)
{
    // waits until the difference between the start and
    // current time catches up to the timeout
    ros::Rate r(100);
    ros::Time start = ros::Time::now();

    while(ros::ok())
    {
        ROS_DEBUG("Waiting...");
        if (ros::Time::now() - start > _timeout) { return true; };

        r.sleep();
    }

    return false;
}

Gripper::~Gripper()
{
    spinner.stop();
}
