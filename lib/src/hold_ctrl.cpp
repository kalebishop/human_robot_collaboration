#include "robot_interface/hold_ctrl.h"

using namespace std;

HoldCtrl::HoldCtrl(std::string _name, std::string _limb) : 
             ArmCtrl(_name,_limb), hand_over_state("done")
{
    if (!goHome()) setState(ERROR);
}

bool HoldCtrl::doAction(int s, std::string a)
{
    if (a == ACTION_HOLD && (s == START ||
                             s == ERROR ||
                             s == DONE  ))
    {
        if (holdObject())
        {
            setState(DONE);
            return true;
        }   
        else recoverFromError();
    }
    else if (a == ACTION_HAND_OVER && (s == START ||
                                       s == ERROR ||
                                       s == DONE  ))
    {
        if (handOver())
        {
            setState(DONE);
            return true;
        }
        else recoverFromError();
    }
    else
    {
        ROS_ERROR("[%s] Invalid State %i", getLimb().c_str(), s);
    }

    return false;
}

bool HoldCtrl::handOver()
{
    hand_over_state = "start";
    if (!prepare4HandOver())                   return false;
    hand_over_state = "ready";
    if (!waitForOtherArm(120.0, true))         return false;
    if (!gripObject())                         return false;
    ros::Duration(1.0).sleep();
    if (!goHoldPose(0.24))                     return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction(180.0))       return false;
    if (!releaseObject())                      return false;
    ros::Duration(1.0).sleep();
    if (!goHome())                             return false;

    return true;
}

bool HoldCtrl::holdObject()
{
    if (!goHoldPose(0.30))                return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction(30.0))   return false;
    if (!gripObject())                    return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction(180.0))  return false;
    if (!releaseObject())                 return false;
    ros::Duration(1.0).sleep();
    if (!goHome())                        return false;

    return true;
}

bool HoldCtrl::waitForOtherArm(double _wait_time, bool disable_coll_av)
{
    ros::Time _init = ros::Time::now();

    while(ros::ok())
    {
        if (disable_coll_av)      suppressCollisionAv();
        
        if (hand_over_state == "gripped")   return true;

        ros::spinOnce();
        ros::Rate(100).sleep();

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("No feedback from other arm has been received in %gs!",_wait_time);
            return false;
        }
    }    
}

bool HoldCtrl::serviceOtherLimbCb(baxter_collaboration::AskFeedback::Request  &req,
                                  baxter_collaboration::AskFeedback::Response &res)
{
    if (req.ask == "ready")
    {
        if (hand_over_state == "start") res.reply = "wait";
        if (hand_over_state == "ready")
        {
            hand_over_state = "gripped";
            res.reply = "gripped";
        }
    }
    return true;
}

bool HoldCtrl::prepare4HandOver()
{
    return ROSThread::goToPose(0.65, 0.1, Z_LOW,
                                  HANDOVER_ORI_R);
}

bool HoldCtrl::goHoldPose(double height)
{
    return ROSThread::goToPose(0.80, -0.4, height,
                                HORIZONTAL_ORI_R);
}

HoldCtrl::~HoldCtrl()
{

}
