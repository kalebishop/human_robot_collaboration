#ifndef __ROS_THREAD_H__
#define __ROS_THREAD_H__

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <pthread.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>

#include "utils.h"

/**
 * @brief A ROS Thread class
 * @details This class initializes overhead functions necessary to start a thread
 *          from within a class, and overhead ROS features: subscriber/publishers,
 *          services, callback functions etc.
 */
class ROSThread
{
private:
    ros::Time _init_time;
    State _state;
    std::string _limb;

    pthread_t _thread;
    static void * InternalThreadEntryFunc(void * This);

    ros::AsyncSpinner spinner;

    ros::Subscriber _endpt_sub;
    ros::Subscriber _ir_sub;
    ros::ServiceClient _ik_client;

protected:

    ros::NodeHandle _n;

    geometry_msgs::Pose   _curr_pose;
    geometry_msgs::Point  _curr_position;
    geometry_msgs::Wrench _curr_wrench;

    std::vector<double> _filt_force;

    float _curr_range, _curr_max_range, _curr_min_range;

    ros::Publisher  _joint_cmd_pub;

    void pause();

    /*
     * Function that will be spun out as a thread
     */
    virtual void InternalThreadEntry() = 0;

    /*
     * Uses built in IK solver to find joint angles solution for desired pose
     * 
     * @param     requested PoseStamped
     * @param     array of joint angles solution
     * @return    true/false if success/failure
     */
    bool getJointAngles(geometry_msgs::PoseStamped& pose_stamped, std::vector<double>& joint_angles);

    /*
     * Moves arm to the requested pose
     * 
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @param  mode (either loose or strict, it checks for the final desired position)
     * @return true/false if success/failure
     */
    bool goToPose(double px, double py, double pz,
                  double ox, double oy, double oz, double ow, std::string mode="loose");

    /*
     * Sets the joint names of a JointCommand
     * 
     * @param    joint_cmd the joint command
     */
    void setJointNames(baxter_core_msgs::JointCommand& joint_cmd);

    /*
     * Detects if the force overcame a set threshold in either one of its three axis
     * 
     * @return true/false if the force overcame the threshold
     */
    bool detectForceInteraction();

    /*
     * Waits for a force interaction to occur.
     * 
     * @return true when the force interaction occurred
     * @return false if no force interaction occurred after 10s
     */
    bool waitForForceInteraction(double _wait_time = 10.0);

    /*
     * Prevents any following code from being executed before thread is exited
     * 
     * @param      N/A
     * @return     true if thread was successfully launched; false otherwise
     */      
    void WaitForInternalThreadToExit();

    /*
     * Callback function that sets the current pose to the pose received from 
     * the endpoint state topic
     * 
     * @param      N/A
     * @return     N/A
     */
    void endpointCallback(const baxter_core_msgs::EndpointState& msg);

    /*
     * Infrared sensor callback function that sets the current range to the range received
     * from the left hand range state topic
     * 
     * @param      The message
     * @return     N/A
     */
    void IRCallback(const sensor_msgs::RangeConstPtr& msg);

    /*
     * Filters the forces with a very simple low pass filter
     */
    void filterForces();

    /*
     * hover arm above tokens
     * 
     * @param      double indicating requested height of arm (z-axis)
     * return     N/A
     */
    void hoverAboveTokens(double height);

public:
    ROSThread(std::string limb);
    virtual ~ROSThread();

    /*
     * Starts thread that executes the internal thread entry function
     * 
     * @param      N/A
     * @return     true if thread was successfully launched; false otherwise
     */        
    bool startInternalThread();

    /*
     * Self-explaining "setters"
     */   
    void setState(int state);

    /*
     * Self-explaining "getters"
     */
    State       getState() { return _state; };
    std::string getLimb()  { return _limb;  };
};

/**
 * @brief A ROS Thread with an image callbck
 * @details This class inherits from ROSThread, but it adds also an image callback
 *          to be overwritten by its children. Useful to to visual processing.
 */
class ROSThreadImage : public ROSThread
{
private:
    image_transport::ImageTransport _img_trp;
    image_transport::Subscriber _img_sub;

protected:
    cv::Mat _curr_img;
    cv::Size _curr_img_size;
    bool _curr_img_empty;

    pthread_mutex_t _mutex_img;

public:
    ROSThreadImage(std::string limb);
    ~ROSThreadImage();

    /*
     * image callback function that displays the image stream from the hand camera 
     * 
     * @param      The image
     * @return     N/A
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif