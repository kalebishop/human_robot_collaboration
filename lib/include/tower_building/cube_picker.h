#ifndef __CUBE_PICKER_H__
#define __CUBE_PICKER_H__

#include <flatpack_furniture/artag_ctrl.h>

class CubePicker : public ArmCtrl, public ARucoClient
{
private:
    double elap_time;

    /**
     * [moveObjectTowardHuman description]
     * @return true/false if success/failure
     */
    bool moveObjectTowardHuman();

    /**
     * [pickARTag description]
     * @return true/false if success/failure
     */
    bool pickARTag();

    /**
     * [pickObject description]
     * @return true/false if success/failure
     */
    bool pickObject();

    /**
     * [passObject description]
     * @return true/false if success/failure
     */
    bool passObject();

    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

protected:
    /**
     * Executes the arm-specific and task-specific actions.
     *
     * @param  s the state of the system before starting the action
     * @param  a the action to do
     * @return   true/false if success/failure
     */
    bool doAction(int s, std::string a);

public:
    /**
     * Constructor
     */
    CubePicker(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~CubePicker();

    void setObject(int _obj);
};

#endif
