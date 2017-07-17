/**
 * Copyright (C) 2017 Social Robotics Lab, Yale University
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * website: www.scazlab.yale.edu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2.1 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

#ifndef __HOLD_CTRL_IMPL_H__
#define __HOLD_CTRL_IMPL_H__

#include "flatpack_furniture/hold_ctrl.h"

class HoldCtrlImpl : public HoldCtrl
{
private:

public:
    /**
     * Constructor
     */
    HoldCtrlImpl(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~HoldCtrlImpl();
};

#endif