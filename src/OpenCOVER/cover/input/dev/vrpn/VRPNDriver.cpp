/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

/*
 * inputhdw.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: svnvlad
 */
#include "VRPNDriver.h"

#include <config/CoviseConfig.h>

#include <iostream>
#include <osg/Matrix>

#include <OpenVRUI/osg/mathUtils.h> //for MAKE_EULER_MAT

using namespace std;
using namespace covise;

#include <util/unixcompat.h>
#include <iostream>

#include <quat.h>

using namespace std;
void VRPNDriver::vrpnCallback(void *userdata, const vrpn_TRACKERCB t)
{
    VRPNDriver *vrpn = reinterpret_cast<VRPNDriver *>(userdata);
    vrpn->processTrackerData(t);
}

void VRPNDriver::vrpnButtonCallback(void *userdata, const vrpn_BUTTONCB t)
{
    VRPNDriver *vrpn = reinterpret_cast<VRPNDriver *>(userdata);
    vrpn->processButtonData(t);
}

VRPNDriver::VRPNDriver(const std::string &config)
    : InputDevice(config)
{
    
    vrpnTracker=NULL;
    vrpnButton=NULL;

    std::string host = coCoviseConfig::getEntry("host", configPath(), "localhost");
    std::string tracker = coCoviseConfig::getEntry("tracker", configPath(), "");
    std::string button = coCoviseConfig::getEntry("button", configPath(), "");

    trackerid = tracker + "@" + host;
    buttonid = button + "@" + host;
    if(tracker.size()>0)
    {
        vrpnTracker = new vrpn_Tracker_Remote(trackerid.c_str());
        vrpnTracker->register_change_handler(this, (vrpn_TRACKERCHANGEHANDLER)vrpnCallback);
    }
    if(button.size()>0)
    {
        vrpnButton = new vrpn_Button_Remote (buttonid.c_str());
        vrpnButton->register_change_handler(this, (vrpn_BUTTONCHANGEHANDLER)vrpnButtonCallback);
    }

}

//====================END of init section============================


VRPNDriver::~VRPNDriver()
{
    stopLoop();
    delete vrpnTracker;
    delete vrpnButton;
}

//==========================main loop =================

/**
 * @brief VRPNDriver::run ImputHdw main loop
 *
 * Gets the status of the input devices
 */
bool VRPNDriver::poll()
{
    if (vrpnTracker==NULL && vrpnButton==NULL)
        return false;
    vrpnTracker->mainloop();
    vrpnButton->mainloop();
    return true;
}

void VRPNDriver::processTrackerData(const vrpn_TRACKERCB &vrpnData)
{

    m_mutex.lock();
    qgl_matrix_type mat;
    q_type quat;
    for (int n = 0; n < 4; ++n)
        quat[n] = vrpnData.quat[n];
    qgl_to_matrix(mat, quat);
    osg::Matrix matrix;
    for(int n=0;n<3;n++)
        for(int m=0;m<3;m++)
            matrix(n,m) = mat[n][m];
    matrix(0,3)=vrpnData.pos[0];
    matrix(1,3)=vrpnData.pos[1];
    matrix(2,3)=vrpnData.pos[2];
    m_bodyMatricesValid[vrpnData.sensor]=true;
    m_bodyMatrices[vrpnData.sensor]=matrix;
    m_mutex.unlock();

}

void VRPNDriver::processButtonData(const vrpn_BUTTONCB &vrpnButtonData)
{

    m_mutex.lock();
    m_buttonStates[vrpnButtonData.button]=vrpnButtonData.state!=0;
    m_mutex.unlock();

}


INPUT_PLUGIN(VRPNDriver)
