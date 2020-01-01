/* This file is part of COVISE.
 *

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#pragma once
#include <iostream>
#include <string>

#include<osg/Node>
#include <osg/ShapeDrawable>
#include<osgFX/Outline>

#include<EKU.h>
//#include<Sensor.h>
using namespace opencover;
class Equipment
{
public:
    Equipment(std::string name,osg::Matrix position,osg::ref_ptr<osg::Node> node);
    void preFrame();
    void showOutline(bool status);

private:
    std::string name;
    osg::Matrix pos;
    osg::ref_ptr<osg::MatrixTransform> matrix;
    osg::ref_ptr<osg::Switch> switchNode;
    osg::ref_ptr<osgFX::Outline> outline;



    //user Interaction
    mySensor *aSensor;
    vrui::coTrackerButtonInteraction *myinteractionA;
    bool interActingA;
    coSensorList sensorList;

};
