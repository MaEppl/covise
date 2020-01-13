/* This file is part of COVISE.
 *

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#pragma once
#include <iostream>
#include <string>

#include<osg/Node>
#include<osg/ShapeDrawable>
#include<osgFX/Outline>

#include<EKU.h>

using namespace opencover;
class Equipment
{
public:
    Equipment(std::string name,osg::Matrix position,osg::ref_ptr<osg::Node> node);
    void preFrame();
    void showOutline(bool status);
    osg::Matrix getPosition(){return matrix.get()->getMatrix();}

protected:
    std::string name;
    osg::ref_ptr<osg::Group> group;
    osg::ref_ptr<osg::Switch> switchNode;
    osg::ref_ptr<osg::MatrixTransform> matrix;
    osg::ref_ptr<osgFX::Outline> outline;

    //user Interaction
    mySensor *aSensor;
    vrui::coTrackerButtonInteraction *myinteractionA;
    bool interActingA;
    coSensorList sensorList;

};

class CamPosition;
class EquipmentWithCamera: public Equipment
{

public:
    EquipmentWithCamera(std::string name,osg::Matrix position,osg::ref_ptr<osg::Node> node,std::vector<osg::Matrix> camMatrixes);
    void preFrame();
private:
    std::vector<std::weak_ptr<CamPosition>> cameraPositions;
};



