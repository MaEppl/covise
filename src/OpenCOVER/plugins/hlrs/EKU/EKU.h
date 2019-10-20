/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef EKUEXAMPLEPLUGIN_H
#define EKUEXAMPLEPLUGIN_H
/****************************************************************************\
 **                                                            (C)2018 HLRS  **
 **                                                                          **
 ** Description: EKU camera position optimization for a bore field           **
 **                                                                          **
 **                                                                          **
 ** Author: Matthias Epple	                                                 **
 **                                                                          **
 ** History:  								                                 **
 ** Juni 2019  v1	    				       		                         **
 **                                                                          **
 **                                                                          **
\****************************************************************************/
namespace opencover
{
namespace ui
{
class Button;
class Menu;
class Group;
class Slider;
class Label;
class Action;
}
}

#include <cover/coVRPlugin.h>
#include <cover/coVRMSController.h>
#include <cover/ui/Button.h>
#include <cover/ui/Slider.h>
#include <cover/ui/Group.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Label.h>
#include <cover/ui/Action.h>

#include <cover/coVRPluginSupport.h>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Vec4>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include<osg/Node>


#include <iostream>
#include <vector>
#include<functional>

#include <cover/ui/Owner.h>
#include<Cam.h>
#include<SafetyZone.h>
#include<GA.hpp>
#include<FileReader.hpp>
#include<Sensor.h>


using namespace opencover;
class Pump
{
public:
    static size_t counter;

    Pump(osg::ref_ptr<osg::Node> truck,osg::ref_ptr<osg::Node> PumptruckSurface, osg::ref_ptr<osg::Node> cabine, osg::Vec3 pos, int rotZ);
    ~Pump();

    std::vector<CamPosition*> possibleCamLocations;
    std::vector<CamDrawable*> placedCameras;
    std::array<SafetyZone*,2> safetyZones;

    osg::Vec3 getPos()const{return position;}
    int getRot()const{return rotZ;}
    osg::ref_ptr<osg::Group> getPumpDrawable()const{return fullMat;}

    //User Interaction
    void preFrame();

private:
    std::string name;
    osg::Vec3 position;
    int rotZ;
    osg::ref_ptr<osg::Node> truck;
    osg::ref_ptr<osg::Node> truckSurfaceBox;
    osg::ref_ptr<osg::Node> truckCabine;

    osg::ref_ptr<osg::MatrixTransform> transMat;
    osg::ref_ptr<osg::MatrixTransform> rotMat;
    osg::ref_ptr<osg::MatrixTransform> fullMat;
    osg::ref_ptr<osg::MatrixTransform> transCabine;
    osg::ref_ptr<osg::MatrixTransform> rotCabine;

    osg::ref_ptr<osg::Group> group;
    osg::ref_ptr<osg::Group> group1;

    //user Interaction
    mySensor *aSensor;
    vrui::coTrackerButtonInteraction *myinteraction;
    bool interActing;
    coSensorList sensorList;
};

class mySensor;
class EKU: public opencover::coVRPlugin, public opencover::ui::Owner
{
    friend class mySensor;
public:
    EKU();
    ~EKU();
    bool init();
    void doAddTruck();
    void doRemoveTruck();
    void doAddCam();
    void doRemoveCamera();
    std::vector<osg::Vec3>& getObservationPoints(){return observationPoints;}
    //osg::Material *mtl;
    virtual void preFrame();

    std::vector<SafetyZone*> safetyZones;

    std::vector<Cam*> cameras;
    std::vector<CamDrawable*> finalCams;
    std::vector<Pump*> allPumps;

    GA *ga;
    static EKU *plugin;
    osg::ref_ptr<osg::Group> finalScene;

private:
    //UI
    ui::Menu *EKUMenu  = nullptr;
    ui::Action *AddTruck = nullptr, *RmvTruck = nullptr, *AddCam = nullptr,*OptOrient = nullptr,*OptNbrCams = nullptr;
    ui::Slider *FOVRegulator = nullptr, *VisibilityRegulator = nullptr;
    ui::Group *Frame = nullptr;
    ui::Label *Label = nullptr;
    ui::Button *MakeCamsInvisible = nullptr;

    std::vector<SafetyZone::Priority> priorityList;
    std::vector<osg::Vec3> observationPoints;

    osg::MatrixTransform *mymtf;
    vrui::coTrackerButtonInteraction *myinteraction;
    bool interActing;
    coSensorList sensorList;
    std::vector<mySensor*> userInteraction;

     //Landscape
    void createScene();
    osg::ref_ptr<osg::Node> silo1;
    osg::ref_ptr<osg::Node> silo2;
    osg::ref_ptr<osg::Node> container;


    osg::ref_ptr<osg::Node> truck;
    osg::ref_ptr<osg::Node> truckSurfaceBox;
    osg::ref_ptr<osg::Node> truckCabine;


    void createSafetyZone(float xpos,float ypos);

    void updateObservationPointPosition();
    void updateAllCameras();
    //Raycasting for intersection calculation is too slow with many vertices
    void disactivateDetailedRendering(){
         truck->setNodeMask(0);
         truckSurfaceBox->setNodeMask(UINT_MAX);
    }
    void activateDetailedRendering(){
         truck->setNodeMask(UINT_MAX);
         truckSurfaceBox->setNodeMask(0);
    }

    void calcPercentageOfCoveredSafetyZones();



    void removeCamDrawable(CamDrawable *cam);

  //  FileReaderWriter *readerWriter;
  //  FindNamedNode fnn;//NOTE: make to pointer

};




#endif
