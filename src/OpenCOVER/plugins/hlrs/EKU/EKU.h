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

#include <OpenVRUI/osg/mathUtils.h>
#include <PluginUtil/coVR3DTransRotInteractor.h>
#include <cover/coVRPluginSupport.h>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Vec4>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include<osg/Node>
#include<osg/Switch>
#include<osgFX/Outline>

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

//returns the highest z value of a safetyZone
double getZvalueOfSZ();


class Pump
{
public:
    static size_t counter;

    Pump(std::vector<std::shared_ptr<CamPosition>>& allCams,std::vector<std::shared_ptr<SafetyZone>> &allSZ,osg::ref_ptr<osg::Node> truck,osg::ref_ptr<osg::Node> PumptruckSurface, osg::ref_ptr<osg::Node> cabine, osg::Vec3 pos, int rotZ);
    ~Pump();

    //std::vector<CamPosition*> possibleCamLocations;
    //std::vector<CamDrawable*> placedCameras;
    //std::array<SafetyZone*,2> safetyZones;

    osg::Vec3 getPos()const{return position;}
    int getRot()const{return rotZ;}
    osg::ref_ptr<osg::Group> getPumpDrawable()const{return upperGroup;}

    std::weak_ptr<SafetyZone> szLeft;
    std::weak_ptr<SafetyZone> szRight;
    std::weak_ptr<CamPosition> camLeft;
    std::weak_ptr<CamPosition> camRight;



    //User Interaction
    void preFrame();

private:
    std::string name;
    osg::Vec3 position;
    int rotZ;
    osg::ref_ptr<osg::Node> truck;
    osg::ref_ptr<osg::Node> truckSurfaceBox;
    osg::ref_ptr<osg::Node> truckCabine;
    osg::ref_ptr<osgFX::Outline> outline;

    osg::ref_ptr<osg::MatrixTransform> transMat;
    osg::ref_ptr<osg::MatrixTransform> rotMat;
    osg::ref_ptr<osg::MatrixTransform> fullMat;
    osg::ref_ptr<osg::MatrixTransform> transCabine;
    osg::ref_ptr<osg::MatrixTransform> rotCabine;

    osg::ref_ptr<osg::Group> group;
    osg::ref_ptr<osg::Group> group1;
    osg::ref_ptr<osg::Group> upperGroup;
    osg::ref_ptr<osg::Switch> switchBetween;

    std::vector<std::shared_ptr<CamPosition>> &allCams;
    std::vector<std::shared_ptr<SafetyZone>> &allSZ;



    //user Interaction
    mySensor *aSensor;

    vrui::coTrackerButtonInteraction *myinteractionA;
    bool interActingA;
    coSensorList sensorList;


};

class mySensor;
class EKU: public opencover::coVRPlugin, public opencover::ui::Owner
{
    friend class mySensor;
public:
    static bool modifyScene;
    EKU();
    ~EKU();
    bool init();
    void doAddTruck();
    void doRemoveTruck(std::unique_ptr<Pump> &t);
    void doAddCam();
    void doRemoveCam(std::shared_ptr<CamPosition> &c);
    void doAddPRIO1(osg::Vec3 pos, double l, double w, double h);
    void doAddPRIO2(osg::Vec3 pos, double l, double w, double h);
    void doRemovePRIOZone(std::shared_ptr<SafetyZone> &s);
    void doCalcVisMat();
    void findNotVisiblePoints();

    virtual void preFrame();

    static std::vector<std::shared_ptr<SafetyZone>> safetyZones;
    static std::vector<std::shared_ptr<CamPosition>> allCamPositions;
    static std::vector<std::unique_ptr<Pump>> allPumps;



    //std::vector<Cam*> cameras;
    std::vector<CamDrawable*> finalCams;


    GA *ga =nullptr;
    static EKU *plugin;
    osg::ref_ptr<osg::Group> finalScene;
    void restrictMovement(osg::Matrix &mat);
private:
    //UI
    ui::Menu *EKUMenu  = nullptr;
    ui::Action *AddTruck = nullptr, *RmvTruck = nullptr,*RmvCam = nullptr, *AddCam = nullptr,*OptOrient = nullptr,*OptNbrCams = nullptr,*AddPRIO1 = nullptr,*AddPRIO2 = nullptr, *RmvSafetyZone = nullptr,*calcVisMat = nullptr,*StopGA = nullptr;
    ui::Slider *FOVRegulator = nullptr, *VisibilityRegulator = nullptr;
    ui::Group *Frame = nullptr;
    ui::Label *Label = nullptr;
    ui::Button *MakeCamsInvisible = nullptr, *ShowSearchSpace = nullptr,*ModifyScene = nullptr;


    ui::Menu *Optimize = nullptr;
    ui::Slider *penalty = nullptr, *weighting = nullptr;


    ui::Menu *Test = nullptr;
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
    osg::ref_ptr<osg::Node> christmasTree;



    osg::ref_ptr<osg::Node> truck;
    osg::ref_ptr<osg::Node> truckSurfaceBox;
    osg::ref_ptr<osg::Node> truckCabine;


    void createSafetyZone(float xpos,float ypos,SafetyZone::Priority prio);

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

    CamPosition *testCam;


};




#endif
