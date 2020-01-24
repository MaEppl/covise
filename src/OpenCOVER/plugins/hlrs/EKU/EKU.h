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
#include<Equipment.h>

using namespace opencover;

void restrictMovement(osg::Matrix &mat);

class mySensor;
class Equipment;
class EKU: public opencover::coVRPlugin, public opencover::ui::Owner
{
    friend class mySensor;
public:
    static bool modifyScene;
    static bool deleteObjects;
    EKU();
    ~EKU();
    bool init();
    void doAddTruck(osg::Matrix pos);
    void doRemoveTruck(std::unique_ptr<EquipmentWithCamera> &t);
    void doAddCam();
    void doRemoveCam(std::shared_ptr<CamPosition> &c);
    void doRemoveAllCams();
    void doAddPRIO1(osg::Vec3 pos, double l, double w, double h);
    void doAddPRIO2(osg::Vec3 pos, double l, double w, double h);
    void doRemovePRIOZone(std::shared_ptr<SafetyZone> &s);
    void doRemoveAllPRIOzones();

    void doCalcVisMat();
    void findNotVisiblePoints();

    virtual void preFrame();

    static std::vector<std::shared_ptr<SafetyZone>> safetyZones;
    static std::vector<std::shared_ptr<CamPosition>> allCamPositions;
    static std::vector<std::unique_ptr<Equipment>> equipment;
    static std::vector<std::unique_ptr<EquipmentWithCamera>> equipmentWithCamera;

    //std::vector<Cam*> cameras;
    std::vector<CamDrawable*> finalCams;


    GA *ga =nullptr;
    static EKU *plugin;
    osg::ref_ptr<osg::Group> finalScene;

    static void updateCoverage(float totalCoverage, float prio1Coverage, float prio2Coverage);
    static void updateNbrCams();
    static void updateNbrPoints();
    static void updateOptimizationResults(float totalCoverage, float prio1Coverage, float prio2Coverage, float fitness, float time);


private:
    std::string path="/home/AD.EKUPD.COM/matthias.epple/Schreibtisch/MA3dModels/";
    //UI
    ui::Menu *EKUMenu  = nullptr;
    ui::Action *AddTruck = nullptr, *RmvTruck = nullptr,*RmvCam = nullptr, *AddCam = nullptr,*OptOrient = nullptr,
    *OptNbrCams = nullptr,*AddPRIO1 = nullptr,*AddPRIO2 = nullptr, *RmvSafetyZone = nullptr,*StopGA = nullptr,
            *rmvAllSafetyZones = nullptr, *rmvAllCameras = nullptr;
    ui::Group *Frame = nullptr;
    ui::Label *Label = nullptr;
    ui::Button *ModifyScene = nullptr,*Delete =nullptr;//*MakeCamsInvisible = nullptr;

    //Menu for Optimization parameters
    ui::Menu *Optimize = nullptr;
    ui::Slider *populationSize = nullptr;
    ui::Slider *penalty = nullptr, *weighting = nullptr;
    ui::Button *dynamicThreading = nullptr;

    //Menu for optimization results
    ui::Menu *results = nullptr;
    ui::Label *r_totalCoverage = nullptr, *r_prio1Coverage = nullptr,*r_prio2Coverage = nullptr,*r_fitness = nullptr,*r_nbrCams = nullptr,*r_nbrControlPoints = nullptr,*r_optimizationTime = nullptr;//*r_nbrOrientations = nullptr;

    ui::Menu *Camera = nullptr;
    ui::Slider *FOVRegulator = nullptr, *VisibilityRegulator = nullptr;
    ui::Button  *ShowSearchSpace = nullptr,*ShowDeletedSearchSpace = nullptr,*ShowRealSize = nullptr;
    ui::Action *calcVisMat = nullptr;

    //Equipment
    void createScene();
    osg::ref_ptr<osg::Node> silo1;
    osg::ref_ptr<osg::Node> container;
    osg::ref_ptr<osg::Node> christmasTree;
    osg::ref_ptr<osg::Node> manifold;
    osg::ref_ptr<osg::Node> blender;
    osg::ref_ptr<osg::Node> dataVan;
    osg::ref_ptr<osg::Node> truck;
    osg::ref_ptr<osg::Node> truckSurfaceBox;
    osg::ref_ptr<osg::Node> truckCabine;
    osg::ref_ptr<osg::Node> landscape;


    void createSafetyZone(float xpos,float ypos,SafetyZone::Priority prio);
    void calcPercentageOfCoveredSafetyZones();
    void changeStatusOfInteractors(bool status);

};




#endif
