#pragma once

#include<osg/ShapeDrawable>
#include<OpenVRUI/coTrackerButtonInteraction.h>
#include<cover/coVRPluginSupport.h>
#include<cover/coVRSelectionManager.h>
#include<PluginUtil/coSensor.h>

#include<Cam.h>
#include<Truck.h>
using namespace covise;
using namespace opencover;

class Truck;
class CamDrawable;
class mySensor : public coPickSensor
{
public:
    mySensor(osg::Node *node, std::string name,vrui::coTrackerButtonInteraction *_interactionA, osg::ShapeDrawable *cSphDr);
    mySensor(osg::Node *node, std::string name,vrui::coTrackerButtonInteraction *_interactionA, CamDrawable *camDr,std::vector<Truck*> *observationPoints, std::vector<CamDrawable*> *cams);
    mySensor(osg::Node *node,int pos, std::string name,vrui::coTrackerButtonInteraction *_interactionA, Truck *safetyDraw ,std::vector<CamDrawable*> *cams);
    mySensor(osg::Node *node, std::string name,vrui::coTrackerButtonInteraction *_interactionA);
    ~mySensor();

    void activate();
    void disactivate();
    std::string getSensorName();
    bool isSensorActive();

private:
    std::string sensorName;
    bool isActive;
    vrui::coTrackerButtonInteraction *_interA;
    osg::ShapeDrawable *shapDr = nullptr;
    CamDrawable *camDr = nullptr;
    Truck *safetyDr = nullptr;
    std::vector<Truck*> *observationPoints=nullptr;
    std::vector<CamDrawable*> *cams=nullptr;
    int pos;
};

