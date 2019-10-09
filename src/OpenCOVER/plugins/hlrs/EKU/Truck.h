#pragma once

#include <cover/coVRPluginSupport.h>
#include <cover/coVRFileManager.h>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Vec4>
#include<osgText/Font>
#include<osgText/Text>

#include<Sensor.h>
using namespace opencover;

class Truck
{
    friend class mySensor;
public:

    osg::Box *truck;

    enum Priority{
        PRIO2 = 1, // must be observed with at least 1 camera
        PRIO1 = 2, //must be observed with at least 2 cameras
    };

    Truck(osg::Vec3 pos, Priority priority);
   // Truck(const Truck&, Priority priority) { ++count; }
    ~Truck();
    virtual bool destroy();
    static size_t count;

    osg::ref_ptr<osg::Geode> getTruckDrawable()const{return truckGeode;}
    void preFrame();
    void updateColor();
    void resetColor();
    void updatePosInWorld();
    osg::Vec3 getPosition(){
        updatePosInWorld();
       // std::cout<<worldPosition.x()<<" "<<worldPosition.y()<<" "<<worldPosition.z()<<std::endl;
        return pos;}
private:
    const float length = 2.0f;//8
    const float width = 2.0f;//2
    const float height = 8.0f;//2
    const int priority;
    osg::Vec3 pos;
    osg::ref_ptr<osg::Geode> truckGeode;
    osg::ref_ptr<osgText::Text> text;
    osg::ref_ptr<osg::TessellationHints> hint;
    osg::ShapeDrawable *truckDrawable;
    void setStateSet(osg::StateSet *stateSet);

    //user Interaction
 /*   mySensor *aSensor;
    vrui::coTrackerButtonInteraction *myinteraction;
    bool interActing;
    coSensorList sensorList;
    */
};



