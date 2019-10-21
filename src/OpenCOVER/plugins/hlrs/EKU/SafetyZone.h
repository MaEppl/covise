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

class SafetyZone
{
    friend class mySensor;
public:

    osg::Box *zone;

    enum Priority{
        PRIO2 = 1, // must be observed with at least 1 camera
        PRIO1 = 2, //must be observed with at least 2 cameras
    };

    SafetyZone(osg::Vec3 pos, Priority priority, float length, float width, float height);
   // SafetyZone(const SafetyZone&, Priority priority) { ++count; }
    ~SafetyZone();
    virtual bool destroy();
    static size_t count;

    osg::ref_ptr<osg::Geode> getSafetyZoneDrawable()const{return safetyZoneGeode;}
    void preFrame();
    void updateColor();
    void resetColor();
    void updatePosInWorld();
    osg::Vec3 getPosition(){
      //  updatePosInWorld();
    //    std::cout<<name<<pos.x()<<" "<<pos.y()<<" "<<pos.z()<<std::endl;
        std::cout<<name<<" SZ: "<<pos.x()<<", "<<pos.y()<<", "<<pos.z()<<std::endl;

        return pos;}
    void setPosition( osg::Matrix matrix1)
    {
        pos=safetyZoneGeode->getBound().center()*matrix1;
        pos.z()=1.0;
        std::cout<<name<<" SZ: "<<pos.x()<<", "<<pos.y()<<", "<<pos.z()<<std::endl;
    }
    int getPriority(){return priority;}
private:
    float length = 2;//8
    float width = 2;//2
    float height = 8;//2sss
    const int priority;
    std::string name;
    osg::Vec3 pos;
    osg::ref_ptr<osg::Geode> safetyZoneGeode;
    osg::ref_ptr<osgText::Text> text;
    osg::ref_ptr<osg::TessellationHints> hint;
    osg::ShapeDrawable *safetyZoneDrawable;
    void setStateSet(osg::StateSet *stateSet);
    osg::ref_ptr<osg::MatrixTransform> transMat;


    //user Interaction
 /*   mySensor *aSensor;
    vrui::coTrackerButtonInteraction *myinteraction;
    bool interActing;
    coSensorList sensorList;
    */
};



