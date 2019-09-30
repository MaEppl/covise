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
    osg::Vec3 pos;
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
    void updateColor();
    void resetColor();

private:
    const float length = 2.0f;//8
    const float width = 2.0f;//2
    const float height = 8.0f;//2
    const int priority;
    osg::ref_ptr<osg::Geode> truckGeode;
    osg::ref_ptr<osgText::Text> text;
    osg::ref_ptr<osg::TessellationHints> hint;
    osg::ShapeDrawable *truckDrawable;
    void setStateSet(osg::StateSet *stateSet);
};



