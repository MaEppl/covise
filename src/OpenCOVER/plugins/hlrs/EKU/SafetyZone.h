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
       // pos=osg::Vec3{-2.3,0.0,9.0}*matrix1;
        pos=safetyZoneGeode->getBound().center()*matrix1;
        pos.z()=1.0;

       std::cout<<name<<" SZ: "<<pos.x()<<", "<<pos.y()<<", "<<pos.z()<<std::endl;

     /*    updatePosInWorld();
        osg::Matrix matrix2;
        matrix2.setTrans(2.3,0,9);
        matrix2.get
        auto test =matrix2*matrix1;
     */  // std::cout<<"test"<<test.x()<<" "<<test.y()<<" "<<test.z()<<std::endl;
      //  pos ={(float)test.getTrans().x(),(float)test.getTrans().y(),(float)test.getTrans().z()};

//        pos ={(float)matrix1.getTrans().x()+2.3f,(float)matrix1.getTrans().y(),(float)matrix1.getTrans().z()+9.0f};
    }
   // pos ={1.0,1.0,1.0};
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



