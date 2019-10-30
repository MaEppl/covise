#pragma once

#include <PluginUtil/coVR3DTransRotInteractor.h>
#include <PluginUtil/coVR3DTransInteractor.h>

#include <OpenVRUI/osg/mathUtils.h>


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
        //pos.z()=1.0;
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
using namespace osg;
class SZ
{
private:
    std::string name; //< name of the point
    int id_;
    float scale_;
    float depthscale_;
    osg::Matrix xformMat_;


    bool hasScale_;
    bool isViewAll_;
    bool hasPosition_;
    bool hasOrientation_;
    bool hasMatrix_;

    bool viewpointVisible;
    bool hasGeometry_;

    //Vec3 eyepoint{0.0,-50,0.0};
    Vec3 eyepoint{0.0,0,0.0};
    Vec3 scaleVec;



    osg::ref_ptr<osg::Geode> viewpointgeode;

    osg::ref_ptr<osg::Geometry> line1;
    osg::ref_ptr<osg::Geometry> line2;
    osg::ref_ptr<osg::Geometry> line3;
    osg::ref_ptr<osg::Geometry> line4;

    osg::ref_ptr<osg::Vec3Array> lineEyetoLeftDown;
    osg::ref_ptr<osg::Vec3Array> lineEyetoRightDown;
    osg::ref_ptr<osg::Vec3Array> lineEyetoRightUp;
    osg::ref_ptr<osg::Vec3Array> lineEyetoLeftUp;

    ref_ptr<StateSet> line1_state;
    ref_ptr<StateSet> line2_state;
    ref_ptr<StateSet> line3_state;
    ref_ptr<StateSet> line4_state;

    osg::ref_ptr<osg::Geometry> viewpointPlaneGeoset;
    osg::ref_ptr<osg::Geometry> viewpointPlaneBorderGeoset;
    ref_ptr<osg::Geometry> viewpointGeoset;
    ref_ptr<Vec3Array> viewpointCoords; // coordinates of viewpoint-box(Plane)
    ref_ptr<Vec3Array> viewpointBorderCoords; //// coordinates of viewpoint-box(Border)

    ref_ptr<StateSet> viewpointPlaneGeoset_state;
    ref_ptr<StateSet> viewpointGeoset_state;
    ref_ptr<StateSet> viewpointPlaneBorderGeoset_state;

    //interactors
    coVR3DTransRotInteractor *viewpointInteractor; // Angriffspunkt in Mitte
    coVR3DTransInteractor *scaleInteractor;
    coVR3DTransInteractor *depthInteractor;



    void loadUnlightedGeostate(ref_ptr<StateSet> state);

public:
    static double imgWidth;
    static double imgHeight;
    static double imgWidthPixel;
    static double imgHeightPixel;
    static double fov;
    static double depthView;
    static double focalLengthPixel;

    coCoord coord; //< pos + orientation
    ref_ptr<MatrixTransform> localDCS;

    SZ(const char *name, int id, osg::Vec3 pos,osg::Vec3 hpr, float scale,float depthscale);

    bool hasGeometry() {return hasGeometry_;}
    void createGeometry(osg::Vec3 pos, osg::Vec3 euler);
    void updateGeometry();
    void showGeometry(bool);
    void setPosition(coCoord pos);
    void setPosition(osg::Vec3 pos);
    void setEuler(coCoord euler);
    void setEuler(osg::Vec3 euler);
    void setScale(float scale);

    void preFrame();
    void showInteractors(bool state);

};
class Wireframe;
class SZ2
{
private:
    osg::Vec3 position;
    coVR3DTransInteractor *widthInteractor;
    float interactorSize;
public:
    SZ2(osg::Vec3 initialPosition);
    ~SZ2();
    osg::Vec3 getPosition()
    {
        return position;
    }
    coVR3DTransInteractor *getInteractor()
    {
        return widthInteractor;
    }

    // direct interaction
    void preFrame();
    void updatePosition(osg::Vec3 newPosition);
    void showInteractors(bool state);

};

class Wireframe
{
private:
    Vec3 pos;
    osg::ref_ptr<osg::Vec3Array> vertices;
    osg::ref_ptr<osg::Vec3Array> normals;
    osg::ref_ptr<osg::Vec4Array> colors;
    osg::ref_ptr<osg::Geometry> quad ;


public:
    Wireframe();
    osg::ref_ptr<osg::Geode> wireframe;
    void createGeometry();
    osg::ref_ptr<osg::Geode> getGeode(){return wireframe;}


  void setPosition();
  void getPosition();

  void scaleHight();
  void scaleWidth();
  void scaleLength();

};


