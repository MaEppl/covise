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
class Point;
class SafetyZone
{   /*
     *
     *    4*    *7
     *
     * 5*----*6
     *  |0*  |  *3
     *  |    |
     * 1*----*2
     */
    friend class mySensor;
public:

    osg::Box *zone;

    enum Priority{
        PRIO2 = 1, // must be observed with at least 1 camera
        PRIO1 = 2, //must be observed with at least 2 cameras
    };

    SafetyZone(osg::Matrix m, Priority priority, float length, float width, float height);
    ~SafetyZone();
    static size_t count;

    osg::ref_ptr<osg::Geode> getSafetyZoneDrawable()const{return safetyZoneGeode;}
    void updateColor(const std::vector<double> &update)const;
    void resetColor(const std::vector<double> &reset)const;
    void pointsVisibleForEnoughCameras(const std::vector<double>& update)const;
    osg::Vec3 getPosition(){return mat.getTrans();}
    int getPriority(){return priority;}
    void setPosition( osg::Matrix matrix1)
    {
        interactor->updateTransform(matrix1);
        localDCS->setMatrix(matrix1);
        updateWorldPosOfAllObservationPoints();
    }
    osg::Matrix getMatrix(){return localDCS.get()->getMatrix();}
    std::vector<osg::Vec3>& getWorldPosOfAllObservationPoints(){return worldPosOfAllObservationPoints;}
    std::string &getName(){return name;}

    void preFrame();
    osg::ref_ptr<osg::MatrixTransform> localDCS;
    osg::ref_ptr<osg::MatrixTransform> getSZ()const{return localDCS.get();}
   // osg::ref_ptr<osg::Geode> getSZ()const{return safetyZoneGeode.get();}

private:
    float length = 2;//8
    float width = 2;//2
    float height = 8;//2ss
    const double distance =5.0;
    const int priority;
    std::string name;
    std::vector<osg::Vec3> worldPosOfAllObservationPoints;
    void updateWorldPosOfAllObservationPoints();

    osg::Vec4 color;
    osg::Matrix mat;
    osg::ref_ptr<osg::Geode> safetyZoneGeode;
    void setStateSet(osg::StateSet *stateSet);

    coVR3DTransRotInteractor *interactor;
    coVR3DTransInteractor *sizeYInteractor;
    coVR3DTransInteractor *sizeXInteractor;
    osg::ref_ptr<osg::Vec3Array> verts;
    void updateGeometryY(osg::Vec3 tmp);
    void updateGeometryX(osg::Vec3 tmp);
    osg::Geode* addPoint(osg::Vec3 pos);

    osg::Vec3 distanceInteractors;
    double distanceY;
    double distanceX;
    //std::vector<std::unique_ptr<Point>> pointsVerts;
    std::vector<std::vector<std::unique_ptr<Point>>> points;

    osg::Geode* plotSafetyZone(osg::Matrix m);
    void createPoints();
 /* //user Interaction
    mySensor *aSensor;
    vrui::coTrackerButtonInteraction *myinteraction;
    bool interActing;
    coSensorList sensorList;
  */
};
class Point
{
public:
   Point(osg::Vec3 pos,osg::Vec4 color);
   ~Point();
   void setPos(osg::Vec3 p);
   osg::Vec3 getPos(){return localDCS->getMatrix().getTrans();}
   void updateColor();
   void resetColor();
   void visible(bool status);
   osg::ref_ptr<osg::MatrixTransform> getPoint()const{return localDCS.get();}
   osg::ref_ptr<osg::MatrixTransform> localDCS;
private:
   bool visibleForEnoughCameras;
   osg::ref_ptr<osg::Geode> geode;
   osg::ref_ptr<osg::Sphere> sphere;
   osg::ref_ptr<osg::ShapeDrawable> sphereDrawable;
   osg::Vec4 color;

   void setStateSet(osg::StateSet *stateSet);

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



