#pragma once

#include <PluginUtil/coVR3DTransRotInteractor.h>
#include <PluginUtil/coVR3DTransInteractor.h>
#include <PluginUtil/coVR3DRotCenterInteractor.h>
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
using namespace osg;
osg::Vec3 calcDirectionVec(osg::Matrix& m); //returns direction Vector of coVR3DTransRotInteractor

class Point;
class SafetyZone
{   /* Coordinate System is at 6
     * width = 7-6
     * length =5-6
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
    void changeInteractorStatus(bool status);
    void pointsVisibleForEnoughCameras(const std::vector<double>& update)const;
   // osg::Vec3 getPosition(){return mat.getTrans();}
    int getPriority(){return priority;}
    void setPosition( osg::Matrix matrix1)
    {
        interactor->updateTransform(matrix1);
        localDCS->setMatrix(matrix1);
        updateWorldPosOfAllObservationPoints();
    }
    osg::Matrix getMatrix(){return localDCS.get()->getMatrix();}
    std::vector<osg::Vec3>& getWorldPosOfAllObservationPoints(){return worldPosOfAllObservationPoints;}
    osg::Vec3 &getPreferredDirection(){return preferredDirection;}

    std::string &getName(){return name;}

    void preFrame();
    osg::ref_ptr<osg::MatrixTransform> localDCS;
    osg::ref_ptr<osg::MatrixTransform> getSZ()const{return localDCS.get();}
   // osg::ref_ptr<osg::Geode> getSZ()const{return safetyZoneGeode.get();}

private:
    float length = 2;
    float width = 2;
    float height = 8;
    const double distance =4.0; //distance between Points
    const int priority;
    osg::Vec3 preferredDirection;//Direction from which SZ should be observed (x,y,z)
    std::string name;
    std::vector<osg::Vec3> worldPosOfAllObservationPoints;

    osg::Vec4 color;
    osg::ref_ptr<osg::Geode> safetyZoneGeode;
    osg::ref_ptr<osg::Vec3Array> verts;
    std::vector<std::vector<std::unique_ptr<Point>>> points; //vector of control points


    coVR3DTransRotInteractor *interactor;
    coVR3DTransInteractor *sizeYInteractor;
    coVR3DTransInteractor *sizeXInteractor;
    coVR3DTransRotInteractor *preferredDirectionInteractor;
    coVR3DTransRotInteractor *preferredDirectionInteractor2;//2. Interactor which shows in opposite direction


    osg::Geode* plotSafetyZone();
    void updateGeometryY(double tmp);
    void updateGeometryX(double tmp);
    void createPoints();
    void deletePoints();
    void updateWorldPosOfAllObservationPoints();
    void setStateSet(osg::StateSet *stateSet);

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
