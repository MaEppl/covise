/* This file is part of COVISE.
 *

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#pragma once

#include <unistd.h>

#include<cover/coVRPluginSupport.h>
#include <PluginUtil/coVR3DTransRotInteractor.h>
#include <OpenVRUI/coTrackerButtonInteraction.h>
#include <PluginUtil/coSensor.h>


#include <OpenVRUI/osg/mathUtils.h>

#include <climits>



#include<osg/ShapeDrawable>
#include<osg/Vec4>
#include<osg/NodeCallback>
#include<osg/PositionAttitudeTransform>
#include<osg/Material>
#include<osg/MatrixTransform>
#include<osg/Quat>
#include<osg/BlendFunc>
#include<osgText/Font>
#include<osgText/Text>
#include<osg/ShadeModel>
#include <osg/LightModel>
#include<osgFX/Scribe>

#include<Sensor.h>

using namespace opencover;

class Cam
{   
public:
    static double imgWidth;
    static double imgHeight;
    static double imgWidthPixel;
    static double imgHeightPixel;
    static double fov;
    static double depthView;
    static double focalLengthPixel;

    Cam(coCoord m,const std::string name);
    ~Cam();
    std::vector<double> getVisMat(){return visMat;}


    osg::Vec2 rot; // [0]=alpha =zRot, [1]=beta =yRot
    osg::Vec3 pos;

    //check if points are visible for this camera
    void calcVisMat();
    std::vector<double> visMat;
    std::string getName()const{return name;}
    void setPosition(coCoord& m);
protected:
    const std::string name;
private:
    //osg::Box *mySphere; // for visualization
    // Calculates if Obstacles are in line of sigth betwenn camera and observation Point
    bool calcIntersection(const osg::Vec3d& end);
    // sensor will gather the most relevant data only at a particular distance with gradually fading efficiency on either side of it
    double calcRangeDistortionFactor(const osg::Vec3d& point);


};
class mySensor;
class CamPosition;
class CamDrawable
{
private:
    osg::ref_ptr<osg::Vec3Array> verts;
    osg::ref_ptr<osg::Vec4Array> colors;
    osg::ref_ptr<osg::Geode> camGeode;
   // osg::ref_ptr<osg::Geode> interactorGeode;


  /* for an additional Sensor
    osg::Sphere *mySphere;
    mySensor *aSensor;
    vrui::coTrackerButtonInteraction *myinteraction;
    coSensorList sensorList;
    */
    void setStateSet(osg::StateSet *stateSet);


public:
    static size_t count;
    std::unique_ptr<Cam> cam;
    osg::Geode* plotCam();
    CamDrawable(coCoord& m);
    ~CamDrawable();

    osg::ref_ptr<osg::Geode> getCamGeode()const{return camGeode;}

    void preFrame();
    void updateFOV(float value);
    void updateVisibility(float value);
    void updateColor();
    void resetColor();
    void activate(){camGeode->setNodeMask(UINT_MAX);}
    void disactivate(){camGeode->setNodeMask(0);}
};

class Pump;
class CamPosition
{
public:
    static size_t counter;
    bool searchSpaceState;
    bool isFinalCamPos;
    CamPosition(osg::Matrix m,Pump *pump);
    CamPosition(osg::Matrix m);

    ~CamPosition();

    osg::ref_ptr<osg::MatrixTransform> getCamGeode()const{return localDCS;}

    osg::Vec3 getPosition(){
        return localDCS.get()->getMatrix().getTrans();}

    osg::Matrix getMatrix(){return viewpointInteractor->getMatrix();}
    std::string getName(){return name;}
    void setPosition( osg::Matrix matrix1)
    {
        viewpointInteractor->updateTransform(matrix1);
        localDCS->setMatrix(matrix1);
        osg::Vec3 poslocal= viewpointInteractor->getMatrix().getTrans();
        std::cout<<"Camera in World: "<<name<<poslocal.x()<<"|"<<poslocal.y()<<"|"<<poslocal.z()<<std::endl;
        updateCamMatrixes();
    }

    void preFrame();
    void createCamsInSearchSpace();
    void setSearchSpaceState(bool state);
    void updateCamMatrixes();

    std::vector<std::unique_ptr<Cam>> allCameras;
    std::unique_ptr<CamDrawable> camDraw;

private:
    std::string name;
    Pump* myPump = nullptr;


    coVR3DTransRotInteractor *viewpointInteractor;
    osg::ref_ptr<osg::MatrixTransform> localDCS;
    osg::ref_ptr<osg::Group> searchSpaceGroup;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> searchSpace;






};
