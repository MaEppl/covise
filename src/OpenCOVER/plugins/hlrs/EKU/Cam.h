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
#include<osg/Switch>

#include<Sensor.h>
#include<SafetyZone.h>
using namespace opencover;
void printCoCoord(coCoord m);
class Cam
{   
public:
    static size_t count;

    static double imgWidth;
    static double imgHeight;
    static double imgWidthPixel;
    static double imgHeightPixel;
    static double fov;
    static double depthView;
    static double focalLengthPixel;
    static double rangeDistortionDepth;
    Cam(coCoord m,std::vector<std::vector<double>> visMat,const std::string name);
    ~Cam();
    std::vector<std::vector<double>> getVisMat(){return visMat;}
    std::vector<int> getVisMatPrio1(){return visMatPrio1;}
    std::vector<int> getVisMatPrio2(){return visMatPrio2;}
    const int id=count;
    osg::Vec2 rot; // [0]=alpha =zRot, [1]=beta =yRot
    osg::Vec3 pos;
    osg::Vec3 directionVec; //direction Vector
    osg::Matrix mat;
    //check if points are visible for this camera
    void calcVisMat();
    std::vector<std::vector<double>> visMat; //outer Vector is for SafetyZones, inner for dots in each SafetyZone
    std::vector<int> visMatPrio1;
    std::vector<int> visMatPrio2;
    std::vector<double> distortionValuePrio1;
    std::vector<double> distortionValuePrio2;

    std::string getName()const{return name;}
    void setPosition(coCoord& m, std::vector<std::vector<double>> visMatInput);
    osg::Matrix getMatrix(){
        coCoord euler = mat;
        //std::cout<<"Cam "<<count<<" :"<<euler.hpr[0]<<","<<euler.hpr[1]<<","<<euler.hpr[2]<<std::endl;
        return mat;}
    int getID(){return id;}
   // void preFrame();
protected:
     std::string name;
private:
    //coVR3DTransRotInteractor *viewpointInteractor; // for Degugging: Visualization
    osg::Box *mySphere; // for visualization
    // Calculates if Obstacles are in line of sigth betwenn camera and observation Point
    bool calcIntersection(const osg::Vec3d& end);
    // sensor will gather the most relevant data only at a particular distance with gradually fading efficiency on either side of it
    double calcRangeDistortionFactor(const osg::Vec3d& point) const;
    double calcPreferredDirectionFactor(osg::Vec3 directionOfPoint);


};
class EquipmentWithCamera;
class mySensor;
class CamPosition;
class CamDrawable
{
private:
    osg::ref_ptr<osg::Vec3Array> verts;
    osg::ref_ptr<osg::Vec4Array> colors;
    osg::ref_ptr<osg::Geode> camGeode;

 /*  osg::ref_ptr<osg::Vec3Array> vertsSRC;
    osg::ref_ptr<osg::Vec4Array> colorsSRC;
    osg::ref_ptr<osg::Geode> SRC;
   */ int scale = 15;
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
    osg::Geode* plotCam(bool showLines, osg::Vec4 color);
    CamDrawable(coCoord& m,std::vector<std::vector<double>> visMat,bool showLines,osg::Vec4 color);
    ~CamDrawable();
    bool _showRealSize=false;

    osg::ref_ptr<osg::Geode> getCamGeode()const{return camGeode;}

    void updateFOV(float value);
    void updateVisibility(float value);
    void showRealSize();
    void resetSize();
    void activate(){camGeode->setNodeMask(UINT_MAX);
                    camGeode->setNodeMask(camGeode->getNodeMask() & (~Isect::Intersection) & (~Isect::Pick));
                   }
    void disactivate(){camGeode->setNodeMask(0);}
};
class EquipmentWithCam;
//class Pump;
class CamPosition
{
public:
    static size_t counter;
    bool searchSpaceState;
    CamPosition(osg::Matrix m);
    CamPosition(osg::Matrix m,EquipmentWithCamera *pump);



    ~CamPosition();

    osg::ref_ptr<osg::Switch> getCamGeode()const{return switchNode;}

    osg::Vec3 getPosition(){
        return localDCS.get()->getMatrix().getTrans();}

    osg::Matrix getMatrix(){return viewpointInteractor->getMatrix();}
    osg::Vec3 &getDirectionVec(){return directionVec;}

    std::string getName(){return name;}
    void setPosition( osg::Matrix matrix1)
    {
        viewpointInteractor->updateTransform(matrix1);
        localDCS->setMatrix(matrix1);
      //  updateVisibleCam();
    }

    void preFrame();
    void createCamsInSearchSpace();
    void setSearchSpaceState(bool state);
    void updateCamMatrixes();
    void updateVisibleCam();
    std::vector<std::shared_ptr<Cam>> allCameras;
    std::unique_ptr<CamDrawable> camDraw;
    std::unique_ptr<CamDrawable> searchSpaceDrawable;
    std::unique_ptr<CamDrawable> deletedOrientationsDrawable;


    void activate();
    void disactivate();

    void calcIntersection();
    int calcVisibility(osg::Matrix cam,osg::Vec3 point);

  //  Cam& CamPosition::compareCams(Cam &camA ,Cam &camB);
private:
    bool status;
    std::string name;

    std::vector<std::vector<double>> visMat;

   // std::vector<std::pair<std::shared_ptr<SafetyZone,std::vector<int>>> visMatPerSafetyZone;

    coVR3DTransRotInteractor *viewpointInteractor;
    osg::ref_ptr<osg::Switch> switchNode;
    osg::ref_ptr<osg::MatrixTransform> localDCS;
    osg::ref_ptr<osg::Group> searchSpaceGroup;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> searchSpace;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> deletedOrientations;

    osg::Vec3 directionVec; // Direction Vector of camera (direction of arrow)

    std::shared_ptr<Cam> createCamFromMatrix(coCoord& euler);
    bool isVisibilityMatrixEmpty(const std::shared_ptr<Cam> &cam);
    void compareCamsNew(std::shared_ptr<Cam> newCam, std::shared_ptr<Cam> oldCam);
    void addCamToVec(std::shared_ptr<Cam> cam);
    void removeCamFromVec(std::shared_ptr<Cam> cam);
    void createDrawableForEachCamOrientation();
    void createDrawableForEachDeletedCamOrientation();





};
