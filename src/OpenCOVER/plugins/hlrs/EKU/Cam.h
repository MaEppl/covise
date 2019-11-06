/* This file is part of COVISE.
 *

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#pragma once

#include <unistd.h>

#include<cover/coVRPluginSupport.h>
#include <PluginUtil/coVR3DTransRotInteractor.h>
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

    Cam(const osg::Vec3 pos, const osg::Vec2 rot, const std::vector<osg::Vec3> &observationPoints, const std::string name);
    Cam(const osg::Vec3 pos, const osg::Vec2 rot,const std::string name);
    Cam(coCoord m,const std::string name);
    ~Cam();
    std::vector<double> getVisMat(){return visMat;}


    const osg::Vec2 rot; // [0]=alpha =zRot, [1]=beta =yRot
    const osg::Vec3 pos;

    //const osg::Vec3Array* obsPoints =nullptr; // NOTE: remove later

    //check if points are visible for this camera
    void calcVisMat(const std::vector<osg::Vec3> &observationPoints);
    std::vector<double> visMat;
    std::string getName()const{return name;}
protected:
    const std::string name;
private:

    // Calculates if Obstacles are in line of sigth betwenn camera and observation Point
    bool calcIntersection(const osg::Vec3d& end);
    // sensor will gather the most relevant data only at a particular distance with gradually fading efficiency on either side of it
    double calcRangeDistortionFactor(const osg::Vec3d& point);


};
class CamPosition;
class CamDrawable
{
private:
    osg::ref_ptr<osg::Vec3Array> verts;
    osg::ref_ptr<osg::Vec4Array> colors;
    osg::ref_ptr<osg::Geode> camGeode;


public:
    static size_t count;
    Cam* cam=nullptr;
    osg::Geode* plotCam();
    void updateFOV(float value);
    void updateVisibility(float value);
    void updateColor();
    void resetColor();
    void activate(){camGeode->setNodeMask(UINT_MAX);}
    void disactivate(){camGeode->setNodeMask(0);}
    CamDrawable(Cam* cam);
    CamDrawable();
    ~CamDrawable();

    osg::ref_ptr<osg::Geode> getCamGeode()const{return camGeode;}
};


class CamPosition
{
public:
    static size_t counter;
    bool searchSpaceState;
    bool isFinalCamPos;
    CamPosition(osg::Matrix m);
    ~CamPosition(){}

    osg::ref_ptr<osg::MatrixTransform> getCamGeode()const{return localDCS;}

    osg::Vec3 getPosition(){
        return localDCS.get()->getMatrix().getTrans();}

    osg::Matrix getMatrix(){return viewpointInteractor->getMatrix();}

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

    std::vector<Cam*> allCameras;

private:
    std::string name;

    CamDrawable* camDraw;

    coVR3DTransRotInteractor *viewpointInteractor;
    osg::ref_ptr<osg::MatrixTransform> localDCS;
    osg::ref_ptr<osg::Group> searchSpaceGroup;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> searchSpace;






};
