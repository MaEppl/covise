/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _BLOOD_PLUGIN_H
#define _BLOOD_PLUGIN_H
/****************************************************************************\ 
 **                                                            (C)2018 HLRS  **
 **                                                                          **
 ** Description: BloodPlugin                                                 **
 **                                                                          **
 **                                                                          **
 ** Author: U.Woessner		                                                 **
 **                                                                          **
 ** History:  								                                 **
 ** August 2018 v1	    				                                     **
 **                                                                          **
 **                                                                          **
\****************************************************************************/
#include <cover/coVRPlugin.h>
#include <cmath>

#include <cover/ui/Menu.h>
#include <cover/ui/Action.h>
#include <cover/ui/Button.h>
#include <cover/ui/Slider.h>
#include <cover/ui/Label.h>
#include <cover/coVRPluginSupport.h>
#include <cover/coVRFileManager.h>

#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/AnimationPath>
#include <osg/PositionAttitudeTransform> 
#include <osg/Material>

#include <Blood.h>

using namespace opencover;

class BloodPlugin : public opencover::coVRPlugin, public ui::Owner
{
public:
    osg::Vec3 startPosition = osg::Vec3(-100, 0, 100);
	
    //member variables for displaying blood
    osg::ref_ptr<osg::Geode> bloodGeode;
	osg::Vec4 bloodColor = osg::Vec4(1.0, 0.0, 0.0, 1.0);
	osg::ref_ptr<osg::MatrixTransform> bloodTransform;
	osg::Matrix bloodBaseTransform;
	osg::ref_ptr<osg::Sphere> bloodSphere;
	osg::ref_ptr<osg::ShapeDrawable> bloodShapeDrawable;
	osg::ref_ptr<osg::StateSet> bloodStateSet;
	osg::ref_ptr<osg::Material> bloodMaterial;
	
    //member variables for displaying knife
    osg::ref_ptr<osg::MatrixTransform> knifeTransform;
    osg::Matrix knifeBaseTransform;

    BloodPlugin();
    ~BloodPlugin(); 
    virtual bool init();
    virtual bool update();
    double isSliding(osg::Vec3 deltaPos);
    void doAddBlood();
    osg::ref_ptr<osg::Group> bloodNode;
    std::list<Blood*> bloodJunks;
    static BloodPlugin *instance();
private:
    static BloodPlugin *inst;
    ui::Menu* bloodMenu = nullptr;
    ui::Action* addBlood = nullptr;
    
    Droplet particle;
    Weapon knife;
    osg::Vec3 particlePosition;
    osg::Vec3 velocity;
    osg::Vec3 acc;
};
#endif
