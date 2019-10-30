 /* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#include <iostream>
#include <math.h>

#include <Cam.h>
#include <EKU.h>
using namespace opencover;

double Cam::imgHeightPixel = 1080;
double Cam::imgWidthPixel = 1920;
double Cam::fov = 60;
double Cam::depthView = 40;
double Cam::focalLengthPixel = Cam::imgWidthPixel*0.5/(std::tan(Cam::fov*0.5*M_PI/180));
double Cam::imgWidth = 2*depthView*std::tan(Cam::fov/2*osg::PI/180);
double Cam::imgHeight = Cam::imgWidth/(Cam::imgWidthPixel/Cam::imgHeightPixel);


Cam::Cam(const osg::Vec3 pos, const osg::Vec2 rot, const std::vector<osg::Vec3> &observationPoints,const std::string name):pos(pos),rot(rot),name(name)
{

    calcVisMat(observationPoints);

}

Cam::Cam(const osg::Vec3 pos, const osg::Vec2 rot, const std::string name):pos(pos),rot(rot),name(name)
{

}

Cam::~Cam()
{

}


void Cam::calcVisMat(const std::vector<osg::Vec3> &observationPoints)
{
     visMat.clear();

    osg::Matrix T = osg::Matrix::translate(-pos);
    osg::Matrix zRot = osg::Matrix::rotate(-rot.x(), osg::Z_AXIS);
    osg::Matrix yRot = osg::Matrix::rotate(-rot.y(), osg::Y_AXIS);
    // BUGFIX: still problem at borders?

    size_t cnt =1;
    std::cout<<name<<": ";
    for(const auto& p : observationPoints)
    {


        auto newPoint = p*T*zRot*yRot;
        if((newPoint.x()<=Cam::depthView ) && (newPoint.x()>=0) &&
           (std::abs(newPoint.y()) <= Cam::imgWidth/2 * newPoint.x()/Cam::depthView) &&
           (std::abs(newPoint.z())<=Cam::imgHeight/2 * newPoint.x()/Cam::depthView))
        {
            if(calcIntersection(p)==false)
                visMat.push_back(1);//*calcRangeDistortionFactor(newPoint));//*calcRangeDistortionFactor(newPoint));
            else
                visMat.push_back(0);
        }
        else
            visMat.push_back(0);


        std::cout <<"P"<<cnt<<": "<<visMat.back()<<" ";
        cnt++;
    }
    std::cout<<" "<<std::endl;
}
bool Cam::calcIntersection(const osg::Vec3d& end)
{
    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(pos,end);
    intersector->setIntersectionLimit(osgUtil::Intersector::IntersectionLimit::LIMIT_ONE_PER_DRAWABLE);
    osgUtil::IntersectionVisitor visitor(intersector);
   // EKU::plugin->finalScene->accept(visitor);// NOTE: how to do this rigth ? wihout acces to class EKU?
    cover->getObjectsRoot()->accept(visitor);
    const osgUtil::LineSegmentIntersector::Intersections hits = intersector->getIntersections();
    std::cout<<"Intersect: "<<hits.size()<<" with ";
    size_t numberOfNonRelevantObstacles = 0;
    for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr =hits.begin(); hitr!=hits.end();++hitr)
    {
        std::string name = hitr->nodePath.back()->getName();

        // Nodes with cx and px are safety areas or possible camera positions, these are no real obstacles
        if((name.find("Cam") != std::string::npos) || (name.find("SafetyZone") != std::string::npos) || (name.find("Pyramid") != std::string::npos))
            ++numberOfNonRelevantObstacles;
        std::cout<<hitr->nodePath.back()->getName()<<" & ";
    }
    if(hits.size()-numberOfNonRelevantObstacles>0)
        return true;
    else
        return false;
}

double Cam::calcRangeDistortionFactor(const osg::Vec3d &point)
{
    double x = point.x(); //distance between point and sensor in x direction
    double sigma = 14; //

    //SRC = Sensor Range Coefficient
    //normalized Rayleigh distribution function
    double SRC = sigma*exp(0.5) * (x / pow(sigma,2)) * exp(-(pow(x,2)) / (2*pow(sigma,2)));
    return 1/SRC;
}

size_t CamDrawable::count=0;

CamDrawable::CamDrawable(Cam* cam):cam(cam)
{
    count++;
    fprintf(stderr, "new CamDrawable from Point\n");
    group = new osg::Group;
    group->setName("Cam"+std::to_string(CamDrawable::count));

    text = new osgText::Text;
   // text->setName("Text");
   // text->setText("Cam"+std::to_string(CamDrawable::count));
    //text->setColor()
  //  text->setCharacterSize(17);

    camGeode = plotCam();

    //Translation
    transMat= new osg::MatrixTransform();
    transMat->setName("Translation");
    //group->addChild(transMat);
    osg::Matrix m;
    m.setTrans(cam->pos.x(),cam->pos.y(),cam->pos.z());
    transMat->setMatrix(m);

    //Rotation
    rotMat = new osg::MatrixTransform();
    rotMat ->setName("Rotation");
    //group->addChild(rotMat);
    osg::Matrix r;
    osg::Quat yRot, zRot;
    zRot.makeRotate((float)cam->rot.x(), osg::Z_AXIS);
    yRot.makeRotate((float)cam->rot.y(), osg::Y_AXIS);
    osg::Quat fullRot = yRot*zRot; //NOTE: Be careful, changed order of Matrix Multiplication here
    r.setRotate(fullRot);
    rotMat->setMatrix(r);
    //OpenGL first rotate than translate
    //camGeode->addChild(text.get()); //in older OSG versions osg::Geode doesn't have function addChild
    rotMat->addChild(camGeode.get());
    transMat->addChild(rotMat.get());
    group->addChild(transMat.get());

}



CamDrawable::~CamDrawable()
{
    std::cout<<"CamDrawable Destructor called"<<std::endl;
    count--;
  //  delete cam;
    cover->getObjectsRoot()->removeChild(group.get());
//    cover->getObjectsRoot()->removeChild(transMat.get());


}

osg::Geode* CamDrawable::plotCam()
{
    // The Drawable geometry is held under Geode objects.
    osg::Geode* geode = new osg::Geode();
    geode->setName("Pyramid");
    osg::Geometry* geom = new osg::Geometry();
    osg::StateSet *stateset = geode->getOrCreateStateSet();
    //necessary for dynamic redraw (command:dirty)
    geom->setDataVariance(osg::Object::DataVariance::DYNAMIC) ;
    geom->setUseDisplayList(false);
    geom->setUseVertexBufferObjects(true);
    //stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //stateset->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
    // Associate the Geometry with the Geode.
    geode->addDrawable(geom);
    geode->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    geode->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    // Declare an array of vertices to create a simple pyramid.
    verts = new osg::Vec3Array;
    verts->push_back( osg::Vec3(Cam::depthView, -Cam::imgWidth/2, Cam::imgHeight/2 ) ); // 0 upper  front base
    verts->push_back( osg::Vec3(Cam::depthView, -Cam::imgWidth/2,-Cam::imgHeight/2 ) ); // 1 lower front base
    verts->push_back( osg::Vec3(Cam::depthView,  Cam::imgWidth/2,-Cam::imgHeight/2 ) ); // 3 lower  back  base
    verts->push_back( osg::Vec3(Cam::depthView,  Cam::imgWidth/2, Cam::imgHeight/2 ) ); // 2 upper back  base
    verts->push_back( osg::Vec3( 0,  0,  0) ); // 4 peak


    // Associate this set of vertices with the Geometry.
    geom->setVertexArray(verts);

    // Next, create primitive sets and add them to the Geometry.
    // Each primitive set represents a face or a Line of the Pyramid
    // 0 base
    osg::DrawElementsUInt* face =
       new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    face->push_back(3);
    face->push_back(2);
    face->push_back(1);
    face->push_back(0);
    geom->addPrimitiveSet(face);

    osg::DrawElementsUInt* line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(4);
    line->push_back(3);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(4);
    line->push_back(2);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(4);
    line->push_back(1);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(4);
    line->push_back(0);
    geom->addPrimitiveSet(line);

    osg::Vec3Array* normals = new osg::Vec3Array();
    normals->push_back(osg::Vec3(-1.f ,-1.f, 0.f)); //left front
    normals->push_back(osg::Vec3(1.f ,-1.f, 0.f)); //right front
    normals->push_back(osg::Vec3(1.f ,1.f, 0.f));//right back
    normals->push_back(osg::Vec3(-1.f ,1.f, 0.f));//left back
    normals->push_back(osg::Vec3(0.f ,0.f, 1.f));//peak
    geom->setNormalArray(normals);
   // geom->setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
     geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    //create Material
    osg::Material *material = new osg::Material;
    material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0f, 0.2f, 0.2f, 1.0f));
    material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.1f, 0.1f, 0.1f, 1.0f));
    material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0, 1.0, 1.0, 1.0));
    material->setShininess(osg::Material::FRONT_AND_BACK, 25.0);
    material->setTransparency(osg::Material::FRONT_AND_BACK,0.2);
    material->setAlpha(osg::Material::FRONT_AND_BACK,0.2);
    stateset->setAttributeAndModes(material);
    stateset->setNestRenderBins(false);

    // Create a separate color for each face.
    colors = new osg::Vec4Array; //magenta 1 1 0; cyan 0 1 1; black 0 0 0
    colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 0.5f) ); // magenta - back
    colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ); // magenta - back
    colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ); // magenta - back
    colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ); // magenta - back
    colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ); // yellow  - base
    // Assign the color indices created above to the geometry and set the
    // binding mode to _PER_PRIMITIVE_SET.
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    //geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    //LineWidth
    LineWidth *lw = new LineWidth(2.0);
    stateset->setAttribute(lw);
    // return the geode as the root of this geometry.
    return geode;
}

void CamDrawable::updateFOV(float value)
{
    cam->fov = value;
    cam->imgWidth = 2*cam->depthView*std::tan(cam->fov/2*osg::PI/180);
    cam->imgHeight = cam->imgWidth/(cam->imgWidthPixel/cam->imgHeightPixel);
    verts->resize(0);
    verts->push_back( osg::Vec3(cam->depthView, -cam->imgWidth/2, cam->imgHeight/2 ) ); // 0 upper  front base
    verts->push_back( osg::Vec3(cam->depthView, -cam->imgWidth/2,-cam->imgHeight/2 ) ); // 1 lower front base
    verts->push_back( osg::Vec3(cam->depthView,  cam->imgWidth/2,-cam->imgHeight/2 ) ); // 3 lower  back  base
    verts->push_back( osg::Vec3(cam->depthView,  cam->imgWidth/2, cam->imgHeight/2 ) ); // 2 upper back  base
    verts->push_back( osg::Vec3( 0,  0,  0) ); // 4 peak
    verts->dirty();

}

void CamDrawable::updateVisibility(float value)
{
    cam->depthView = value;
    cam->imgWidth = 2*cam->depthView*std::tan(cam->fov/2*osg::PI/180);
    cam->imgHeight = cam->imgWidth/(cam->imgWidthPixel/cam->imgHeightPixel);
    verts->resize(0);
    verts->push_back( osg::Vec3(cam->depthView, -cam->imgWidth/2, cam->imgHeight/2 ) ); // 0 upper  front base
    verts->push_back( osg::Vec3(cam->depthView, -cam->imgWidth/2,-cam->imgHeight/2 ) ); // 1 lower front base
    verts->push_back( osg::Vec3(cam->depthView,  cam->imgWidth/2,-cam->imgHeight/2 ) ); // 3 lower  back  base
    verts->push_back( osg::Vec3(cam->depthView,  cam->imgWidth/2, cam->imgHeight/2 ) ); // 2 upper back  base
    verts->push_back( osg::Vec3( 0,  0,  0) ); // 4 peak
    verts->dirty();
}

void CamDrawable::updateColor()
{

    colors->resize(0);
    colors->push_back( osg::Vec4(1.0f, 0.0f, 1.0f, 0.5f) );
    colors->push_back( osg::Vec4(1.0f, 0.0f, 1.0f, 0.5f) );
    colors->push_back( osg::Vec4(1.0f, 0.0f, 1.0f, 0.5f) );
    colors->push_back( osg::Vec4(1.0f, 0.0f, 1.0f, 0.5f) );
    colors->push_back( osg::Vec4(1.0f, 0.0f, 1.0f, 0.5f) );
    colors->dirty();
}

void CamDrawable::resetColor()
{

    colors->resize(0);
    colors->push_back( osg::Vec4(0.0f, 1.0f, 1.0f, 0.5f) ); // yellow  - base
    colors->push_back( osg::Vec4(0.0f, 1.0f, 1.0f, 0.5f) ); // cyan    - left
    colors->push_back( osg::Vec4(0.0f, 1.0f, 1.0f, 0.5f) ); // cyan    - right
    colors->push_back( osg::Vec4(0.0f, 1.0f, 1.0f, 0.5f) ); // magenta - front
    colors->push_back( osg::Vec4(0.0f, 1.0f, 1.0f, 0.5f) ); // magenta - back
    colors->dirty();
}

size_t CamPosition::counter =0;
CamPosition::CamPosition(osg::Vec3 pos):worldPosition(pos)
{
    counter ++;
    //Creating an osg::Sphere
  //  mySphere = new osg::Sphere(position, 3.);
    sphere = new osg::Sphere(worldPosition,0.15);
    shapDr = new osg::ShapeDrawable(sphere);
    shapDr->setColor(osg::Vec4(0., 1., 0., 1.0f));
    geode = new osg::Geode();
    osg::StateSet *mystateSet = geode->getOrCreateStateSet();
    setStateSet(mystateSet);
    geode->setName("Cam "+std::to_string(CamPosition::counter));
    geode->addDrawable(shapDr);

  /*  osg::Matrix localMat;
    localMat.setTrans(pos);
    //create Interactors
    float _interSize = cover->getSceneSize() / 25;
    viewpointInteractor = new coVR3DTransRotInteractor(localMat, _interSize/2, vrui::coInteraction::ButtonA, "hand", "CamInteractor", vrui::coInteraction::Medium);
    viewpointInteractor->show();
    viewpointInteractor->enableIntersection();

    */
}
void CamPosition::preFrame()
{
    //viewpointInteractor->preFrame();
}
void CamPosition::updatePosInWorld()
{
    worldPosition = geode->getBound().center() * osg::computeLocalToWorld(geode->getParentalNodePaths()[0])/1000;
    std::cout<<"Camera in World: "<<name<<worldPosition.x()<<"|"<<worldPosition.y()<<"|"<<worldPosition.z()<<std::endl;

}
void CamPosition::setStateSet(osg::StateSet *stateSet)
{
    osg::Material *material = new osg::Material();
    material->setColorMode(osg::Material::DIFFUSE);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0f, 0.0f, 0.0f, 0.5f));
    osg::LightModel *defaultLm;
    defaultLm = new osg::LightModel();
    defaultLm->setLocalViewer(true);
    defaultLm->setTwoSided(true);
    defaultLm->setColorControl(osg::LightModel::SINGLE_COLOR);
    stateSet->setAttributeAndModes(material, osg::StateAttribute::ON);
    stateSet->setAttributeAndModes(defaultLm, osg::StateAttribute::ON);
}
