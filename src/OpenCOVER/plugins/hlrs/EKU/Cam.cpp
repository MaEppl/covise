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

    camGeode = plotCam();
    camGeode->setName("CamDrawable"+std::to_string(CamDrawable::count));

}

CamDrawable::CamDrawable()
{
    count++;
    fprintf(stderr, "new CamDrawable from Point\n");

    camGeode = plotCam();
    camGeode->setName("CamDrawable"+std::to_string(CamDrawable::count));

}


CamDrawable::~CamDrawable()
{
    std::cout<<"CamDrawable Destructor called"<<std::endl;
    count--;
    cover->getObjectsRoot()->removeChild(camGeode.get());


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
    verts->push_back( osg::Vec3( -Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2 ) ); // 0 upper  front base
    verts->push_back( osg::Vec3( -Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2 ) ); // 1 lower front base
    verts->push_back( osg::Vec3(  Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2 ) ); // 3 lower  back  base
    verts->push_back( osg::Vec3(  Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2 ) ); // 2 upper back  base
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
CamPosition::CamPosition(osg::Matrix m)
{
    counter ++;
    searchSpaceState = false;

    camDraw = new CamDrawable();


    localDCS = new osg::MatrixTransform();
    localDCS->setName("CamInteractorMatrix");
    localDCS->setMatrix(m);


    //create Interactors
    float _interSize = cover->getSceneSize() / 25;
    viewpointInteractor = new coVR3DTransRotInteractor(m, _interSize/2, vrui::coInteraction::ButtonA, "hand", "CamInteractor", vrui::coInteraction::Medium);
    viewpointInteractor->show();
    viewpointInteractor->enableIntersection();

    localDCS->addChild(camDraw->getCamGeode());

    searchSpaceGroup = new osg::Group;
    searchSpaceGroup->setName("SearchSpace");
    localDCS->addChild(searchSpaceGroup.get());
    
}
void CamPosition::preFrame()
{
    viewpointInteractor->preFrame();
    if(viewpointInteractor->isRunning())
    {
        osg::Matrix local = viewpointInteractor->getMatrix();
        coCoord localEuler = local;
        //restrict rotation around y
    //    localEuler.hpr[2]=0.0;
        //cameras are always looking downwards -> no neg rotation around x
    /*    if(localEuler.hpr[1] < 0.0)
            localEuler.hpr[1] = 0.0;
    */    localEuler.makeMat(local);
        localDCS->setMatrix(local);

        std::cout<<"Rotation(around global axes): "<<"z:"<<localEuler.hpr[0]<< " x:"<<localEuler.hpr[1]<<" y:"<<localEuler.hpr[2]<<std::endl;

    }
}
void CamPosition::createCamsInSearchSpace()
{
    searchSpaceState = true;
    //around z axis
    int zMax = 40;
    int stepSizeZ = 10; //in Degree

    int xMax = 20;
    int stepSizeX = 10; //in Degree

    osg::Matrix m = localDCS.get()->getMatrix();
    coCoord coord=m;
    coCoord newCoordPlus,newCoordMinus;

    std::cout<<"StartPos: "<<"z:"<<coord.hpr[0]<< " x:"<<coord.hpr[1]<<" y:"<<coord.hpr[2]<<std::endl;

    osg::Matrix m_new;

    int count =0;

    for(int cnt = 0; cnt<zMax/stepSizeZ; cnt++)
    {

        if(count == 0)
        {
            newCoordPlus.hpr[0]=0;
            newCoordMinus.hpr[0]-= stepSizeZ;


        }else
        {
            newCoordPlus.hpr[0] += stepSizeZ;
            newCoordMinus.hpr[0] -= stepSizeZ;
        }

        newCoordPlus.hpr[1]=newCoordMinus.hpr[1]=0;
        count ++;
        int countX =0;
        for(int cnt2 = 0; cnt2<xMax/stepSizeX; cnt2++)
        {
            if(countX==0)
                 newCoordPlus.hpr[1]=0;
            else
                newCoordPlus.hpr[1] += stepSizeX;

            newCoordPlus.makeMat(m_new);
            searchSpace.push_back(new osg::MatrixTransform );
            searchSpaceGroup->addChild(searchSpace.back().get());
            searchSpace.back()->setMatrix(m_new);
            searchSpace.back()->setName("+MATRIX Z:" + std::to_string( newCoordPlus.hpr[0])+ " X:" +std::to_string( newCoordPlus.hpr[1]));
            searchSpace.back()->addChild(camDraw->getCamGeode().get());
            std::cout<<"Rotation+++++++++++++: "<<"z:"<<newCoordPlus.hpr[0]<< " x:"<<newCoordPlus.hpr[1]<<" y:"<<newCoordPlus.hpr[2]<<std::endl;
           // std::cout<<"Tramslation############: "<<"z:"<<coord.xyz[0]<< " x:"<<coord.xyz[1]<<" y:"<<coord.xyz[2]<<std::endl;


            if(countX==0)
                 newCoordMinus.hpr[1]=0;
            else
                newCoordMinus.hpr[1] += stepSizeX;

            // rot around z in - direction

            newCoordMinus.makeMat(m_new);
            searchSpace.push_back(new osg::MatrixTransform );
            searchSpaceGroup->addChild(searchSpace.back().get());
            searchSpace.back()->setMatrix(m_new);
            searchSpace.back()->setName("-MATRIX Z:" + std::to_string( newCoordMinus.hpr[0])+ " X:" +std::to_string( newCoordMinus.hpr[1]));
            searchSpace.back()->addChild(camDraw->getCamGeode().get());
            std::cout<<"Rotation-------------: "<<"z:"<<newCoordMinus.hpr[0]<< " x:"<<newCoordMinus.hpr[1]<<" y:"<<newCoordMinus.hpr[2]<<std::endl;
            //  std::cout<<"Tramslation############: "<<"z:"<<coord.xyz[0]<< " x:"<<coord.xyz[1]<<" y:"<<coord.xyz[2]<<std::endl;


            countX++;

        }
    }
}

void CamPosition::setSearchSpaceState(bool state)
{
    //for(auto &x : searchSpace)
      //  searchSpaceGroup->removeChild(x);

    if(searchSpaceState == false)
        createCamsInSearchSpace();

    if(state)
        searchSpaceGroup->setNodeMask(UINT_MAX);
    else
        searchSpaceGroup->setNodeMask(0);



}


