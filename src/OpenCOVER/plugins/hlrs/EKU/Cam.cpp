 /* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#include <iostream>
#include <math.h>
#include <numeric>

#include <Cam.h>
#include <EKU.h>
using namespace opencover;

void printCoCoord(coCoord m)
{
    std::cout<<"translation: "<<"x:"<<m.xyz[0]<< " y:"<<m.xyz[1]<<" z:"<<m.xyz[2]<<std::endl;
    std::cout<<"rotation: "<<"z:"<<m.hpr[0]<< " x:"<<m.hpr[1]<<" y:"<<m.hpr[2]<<std::endl;
}
double Cam::imgHeightPixel = 1080;
double Cam::imgWidthPixel = 1920;
double Cam::fov = 60;
double Cam::depthView = 70;
double Cam::focalLengthPixel = Cam::imgWidthPixel*0.5/(std::tan(Cam::fov*0.5*M_PI/180));
double Cam::imgWidth = 2*depthView*std::tan(Cam::fov/2*osg::PI/180);
double Cam::imgHeight = Cam::imgWidth/(Cam::imgWidthPixel/Cam::imgHeightPixel);
double Cam::rangeDistortionDepth =22;
size_t Cam::count=0;

Cam::Cam(coCoord matrix,std::string name):pos(matrix.xyz),rot(matrix.hpr[0],matrix.hpr[1]),name(name)
{
 //   std::cout<<"new Cam:" <<id<<std::endl;
    count++;
    matrix.makeMat(mat);
    directionVec = calcDirectionVec(mat);

}
Cam::~Cam()
{

}
void Cam::setPosition(coCoord& m)
{
    pos = m.xyz;
    rot.set(m.hpr[0],m.hpr[1]);
    m.makeMat(mat);
    directionVec = calcDirectionVec(mat);
}


void Cam::calcVisMat()
{   visMatPrio1.clear();
    visMatPrio2.clear();
    visMat.clear();
    distortionValuePrio1.clear();
    distortionValuePrio2.clear();
    visMat.reserve(EKU::safetyZones.size());
    osg::Matrix T = osg::Matrix::translate(-pos);
    osg::Matrix zRot = osg::Matrix::rotate(-osg::DegreesToRadians(rot.x()), osg::Z_AXIS);
    osg::Matrix yRot = osg::Matrix::rotate(-osg::DegreesToRadians(rot.y()), osg::X_AXIS);
    // BUGFIX: still problem at borders?

    size_t cnt =1;
    for(const auto& p : EKU::safetyZones)
    {
        std::vector<double> visMatForThisSafetyZone;
        visMatForThisSafetyZone.reserve(p->getWorldPosOfAllObservationPoints().size());

        //Preferred Direction Coefficient is equal for whole safety zone
        double PDC = calcPreferredDirectionFactor(p->getPreferredDirection());

        for(const auto& p1 :p->getWorldPosOfAllObservationPoints())
        {
            osg::Vec3 newPoint = p1*T*zRot*yRot;
          // For Visualization of transfered Point
          /*  mySphere = new osg::Box(newPoint,2,2,2);
            osg::ShapeDrawable *mySphereDrawable = new osg::ShapeDrawable(mySphere);
            mySphereDrawable->setColor(osg::Vec4(1., 1., 0., 1.0f));
            //red color
            mySphereDrawable->setColor(osg::Vec4(1., 0., 0., 1.0f));
            osg::Geode *myGeode = new osg::Geode();
            myGeode->addDrawable(mySphereDrawable);
            myGeode->setName("SafetyZone");
            cover->getObjectsRoot()->addChild(myGeode);
            std::cout<<"D: "<<newPoint.y()<<" <= "<<Cam::depthView<<" && "<<newPoint.y()<<" >= "<<0<<std::endl;
            std::cout<<"x: "<<std::abs(newPoint.x()) <<" <= "<<Cam::imgWidth/2 * newPoint.y()/Cam::depthView<<std::endl;
            std::cout<<"H: "<<std::abs(newPoint.z())<<" <= "<<Cam::imgHeight/2 * newPoint.y()/Cam::depthView<<std::endl;
            */
            newPoint.set(newPoint.x(),newPoint.y()*-1,newPoint.z());
            if((newPoint.y()<=Cam::depthView ) && (newPoint.y()>=0) &&
               (std::abs(newPoint.x()) <= Cam::imgWidth/2 * newPoint.y()/Cam::depthView) &&
               (std::abs(newPoint.z()) <=Cam::imgHeight/2 * newPoint.y()/Cam::depthView))
            {
                if(calcIntersection(p1)==false)
                {
                    visMatForThisSafetyZone.push_back(1);//*calcRangeDistortionFactor(newPoint));//*calcRangeDistortionFactor(newPoint));
                    if(p->getPriority() == SafetyZone::PRIO1)
                    {
                        visMatPrio1.push_back(1);
                        double SRC = calcRangeDistortionFactor(newPoint);
                        distortionValuePrio1.push_back(SRC*PDC);
                       // std::cout<<"Total Value: "<<SRC*PDC<<std::endl;

                    }
                    else if(p->getPriority() ==SafetyZone::PRIO2)
                    {
                        visMatPrio2.push_back(1);
                        double SRC = calcRangeDistortionFactor(newPoint);
                        distortionValuePrio2.push_back(SRC*PDC);
                        //std::cout<<"Total Value: "<<SRC*PDC<<std::endl;

                    }
                }
                else
                {
                    visMatForThisSafetyZone.push_back(0);
                    if(p->getPriority() == SafetyZone::PRIO1)
                    {
                        visMatPrio1.push_back(0);
                        distortionValuePrio1.push_back(0);
                    }
                    else if(p->getPriority() ==SafetyZone::PRIO2)
                    {
                        visMatPrio2.push_back(0);
                        distortionValuePrio2.push_back(0);

                    }
                }
            }
            else
            {
                visMatForThisSafetyZone.push_back(0);
                if(p->getPriority() == SafetyZone::PRIO1)
                {
                    visMatPrio1.push_back(0);
                    distortionValuePrio1.push_back(0);

                }
                else if(p->getPriority() ==SafetyZone::PRIO2)
                {
                    visMatPrio2.push_back(0);
                    distortionValuePrio2.push_back(0);

                }
            }

 //           std::cout <<"P"<<cnt<<": "<<visMatForThisSafetyZone.back()<<" ";
            cnt++;
        }
        visMat.push_back(visMatForThisSafetyZone);

    }


 //   std::cout<<" "<<std::endl;
}
bool Cam::calcIntersection(const osg::Vec3d& end)
{
    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(pos,end);
    intersector->setIntersectionLimit(osgUtil::Intersector::IntersectionLimit::LIMIT_ONE_PER_DRAWABLE);
    osgUtil::IntersectionVisitor visitor(intersector);
   // EKU::plugin->finalScene->accept(visitor);// NOTE: how to do this rigth ? wihout acces to class EKU?
    cover->getObjectsRoot()->accept(visitor);
    const osgUtil::LineSegmentIntersector::Intersections hits = intersector->getIntersections();
 //   std::cout<<"Intersect: "<<hits.size()<<" with ";
    size_t numberOfNonRelevantObstacles = 0;
    for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr =hits.begin(); hitr!=hits.end();++hitr)
    {
        std::string name = hitr->nodePath.back()->getName();

        // Nodes with cx and px are safety areas or possible camera positions, these are no real obstacles
        if(((name.find("Point") != std::string::npos) ||name.find("Wireframe") != std::string::npos) ||(name.find("Cam") != std::string::npos) || (name.find("SafetyZone") != std::string::npos) || (name.find("Pyramid") != std::string::npos))
            ++numberOfNonRelevantObstacles;
//        std::cout<<hitr->nodePath.back()->getName()<<" & ";
    }
    if(hits.size()-numberOfNonRelevantObstacles>0)
        return true;
    else
        return false;
}

double Cam::calcRangeDistortionFactor(const osg::Vec3d &point) const
{
    double y = point.y(); //distance between point and sensor in depth direction
   // std::cout<<"distance"<<y<<std::endl;
    //SRC = Sensor Range Coefficient
    double calibratedValue = 70; // Parameter rangeDisortionDepth was calibrated for DephtView of 70;
    double omega = rangeDistortionDepth * depthView / calibratedValue; // adapt calibratedValue to new depthView
    //normalized Rayleigh distribution function
    double SRC = omega*exp(0.5) * (y / pow(omega,2)) * exp(-(pow(y,2)) / (2*pow(omega,2)));

   // std::cout<<"DepthView: "<<Cam::depthView<<std::endl;
   // std::cout<<"SRC: "<<SRC<<std::endl;

    return SRC;
}

double Cam::calcPreferredDirectionFactor( osg::Vec3 directionOfSafetyZone)
{
    //Prefered Direction Coefficient
    double c =0.0;
    double PDC =0.0;
    directionOfSafetyZone.normalize();
    if(directionOfSafetyZone.z()>0.6)
        PDC = 0.5;
    else
         PDC = std::pow((directionVec.operator *(directionOfSafetyZone)/(directionVec.normalize()*directionOfSafetyZone.normalize())),2) + c;
  //  std::cout<<"PDC: "<<PDC<<std::endl;
    return PDC;
}

size_t CamDrawable::count=0;

CamDrawable::CamDrawable(coCoord &m)
{
    count++;
//    fprintf(stderr, "new CamDrawable from Point\n");
    cam = std::unique_ptr<Cam>(new Cam(m,"Original from CamDrawable"));
    //create pyramide
    camGeode = plotCam();
    camGeode->setName("CamDrawable"+std::to_string(CamDrawable::count));
    camGeode->setNodeMask(camGeode->getNodeMask() & (~Isect::Intersection) & (~Isect::Pick));
    //create interactor
  /*  mySphere = new osg::Sphere(verts.get()->at(0), 1.);
    osg::ShapeDrawable *mySphereDrawable = new osg::ShapeDrawable(mySphere);
    mySphereDrawable->setColor(osg::Vec4(1., 0., 0., 1.0f));
    interactorGeode = new osg::Geode();
    interactorGeode->setName("Interactor");
    interactorGeode->addDrawable(mySphereDrawable);
    osg::StateSet *mystateSet = interactorGeode->getOrCreateStateSet();
    setStateSet(mystateSet);
    aSensor = new mySensor(interactorGeode, "showSZ", myinteraction, mySphereDrawable);
    sensorList.append(aSensor);
    */

}
void CamDrawable::setStateSet(osg::StateSet *stateSet)
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
void CamDrawable::preFrame()
{
}

CamDrawable::~CamDrawable()
{
    std::cout<<"deleted CamDrawable"<<std::endl;
    count--;
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

    verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 0 upper  front base
    verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 1 lower front base
    verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 3 lower  back  base
    verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 2 upper back  base
    verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak

    verts->dirty();

}

void CamDrawable::updateVisibility(float value)
{
    cam->depthView = value;
    cam->imgWidth = 2*cam->depthView*std::tan(cam->fov/2*osg::PI/180);
    cam->imgHeight = cam->imgWidth/(cam->imgWidthPixel/cam->imgHeightPixel);

    verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 0 upper  front base
    verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 1 lower front base
    verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 3 lower  back  base
    verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 2 upper back  base
    verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak

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
    name = "CamPosition"+std::to_string(counter);

    coCoord mEuler= m;
    camDraw = std::unique_ptr<CamDrawable>(new CamDrawable(mEuler));

    localDCS = new osg::MatrixTransform();
    localDCS->setName(name);
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

    createCamsInSearchSpace();
    searchSpaceGroup->setNodeMask(0);
    directionVec = calcDirectionVec(m);
    updateCamMatrixes();
    cover->getObjectsRoot()->addChild(localDCS.get());


    
}
CamPosition::CamPosition(osg::Matrix m,Pump *pump ):myPump(pump)
{
  //  std::cout<<"The next CamPosition is created from a Truck"<<std::endl;
    status =true;
    counter ++;
    searchSpaceState = false;
    name = "CamPosition"+std::to_string(counter);

    coCoord mEuler= m;
    camDraw = std::unique_ptr<CamDrawable>(new CamDrawable(mEuler));



    localDCS = new osg::MatrixTransform();
    localDCS->setName(name);
    localDCS->setMatrix(m);


    //create Interactors
    float _interSize = cover->getSceneSize() / 25;
    viewpointInteractor = new coVR3DTransRotInteractor(m, _interSize/2, vrui::coInteraction::ButtonA, "hand", "CamInteractor", vrui::coInteraction::Medium);
    viewpointInteractor->show();
    viewpointInteractor->enableIntersection();

    localDCS->addChild(camDraw->getCamGeode().get());

    searchSpaceGroup = new osg::Group;
    searchSpaceGroup->setName("SearchSpace");
    localDCS->addChild(searchSpaceGroup.get());

    createCamsInSearchSpace();
    searchSpaceGroup->setNodeMask(0);
    directionVec = calcDirectionVec(m);
    updateCamMatrixes();
}
void CamPosition::activate()
{
    status = true;
    viewpointInteractor->show();
    viewpointInteractor->enableIntersection();
    camDraw->activate();
}
void CamPosition::disactivate()
{
    status = false;
    viewpointInteractor->disableIntersection();
    viewpointInteractor->hide();
    camDraw->disactivate();
}
CamPosition::~CamPosition()
{

    localDCS->getParent(0)->removeChild(localDCS);
     delete viewpointInteractor;

    std::cout<<"deleted Camposition: "<<name<<std::endl;

}
void CamPosition::preFrame()
{

    viewpointInteractor->preFrame();

    //change positions of cameras
    if(EKU::modifyScene==true)
    {
        viewpointInteractor->setNoTranslationNoRotation(false);
        coCoord testEuler;
        if(viewpointInteractor->wasStarted())
        {
           osg::Matrix test = viewpointInteractor->getMatrix();
            testEuler = test;
            std::cout<<"START"<<std::endl;
        }
        if(viewpointInteractor->isRunning())
        {
            osg::Matrix local = viewpointInteractor->getMatrix();
            coCoord localEuler = local;
            //restrict rotation around y
           // localEuler.hpr[2]=testEuler.hpr[2];
            //cameras are always looking downwards -> no neg rotation around x
            //if(localEuler.hpr[1] < 0.0)
            //    localEuler.hpr[1] = 0.0;
            localEuler.makeMat(local);
            localDCS->setMatrix(local);
           // viewpointInteractor->updateTransform(local);
    //        std::cout<<"Rotation(around global axes): "<<"z:"<<localEuler.hpr[0]<< " x:"<<localEuler.hpr[1]<<" y:"<<localEuler.hpr[2]<<std::endl;
        }
        if(viewpointInteractor->wasStopped())
        {

            osg::Matrix local = viewpointInteractor->getMatrix();
            directionVec = calcDirectionVec(local);
            updateCamMatrixes();

        }

    }
    // use interactor to show all visible SZ
    else
    {        viewpointInteractor->setNoTranslationNoRotation(true);

        if(viewpointInteractor->wasStarted())
        {
            camDraw->activate();
            size_t cnt=0;
            double visible =1;
            for(const auto &x : camDraw->cam->visMat)
            {
                if(std::find(x.begin(), x.end(), visible) != x.end())
                    EKU::safetyZones.at(cnt)->updateColor(x);
                cnt++;
            }
        }
        if(viewpointInteractor->wasStopped())
        {
            camDraw->disactivate();
            size_t cnt=0;
            double visible =1;
            for(const auto &x : camDraw->cam->visMat)
            {
                if(std::find(x.begin(), x.end(), visible) != x.end())
                   EKU::safetyZones.at(cnt)->resetColor(x);

                cnt++;
            }

        }
    }
    //camDraw->preFrame();
}
void CamPosition::createCamsInSearchSpace()
{
    //around z axis
    int zMax = 180;
    int stepSizeZ = 5; //in Degree

    int xMax = 20;
    int stepSizeX = 10; //in Degree

    osg::Matrix m = localDCS.get()->getMatrix();
    coCoord coord=m;
    coCoord newCoordPlus,newCoordMinus;
    newCoordPlus.hpr[2] =0;
    newCoordMinus.hpr[2] =0;

    //std::cout<<"StartPos: "<<"z:"<<coord.hpr[0]<< " x:"<<coord.hpr[1]<<" y:"<<coord.hpr[2]<<std::endl;

    int nbrOfCameras =0;
    osg::Matrix m_new;

 /* //For debugging: only 1 cam in search space
    newCoordPlus.makeMat(m_new);
    searchSpace.push_back(new osg::MatrixTransform );
    searchSpaceGroup->addChild(searchSpace.back().get());
    searchSpace.back()->setMatrix(m_new);
    searchSpace.back()->setName(std::to_string(nbrOfCameras)+"+MATRIX Z:" + std::to_string( newCoordPlus.hpr[0])+ " X:" +std::to_string( newCoordPlus.hpr[1]));
    searchSpace.back()->addChild(camDraw->getCamGeode().get());
*/
    int count =0;
    for(int cnt = 0 ; cnt<zMax/stepSizeZ; cnt++)//############## ===cnt = 0!!!!!!!!!!!muss hier hin
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

            nbrOfCameras++;
            newCoordPlus.makeMat(m_new);
            searchSpace.push_back(new osg::MatrixTransform );
            searchSpaceGroup->addChild(searchSpace.back().get());
            searchSpace.back()->setMatrix(m_new);
            searchSpace.back()->setName(std::to_string(nbrOfCameras)+"+MATRIX Z:" + std::to_string( newCoordPlus.hpr[0])+ " X:" +std::to_string( newCoordPlus.hpr[1]));
            searchSpace.back()->addChild(camDraw->getCamGeode().get());
       //     std::cout<<" + Search space Matrix"<<std::endl;
       //     printCoCoord(newCoordPlus);


            if(countX==0)
                 newCoordMinus.hpr[1]=0;
            else
                newCoordMinus.hpr[1] += stepSizeX;

            nbrOfCameras++;
            newCoordMinus.makeMat(m_new);
            searchSpace.push_back(new osg::MatrixTransform );
            searchSpaceGroup->addChild(searchSpace.back().get());
            searchSpace.back()->setMatrix(m_new);
            searchSpace.back()->setName(std::to_string(nbrOfCameras)+"-MATRIX Z:" + std::to_string( newCoordMinus.hpr[0])+ " X:" +std::to_string( newCoordMinus.hpr[1]));
            searchSpace.back()->addChild(camDraw->getCamGeode().get());
        //    std::cout<<" - Search space Matrix"<<std::endl;
        //    printCoCoord(newCoordMinus);


            countX++;

        }
    }
}
void CamPosition::updateCamMatrixes()
{

    if(allCameras.empty())
    {
        int count = 0;
        for(const auto& x :searchSpace)
        {
            count++;
            std::string name = std::to_string(count);
            osg::Quat q = x->getMatrix().getRotate()*localDCS.get()->getMatrix().getRotate();
            osg::Matrix tmp;
            tmp.setRotate(q);
            tmp.setTrans(localDCS.get()->getMatrix().getTrans());
            coCoord euler = tmp;
            std::shared_ptr<Cam> camera = std::make_shared<Cam>(euler,name);
            //std::unique_ptr<Cam> camera(new Cam(euler,name));
            allCameras.push_back(std::move(camera));
        }
    }
    else
    {
        int cnt =0; //important!!
        for(const auto& x :searchSpace)
        {
            osg::Quat q = x->getMatrix().getRotate()*localDCS.get()->getMatrix().getRotate();
            osg::Matrix tmp;
            tmp.setRotate(q);
            tmp.setTrans(localDCS.get()->getMatrix().getTrans());
            coCoord euler = tmp;
            allCameras.at(cnt)->setPosition(euler);
      //      printCoCoord(euler);
            cnt++;

        }
    }



    //update pos of camDraw
    coCoord euler =localDCS->getMatrix();
    camDraw->cam->setPosition(euler);
    std::cout<<"All cam positions are updated!"<<std::endl;
}

void CamPosition::setSearchSpaceState(bool state)
{

   // if(searchSpaceState == false)
        //createCamsInSearchSpace();

    if(state)
        searchSpaceGroup->setNodeMask(UINT_MAX);
    else
        searchSpaceGroup->setNodeMask(0);

}




