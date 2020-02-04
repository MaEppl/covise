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
    //std::cout<<"translation: "<<"x:"<<m.xyz[0]<< " y:"<<m.xyz[1]<<" z:"<<m.xyz[2]<<std::endl;
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

Cam::Cam(coCoord matrix,std::vector<std::vector<double>> visMat,std::string name):pos(matrix.xyz),rot(matrix.hpr[0],matrix.hpr[1]),visMat(visMat),name(name)
{
    count++;
    matrix.makeMat(mat);
    calcVisMat();
    directionVec = calcDirectionVec(mat);

    // Debugging:
    /*
        float _interSize = cover->getScNeneSize() / 25;
        viewpointInteractor = new coVR3DTransRotInteractor(mat, _interSize/2, vrui::coInteraction::ButtonA, "hand", "CamInteractor", vrui::coInteraction::Medium);
        viewpointInteractor->hide();
        viewpointInteractor->enableIntersection();
    */
}
Cam::~Cam()
{

}
void Cam::setPosition(coCoord& m, std::vector<std::vector<double> > visMatInput)
{
    pos = m.xyz;
    rot.set(m.hpr[0],m.hpr[1]);
    m.makeMat(mat);
    directionVec = calcDirectionVec(mat);
    visMat = visMatInput;
    calcVisMat();
 /*   for(const auto& x:visMat)
    {std::cout<<"Cam"<< name <<": ";
        for (const auto& x1 :x)
            std::cout<<x1<<",";
    }
    std::cout<<" "<<std::endl;
   */// viewpointInteractor->updateTransform(mat);
}
/*void Cam::preFrame()
{
    viewpointInteractor->preFrame();
}
*/
void Cam::calcVisMat()
{   visMatPrio1.clear();
    visMatPrio2.clear();
  //  visMat.clear(); do not clear anymore!
    distortionValuePrio1.clear();
    distortionValuePrio2.clear();
    if(!visMat.empty()){
  //  visMat.reserve(EKU::safetyZones.size());
        osg::Matrix T = osg::Matrix::translate(-pos);
        osg::Matrix zRot = osg::Matrix::rotate(-osg::DegreesToRadians(rot.x()), osg::Z_AXIS);
        osg::Matrix yRot = osg::Matrix::rotate(-osg::DegreesToRadians(rot.y()), osg::X_AXIS);
        coCoord eulerTest = mat;
        double test = eulerTest.hpr[2];
        osg::Matrix testRot = osg::Matrix::rotate(-osg::DegreesToRadians(test), osg::Y_AXIS);
        // BUGFIX: still problem at borders?
        size_t countSZ=0;
        for(const auto& p : EKU::safetyZones)
        {
           // std::vector<double> visMatForThisSafetyZone;
           // visMatForThisSafetyZone.reserve(p->getWorldPosOfAllObservationPoints().size());

            //Preferred Direction Coefficient is equal for whole safety zone
            double PDC = 1.0;//calcPreferredDirectionFactor(p->getPreferredDirection()); // not using PDC
            size_t count=0;
            for(const auto& p1 :p->getWorldPosOfAllObservationPoints())
            {
                if(visMat.at(countSZ).at(count) == 0) // Point is not visible because of intersection test
                {
                   // visMatForThisSafetyZone.push_back(0);
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
                    //count++;
                }
                else{ //point is Visible from CamPos now check if it's in FOV
                        osg::Vec3 newPoint = p1*T*zRot*yRot*testRot;
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
                        {   //if point is in FOV
                            //visMat.at(countSZ).at(count)=1; not necessary, it's already 1

                            if(p->getPriority() == SafetyZone::PRIO1)
                            {
                                visMatPrio1.push_back(1);
                                double SRC = calcRangeDistortionFactor(newPoint);
                                double SWC = calcWidthDistortionFactor(newPoint);
                                double SHC = calcHeightDistortionFactor(newPoint);
                                distortionValuePrio1.push_back(SRC*SWC*SHC*PDC);
                               // std::cout<<"Total Value: "<<SRC*PDC<<std::endl;

                            }
                            else if(p->getPriority() ==SafetyZone::PRIO2)
                            {
                                visMatPrio2.push_back(1);
                                double SRC = calcRangeDistortionFactor(newPoint);
                                double SWC = calcWidthDistortionFactor(newPoint);
                                double SHC = calcHeightDistortionFactor(newPoint);
                                distortionValuePrio2.push_back(SRC*SWC*SHC*PDC);
                                //std::cout<<"Total Value: "<<SRC*PDC<<std::endl;

                            }
                         }
                        else // not in FOV
                        {
                            visMat.at(countSZ).at(count)=0;
                            if(p->getPriority() == SafetyZone::PRIO1)
                            {
                                visMatPrio1.push_back(0);
                                distortionValuePrio1.push_back(0);
                               // std::cout<<"Total Value: "<<SRC*PDC<<std::endl;

                            }
                            else if(p->getPriority() ==SafetyZone::PRIO2)
                            {
                                visMatPrio2.push_back(0);
                                distortionValuePrio2.push_back(0);
                                //std::cout<<"Total Value: "<<SRC*PDC<<std::endl;

                            }
                        }
                        //count ++;
                    }count++;
                }
            countSZ++;
            }
    }
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
double Cam::calcWidthDistortionFactor(const osg::Vec3d &point)
{
    double widthFOVatPoint = point.y()*std::tan(Cam::fov/2*osg::PI/180);
    double x = std::abs(point.x()); //distance between point and sensor in width direction

    double x_scaled = scale(0,widthFOVatPoint,0,1,x);
    //SWC = Sensor Width Coefficient SWC = -x² +1
    double SWC = - std::pow(x_scaled,2) + 1;
    return SWC;
}
double Cam::calcHeightDistortionFactor(const osg::Vec3d &point)
{
    double widthFOVatPoint = point.y()*std::tan(Cam::fov/2*osg::PI/180);
    double heightFOVatPoint = widthFOVatPoint/(Cam::imgWidthPixel/Cam::imgHeightPixel);
    double z = std::abs(point.z()); //distance between point and sensor in width direction

    double z_scaled = scale(0,heightFOVatPoint,0,1,z);
    //SWC = Sensor Width Coefficient SWC = -x² +1
    double SHC = - std::pow(z_scaled,2) + 1;
    return SHC;
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
double Cam::scale(double oldMin, double oldMax, double newMin, double newMax, double oldValue)
{
    double oldRange = oldMax - oldMin;
    double newRange = newMax - newMin;
    double newValue = (((oldValue - oldMin) * newRange) / oldRange )+ newMin;
    return newValue;
}

size_t CamDrawable::count=0;

CamDrawable::CamDrawable(coCoord &m, std::vector<std::vector<double> > visMat, bool showLines, Vec4 color)
{
    count++;
//    fprintf(stderr, "new CamDrawable from Point\n");
    cam = std::make_shared<Cam>(m,visMat,"Original from CamDrawable");
    //create pyramide
    camGeode = plotCam(showLines,color);
    camGeode->setName("CamDrawable"+std::to_string(CamDrawable::count));
    camGeode->setNodeMask(camGeode->getNodeMask() & (~Isect::Intersection) & (~Isect::Pick));

    SRCgeode = plotSRC();
    SRCgeode->setName("SRCDrawable"+std::to_string(CamDrawable::count));
    SRCgeode->setNodeMask(camGeode->getNodeMask() & (~Isect::Intersection) & (~Isect::Pick));
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
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}


CamDrawable::~CamDrawable()
{
    count--;
}

/*osg::Vec3 calcPointOnPyramidAtSpecificDistance(double distance)
{
    double x = distance*std::tan(Cam::fov/2*osg::PI/180)/(2*scale);
    double y = x/(Cam::imgWidthPixel/Cam::imgHeightPixel)/(2*scale);
    osg::Vec3 point = {x,y,dof};
    return point;
}
*/
/*osg::Geode* CamDrawable::plotSRC()
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


    double TimgWidth = 2*Cam::depthView/2*std::tan(Cam::fov/2*osg::PI/180);
    double TimgHeight = TimgWidth/(Cam::imgWidthPixel/Cam::imgHeightPixel);
    verts->push_back(osg::Vec3(TimgWidth/2,-Cam::depthView/2,TimgHeight/2)/scale );
    verts->push_back(osg::Vec3(-TimgWidth/2,-Cam::depthView/2,TimgHeight/2)/scale );

    vertsSRC = new osg::Vec3Array;
    verts->push_back( osg::Vec3( -Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2 )/scale ); // 0 upper  front base
    verts->push_back( osg::Vec3( -Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2 )/scale ); // 1 lower front base
    verts->push_back( osg::Vec3(  Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2 )/scale ); // 3 lower  back  base
    verts->push_back( osg::Vec3(  Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2 )/scale ); // 2 upper back  base
    verts->push_back( osg::Vec3( 0,  0,  0) ); // 4 peak
}
*/
osg::Geode* CamDrawable::plotCam(bool showLines,osg::Vec4 color)
{
    // The Drawable geometry is held under Geode objects.
    osg::Geode* geode = new osg::Geode();
    geode->setName("Pyramid");
    geom = new osg::Geometry();
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
    verts->push_back( osg::Vec3( -Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2 )/scale ); // 0 upper  front base
    verts->push_back( osg::Vec3( -Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2 )/scale ); // 1 lower front base
    verts->push_back( osg::Vec3(  Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2 )/scale ); // 3 lower  back  base
    verts->push_back( osg::Vec3(  Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2 )/scale ); // 2 upper back  base
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

    if(showLines)
    {
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
    }

        // Create a separate color for each face.
     colors = new osg::Vec4Array; //magenta 1 1 0; cyan 0 1 1; black 0 0 0
     osg::Vec4 makeTransparent = color;
     makeTransparent.set(color.x(),color.y(),color.z(),0.5);
     colors->push_back( makeTransparent ); // magenta - back
     colors->push_back( color ); // magenta - back
     colors->push_back( color ); // magenta - back
     colors->push_back( color ); // magenta - back
     colors->push_back( color ); // yellow  - base


    // Assign the color indices created above to the geometry and set the
    // binding mode to _PER_PRIMITIVE_SET.
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

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

    //LineWidth
    LineWidth *lw = new LineWidth(3.0);
    stateset->setAttribute(lw);
    // return the geode as the root of this geometry.
    return geode;
}

osg::Geode* CamDrawable::plotSRC()
{
    // The Drawable geometry is held under Geode objects.
    osg::Geode* geode = new osg::Geode();
    geode->setName("SRC");
    geomSRC = new osg::Geometry();
    osg::StateSet *stateset = geode->getOrCreateStateSet();
    //necessary for dynamic redraw (command:dirty)
    geomSRC->setDataVariance(osg::Object::DataVariance::DYNAMIC) ;
    geomSRC->setUseDisplayList(false);
    geomSRC->setUseVertexBufferObjects(true);
    //stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //stateset->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
    // Associate the Geometry with the Geode.
    geode->addDrawable(geomSRC);
    geode->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    geode->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    geode->getStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    vertsSRC = new osg::Vec3Array;


   //  vertsSRC->push_back( osg::Vec3( -Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2 )/scale );
   //   vertsSRC->push_back( osg::Vec3(  Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2 )/scale );
    std::pair<osg::Vec3,osg::Vec3> lastVertexes = calcPointsOnPyramidAtDistance(Cam::depthView,osg::Y_AXIS);
//    vertsSRC->push_back(lastVertexes.first);
//    vertsSRC->push_back(lastVertexes.second);

    vertsSRC->push_back(osg::Vec3(-Cam::imgWidth/4,-Cam::depthView/2,Cam::imgHeight)/scale);
    vertsSRC->push_back(osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,Cam::imgHeight)/scale);
    vertsSRC->push_back(osg::Vec3(Cam::imgWidth/2,-Cam::depthView,Cam::imgHeight)/scale);
    vertsSRC->push_back(osg::Vec3(Cam::imgWidth/4,-Cam::depthView/2,Cam::imgHeight)/scale);

    vertsSRC->push_back(osg::Vec3(-Cam::imgWidth/8,0,Cam::imgHeight)/scale);
    vertsSRC->push_back(osg::Vec3(-Cam::imgWidth/4,-Cam::depthView/2,Cam::imgHeight)/scale);
    vertsSRC->push_back(osg::Vec3(Cam::imgWidth/4,-Cam::depthView/2,Cam::imgHeight)/scale);
    vertsSRC->push_back(osg::Vec3(Cam::imgWidth/8,0,Cam::imgHeight)/scale);


    std::vector<float> distances;
        distances.push_back(0.f);



 /*   std::vector<float> distances;
    distances.push_back(3.3f);
    distances.push_back(6.5f);
    distances.push_back(10.2f);
    distances.push_back(15.f);
    distances.push_back(39.f);
    distances.push_back(46.f);
    distances.push_back(53.f);
    distances.push_back(63.f);
*/
/*    for(const auto x : distances)
    {
        std::pair<osg::Vec3,osg::Vec3> test = calcPointsOnPyramidAtDistance(x,osg::Y_AXIS);
        vertsSRC->push_back(test.first);
        vertsSRC->push_back(test.second);
    }
  */  vertsSRC->push_back( osg::Vec3( 0,  0,  0) ); // 4 peak


    geomSRC->setVertexArray(vertsSRC);




 /*   osg::DrawElementsUInt* quad =
       new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    for(int i =0;i<=vertsSRC.get()->size()-4;i+=4)
    {
   */     osg::DrawElementsUInt* quad = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
        quad->push_back(0);
        quad->push_back(0+1);
        quad->push_back(0+2);
        quad->push_back(0+3);
        geomSRC->addPrimitiveSet(quad);

        quad = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
                quad->push_back(4);
                quad->push_back(5);
                quad->push_back(6);
                quad->push_back(7);
                geomSRC->addPrimitiveSet(quad);

  //  }
    osg::DrawElementsUInt* triangle =
       new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    int count = vertsSRC.get()->size();
    triangle->push_back(4);
    triangle->push_back(7);
    triangle->push_back(8);
    geomSRC->addPrimitiveSet(triangle);
   // Create a separate color for each face
    colorsSRC = new osg::Vec4Array;
    osg::Vec4 color = {0.0,0.0,0.0,0.3};
    float colorChange =0.0f;

    colorsSRC->push_back(color+osg::Vec4{0.f,colorChange,0.0f,0.f});
    colorsSRC->push_back(color+osg::Vec4{0.f,0.5,0.0f,0.f});
    colorsSRC->push_back(color+osg::Vec4{0.f,0.5,1.0f,0.f});



 /*   for(int i=1;i<=distances.size()/2+1;i++)
    {

        colorsSRC->push_back(color+osg::Vec4{0.f,colorChange,0.0f,0.f});
        colorChange+=0.2;
    }

    colorsSRC->push_back(colorsSRC.get()->at(3));
    colorsSRC->push_back(colorsSRC.get()->at(2));
    colorsSRC->push_back(colorsSRC.get()->at(1));
    colorsSRC->push_back(colorsSRC.get()->at(0));

*/

   // Assign the color indices created above to the geometry and set the
   // binding mode to _PER_PRIMITIVE_SET.
   geomSRC->setColorArray(colorsSRC);
   geomSRC->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
   return geode;
}
std::pair<osg::Vec3, osg::Vec3> CamDrawable::calcPointsOnPyramidAtDistance(const float distance,const osg::Vec3 fixAxis)
{
    osg::Vec3 vert1;
    osg::Vec3 vert2;
    if(fixAxis == osg::X_AXIS)
    {

    }
    else if(fixAxis == osg::Y_AXIS)
    {
       // float width = 2*distance*std::tan(Cam::fov/2*osg::PI/180);
        float width = 2*70*std::tan(Cam::fov/2*osg::PI/180);

        float heigth = width/(Cam::imgWidthPixel/Cam::imgHeightPixel);
        // vert1 = osg::Vec3(-width/2,-distance,heigth/2)/scale;
        // vert2 = osg::Vec3(width/2,-distance,heigth/2)/scale;
        vert1 = osg::Vec3(-width/2,-distance,heigth/2)/scale;
        vert2 = osg::Vec3(width/2,-distance,heigth/2)/scale;
    }
    else if(fixAxis == osg::Z_AXIS)
    {

    }
    else
    {
        //else trow Exeption!s
    }

    return std::make_pair(vert1,vert2);
}

void CamDrawable::updateFOV(float value)
{
    cam->fov = value;
    cam->imgWidth = 2*cam->depthView*std::tan(cam->fov/2*osg::PI/180);
    cam->imgHeight = cam->imgWidth/(cam->imgWidthPixel/cam->imgHeightPixel);

    if(_showRealSize)
    {
        verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 0 upper  front base
        verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 1 lower front base
        verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 3 lower  back  base
        verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 2 upper back  base
        verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak
    }
    else
    {
        verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2)/scale; // 0 upper  front base
        verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2)/scale; // 1 lower front base
        verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2)/scale; // 3 lower  back  base
        verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2)/scale; // 2 upper back  base
        verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak
    }
    verts->dirty();
    geom->dirtyBound();

}

void CamDrawable::updateVisibility(float value)
{
    cam->depthView = value;
    cam->imgWidth = 2*cam->depthView*std::tan(cam->fov/2*osg::PI/180);
    cam->imgHeight = cam->imgWidth/(cam->imgWidthPixel/cam->imgHeightPixel);

    if(_showRealSize)
    {
        verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 0 upper  front base
        verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 1 lower front base
        verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 3 lower  back  base
        verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 2 upper back  base
        verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak
    }
    else
    {
        verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2)/scale; // 0 upper  front base
        verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2)/scale; // 1 lower front base
        verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2)/scale; // 3 lower  back  base
        verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2)/scale; // 2 upper back  base
        verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak
    }
    verts->dirty();
    geom->dirtyBound();
}
void CamDrawable::showRealSize()
{
    verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2);
    verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2);
    verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2);
    verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2);
    verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak                             ;

    verts->dirty();
    geom->dirtyBound();
}
void CamDrawable::resetSize()
{
    if(_showRealSize)
    {
        verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 0 upper  front base
        verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 1 lower front base
        verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2); // 3 lower  back  base
        verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2); // 2 upper back  base
        verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak
    }
    else
    {
        verts->at(0) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2)/scale; // 0 upper  front base
        verts->at(1) = osg::Vec3(-Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2)/scale; // 1 lower front base
        verts->at(2) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView,-Cam::imgHeight/2)/scale; // 3 lower  back  base
        verts->at(3) = osg::Vec3( Cam::imgWidth/2,-Cam::depthView, Cam::imgHeight/2)/scale; // 2 upper back  base
        verts->at(4) = osg::Vec3( 0,  0,  0); // 4 peak
    }
    verts->dirty();
    geom->dirtyBound();

}

size_t CamPosition::counter =0;
CamPosition::CamPosition(osg::Matrix m)
{
    counter ++;
    name = "CamPosition"+std::to_string(counter);

    coCoord mEuler= m;
    localDCS = new osg::MatrixTransform();
    localDCS->setName(name);
    localDCS->setMatrix(m);
    //check Intersection of this Camposition with all points in SZ
    calcIntersection();

    camDraw = std::unique_ptr<CamDrawable>(new CamDrawable(mEuler,visMat,true,osg::Vec4(0.0f, 1.0f, 0.0f, 1.f)));
    coCoord empty;
    searchSpaceDrawable = std::unique_ptr<CamDrawable>(new CamDrawable(empty,visMat,false,osg::Vec4(1.0f, 1.0f, 0.0f, 1.f)));
    deletedOrientationsDrawable = std::unique_ptr<CamDrawable>(new CamDrawable(empty,visMat,false,osg::Vec4(1.f, 0.0f, 0.0f, 1.f)));

    //create Interactors
    float _interSize = cover->getSceneSize() / 25;
    viewpointInteractor = new coVR3DTransRotInteractor(m, _interSize/2, vrui::coInteraction::ButtonA, "hand", "CamInteractor", vrui::coInteraction::Medium);
    viewpointInteractor->show();
    viewpointInteractor->enableIntersection();
    localDCS->addChild(camDraw->getCamGeode());

    searchSpaceGroup = new osg::Group;
    searchSpaceGroup->setName("SearchSpace");
    searchSpaceGroup->addChild(searchSpaceDrawable->getCamGeode().get());

    searchSpaceGroupDeleted = new osg::Group;
    searchSpaceGroupDeleted->setName("DeletedSearchSpace");
    searchSpaceGroupDeleted->addChild(deletedOrientationsDrawable->getCamGeode().get());


    switchNode = new osg::Switch();
    switchNode->setName("CamPosSwitch");
    switchNode->addChild(localDCS.get(),true);
    switchNode->addChild(searchSpaceGroup.get(),false);
    switchNode->addChild(searchSpaceGroupDeleted.get(),false);

    createCamsInSearchSpace();
    directionVec = calcDirectionVec(m);
    //updateCamMatrixes();
    updateVisibleCam();
    cover->getObjectsRoot()->addChild(switchNode.get());



    
}

CamPosition::CamPosition(osg::Matrix m,EquipmentWithCamera *pump )
{
  //  std::cout<<"The next CamPosition is created from a Truck"<<std::endl;
    status =true;
    counter ++;
    name = "CamPosition"+std::to_string(counter);

    coCoord mEuler= m;
    localDCS = new osg::MatrixTransform();
    localDCS->setName(name);
    localDCS->setMatrix(m);


    camDraw = std::unique_ptr<CamDrawable>(new CamDrawable(mEuler,visMat,true,osg::Vec4(0.0f, 1.0f, 0.0f, 1.f)));
    coCoord empty;
    searchSpaceDrawable = std::unique_ptr<CamDrawable>(new CamDrawable(empty,visMat,false,osg::Vec4(1.0f, 1.0f, 0.0f, 1.f)));
    deletedOrientationsDrawable = std::unique_ptr<CamDrawable>(new CamDrawable(empty,visMat,false,osg::Vec4(1.0f, 0.0f, 0.0f, 1.f)));

    switchNode2 = new osg::Switch();
    switchNode2->setName("CamSRCSwitch");

    //create Interactors
    float _interSize = cover->getSceneSize() / 25;
    viewpointInteractor = new coVR3DTransRotInteractor(m, _interSize/3, vrui::coInteraction::ButtonA, "hand", "CamInteractor", vrui::coInteraction::Medium);
    viewpointInteractor->show();
    viewpointInteractor->enableIntersection();
    localDCS->addChild(switchNode2.get());
    switchNode2->addChild(camDraw->getCamGeode().get(),true);
    switchNode2->addChild(camDraw->getSRCgeode().get(),true);
 //   localDCS->addChild(camDraw->getSRCgeode().get());

    searchSpaceGroup = new osg::Group;
    searchSpaceGroup->setName("SearchSpace");
    searchSpaceGroup->addChild(searchSpaceDrawable->getCamGeode().get());

    searchSpaceGroupDeleted = new osg::Group;
    searchSpaceGroupDeleted->setName("DeletedSearchSpace");
    searchSpaceGroupDeleted->addChild(deletedOrientationsDrawable->getCamGeode().get());


    switchNode = new osg::Switch();
    switchNode->setName("CamPosSwitch");
    switchNode->addChild(localDCS.get(),true);
    switchNode->addChild(searchSpaceGroup.get(),false);
    switchNode->addChild(searchSpaceGroupDeleted.get(),false);

    //check Intersection of this Camposition with all points in SZ
    calcIntersection();
    createCamsInSearchSpace();
    directionVec = calcDirectionVec(m);
    updateVisibleCam();
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

    switchNode->getParent(0)->removeChild(switchNode.get());
     delete viewpointInteractor;

}
bool CamPosition::preFrame()
{

    viewpointInteractor->preFrame();

    //change positions of cameras
    if(EKU::modifyScene==true)
    {
        viewpointInteractor->setNoTranslationNoRotation(false);
        coCoord testEuler;
        if(viewpointInteractor->wasStarted())
        {
            if(EKU::deleteObjects)
                return false;

            deleteSearchSpace();

        }
        if(viewpointInteractor->isRunning())
        {
            osg::Matrix local = viewpointInteractor->getMatrix();
            coCoord localEuler = local;
            //restrict rotation around y
          //  localEuler.hpr[2]=testEuler.hpr[2];
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
            setPosition(viewpointInteractor->getMatrix());
            osg::Matrix local = viewpointInteractor->getMatrix();
            directionVec = calcDirectionVec(local);
           // calcIntersection();

            updateVisibleCam();
            createCamsInSearchSpace();
        }

    }
    // use interactor to show all visible SZ
    else
    {
        viewpointInteractor->setNoTranslationNoRotation(true);

        if(viewpointInteractor->wasStarted())
        {
           // camDraw->activate();
            camDraw->showRealSize();

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
            //camDraw->disactivate();
            camDraw->resetSize();
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
    return true;
}

/*void CamPosition::createCams(bool rotX=true, bool rotY=true, bool rotZ=true, double limitX=90, double limitY =90, double limitZ = 90)
{

};
*/

/* void rotMatrix(coCoord& euler, int stepSize, osg::Vec3f& axis)
 {
     switch (axis) {
     case osg::X_AXIS:
         euler.hpr[1] +=stepSize;
         break;
     case osg::Y_AXIS:
         euler.hpr[2] +=stepSize;
         break;
     case osg::Z_AXIS:
         euler.hpr[0] +=stepSize;
         break;
     default:
         break;
     }
 }
 */
void CamPosition::createCamsInSearchSpace()
{
    deleteSearchSpace();

    //around z axis
    int zMax = 180;
    int stepSizeZ = 5; //in Degree

    int xMax = 80;
    int stepSizeX = 10; //in Degree

    int yMax = 135;
    int stepSizeY = 45; //in Degree

    osg::Matrix m = localDCS.get()->getMatrix();
    coCoord coord;//=m;
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
    searchSpace.back()->addChild(searchSpaceDrawable->getCamGeode().get());
*/
    int count =0;
    for(int cnt = 0 ; cnt<zMax/stepSizeZ; cnt++)//############## ===cnt = 0!!!!!!!!!!!muss hier hin
    {

        if(count == 0)
        {
            newCoordPlus.hpr[0]=0;
            newCoordMinus.hpr[0]-= stepSizeZ;
        }
        else
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
            {
                 newCoordPlus.hpr[1]=0;
                 newCoordMinus.hpr[1]=0;

            }
            else
            {
                newCoordPlus.hpr[1] += stepSizeX;
                newCoordMinus.hpr[1] += stepSizeX;
            }
            countX++;

            //Rotation round Y
            int countY =0;
            for(int cnt3 = 0; cnt3<=yMax/stepSizeY; cnt3++)
            {
                if(countY==0)
                {
                     newCoordPlus.hpr[2]=0;
                     newCoordMinus.hpr[2]=0;

                }
                else
                {
                    newCoordPlus.hpr[2] += stepSizeY;
                    newCoordMinus.hpr[2] += stepSizeY;

                }

                std::shared_ptr<Cam> camPlus = createCamFromMatrix(newCoordPlus);
                std::shared_ptr<Cam> camMinus = createCamFromMatrix(newCoordMinus);

                if(!isVisibilityMatrixEmpty(camPlus))
                {
                    if(allCameras.empty())
                        addCamToVec(camPlus);
                    else
                    {
                       compareCams(camPlus);
                    }

                }
                camPlus.reset();

                if(!isVisibilityMatrixEmpty(camMinus))
                {
                    if(allCameras.empty())
                        addCamToVec(camMinus);
                    else
                    {
                       compareCams(camMinus);
                    }

                }
                camMinus.reset();

                countY++;
            }

          }
    }

    createDrawableForEachCamOrientation();
    EKU::updateNbrCams();

}
std::shared_ptr<Cam> CamPosition::createCamFromMatrix(coCoord& euler)
{
    static int i =0;
    osg::Matrix m;
    euler.makeMat(m);
    osg::Matrix translationViewpoint;
    translationViewpoint.setTrans(viewpointInteractor->getMatrix().getTrans());
    std::shared_ptr<Cam> cam =std::make_shared<Cam>(m*translationViewpoint,visMat,"cam"+std::to_string(i));
    i++;
    return cam;
}
bool CamPosition::isVisibilityMatrixEmpty(const std::shared_ptr<Cam>& cam)
{

    bool emptyPrio1 = std::all_of(cam->visMatPrio1.begin(), cam->visMatPrio1.end(), [](int i) { return i==0; });
    bool emptyPrio2 = std::all_of(cam->visMatPrio2.begin(), cam->visMatPrio2.end(), [](int i) { return i==0; });
    if(emptyPrio1 && emptyPrio2)
        return true;
    else
        return false;
}

bool Cam::operator>>(const Cam& other)const
{
    int onlyInThisCam =0;
    int onlyInOtherCam =0;
    //check PRIO1 zone
    auto ItThisCam = visMatPrio1.begin();
    auto ItOtherCam = other.visMatPrio1.begin();
    while(ItThisCam != visMatPrio1.end() || ItOtherCam != other.visMatPrio1.end() )
    {

        if(*ItThisCam > *ItOtherCam)
            ++onlyInThisCam;
        if(*ItThisCam < *ItOtherCam)
            ++onlyInOtherCam;
        if((onlyInThisCam && onlyInOtherCam) !=0) //each camera can see points, which the other can't see --> keep both cameras
        {
            return false;
        }

        // Loop further
        if(ItThisCam != visMatPrio1.end())
        {
            ++ItThisCam;
        }
        if(ItOtherCam != other.visMatPrio1.end())
        {
            ++ItOtherCam;
        }
    }

    //check PRIO2 zone
    auto ItThisCam2 = visMatPrio2.begin();
    auto ItOtherCam2 = other.visMatPrio2.begin();
    while(ItThisCam2 != visMatPrio2.end() || ItOtherCam2 != other.visMatPrio2.end() )
    {

        if(*ItThisCam2 > *ItOtherCam2)
            ++onlyInThisCam;
        if(*ItThisCam2 < *ItOtherCam2)
            ++onlyInOtherCam;
        if((onlyInThisCam && onlyInOtherCam) !=0) //each camera can see points, which the other can't see --> keep both cameras
        {
            return false;
        }

        // Loop further
        if(ItThisCam2 != visMatPrio2.end())
        {
            ++ItThisCam2;
        }
        if(ItOtherCam2 != other.visMatPrio2.end())
        {
            ++ItOtherCam2;
        }
    }

    if(onlyInThisCam != 0 && onlyInOtherCam ==0)// this cam can see all points, that the other also can see plus some additional
        return true;
    else if(onlyInThisCam ==0 && onlyInOtherCam !=0)// other cam can see points, which this cam can't see
       return false;
    else if(onlyInThisCam ==0 && onlyInOtherCam ==0) //both cameras see exactly the same points --> check other Cam characteristics
    {
        coCoord eulerThis = mat;
        double angleThis = eulerThis.hpr[2];
        coCoord eulerOther = other.mat;
        double angleOther = eulerOther.hpr[2];
        if(angleThis ==0 && angleOther != 0) //rotation around own axis not wanted, because it's unrealistic
            return true;
        else if(angleOther ==0 && angleThis != 0)
            return false;
        else
        {
            double coefThis = std::accumulate(distortionValuePrio1.begin(),distortionValuePrio1.end(),0.0) + std::accumulate(distortionValuePrio2.begin(),distortionValuePrio2.end(),0.0);
            double coefOther = std::accumulate(other.distortionValuePrio1.begin(),other.distortionValuePrio1.end(),0.0) + std::accumulate(other.distortionValuePrio2.begin(),other.distortionValuePrio2.end(),0.0);
            if(coefOther > coefThis)
                return false;
            else
                return true;
        }
    }

}
void CamPosition::compareCams(std::shared_ptr<Cam> newCam)
{
    auto it = allCameras.begin();
    //for (auto it = allCameras.begin(); it != allCameras.end(); it++)
    while(it != allCameras.end())
    {
        if(*it->get() >> *newCam.get())
        {
            drawRemovedCam(newCam);
            return; // a better Cam already exist --> don't need this one!
        }
        if(*newCam.get()>>*it->get())
        {
            if(allCameras.size()>=2)
              replaceCamWithLastElement(std::distance(allCameras.begin(),it));// element is replaced by last element & iterator is not incremented to check same element index again!
            else
              removeCamFromVec(*it);

            continue; // do not increment iterator!
        }
        else
        {
            ++it;
        }

    }
    addCamToVec(newCam);
}

void CamPosition::addCamToVec(std::shared_ptr<Cam> cam)
{
    long refs = cam.use_count();
    osg::Matrix m = cam->getMatrix();
    allCameras.push_back(std::move(cam));
    auto test = allCameras.back()->getID();
    int a =2;
    //now cam is nullptr
}
void CamPosition::removeCamFromVec(std::shared_ptr<Cam> cam)
{
    drawRemovedCam(cam);
    if(!allCameras.empty())
    {
        allCameras.erase(std::remove_if(allCameras.begin(),allCameras.end(),[&cam](std::shared_ptr<Cam>const& it){return cam == it;}));
    }
}
void CamPosition::drawRemovedCam(std::shared_ptr<Cam> cam)
{
    deletedOrientations.push_back(new osg::MatrixTransform);
    searchSpaceGroupDeleted->addChild(deletedOrientations.back().get());
    deletedOrientations.back()->setMatrix(cam->getMatrix());
    deletedOrientations.back()->addChild(deletedOrientationsDrawable->getCamGeode().get());
}

void CamPosition::replaceCamWithLastElement(int index)
{
    deletedOrientations.push_back(new osg::MatrixTransform);
    searchSpaceGroupDeleted->addChild(deletedOrientations.back().get());
    deletedOrientations.back()->setMatrix(allCameras.at(index).get()->getMatrix());
    deletedOrientations.back()->addChild(deletedOrientationsDrawable->getCamGeode().get());
    if(!allCameras.empty())
    {
        allCameras.at(index) = allCameras.back();
        allCameras.pop_back();
    }
}

void CamPosition::createDrawableForEachCamOrientation()
{
    if(!allCameras.empty())
    {
        for(const auto& x :allCameras)
        {
            searchSpace.push_back(new osg::MatrixTransform );
            searchSpaceGroup->addChild(searchSpace.back().get());
            searchSpace.back()->setMatrix(x->getMatrix());
           // searchSpace.back()->setName(std::to_string(nbrOfCameras)+"+MATRIX Z:" + std::to_string( newCoordPlus.hpr[0])+ " X:" +std::to_string( newCoordPlus.hpr[1])+ " Y:" +std::to_string( newCoordPlus.hpr[2]));
            searchSpace.back()->addChild(searchSpaceDrawable->getCamGeode().get());
        }
    }
}
void CamPosition::createDrawableForEachDeletedCamOrientation()
{
    for(const auto& x :deletedOrientations)
    {
        deletedOrientations.push_back(new osg::MatrixTransform );
        searchSpaceGroupDeleted->addChild(deletedOrientations.back().get());
        deletedOrientations.back()->setMatrix(x->getMatrix());
       // searchSpace.back()->setName(std::to_string(nbrOfCameras)+"+MATRIX Z:" + std::to_string( newCoordPlus.hpr[0])+ " X:" +std::to_string( newCoordPlus.hpr[1])+ " Y:" +std::to_string( newCoordPlus.hpr[2]));
        deletedOrientations.back()->addChild(deletedOrientationsDrawable->getCamGeode().get());
    }
}
void CamPosition::deleteSearchSpace()
{
    allCameras.clear();
    auto i = searchSpaceGroup.get()->getNumChildren();
    if(searchSpaceGroup.get()->getNumChildren() !=0)
        searchSpaceGroup->removeChildren(0,searchSpaceGroup.get()->getNumChildren());
    searchSpace.clear();

    if(searchSpaceGroupDeleted.get()->getNumChildren() !=0)
         searchSpaceGroupDeleted->removeChildren(0,searchSpaceGroupDeleted.get()->getNumChildren());
    deletedOrientations.clear();


}

void CamPosition::updateCamMatrixes()
{
 /*   std::cout<<"update cameras"<<std::endl;
    if(allCameras.empty())
    {
        int count = 0;
        for(const auto& x :searchSpace)
        {
            count++;
            std::string name = std::to_string(count);

            osg::Matrix total=x->getMatrix()*viewpointInteractor->getMatrix();
            total.setTrans(viewpointInteractor->getMatrix().getTrans());
            coCoord eulerNew = total;
            std::shared_ptr<Cam> camera = std::make_shared<Cam>(eulerNew,visMat,name);

            allCameras.push_back(std::move(camera));
        }
    }
    else
    {
        int cnt =0; //important!!
        for(const auto& x :searchSpace)
        {
            osg::Matrix total=x->getMatrix()*viewpointInteractor->getMatrix();
            total.setTrans(viewpointInteractor->getMatrix().getTrans());
            coCoord eulerNew = total;
            allCameras.at(cnt)->setPosition(eulerNew,visMat);
          //  std::cout<<"total"<<std::endl;
          //  printCoCoord(eulerNew);
            cnt++;

        }
    }

*/


    //std::cout<<"All cam positions are updated!"<<std::endl;
}
void CamPosition::updateVisibleCam()
{
    //update pos of camDraw
    coCoord euler =localDCS->getMatrix();
    //new intersection Test for this CamPos
    calcIntersection();
    camDraw->cam->setPosition(euler,visMat);
}

void CamPosition::setSearchSpaceState(bool state)
{
    if(state)
        switchNode->setChildValue(searchSpaceGroup.get(),true);
    else
        switchNode->setChildValue(searchSpaceGroup.get(),false);
}
void CamPosition::setDeletedSearchSpaceState(bool state)
{
    if(state)
        switchNode->setChildValue(searchSpaceGroupDeleted.get(),true);
    else
        switchNode->setChildValue(searchSpaceGroupDeleted.get(),false);
}

void CamPosition::calcIntersection()
{
   osg::Timer_t startTick = osg::Timer::instance()->tick();
   visMat.clear();
   osg::ref_ptr<osgUtil::IntersectorGroup> intersectorGroup = new osgUtil::IntersectorGroup();
   osg::Vec3 start = this->getPosition();
   std::vector<double> nbrPointPerSZ;
   for(const auto& x : EKU::safetyZones)
   {
       nbrPointPerSZ.push_back(x->getWorldPosOfAllObservationPoints().size());

       for(const auto& end : x->getWorldPosOfAllObservationPoints())
       {
            osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
            intersectorGroup->addIntersector( intersector.get() );
       }
   }

  osgUtil::IntersectionVisitor visitor(intersectorGroup.get());
  cover->getObjectsRoot()->accept(visitor);
  register int count = 0;
  register int counterSZ = 0;
  std::vector<double> visMatForThisSafetyZone;
  osgUtil::IntersectorGroup::Intersectors& intersectors = intersectorGroup->getIntersectors();
  for(osgUtil::IntersectorGroup::Intersectors::iterator intersector_itr = intersectors.begin();
                  intersector_itr != intersectors.end();
                  ++intersector_itr)
    {
        osgUtil::LineSegmentIntersector* lsi = dynamic_cast<osgUtil::LineSegmentIntersector*>(intersector_itr->get());

        if(lsi)
        {
            osgUtil::LineSegmentIntersector::Intersections& hits = lsi->getIntersections();
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
                visMatForThisSafetyZone.push_back(0);
            else
                visMatForThisSafetyZone.push_back(1);

            count++;
            if(count == nbrPointPerSZ.at(counterSZ))
            {
                count =0;
                counterSZ++;
                visMat.push_back(visMatForThisSafetyZone);
                visMatForThisSafetyZone.clear();
            }
          //  std::cout <<count<<std::endl;
        }

    }
   osg::Timer_t endTick = osg::Timer::instance()->tick();
/*   std::cout<<"nbr SZ: "<<visMat.size()<<std::endl;
   std::cout<<"Completed in "<<osg::Timer::instance()->delta_s(startTick,endTick)<<std::endl;
   std::cout<<"Camposition : "<<std::endl;
   for(const auto& x : visMat)
   {
       std::cout<<"Zone (size: "<<x.size()<<") :";
       for(const auto& x1 : x)
           std::cout<<x1<<", ";
   std::cout<<" " <<std::endl;
   }
*/
}




/*void CamPosition::createCams(bool rotx, bool roty, bool rotz)
{


 //   calcVisibility();
}
*/


















