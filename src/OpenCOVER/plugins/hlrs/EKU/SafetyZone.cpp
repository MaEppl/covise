#include <SafetyZone.h>
#include <EKU.h>
using namespace opencover;
#include <algorithm>

size_t SafetyZone:: count = 0;
osg::Vec3 calcDirectionVec(osg::Matrix &m)
{
    osg::Matrix rotation = osg::Matrix::rotate(m.getRotate());
    osg::Vec3 direction = osg::Vec3{0,-1,0} *rotation; //{0,1,0} is init Vec of coVR3DTransRotInteractor
    direction.normalize();
//    std::cout<<"Direction Vector: "<<direction.x()<<", "<<direction.y()<<", "<<direction.z()<<std::endl;
    return direction;
}

SafetyZone::SafetyZone(osg::Matrix m, Priority priority, float length = 2, float width =2, float height = 8):priority(priority),length(length),width(width),height(height)
{

    count++;
    fprintf(stderr, "new SafetyZone\n");

    name = "Zone "+std::to_string(SafetyZone::count);
    if(priority == PRIO1)
        color = {0.5 ,0.0, 0.0,1};
    else if(priority == PRIO2)
        color = {0.9,0.6,0.0,1.0};

    localDCS =new osg::MatrixTransform();
    localDCS->setMatrix(m);
    localDCS->setName(name);
    safetyZoneGeode = plotSafetyZone();
    safetyZoneGeode->setName("Wireframe");
    localDCS->addChild(safetyZoneGeode);

    //create points at vertices
    createPoints();

    // interactors
    float _interSize = cover->getSceneSize() / 25;

    interactor= new coVR3DTransRotInteractor(m, _interSize/2, vrui::coInteraction::ButtonA, "hand", "sizeInteractor", vrui::coInteraction::Medium);
    interactor->show();
    interactor->enableIntersection();


    osg::Vec3 startPosY= verts->at(7)*m;
    sizeYInteractor = new coVR3DTransInteractor(startPosY, _interSize/2, vrui::coInteraction::ButtonA, "hand", "sizeInteractor", vrui::coInteraction::Medium);
    sizeYInteractor->show();
    sizeYInteractor->enableIntersection();

    osg::Vec3 startPosX = verts->at(5)*m;
    sizeXInteractor = new coVR3DTransInteractor(startPosX, _interSize/2, vrui::coInteraction::ButtonA, "hand", "sizeInteractor", vrui::coInteraction::Medium);
    sizeXInteractor->show();
    sizeXInteractor->enableIntersection();

    osg::Vec3 startPosPreferredDirection = osg::Vec3(verts->at(6).x()-length/2,verts->at(6).y()+width/2,height/2)*m;
    osg::Quat q;
    q.makeRotate(osg::DegreesToRadians(-90.0),osg::X_AXIS);
    osg::Matrix local;
    local.setTrans(startPosPreferredDirection);
    local.setRotate(q);
    preferredDirectionInteractor = new coVR3DTransRotInteractor(local,_interSize/3,vrui::coInteraction::ButtonA, "hand", "preferredDirectionInteractor", vrui::coInteraction::Medium);
    preferredDirectionInteractor->hide();
    preferredDirectionInteractor->disableIntersection();
    preferredDirection =  calcDirectionVec(local);

    osg::Quat q2;
    q2.makeRotate(osg::DegreesToRadians(90.0),osg::X_AXIS);
    local.setRotate(q2);
    preferredDirectionInteractor2 = new coVR3DTransRotInteractor(local,_interSize/3,vrui::coInteraction::ButtonA, "hand", "preferredDirectionInteractor2", vrui::coInteraction::Medium);
    preferredDirectionInteractor2->hide();
    preferredDirectionInteractor2->disableIntersection();

    updateWorldPosOfAllObservationPoints();
}
osg::Geode* SafetyZone::plotSafetyZone()
{
    osg::Geode* geode = new osg::Geode();
    geode->setName("Wireframe");
    geom = new osg::Geometry();
    osg::StateSet *stateset = geode->getOrCreateStateSet();
    setStateSet(stateset);
    //necessary for dynamic redraw (command:dirty)
    geom->setDataVariance(osg::Object::DataVariance::DYNAMIC) ;
    geom->setUseDisplayList(false);
    geom->setColorBinding(Geometry::BIND_OVERALL);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geom->setUseVertexBufferObjects(true);
    geode->addDrawable(geom);
    geode->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    geode->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    // Declare an array of vertices to create a simple pyramid.
    verts = new osg::Vec3Array;
    verts->push_back( osg::Vec3( -length, width, -1 ) ); // lower back left
    verts->push_back( osg::Vec3( -length,0, -1 ) );// lower front left
    verts->push_back( osg::Vec3(  0,0, -1 ) );// lower front right
    verts->push_back( osg::Vec3(  0, width, -1 ) ); // lower back right
    verts->push_back( osg::Vec3( -length, width,  1 ) ); // upper back left
    verts->push_back( osg::Vec3( -length,0,  1 ) );// upper front left
    verts->push_back( osg::Vec3(  0,0,  1 ) );// upper front right
    verts->push_back( osg::Vec3(  0, width,  1) ); // upper back right

    // Associate this set of vertices with the Geometry.
    geom->setVertexArray(verts);

    //set normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back((osg::Vec3(0.0,0.0,-1.0)));
    geom->setNormalArray(normals);
    // Next, create primitive sets and add them to the Geometry.
    // Each primitive set represents a Line of the Wireframe

    //Lower Rectangle
    osg::DrawElementsUInt* line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(0);
    line->push_back(1);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(1);
    line->push_back(2);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(2);
    line->push_back(3);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(3);
    line->push_back(0);
    geom->addPrimitiveSet(line);

    //UpperRectangle
    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(4);
    line->push_back(5);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(5);
    line->push_back(6);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(6);
    line->push_back(7);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(7);
    line->push_back(4);
    geom->addPrimitiveSet(line);

    //Vertical Lines
    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(0);
    line->push_back(4);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(1);
    line->push_back(5);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(2);
    line->push_back(6);
    geom->addPrimitiveSet(line);

    line = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0,2);
    line->push_back(3);
    line->push_back(7);
    geom->addPrimitiveSet(line);



    LineWidth *lw = new LineWidth(3.0);
    stateset->setAttribute(lw);
    return geode;
}
void SafetyZone::setStateSet(osg::StateSet *stateSet)
{
    osg::Material *material = new osg::Material();
    material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);//0.5
    material->setSpecular(Material::FRONT_AND_BACK, Vec4(0.9f, 0.9f, 0.9f, 1.f));
    material->setAmbient(Material::FRONT_AND_BACK, Vec4(0.2f, 0.2f, 0.2f, 1.f));
    material->setEmission(Material::FRONT_AND_BACK, Vec4(0.0f, 0.0f, 0.0f, 1.f));
    material->setShininess(Material::FRONT_AND_BACK, 16.f);
    material->setTransparency(Material::FRONT_AND_BACK, 0.0f); // Noch Wert anpassen fuer Transparency
    osg::LightModel *defaultLm;
    defaultLm = new osg::LightModel();
    defaultLm->setLocalViewer(true);
    defaultLm->setTwoSided(true);
    defaultLm->setColorControl(osg::LightModel::SINGLE_COLOR);
    stateSet->setAttributeAndModes(material, osg::StateAttribute::ON);
    stateSet->setAttributeAndModes(defaultLm, osg::StateAttribute::ON);
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);


}
void SafetyZone::createPoints()
{
    std::unique_ptr<Point> p6(new Point(verts.get()->at(6)-osg::Vec3(0,0,height/2),color));
    std::vector<std::unique_ptr<Point>> rowFirst;
    rowFirst.push_back(std::move(p6));
    points.push_back(std::move(rowFirst));

    //create Vectors
    double startValueY = distance;
    double diffy = verts.get()->at(7).y()-verts.get()->at(6).y();
    while(startValueY < width)
    {
        std::vector<std::unique_ptr<Point>> rowUp;
        if(diffy<0){
            std::unique_ptr<Point> newPointUp(new Point(verts.get()->at(6)+osg::Vec3(0,-startValueY,-height/2),color));
            rowUp.push_back(std::move(newPointUp));
            points.push_back(std::move(rowUp));
        }
        else
        {
            std::unique_ptr<Point> newPointUp(new Point(verts.get()->at(6)+osg::Vec3(0,startValueY,-height/2),color));
            rowUp.push_back(std::move(newPointUp));
            points.push_back(std::move(rowUp));
        }
        startValueY +=distance;
    }
    //add last row from point 7
    std::unique_ptr<Point> p7(new Point(verts.get()->at(7)-osg::Vec3(0,0,height/2),color));
    std::vector<std::unique_ptr<Point>> rowLast;
    rowLast.push_back(std::move(p7));
    points.push_back(std::move(rowLast));



    // add points to each existing Vector
    double startValueX = distance;
    double diffx = verts.get()->at(5).x()-verts.get()->at(6).x();

    while(startValueX < length)
    {
        for(auto &x : points)
        {
            if(diffx>0)
            {
                osg::Vec3 newPos = x.front()->getPos()-osg::Vec3(-startValueX,0,0.0);
                std::unique_ptr<Point> newPoint(new Point(newPos,color));
                x.push_back(std::move(newPoint));
            }
            else
            {
                osg::Vec3 newPos = x.front()->getPos()-osg::Vec3(startValueX,0,0.0);
                std::unique_ptr<Point> newPoint(new Point(newPos,color));
                x.push_back(std::move(newPoint));
            }
        }

        startValueX +=distance;
    }

    //add last Point to each row
    for(auto &x : points)
    {
       std::unique_ptr<Point> lastPoint(new Point(osg::Vec3(verts.get()->at(5).x(),x.front()->getPos().y(),0),color));//warum hier height?
       x.push_back(std::move(lastPoint));
    }

    //visualize all points
    for(const auto& x : points )
    {
        for(const auto& x1 :x)
            localDCS->addChild(x1->getPoint());

    }

    std::cout<<"number of vectors: "<<points.size()<<std::endl;
    std::cout<<"number of elements in vetor: "<<points.back().size()<<std::endl;
}

void SafetyZone::preFrame()
{
    interactor->preFrame();
    sizeYInteractor->preFrame();
    sizeXInteractor->preFrame();
    preferredDirectionInteractor->preFrame();
    preferredDirectionInteractor2->preFrame();

    static osg::Vec3 startPos_Interactor_w, startPos_YInteractor_w, startPos_XInteractor_w,startPos_PDInteractor_w;
    static osg::Vec3 startPos_Interactor_o, startPos_YInteractor_o, startPos_XInteractor_o,startPos_PDInteractor_o;
    if(interactor->wasStarted())
    {

        osg::Matrix interactor_to_w = interactor->getMatrix();
        startPos_YInteractor_w = sizeYInteractor->getPos();
        startPos_XInteractor_w = sizeXInteractor->getPos();
        startPos_PDInteractor_w = preferredDirectionInteractor->getMatrix().getTrans();

        osg::Vec3 interactor_pos_w = interactor_to_w.getTrans();
        startPos_YInteractor_o= Matrix::transform3x3(startPos_YInteractor_w-interactor_pos_w, interactor_to_w.inverse(interactor_to_w));
        startPos_XInteractor_o= Matrix::transform3x3(startPos_XInteractor_w-interactor_pos_w, interactor_to_w.inverse(interactor_to_w));
        startPos_PDInteractor_o=Matrix::transform3x3(startPos_PDInteractor_w-interactor_pos_w, interactor_to_w.inverse(interactor_to_w));
    }
    if(interactor->isRunning())
    {
        //update Interactors
        osg::Matrix interactor_to_w = interactor->getMatrix();
        localDCS->setMatrix(interactor_to_w);
        osg::Vec3 interactor_pos_w;
        interactor_pos_w = interactor_to_w.getTrans();

        osg::Vec3 sizeYInteractor_pos_w = Matrix::transform3x3(startPos_YInteractor_o, interactor_to_w);
        sizeYInteractor_pos_w +=interactor_pos_w;
        sizeYInteractor->updateTransform(sizeYInteractor_pos_w);

        osg::Vec3 sizeXInteractor_pos_w = Matrix::transform3x3(startPos_XInteractor_o, interactor_to_w);
        sizeXInteractor_pos_w +=interactor_pos_w;
        sizeXInteractor->updateTransform(sizeXInteractor_pos_w);

        osg::Vec3 preferredDirectionInteractor_pos_w = Matrix::transform3x3(startPos_PDInteractor_o, interactor_to_w);
        preferredDirectionInteractor_pos_w +=interactor_pos_w;
        osg::Matrix m = preferredDirectionInteractor->getMatrix();
        m.setTrans(preferredDirectionInteractor_pos_w);
        preferredDirectionInteractor->updateTransform(m);

        osg::Quat q;
        q.makeRotate(osg::DegreesToRadians(180.0),osg::X_AXIS);
        osg::Matrix m2;
        m2.setRotate(q);
        preferredDirectionInteractor2->updateTransform(m2*m);


    }
    if(interactor->wasStopped())
    {
        updateWorldPosOfAllObservationPoints();
    }

    if(sizeYInteractor->wasStarted())
    {
        //remove all points
        for(const auto& x : points )
        {
            for(const auto& x1 :x)
                localDCS->removeChild(x1->getPoint());

        }
        points.clear();

        osg::Matrix interactor_to_w = interactor->getMatrix();
        startPos_YInteractor_w = sizeYInteractor->getPos();
        startPos_YInteractor_o= Matrix::transform3x3(startPos_YInteractor_w, interactor_to_w.inverse(interactor_to_w));
        startPos_Interactor_w = interactor->getMatrix().getTrans();
        startPos_Interactor_o= Matrix::transform3x3(startPos_Interactor_w, interactor_to_w.inverse(interactor_to_w));


    }

    if(sizeYInteractor->isRunning())
    {
        //restrict directions of sizeYInteractor
        osg::Matrix interactor_to_w = interactor->getMatrix();
        osg::Vec3 sizeYInteractor_pos_o = Matrix::transform3x3(sizeYInteractor->getPos(),interactor_to_w.inverse(interactor_to_w));
        sizeYInteractor_pos_o.x() = startPos_YInteractor_o.x(); //set x value to old x value
        sizeYInteractor_pos_o.z() = startPos_YInteractor_o.z(); //set z value to old z value
        osg::Vec3 sizeYInteractor_pos_w = Matrix::transform3x3(sizeYInteractor_pos_o,interactor_to_w);
        sizeYInteractor->updateTransform(sizeYInteractor_pos_w);

        //update vertices
        osg::Vec3 posY_o= Matrix::transform3x3(sizeYInteractor->getPos()-interactor_to_w.getTrans(), interactor_to_w.inverse(interactor_to_w));
        updateGeometryY(posY_o.y());

        //update PDInteractor (position is always in center of SZ)
        osg::Matrix preferredDirectionInteractor_to_w=preferredDirectionInteractor->getMatrix();
        double diffy = verts.get()->at(7).y()-verts.get()->at(6).y();
        double diffx = verts.get()->at(5).x()-verts.get()->at(6).x();
        if(diffy>0 && diffx<0 )
        {
            osg::Vec3 center_w =Matrix::transform3x3(osg::Vec3(length/2,width/2,height/2),interactor_to_w);
            preferredDirectionInteractor_to_w.setTrans( sizeYInteractor_pos_w-center_w);
        }
        else if(diffy>0 && diffx>0 )
        {
            osg::Vec3 center_w =Matrix::transform3x3(osg::Vec3(-length/2,width/2,height/2),interactor_to_w);
            preferredDirectionInteractor_to_w.setTrans( sizeYInteractor_pos_w-center_w);
        }
        else if(diffy<0 && diffx>0)
        {
            osg::Vec3 center_w =Matrix::transform3x3(osg::Vec3(-length/2,-width/2,height/2),interactor_to_w);
            preferredDirectionInteractor_to_w.setTrans( sizeYInteractor_pos_w-center_w);
        }

        else
        {
            osg::Vec3 center_w =Matrix::transform3x3(osg::Vec3(length/2,-width/2,height/2),interactor_to_w);
            preferredDirectionInteractor_to_w.setTrans( sizeYInteractor_pos_w-center_w);
        }
        preferredDirectionInteractor->updateTransform(preferredDirectionInteractor_to_w);

        osg::Quat q;
        q.makeRotate(osg::DegreesToRadians(180.0),osg::X_AXIS);
        osg::Matrix rotPD2;
        rotPD2.setRotate(q);

        preferredDirectionInteractor2->updateTransform(rotPD2 * preferredDirectionInteractor_to_w);


    }
    if(sizeYInteractor->wasStopped())
    {
        createPoints();
        updateWorldPosOfAllObservationPoints();
    }

    if(sizeXInteractor->wasStarted())
    {
        //remove all points
        for(const auto& x : points )
        {
            for(const auto& x1 :x)
                localDCS->removeChild(x1->getPoint());

        }
        points.clear();

        osg::Matrix interactor_to_w = interactor->getMatrix();
        startPos_XInteractor_w = sizeXInteractor->getPos();
        startPos_XInteractor_o= Matrix::transform3x3(startPos_XInteractor_w, interactor_to_w.inverse(interactor_to_w));
        startPos_Interactor_w = interactor->getMatrix().getTrans();
        startPos_Interactor_o= Matrix::transform3x3(startPos_Interactor_w, interactor_to_w.inverse(interactor_to_w));

    }

    if(sizeXInteractor->isRunning())
    {
        //restrict directions of sizeXInteractor
        osg::Matrix interactor_to_w = interactor->getMatrix();
        osg::Vec3 sizeXInteractor_pos_o = Matrix::transform3x3(sizeXInteractor->getPos(),interactor_to_w.inverse(interactor_to_w));
        sizeXInteractor_pos_o.y() = startPos_XInteractor_o.y(); //set y value to old y value
        sizeXInteractor_pos_o.z() = startPos_XInteractor_o.z(); //set z value to old z value
        osg::Vec3 sizeXInteractor_pos_w = Matrix::transform3x3(sizeXInteractor_pos_o,interactor_to_w);
        sizeXInteractor->updateTransform(sizeXInteractor_pos_w);

        //update vertices
        osg::Vec3 posX_o= Matrix::transform3x3(sizeXInteractor->getPos()-interactor->getMatrix().getTrans(), interactor->getMatrix().inverse(interactor->getMatrix()));
        updateGeometryX(posX_o.x());

        //update PDInteractor (position is always in center of SZ)
        osg::Matrix preferredDirectionInteractor_to_w=preferredDirectionInteractor->getMatrix();
        double diffx = verts.get()->at(5).x()-verts.get()->at(6).x();
        double diffy = verts.get()->at(7).y()-verts.get()->at(6).y();

        if(diffx<0 && diffy >0)
        {
            osg::Vec3 center_w =Matrix::transform3x3(osg::Vec3(length/2,width/2,-height/2),interactor_to_w);
            preferredDirectionInteractor_to_w.setTrans( sizeXInteractor_pos_w+center_w);
        }
        else if(diffx<0 && diffy <0)
        {
            osg::Vec3 center_w =Matrix::transform3x3(osg::Vec3(length/2,-width/2,-height/2),interactor_to_w);
            preferredDirectionInteractor_to_w.setTrans( sizeXInteractor_pos_w+center_w);
        }
        else if(diffx>0 && diffy >0)
        {
            osg::Vec3 center_w =Matrix::transform3x3(osg::Vec3(length/2,-width/2,height/2),interactor_to_w);
            preferredDirectionInteractor_to_w.setTrans( sizeXInteractor_pos_w-center_w);
        }
        else if(diffx>0 && diffy <0)
        {
            osg::Vec3 center_w =Matrix::transform3x3(osg::Vec3(length/2,width/2,height/2),interactor_to_w);
            preferredDirectionInteractor_to_w.setTrans( sizeXInteractor_pos_w-center_w);
        }
        preferredDirectionInteractor->updateTransform(preferredDirectionInteractor_to_w);

        osg::Quat q;
        q.makeRotate(osg::DegreesToRadians(180.0),osg::X_AXIS);
        osg::Matrix rotPD2;
        rotPD2.setRotate(q);

        preferredDirectionInteractor2->updateTransform(rotPD2 * preferredDirectionInteractor_to_w);
    }
    if(sizeXInteractor->wasStopped())
    {
        createPoints();
        updateWorldPosOfAllObservationPoints();
    }

    if(preferredDirectionInteractor->isRunning())
    {
        osg::Matrix m = preferredDirectionInteractor->getMatrix();
        osg::Quat q;
        q.makeRotate(osg::DegreesToRadians(180.0),osg::X_AXIS);
        osg::Matrix rotPD2;
        rotPD2.setRotate(q);

        preferredDirectionInteractor2->updateTransform(rotPD2*m);
    }

    if(preferredDirectionInteractor->wasStopped())
    {
        osg::Matrix m = preferredDirectionInteractor->getMatrix();
        preferredDirection = calcDirectionVec(m);
       // std::cout<<"PD: "<<preferredDirection.x()<<" "<<preferredDirection.y()<<" "<<preferredDirection.z()<<std::endl;

    }
    if(preferredDirectionInteractor2->isRunning())
    {
        osg::Matrix m = preferredDirectionInteractor2->getMatrix();
        osg::Quat q;
        q.makeRotate(osg::DegreesToRadians(-180.0),osg::X_AXIS);
        osg::Matrix rotPD;
        rotPD.setRotate(q);

        preferredDirectionInteractor->updateTransform(rotPD*m);
    }

    if(preferredDirectionInteractor2->wasStopped())
    {
        osg::Matrix m = preferredDirectionInteractor->getMatrix();
        preferredDirection = calcDirectionVec(m);
      //  std::cout<<"PD: "<<preferredDirection.x()<<" "<<preferredDirection.y()<<" "<<preferredDirection.z()<<std::endl;
    }

}



void SafetyZone::updateGeometryY(double y)
{

     verts->at(3) =osg::Vec3(verts->at(3).x(),y,verts->at(3).z());
     verts->at(4) =osg::Vec3(verts->at(4).x(),y,verts->at(4).z());
     verts->at(7) =osg::Vec3(verts->at(7).x(),y,verts->at(7).z());
     verts->at(0) =osg::Vec3(verts->at(0).x(),y,verts->at(0).z());
     verts->dirty();
     geom->dirtyBound();
     width = std::abs(verts->at(7).y()-verts->at(6).y());

}
void SafetyZone::updateGeometryX(double x)
{
    verts->at(0) =osg::Vec3(x,verts->at(0).y(),verts->at(0).z());
    verts->at(4) =osg::Vec3(x,verts->at(4).y(),verts->at(4).z());
    verts->at(5) =osg::Vec3(x,verts->at(5).y(),verts->at(5).z());
    verts->at(1) =osg::Vec3(x,verts->at(1).y(),verts->at(1).z());
    verts->dirty();
    geom->dirtyBound();
    length= std::abs(verts->at(5).x()-verts->at(6).x());
}


void SafetyZone::setPointDistance(double dist)
{
    //remove all points
    for(const auto& x : points )
    {
        for(const auto& x1 :x)
            localDCS->removeChild(x1->getPoint());

    }
    points.clear();

    distance = dist;

    createPoints();
    updateWorldPosOfAllObservationPoints();

}
SafetyZone::~SafetyZone()
{
    delete interactor;
    delete sizeXInteractor;
    delete sizeYInteractor;
    delete preferredDirectionInteractor;
    delete preferredDirectionInteractor2;

    points.clear();
    localDCS->getParent(0)->removeChild(localDCS.get());

    std::cout<<"Removed Safety Zone"<<std::endl;
}

void SafetyZone::updateWorldPosOfAllObservationPoints()
{
    worldPosOfAllObservationPoints.clear();
    nbrControlPoints =0;
    for(const auto& x : points )
    {
        nbrControlPoints += x.size();
        for(const auto& x1 :x)
        {
            osg::Vec3 pos = x1->getPos()*localDCS.get()->getMatrix();
            worldPosOfAllObservationPoints.push_back(pos);
        }
    }

    EKU::updateNbrPoints();
}
void SafetyZone::updateColor(const std::vector<double>& update)const
{
    const int nbrOfPointsInInput = update.size();
    const int nbrOfPointsInVector = points.front().size();
    int counter =0;
    //conter vector<double> to corresponding vector<vector<double>>
    for(int i =0;i<nbrOfPointsInInput;i++)
    {
        if(update.at(i) !=0)//if point is visible
            points.at(i/nbrOfPointsInVector).at(counter)->updateColor();
        counter++;
        if(counter == nbrOfPointsInVector)
        {
            counter =0;
        }
    }
}
void SafetyZone::pointsVisibleForEnoughCameras(const std::vector<double>& update)const
{
    if(!update.empty())
    {
        const int nbrOfPointsInInput = update.size();
        const int nbrOfPointsInVector = points.front().size();
        int counter =0;
        //conter vector<double> to corresponding vector<vector<double>>
        for(int i =0;i<nbrOfPointsInInput;i++)
        {
            if(update.at(i) ==1)//if point is not visible
                points.at(i/nbrOfPointsInVector).at(counter)->visible(false);
            else
                points.at(i/nbrOfPointsInVector).at(counter)->visible(true);
            counter++;
            if(counter == nbrOfPointsInVector)
            {
                counter =0;
            }
        }
    }
    else // no point is visible
    {
        for(const auto& x :points)
        {
            for(const auto & x1 :x)
                x1->visible(false);
        }
    }
}

void SafetyZone::resetColor(const std::vector<double>& reset)const
{
    for(const auto& x : points )
    {
        for(const auto& x1 :x)
        {
            if(x1 !=0)
                x1->resetColor();
        }

    }
}
void SafetyZone::changeInteractorStatus(bool status)
{
    if(status == true)
    {
        interactor->show();
        interactor->enableIntersection();
        sizeYInteractor->show();
        sizeYInteractor->enableIntersection();
        sizeXInteractor->show();
        sizeXInteractor->enableIntersection();
        preferredDirectionInteractor->enableIntersection();
        preferredDirectionInteractor2->enableIntersection();


    }
    else
    {
        interactor->hide();
        interactor->disableIntersection();
        sizeYInteractor->hide();
        sizeYInteractor->disableIntersection();
        sizeXInteractor->hide();
        sizeXInteractor->disableIntersection();
        preferredDirectionInteractor->disableIntersection();
        preferredDirectionInteractor2->disableIntersection();


    }
}
Point::Point(osg::Vec3 pos,osg::Vec4 color):color(color),visibleForEnoughCameras(true)
{
    osg::Matrix local;
    local.setTrans(pos);
    localDCS = new osg::MatrixTransform();
    localDCS->setMatrix(local);
    localDCS->setName("Translation");
    sphere = new osg::Sphere(osg::Vec3(0,0,0), 0.45);
    sphereDrawable = new osg::ShapeDrawable(sphere);
    //red color
    sphereDrawable->setColor(color);
    geode = new osg::Geode();
    osg::StateSet *mystateSet = geode->getOrCreateStateSet();
    setStateSet(mystateSet);
    geode->setName("Point");
    geode->addDrawable(sphereDrawable);

    localDCS->addChild(geode.get());
}
Point::~Point()
{

}
void Point::setStateSet(osg::StateSet *stateSet)
{
    osg::Material *material = new osg::Material();
    material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    material->setSpecular(Material::FRONT_AND_BACK, Vec4(0.9f, 0.9f, 0.9f, 1.f));
    material->setAmbient(Material::FRONT_AND_BACK, Vec4(0.2f, 0.2f, 0.2f, 1.f));
    material->setEmission(Material::FRONT_AND_BACK, Vec4(0.0f, 0.0f, 0.0f, 1.f));
    material->setShininess(Material::FRONT_AND_BACK, 16.f);
    osg::LightModel *defaultLm;
    defaultLm = new osg::LightModel();
    defaultLm->setLocalViewer(true);
    defaultLm->setTwoSided(true);
    defaultLm->setColorControl(osg::LightModel::SINGLE_COLOR);
    stateSet->setAttributeAndModes(material, osg::StateAttribute::ON);
    stateSet->setAttributeAndModes(defaultLm, osg::StateAttribute::ON);
}
void Point::setPos(osg::Vec3 p)
{
    osg::Matrix tmp;
    tmp.setTrans(p);
    localDCS->setMatrix(tmp);
}
void Point::updateColor()
{
    sphereDrawable->setColor(osg::Vec4(0.0,1.0,0.0,1.0));
}
void Point::resetColor()
{
    if(visibleForEnoughCameras)
        sphereDrawable->setColor(color);
    else
        sphereDrawable->setColor(osg::Vec4{0.0,0.0,1.0,1});

}
void Point::visible(bool input)
{
    if(!input)
    {
        visibleForEnoughCameras=false;
        sphereDrawable->setColor(osg::Vec4{0.0,0.0,1.0,1});
    }
    else
    {
        visibleForEnoughCameras =true;
        resetColor();
    }


}
