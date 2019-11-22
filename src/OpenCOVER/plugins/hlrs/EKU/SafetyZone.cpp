#include <SafetyZone.h>

using namespace opencover;
#include <algorithm>
void restrictTranslation(coCoord startPos,osg::Matrix &mat, bool noX,  bool noY, bool noZ)
{
    coCoord coord;
    coord = mat;

    if (noX)
    {
          coord.xyz[0] =startPos.xyz[0];

    }
    if(noY)
    {
         coord.xyz[1] =startPos.xyz[1];
    }
    if(noZ)
    {
         coord.xyz[2] =startPos.xyz[2];
    }

   coord.makeMat(mat);
}

size_t SafetyZone:: count = 0;

SafetyZone::SafetyZone(osg::Matrix m, Priority priority, float length = 2, float width =2, float height = 8):mat(m),priority(priority),length(length),width(width),height(height)
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
    safetyZoneGeode = plotSafetyZone(m);
    safetyZoneGeode->setName("Wireframe");
    localDCS->addChild(safetyZoneGeode);

    //create points at vertices
    createPoints();

 /*   pointsVerts.reserve(8);
    for(int i =0;i<=7;i++)
    {
        std::unique_ptr<Point> newPoint(new Point(verts->at(i)));
        pointsVerts.push_back(std::move(newPoint));
//      localDCS->addChild(pointsVerts.back()->getPoint());

    }
 */   //create Points inside

    // interactors
    float _interSize = cover->getSceneSize() / 25;

    interactor= new coVR3DTransRotInteractor(m, _interSize/3, vrui::coInteraction::ButtonA, "hand", "sizeInteractor", vrui::coInteraction::Medium);
    interactor->show();
    interactor->enableIntersection();


    osg::Vec3 startPosY=osg::Vec3(length/2,0,verts->at(7).z())*m;
    sizeYInteractor = new coVR3DTransInteractor(startPosY, _interSize/2, vrui::coInteraction::ButtonA, "hand", "sizeInteractor", vrui::coInteraction::Medium);
    sizeYInteractor->hide();
    sizeYInteractor->disableIntersection();

    osg::Vec3 startPosX = osg::Vec3(0,width/2,verts->at(7).z())*m;
    sizeXInteractor = new coVR3DTransInteractor(startPosX, _interSize/2, vrui::coInteraction::ButtonA, "hand", "sizeInteractor", vrui::coInteraction::Medium);
    sizeXInteractor->hide();
    sizeXInteractor->disableIntersection();

    distanceInteractors = startPosY-startPosX;
    distanceX = verts->at(7).x() - startPosX.x();
    distanceY = verts->at(7).x() - startPosY.y();

    updateWorldPosOfAllObservationPoints();

    //User Interaction
 /*  myinteraction = new vrui::coTrackerButtonInteraction(vrui::coInteraction::AllButtons, "MoveMode", vrui::coInteraction::Medium);
    interActing = false;
    aSensor = new mySensor(SafetyZoneGeode, name, myinteraction);
    sensorList.append(aSensor);
  */
}
osg::Geode* SafetyZone::plotSafetyZone(osg::Matrix m)
{
    osg::Geode* geode = new osg::Geode();
    geode->setName("Wireframe");
    osg::Geometry* geom = new osg::Geometry();
    osg::StateSet *stateset = geode->getOrCreateStateSet();
    setStateSet(stateset);
    //necessary for dynamic redraw (command:dirty)
    geom->setDataVariance(osg::Object::DataVariance::DYNAMIC) ;
    geom->setUseDisplayList(false);
    geom->setUseVertexBufferObjects(true);
    geode->addDrawable(geom);
    geode->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    geode->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    // Declare an array of vertices to create a simple pyramid.
    verts = new osg::Vec3Array;
    verts->push_back( osg::Vec3( -length/2, width/2, -1 ) ); // lower back left
    verts->push_back( osg::Vec3( -length/2,-width/2, -1 ) );// lower front left
    verts->push_back( osg::Vec3(  length/2,-width/2, -1 ) );// lower front right
    verts->push_back( osg::Vec3(  length/2, width/2, -1 ) ); // lower back right
    verts->push_back( osg::Vec3( -length/2, width/2,  1 ) ); // upper back left
    verts->push_back( osg::Vec3( -length/2,-width/2,  1 ) );// upper front left
    verts->push_back( osg::Vec3(  length/2,-width/2,  1 ) );// upper front right
    verts->push_back( osg::Vec3(  length/2, width/2,  1) ); // upper back right


    // Associate this set of vertices with the Geometry.
    geom->setVertexArray(verts);

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



    LineWidth *lw = new LineWidth(2.0);
    stateset->setAttribute(lw);
    return geode;
}
void SafetyZone::setStateSet(osg::StateSet *stateSet)
{
    osg::Material *material = new osg::Material();
    material->setColorMode(osg::Material::DIFFUSE);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);//0.5
    osg::LightModel *defaultLm;
    defaultLm = new osg::LightModel();
    defaultLm->setLocalViewer(true);
    defaultLm->setTwoSided(true);
    defaultLm->setColorControl(osg::LightModel::SINGLE_COLOR);
    stateSet->setAttributeAndModes(material, osg::StateAttribute::ON);
    stateSet->setAttributeAndModes(defaultLm, osg::StateAttribute::ON);
}
void SafetyZone::createPoints()
{
    std::unique_ptr<Point> p6(new Point(verts.get()->at(6)-osg::Vec3(0,0,height/2),color));
    std::vector<std::unique_ptr<Point>> rowFirst;
    rowFirst.push_back(std::move(p6));
    points.push_back(std::move(rowFirst));

    //create Vectors
    double startValueY = distance;
    while(startValueY < width)
    {
        std::vector<std::unique_ptr<Point>> rowUp;
        std::unique_ptr<Point> newPointUp(new Point(verts.get()->at(6)+osg::Vec3(0,startValueY,-height/2),color));
        rowUp.push_back(std::move(newPointUp));
        points.push_back(std::move(rowUp));
        startValueY +=distance;
    }
    //add last row from point 7
    std::unique_ptr<Point> p7(new Point(verts.get()->at(7)-osg::Vec3(0,0,height/2),color));
    std::vector<std::unique_ptr<Point>> rowLast;
    rowLast.push_back(std::move(p7));
    points.push_back(std::move(rowLast));



    // add points to each existing Vector
    double startValueX = distance;
    while(startValueX < length)
    {
        for(auto &x : points)
        {
            osg::Vec3 newPos = x.front() ->getPos()-osg::Vec3(startValueX,0,0.0);
            std::unique_ptr<Point> newPoint(new Point(newPos,color));
            x.push_back(std::move(newPoint));
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
   // double currentYpos =
    //while(distance < width)
}

void SafetyZone::preFrame()
{
    interactor->preFrame();
    sizeYInteractor->preFrame();
    sizeXInteractor->preFrame();
    static osg::Vec3 startPosY,startPosX;
    if(interactor->wasStarted())
    {
        startPosX = sizeXInteractor->getPos();
    }
    if(interactor->isRunning())
    {

        osg::Matrix localMat = interactor->getMatrix();
        localDCS->setMatrix(localMat);
         osg::Vec3 localVec;
        localVec = localMat.getTrans();

        //legt Position des sizeXInteractors fest!
        osg::Vec3 scaleVec =startPosX;
        //osg::Vec3 scaleVec{0,10,0};

        scaleVec = Matrix::transform3x3(scaleVec, localMat); // ohne das tut rotation nicht!
        scaleVec += localVec;       //ohne das tut translation nicht !

        sizeXInteractor->updateTransform(scaleVec);


    }
    if(interactor->wasStopped())
    {
        updateWorldPosOfAllObservationPoints();
    }
    if(sizeYInteractor->wasStarted())
    {
        std::cout<<"YInteractor started"<<std::endl;

        startPosY = sizeYInteractor->getPos();
    }

    if(sizeYInteractor->isRunning())
    {
        osg::Vec3 tmp = sizeYInteractor->getPos();
        tmp.set(startPosY.x(),tmp.y(),startPosY.z());
        sizeYInteractor->updateTransform(tmp);
        updateGeometryY(tmp);


    }
    if(sizeYInteractor->wasStopped())
    {
        width = std::fabs(verts->at(7).y()-verts->at(6).y());
        double lastYvalue;
        osg::Vec3 endPosY = sizeYInteractor->getPos();
        osg::Vec3 newPos;
        std::vector<std::unique_ptr<Point>> newVec;// hier crasht es

        if(points.empty())
        {
            lastYvalue = 0;
        }
        else
        {
            lastYvalue = points.back().back()->getPos().y();
        }
        //add Points
        if(endPosY.y() > lastYvalue)
        {

             if(points.empty())
             {
                 newPos = startPosY;//+osg::Vec3(0.0,distance,0.0);
                 std::unique_ptr<Point> newPoint(new Point(newPos,color));
                 newVec.push_back(std::move(newPoint));
             }
             else
             {
                 newPos = points.back().at(0)->getPos()+osg::Vec3(0.0,distance,0.0);
             }

             while(newPos.y() < endPosY.y())
             {

                 if(!points.empty())
                 {
                     for(auto &x : points.back())
                     {
                         osg::Vec3 newPosInVector = x->getPos()+osg::Vec3(0.0,distance,0.0);
                         std::unique_ptr<Point> newPointinVector(new Point(newPosInVector,color));
                         newVec.push_back(std::move(newPointinVector));
                     }
                 }
                 points.push_back(std::move(newVec));
                for(const auto& x : points.back())
                 {
                     static int count;
                     count ++;
                     std::cout<<"counter: "<<count<<std::endl;
                     localDCS->addChild(x->getPoint());
                 }
                 newPos += osg::Vec3(0.0,distance,0.0);
             }
             /*//add last Point Position of Lines
             for(auto &x : points.back())
             {
                 osg::Vec3 newPosInVector = osg::Vec3(x->getPos().x(),sizeXInteractor->getPos().y(), x->getPos().z());
                 std::unique_ptr<Point> newPointinVector(new Point(newPosInVector));
                 newVec.push_back(std::move(newPointinVector));
             }
             points.push_back(std::move(newVec));
             for(const auto& x : points.back())
             {
                 static int count;
                 count ++;
                 std::cout<<"counter: "<<count<<std::endl;
                 localDCS->addChild(x->getPoint());
             }
            */

        }
        //removePoints
        else if(endPosY.y() < lastYvalue)
        {
            if(!points.empty())
            {
                //remove all Vectors with Points that have y value smaller endPosY.y()
                points.erase(std::remove_if(points.begin(),points.end(),[&endPosY](std::vector<std::unique_ptr<Point>>const& it){return it.back()->getPos().y()>endPosY.y();}),points.end());
                if(points.empty())
                {
                    points.clear();
                    std::cout<<"cleared"<<std::endl;
                }
            }
            else
                std::cout<<"Safety Zone hast no points to delete!"<<std::endl;
        }
          std::cout<<"number of vectors: "<<points.size()<<std::endl;
          std::cout<<"number of elements in vetor: "<<points.back().size()<<std::endl;

          updateWorldPosOfAllObservationPoints();
    }

    if(sizeXInteractor->wasStarted())
    {
        std::cout<<"XInteractor started"<<std::endl;

        startPosX = sizeXInteractor->getPos();
    }

    if(sizeXInteractor->isRunning())
    {
        osg::Vec3 tmp = sizeXInteractor->getPos();
        tmp.set(tmp.x(),startPosX.y(),startPosX.z());
        sizeXInteractor->updateTransform(tmp);
        updateGeometryX(tmp);

    }
    if(sizeXInteractor->wasStopped())
    {
        length =std::fabs(verts->at(7).x()-verts->at(4).x());
        double lastXvalue;
        osg::Vec3 endPosX = sizeXInteractor->getPos();
        osg::Vec3 newPos;
        std::vector<std::unique_ptr<Point>> newVec;


        if(points.empty())
        {
            lastXvalue = 0;
        }
        else
        {
            lastXvalue = points.back().back()->getPos().x();
        }
        //add Points
        if(endPosX.x() > lastXvalue)
        {
           if(points.empty())
           {
               newPos = startPosX;//+osg::Vec3(distance,0,0.0);
               std::unique_ptr<Point> newPoint(new Point(newPos,color));
               newVec.push_back(std::move(newPoint));
               points.push_back(move(newVec));
               localDCS->addChild(points.back().back()->getPoint());

           }
           for(auto& x : points)
               {
                   newPos = x.back()->getPos()+osg::Vec3(distance,0,0.0);
                   while(newPos.x() < endPosX.x())
                   {
                       std::unique_ptr<Point> newPoint(new Point(newPos,color));
                       x.push_back(std::move(newPoint));
                       localDCS->addChild(x.back()->getPoint());
                       newPos += osg::Vec3(distance,0,0.0);
                   }

               }


           }
        //removePoints
        else if(endPosX.x() < lastXvalue)
        {
            if(!points.empty())
            {
             //remove all Points with y value smaller endPosY.y()
             for(auto & x:points)
             {
                 x.erase(std::remove_if( x.begin(), x.end(),[&endPosX](std::unique_ptr<Point>const& it){return it->getPos().x()>endPosX.x();}),x.end());
             }
             if(points.back().empty())
             {
               points.clear();//Note: tell owner to remove this object ?
               std::cout<<"deleted complete vector!"<<std::endl;
             }
         }
            else
                std::cout<<"Safety Zone hast no points to delete!"<<std::endl;
        }
        std::cout<<"number of vectors: "<<points.size()<<std::endl;
        std::cout<<"number of elements in vetor: "<<points.back().size()<<std::endl;

        updateWorldPosOfAllObservationPoints();
    }

}



void SafetyZone::updateGeometryY(osg::Vec3 tmp)
{
        osg::Matrix m;
        m.setTrans(osg::Vec3(0,0,0));//#################################asfasfasfs
        static int oldDistance;
       //std::cout<<"static: "<<count <<std::endl;
  //  if(std::abs(verts->at(6).y()-tmp.y())>=std::abs(distanceY))
  //  {
        //update verts
        verts->at(7) = (tmp+osg::Vec3(0,distanceY,0))*m;
        verts->at(3) = (tmp+osg::Vec3(0,distanceY,-height))*m;
        verts->at(4) = (tmp+osg::Vec3(-length,distanceY,0))*m;
        verts->at(0) = (tmp+osg::Vec3(-length,distanceY,-height))*m;
        verts->dirty();

        //update Points at verts
     /*   pointsVerts.at(7)->setPos(verts->at(7));
        pointsVerts.at(3)->setPos(verts->at(3));
        pointsVerts.at(4)->setPos(verts->at(4));
        pointsVerts.at(0)->setPos(verts->at(0));
*/
        //update other Interactor
        sizeXInteractor->updateTransform(tmp-distanceInteractors);
  //  }
  /*  else //if(std::abs(verts->at(7).y()-tmp.y())>std::abs(distanceY))
    {
        verts->at(6) = tmp+osg::Vec3(0,-distanceY,0);
        verts->at(2) = tmp+osg::Vec3(0,-distanceY,-height);
        verts->at(5) = tmp+osg::Vec3(-length,-distanceY,0);
        verts->at(1) = tmp+osg::Vec3(-length,-distanceY,-height);
        verts->dirty();
    }
    */
    /*    int distance = std::trunc(std::abs(verts->at(6).y()-tmp.y()));
        std::cout<<"Distanz: " <<distance<<std::endl;
        if(distance % 4 == 0 && distance-oldDistance ==1)
        {
            std::unique_ptr<Point> newPoint(new Point(tmp));
            pointsY.push_back(std::move(newPoint));
            localDCS->addChild(pointsY.back()->getPoint());

            std::unique_ptr<Point> newPoint1(new Point(tmp+osg::Vec3(-length,0,0)));
            pointsY.push_back(std::move(newPoint1));
            localDCS->addChild(pointsY.back()->getPoint());
        }
        //remember old distance:
        oldDistance = distance;
*/
}
void SafetyZone::updateGeometryX(osg::Vec3 tmp)
{
    //update verts don't use tmp weil tmp bereits durch Matrix verschoben
    verts->at(7) = tmp+osg::Vec3(distanceX,0,0);
    verts->at(3) = tmp+osg::Vec3(distanceX,0,-height);
    verts->at(6) = tmp+osg::Vec3(distanceX,-width,0);
    verts->at(2) = tmp+osg::Vec3(distanceX,-width,-height);
    verts->dirty();

    //update Points at verts
/*    pointsVerts.at(7)->setPos(verts->at(7));
    pointsVerts.at(3)->setPos(verts->at(3));
    pointsVerts.at(6)->setPos(verts->at(6));
    pointsVerts.at(2)->setPos(verts->at(2));

  */  //update other Interactor
    sizeYInteractor->updateTransform(tmp+distanceInteractors);

}

SafetyZone::~SafetyZone()
{
    delete interactor;
    delete sizeXInteractor;
    delete sizeYInteractor;
    points.clear();
    localDCS->getParent(0)->removeChild(localDCS.get());

    std::cout<<"Removed Safety Zone"<<std::endl;
}

void SafetyZone::updateWorldPosOfAllObservationPoints()
{
    worldPosOfAllObservationPoints.clear();
    for(const auto& x : points )
    {
        for(const auto& x1 :x)
        {
            osg::Vec3 pos = x1->getPos()*localDCS.get()->getMatrix();
            worldPosOfAllObservationPoints.push_back(pos);
            std::cout<< "Point World Coordinates: " << pos.x() <<" | "<< pos.y()<<" | "<< pos.z()<<std::endl;

        }

    }
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

Point::Point(osg::Vec3 pos,osg::Vec4 color):color(color),visibleForEnoughCameras(true)
{
    osg::Matrix local;
    local.setTrans(pos);
    localDCS = new osg::MatrixTransform();
    localDCS->setMatrix(local);
    localDCS->setName("Translation");
    sphere = new osg::Sphere(osg::Vec3(0,0,0), 0.5);
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
    std::cout<<"Point destructor called"<<std::endl;

}
void Point::setStateSet(osg::StateSet *stateSet)
{
    osg::Material *material = new osg::Material();
    material->setColorMode(osg::Material::DIFFUSE);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
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
using namespace osg;
double SZ::imgHeightPixel = 1080;
double SZ::imgWidthPixel = 1920;
double SZ::fov = 60;
double SZ::depthView = 40;
double SZ::focalLengthPixel = SZ::imgWidthPixel*0.5/(std::tan(SZ::fov*0.5*M_PI/180));
double SZ::imgWidth = (2*depthView*std::tan(SZ::fov/2*osg::PI/180))/2;
double SZ::imgHeight = (SZ::imgWidth/(SZ::imgWidthPixel/SZ::imgHeightPixel))/2;

SZ::SZ(const char *n, int id, Vec3 pos, osg::Vec3 hpr, float scale=1.0, float depthscale=1.0):name(n),id_(id),scale_(scale),depthscale_(depthscale)
{
    hasScale_ = true;
    hasOrientation_ = hasPosition_ = true;
  //  hasMatrix_ = false;
  //  activated_ = false;
    hasGeometry_ = false;
    setPosition(pos);
    setEuler(hpr);

   // hasTangentOut_ = false;
   // hasTangentIn_ = false;



    createGeometry(pos,hpr);
}
void SZ::setScale(float scale)
{
    scale_ = scale;
    hasScale_ = true;

    if (hasGeometry_)
        updateGeometry();
    // if (!hasGeometry_)
            //     createGeometry();
           //  updateGeometry();
}
void SZ::setPosition(coCoord pos)
{
    coord.xyz[0]=pos.xyz[0];
    coord.xyz[1]=pos.xyz[1];
    coord.xyz[2]=pos.xyz[2];
    hasPosition_ = true;
    if (hasGeometry_)
        updateGeometry();
  /* if (!hasGeometry_)
        createGeometry();
    updateGeometry();
    */
}
void SZ::setPosition(osg::Vec3 pos)
{
    coord.xyz[0]=pos.x();
    coord.xyz[1]=pos.y();
    coord.xyz[2]=pos.z();
    hasPosition_ = true;
    if (hasGeometry_)
        updateGeometry();

   // if (!hasGeometry_)
   //     createGeometry();
  //  updateGeometry();
}
void SZ::setEuler(coCoord euler)
{
    coord.hpr[0]=euler.hpr[0];
    coord.hpr[1]=euler.hpr[1];
    coord.hpr[2]=euler.hpr[2];
    hasOrientation_ = true;

    if (hasGeometry_)
        updateGeometry();
  //  if (!hasGeometry_)
  //      createGeometry();
  //  updateGeometry();

}
void SZ::setEuler(osg::Vec3 euler)
{
    coord.hpr[0]=euler.x();
    coord.hpr[1]=euler.y();
    coord.hpr[2]=euler.z();
    hasOrientation_ = true;

    if (hasGeometry_)
        updateGeometry();
    // if (!hasGeometry_)
    //     createGeometry();
   //  updateGeometry();
}
void SZ::updateGeometry()
{
    if (!hasGeometry_)
        return;

  /*  if (cover->debugLevel(3))
        fprintf(stderr, "\nViewDesc::updateGeometry\n");

    ViewPoints::instance()->dataChanged = true;
    tangentlinescoords->at(0) = Vec3(0, 0, 0);
    tangentlinescoords->at(1) = Vec3(tangentOut[0] / scale_, tangentOut[1] / scale_, tangentOut[2] / scale_);
    tangentlinescoords->at(2) = Vec3(0, 0, 0);
    tangentlinescoords->at(3) = Vec3(tangentIn[0] / scale_, tangentIn[1] / scale_, tangentIn[2] / scale_);

    if (shiftFlightpathToEyePoint)
    {
        tangentlinescoords->at(0) = eyepoint / scale_;
        tangentlinescoords->at(1) += eyepoint / scale_;
        tangentlinescoords->at(2) = eyepoint / scale_;
        tangentlinescoords->at(3) += eyepoint / scale_;
    }

    if (tangentlinesgeoset)
    {
        tangentlinesgeoset->setVertexArray(tangentlinescoords.get());
        tangentlinesgeoset->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 4));
    }
*/
    viewpointCoords.get()->at(0) = Vec3(-imgWidth /(scale_), depthView/depthscale_,-imgHeight / (scale_));
    viewpointCoords.get()->at(1) = Vec3(imgWidth / (scale_), depthView/depthscale_, -imgHeight /(scale_));
    viewpointCoords.get()->at(2) = Vec3(imgWidth / (scale_), depthView/depthscale_,  imgHeight /(scale_));
    viewpointCoords.get()->at(3) = Vec3(-imgWidth /(scale_), depthView/depthscale_, imgHeight / (scale_));
    viewpointCoords.get()->at(4) = eyepoint / (scale_);

    viewpointBorderCoords.get()->at(0) = Vec3(-imgWidth / (scale_), depthView/depthscale_, -imgHeight / (scale_));
    viewpointBorderCoords.get()->at(1) = Vec3( imgWidth / (scale_), depthView/depthscale_, -imgHeight / (scale_));
    viewpointBorderCoords.get()->at(2) = Vec3( imgWidth / (scale_), depthView/depthscale_, -imgHeight / (scale_));
    viewpointBorderCoords.get()->at(3) = Vec3( imgWidth / (scale_), depthView/depthscale_,  imgHeight / (scale_));
    viewpointBorderCoords.get()->at(4) = Vec3( imgWidth / (scale_), depthView/depthscale_,  imgHeight / (scale_));
    viewpointBorderCoords.get()->at(5) = Vec3(-imgWidth / (scale_), depthView/depthscale_,  imgHeight / (scale_));
    viewpointBorderCoords.get()->at(6) = Vec3(-imgWidth / (scale_), depthView/depthscale_,  imgHeight / (scale_));
    viewpointBorderCoords.get()->at(7) = Vec3(-imgWidth / (scale_), depthView/depthscale_, -imgHeight / (scale_));

    Vec3 lu = Vec3(-imgWidth / scale_, depthView/depthscale_,-imgHeight / (scale_));
    Vec3 ru = Vec3( imgWidth / scale_, depthView/depthscale_,-imgHeight /(scale_));
    Vec3 ro = Vec3( imgWidth / scale_, depthView/depthscale_, imgHeight /(scale_));
    Vec3 lo = Vec3(-imgWidth / scale_, depthView/depthscale_, imgHeight / (scale_));
    Vec3 eye = (eyepoint / (scale_));

/*    viewpointCoords.get()->at(0) = Vec3(-800 / scale_, 0, -600 / scale_);
    viewpointCoords.get()->at(1) = Vec3(800 / scale_, 0, -600 / scale_);
    viewpointCoords.get()->at(2) = Vec3(800 / scale_, 0, 600 / scale_);
    viewpointCoords.get()->at(3) = Vec3(-800 / scale_, 0, 600 / scale_);
    viewpointCoords.get()->at(4) = eyepoint / scale_;

    viewpointBorderCoords.get()->at(0) = Vec3(-800 / scale_, 0, -600 / scale_);
    viewpointBorderCoords.get()->at(1) = Vec3(800 / scale_, 0, -600 / scale_);
    viewpointBorderCoords.get()->at(2) = Vec3(800 / scale_, 0, -600 / scale_);
    viewpointBorderCoords.get()->at(3) = Vec3(800 / scale_, 0, 600 / scale_);
    viewpointBorderCoords.get()->at(4) = Vec3(800 / scale_, 0, 600 / scale_);
    viewpointBorderCoords.get()->at(5) = Vec3(-800 / scale_, 0, 600 / scale_);
    viewpointBorderCoords.get()->at(6) = Vec3(-800 / scale_, 0, 600 / scale_);
    viewpointBorderCoords.get()->at(7) = Vec3(-800 / scale_, 0, -600 / scale_);

    Vec3 lu = Vec3(-800 / scale_, 0, -600 / scale_);
    Vec3 ru = Vec3(800 / scale_, 0, -600 / scale_);
    Vec3 ro = Vec3(800 / scale_, 0, 600 / scale_);
    Vec3 lo = Vec3(-800 / scale_, 0, 600 / scale_);
    Vec3 eye = (eyepoint / scale_);
*/    lineEyetoLeftDown.get()->at(0) = eye;
    lineEyetoLeftDown.get()->at(1) = lu;

    lineEyetoRightDown.get()->at(0) = eye;
    lineEyetoRightDown.get()->at(1) = ru;

    lineEyetoRightUp.get()->at(0) = eye;
    lineEyetoRightUp.get()->at(1) = ro;

    lineEyetoLeftUp.get()->at(0) = eye;
    lineEyetoLeftUp.get()->at(1) = lo;

    if (viewpointPlaneGeoset && viewpointPlaneBorderGeoset && viewpointGeoset)
    {
        line1->setVertexArray(lineEyetoLeftDown);
        line1->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 2));
        line2->setVertexArray(lineEyetoRightDown);
        line2->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 2));
        line3->setVertexArray(lineEyetoRightUp);
        line3->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 2));
        line4->setVertexArray(lineEyetoLeftUp);
        line4->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 2));

        viewpointPlaneGeoset->setVertexArray(viewpointCoords.get());
        viewpointPlaneGeoset->addPrimitiveSet(new DrawArrays(PrimitiveSet::QUADS, 0, 3));

        viewpointPlaneBorderGeoset->setVertexArray(viewpointBorderCoords.get());
        viewpointPlaneBorderGeoset->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 4));
    }

    Matrix rotMat;
    Matrix transMat;
    Matrix final;

   if (hasMatrix_)
    {
        transMat.makeTranslate(-xformMat_(3, 0) / scale_, -xformMat_(3, 1) / scale_, -xformMat_(3, 2) / scale_);
        Quat quat;
        quat = xformMat_.getRotate();
        double angle, x, y, z;
        quat.getRotate(angle, x, y, z);
        quat.makeRotate(-angle, x, y, z);
        rotMat.makeRotate(quat);

        final.makeIdentity();
        final.postMult(transMat);
        final.postMult(rotMat);
        localDCS->setMatrix(final);
    }
    else
    {
        //fprintf(stderr, "ohne Matrix");
        // rotMat.makeEuler(coord.hpr[0], coord.hpr[1], coord.hpr[2]);
        MAKE_EULER_MAT(rotMat, coord.hpr[0], coord.hpr[1], coord.hpr[2])
        transMat.makeTranslate(-coord.xyz[0] / scale_, -coord.xyz[1] / scale_, -coord.xyz[2] / scale_);
        Quat quat;
        // rotMat.getOrthoQuat(quat);
        quat = rotMat.getRotate();
        double angle, x, y, z;
        quat.getRotate(angle, x, y, z);
        quat.makeRotate(-angle, x, y, z);
        rotMat.makeRotate(quat);

        final.makeIdentity();
        final.postMult(transMat);
        final.postMult(rotMat);
        localDCS->setMatrix(final);
    }

   /* Vec3 tangent = tangentIn;
    tangent = Matrix::transform3x3(final, tangent);
    tangent *= (1 / scale_);
    tangent += Vec3(final(3, 0), final(3, 1), final(3, 2));
*/
    // //update text
    // myLabel->setString(name);
    // myLabel->setPosition(Vec3(600 / scale_, 0, 600 / scale_));

    // Update Interactors
    //=========================================================================================

    Matrix localMat;
    localMat = localDCS->getMatrix();

    Vec3 localVec;
    localVec = localMat.getTrans();

    viewpointInteractor->updateTransform(localMat);

  /*  Vec3 tangentInO;
    Vec3 shiftVec = Vec3(0, 0, 0);
    if (shiftFlightpathToEyePoint)
        shiftVec = -eyepoint;

    tangentInO = Matrix::transform3x3((tangentIn - shiftVec), localMat);
    tangentInO *= (1 / scale_);
    tangentInO += localVec;

    tanInInteractor->updateTransform(tangentInO);

    Vec3 tangentOutO;
    tangentOutO = Matrix::transform3x3((tangentOut - shiftVec), localMat);
    tangentOutO *= (1 / scale_);
    tangentOutO += localVec;
    tanOutInteractor->updateTransform(tangentOutO);
*/
  //  scaleVec = Vec3(800 / scale_, 0, -600 / scale_); // urspruenglich 800 x -600

    //legt Ort für scale Interactor fest!

   //scaleVec = Vec3(imgWidth / scale_, depthView / scale_, imgHeight / scale_); // urspruenglich 800 x -600
   scaleVec= Vec3(0,50,0); //legt Position von Interactor fest!
   scaleVec = Matrix::transform3x3(scaleVec, localMat); // ohne das tut rotation nicht!
    scaleVec += localVec;       //ohne das tut translation nicht !

    scaleInteractor->updateTransform(scaleVec);

}
void SZ::createGeometry(osg::Vec3 pos,osg::Vec3 euler)
{
    localDCS = new osg::MatrixTransform();
    localDCS->setName("VPLoaclDCS");
    viewpointgeode = new Geode();
    viewpointgeode->setName("VPGeode");
    viewpointPlaneGeoset = new osg::Geometry();
    viewpointPlaneBorderGeoset = new osg::Geometry();
    viewpointGeoset = new osg::Geometry();

    line1 = new osg::Geometry();
    line2 = new osg::Geometry();
    line3 = new osg::Geometry();
    line4 = new osg::Geometry();

    osg::Vec3 test1{(float)imgHeight,(float)imgWidth,(float)depthView};
    osg::Matrix localMat;
    localMat.setTrans(pos+test1);
  //  localMat.setRotate(euler);
    localDCS->setMatrix(localMat);

    coCoord test;
    test.xyz[0] =imgHeight;
    test.xyz[1] =imgWidth;
    test.xyz[2] =depthView;

    //create Interactors
    float _interSize = cover->getSceneSize() / 25;
    viewpointInteractor = new coVR3DTransRotInteractor(localMat, _interSize/2, vrui::coInteraction::ButtonA, "hand", "vpInteractor", vrui::coInteraction::Medium);
    scaleInteractor = new coVR3DTransInteractor(test1, _interSize/2, vrui::coInteraction::ButtonA, "hand",
                                                "vpScaleInteractor", vrui::coInteraction::Medium);
    depthInteractor = new coVR3DTransInteractor(test1/2, _interSize/2, vrui::coInteraction::ButtonA, "hand",
                                                "vpDepthInteractor", vrui::coInteraction::Medium);

    //initially hide interactors
    //viewpointInteractor->hide();
    //scaleInteractor->hide();

    lineEyetoLeftDown = new Vec3Array(2);
    lineEyetoRightDown = new Vec3Array(2);
    lineEyetoRightUp = new Vec3Array(2);
    lineEyetoLeftUp = new Vec3Array(2);

    viewpointCoords = new Vec3Array(5);
    viewpointBorderCoords = new Vec3Array(8);

    hasGeometry_ = true;
    updateGeometry();

    LineWidth *lw = new LineWidth(2.0);

    osg::ref_ptr<osg::Vec4Array> viewpointColor = new Vec4Array();
    viewpointColor->push_back(osg::Vec4(0.0, 1.0, 0.0, 0.5)); // green
    ref_ptr<Vec4Array> viewpointboxLineColor = new Vec4Array();
    viewpointboxLineColor->push_back(osg::Vec4(0.0, 1.0, 0.0, 1.0)); // green

    viewpointPlaneGeoset->setVertexArray(viewpointCoords.get());
    viewpointPlaneGeoset->addPrimitiveSet(new DrawArrays(PrimitiveSet::QUADS, 0, 4));
    viewpointPlaneGeoset->setColorArray(viewpointColor.get());
    viewpointPlaneGeoset->setColorBinding(Geometry::BIND_OVERALL);
    viewpointPlaneGeoset_state = viewpointPlaneGeoset->getOrCreateStateSet();
    loadUnlightedGeostate(viewpointPlaneGeoset_state);

    viewpointPlaneBorderGeoset->setVertexArray(viewpointBorderCoords.get());
    viewpointPlaneBorderGeoset->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 8));
    viewpointPlaneBorderGeoset->setColorArray(viewpointboxLineColor.get());
    viewpointPlaneBorderGeoset->setColorBinding(Geometry::BIND_OVERALL);
    viewpointPlaneBorderGeoset_state = viewpointPlaneBorderGeoset->getOrCreateStateSet();
    loadUnlightedGeostate(viewpointPlaneBorderGeoset_state);
    viewpointPlaneBorderGeoset_state->setAttribute(lw);

    line1->setVertexArray(lineEyetoLeftDown);
    line1->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 2));
    line1->setColorArray(viewpointboxLineColor.get());
    line1->setColorBinding(Geometry::BIND_OVERALL);
    line1_state = line1->getOrCreateStateSet();
    loadUnlightedGeostate(line1_state);
    line1_state->setAttribute(lw);

    line2->setVertexArray(lineEyetoRightDown);
    line2->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 2));
    line2->setColorArray(viewpointboxLineColor.get());
    line2->setColorBinding(Geometry::BIND_OVERALL);
    line2_state = line2->getOrCreateStateSet();
    loadUnlightedGeostate(line2_state);
    line2_state->setAttribute(lw);

    line3->setVertexArray(lineEyetoRightUp);
    line3->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 2));
    line3->setColorArray(viewpointboxLineColor.get());
    line3->setColorBinding(Geometry::BIND_OVERALL);
    line3_state = line3->getOrCreateStateSet();
    loadUnlightedGeostate(line3_state);
    line3_state->setAttribute(lw);

    line4->setVertexArray(lineEyetoLeftUp);
    line4->addPrimitiveSet(new DrawArrays(PrimitiveSet::LINES, 0, 2));
    line4->setColorArray(viewpointboxLineColor.get());
    line4->setColorBinding(Geometry::BIND_OVERALL);
    line4_state = line4->getOrCreateStateSet();
    loadUnlightedGeostate(line4_state);
    line4_state->setAttribute(lw);



    viewpointgeode->addDrawable(line1.get());
    viewpointgeode->addDrawable(line2.get());
    viewpointgeode->addDrawable(line3.get());
    viewpointgeode->addDrawable(line4.get());
    viewpointgeode->addDrawable(viewpointPlaneBorderGeoset.get());
    viewpointgeode->addDrawable(viewpointPlaneGeoset.get());
    viewpointgeode->setNodeMask(viewpointgeode->getNodeMask() & (~Isect::Intersection) & (~Isect::Pick));

    localDCS->addChild(viewpointgeode.get()); //Nicht vergessen bei deleteGeometry auch abaendern!
    cover->getObjectsRoot()->addChild(localDCS.get());
    showInteractors(true);
    //showGeometry(true);//viewpointVisible
}

void SZ::loadUnlightedGeostate(ref_ptr<StateSet> state)
{
    ref_ptr<Material> mat = new Material;
    mat->setColorMode(Material::AMBIENT_AND_DIFFUSE);
    mat->setDiffuse(Material::FRONT_AND_BACK, Vec4(0.9f, 0.9f, 0.9f, 1.f));
    mat->setSpecular(Material::FRONT_AND_BACK, Vec4(0.9f, 0.9f, 0.9f, 1.f));
    mat->setAmbient(Material::FRONT_AND_BACK, Vec4(0.2f, 0.2f, 0.2f, 1.f));
    mat->setEmission(Material::FRONT_AND_BACK, Vec4(0.0f, 0.0f, 0.0f, 1.f));
    mat->setShininess(Material::FRONT_AND_BACK, 16.f);
    mat->setTransparency(Material::FRONT_AND_BACK, 1.0f); // Noch Wert anpassen fuer Transparency

    state->setAttributeAndModes(mat, osg::StateAttribute::ON);
    state->setMode(GL_BLEND, osg::StateAttribute::ON);
    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
}

void SZ::showInteractors(bool state)
{

    if (hasGeometry_)
    {
        if (state == true)
        {
            viewpointInteractor->show();
            viewpointInteractor->enableIntersection();
            scaleInteractor->show();
            scaleInteractor->enableIntersection();
            depthInteractor->show();
            depthInteractor->enableIntersection();
        }
        else
        {
            viewpointInteractor->hide();
            viewpointInteractor->disableIntersection();
            depthInteractor->hide();
            depthInteractor->disableIntersection();
        }
          // showMoveInteractorsCheck_->setState(state);
    }
}

void SZ::preFrame()
{
    if (!hasGeometry_)
        return;

  //  tanInInteractor->preFrame();
  //  tanOutInteractor->preFrame();
    viewpointInteractor->preFrame();
    scaleInteractor->preFrame();
    depthInteractor->preFrame();
/*
    if (tanInInteractor->isRunning())
    {
        Matrix local;
        local = localDCS->getMatrix();
        Vec3 localVec;
        localVec = local.getTrans();
        local.invert(local);

        Vec3 objectCoord;
        objectCoord = tanInInteractor->getPos();
        tangentIn = objectCoord - localVec;
        tangentIn = Matrix::transform3x3(tangentIn, local);

        if (shiftFlightpathToEyePoint)
            tangentIn -= eyepoint / scale_;
        tangentIn *= scale_;

        // C1 continuity
        //tangentOut = -tangentIn;

        // G1 continuity
        float outLen = tangentOut.length();
        tangentOut = -tangentIn;
        tangentOut.normalize();
        tangentOut *= outLen;

        updateGeometry();
    }
    if (tanOutInteractor->isRunning())
    {
        Matrix local;
        local = localDCS->getMatrix();
        Vec3 localVec;
        localVec = local.getTrans();
        local.invert(local);

        Vec3 objectCoord;
        objectCoord = tanOutInteractor->getPos();
        tangentOut = objectCoord - localVec;
        tangentOut = Matrix::transform3x3(tangentOut, local);

        if (shiftFlightpathToEyePoint)
            tangentOut -= eyepoint / scale_;
        tangentOut *= scale_;

        // C1 continuity
        //tangentIn = -tangentOut;

        // G1 continuity
        float inLen = tangentIn.length();
        tangentIn = -tangentOut;
        tangentIn.normalize();
        tangentIn *= inLen;

        updateGeometry();
    }
*/    if (scaleInteractor->isRunning())
    {

        Matrix local,test;
       // test.setTrans(imgWidth/2,depthView,imgHeight/2);

        local = localDCS->getMatrix();//*test;
       // local = scaleInteractor->getMatrix();
       // local.postMult(test);

        Vec3 localVec;
        localVec = local.getTrans();
        std::cout<<"TransRotPos:"<<localVec.x()<<","<<localVec.y()<<","<<localVec.z()<<std::endl;

        Vec3 tmp = scaleInteractor->getPos();
        std::cout<<"Old scale Pos:"<<scaleInteractor->getPos().x()<<","<<scaleInteractor->getPos().y()<<","<<scaleInteractor->getPos().z()<<std::endl;

        float length = fabs((localVec - tmp).length());
       // float length = fabs((Vec3(0,20,0) - tmp).length());

        std::cout<<"length"<<length<<std::endl;
        // restrict direction of scaleInteractor:
       // scaleVec = Vec3(800, 0, -600); // urspruenglich 800 x -600
        scaleVec = Vec3(imgWidth / scale_, 0 , imgHeight / scale_);
       // scaleVec = Vec3(imgWidth, 0, -imgHeight); // urspruenglich 800 x -600
        scaleVec.normalize();
        scaleVec *= length;
        scaleVec = Matrix::transform3x3(scaleVec, local);

        scaleInteractor->updateTransform(scaleVec + localVec);
        scaleInteractor->getPos();
        std::cout<<"New scale Pos:"<<scaleInteractor->getPos().x()<<","<<scaleInteractor->getPos().y()<<","<<scaleInteractor->getPos().z()<<std::endl;
        //--> hier ist die Scale Pos noch richtig

        // alten scale herausrechnen | | x  | | scale 2 --> x = 200
        // coord.xyz[0] /= _scale; // | x | scale 1 --> x = 100
        // coord.xyz[1] /= _scale;
        // coord.xyz[2] /= _scale;

        xformMat_(3, 0) /= scale_;
        xformMat_(3, 1) /= scale_;
        xformMat_(3, 2) /= scale_;

       //scale_ = 1000 / scaleVec.length();
        scale_ = 1000/scaleVec.length();
        xformMat_(3, 0) *= scale_;
        xformMat_(3, 1) *= scale_;
        xformMat_(3, 2) *= scale_;

        updateGeometry();
    }
    if(depthInteractor->isRunning())
    {

    }
    if (viewpointInteractor->isRunning())
    {
        Matrix m = viewpointInteractor->getMatrix();
        localDCS->setMatrix(m); //--> hier wird Geometry mit interactor Verknüpft

        // get world coordinates of viewpoint
        m.invert(m);

        xformMat_ = m;
        xformMat_(3, 0) *= scale_;
        xformMat_(3, 1) *= scale_;
        xformMat_(3, 2) *= scale_;
        hasMatrix_=true;
       updateGeometry();
    }
   // vpVis->updateDrawnCurve();


    /*
    if (tanOutInteractor->wasStopped() || tanInInteractor->wasStopped() || viewpointInteractor->wasStopped() || scaleInteractor->wasStopped())
    {
    // Send Changes to GUI
    //=========================================================================================
    if(VRCoviseConnection::covconn && coVRMSController::msController->isMaster())
    {
    char newString[1000];
    sprintf(newString, "%f %f %f %f %f %f %f %f %f %f %f %f %f", scale, coord.xyz[0],
    coord.xyz[1], coord.xyz[2], coord.hpr[0], coord.hpr[1],
    coord.hpr[2], tangentOut[0], tangentOut[1], tangentOut[2], tangentIn[0],
    tangentIn[1], tangentIn[2]);

    // send to GUI
    coGRUpdateViewpointMsg vMsg(id_,newString);

    Message grmsg;
    grmsg.type = UI;
    grmsg.data = (char *)(vMsg.c_str());
    grmsg.length = strlen(grmsg.data)+1;
    CoviseRender::appmod->send_ctl_msg(&grmsg);
    }
    updateGeometry();
    // vpVis->updateDrawnCurve();
    }
    */
}

SZ2::SZ2(osg::Vec3 pos):position(pos)
{
    interactorSize = cover->getSceneSize() / 15;
    widthInteractor = new coVR3DTransInteractor(position, interactorSize, vrui::coInteraction::ButtonA, "hand", "vpTanOutInteractor", vrui::coInteraction::Medium);
    widthInteractor->show();
    widthInteractor->enableIntersection();

}
SZ2::~SZ2()
{
    delete widthInteractor;
}

void SZ2::updatePosition(osg::Vec3 newPosition)
{
    std::cout<<"WidthInteractor: "<<position.x()<<"|"<<position.y()<<"|"<<position.z()<<std::endl;
    position = newPosition;
}

void SZ2::preFrame()
{
    widthInteractor->preFrame();
    static osg::Matrix start;
    static coCoord startEuler;
    if(widthInteractor->wasStarted())//sobald Maus in Cover ist!
        std::cout<<"start"<<std::endl;
        start = widthInteractor->getMatrix();
        startEuler = start;
    if (widthInteractor->isRunning())
    {
        std::cout<<"running"<<std::endl;

        osg::Matrix m = widthInteractor->getMatrix();
        restrictTranslation(startEuler,m,true,true,false);
        osg::Vec3 interpos = m.getTrans();
        updatePosition(interpos);
        widthInteractor->updateTransform(interpos);
    }
    if(widthInteractor->wasStopped())
    {
        std::cout<<"stop"<<std::endl;

    }
}
void SZ2::showInteractors(bool state)
{

        if (state == true)
        {
           // zonepointInteractor->show();
           // zonepointInteractor->enableIntersection();

        }
        else
        {
           // zonepointInteractor->hide();
           // zonepointInteractor->disableIntersection();

        }
}











