#include "EKU.h"


#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <string>
#include <numeric>
#include <array>
#include <cstdlib>

using namespace opencover;

EKU *EKU::plugin = NULL;

void Pump::preFrame()
{
    sensorList.update();
    //Test if button is pressed
    int state = cover->getPointerButton()->getState();
    if (myinteraction->isRunning()) //when interacting the Sphere will be moved
    {
        static osg::Matrix invStartHand;
        static osg::Matrix startPos;
        static osg::Quat xRot,yRot, zRot;
        if (!interActing)
        {
            //remember invStartHand-Matrix, when interaction started and mouse button was pressed
            invStartHand.invert(cover->getPointerMat() * cover->getInvBaseMat());
            startPos = transMat->getMatrix(); //remember position of sphere, when interaction started
            interActing = true; //register interaction
            std::cout<<"Start Pos: " <<startPos.getTrans().x() <<","<<startPos.getTrans().y() <<","<<startPos.getTrans().z() <<std::endl;
        }
        else
        {
            //calc the tranformation matrix when interacting is running and mouse button was pressed
            osg::Matrix trans = startPos * invStartHand * (cover->getPointerMat() * cover->getInvBaseMat());
            //no translation in z
            trans.setTrans(trans.getTrans().x(),trans.getTrans().y(),startPos.getTrans().z());
            //rotation only around z
            zRot.makeRotate(trans.getRotate().z(), osg::Z_AXIS);         //   trans.setRotate();
            yRot.makeRotate(startPos.getRotate().y(), osg::Y_AXIS);         //   trans.setRotate();
            xRot.makeRotate(startPos.getRotate().x(), osg::X_AXIS);         //   trans.setRotate();
            trans.setRotate(xRot*yRot*zRot);
            transMat->setMatrix(trans);
            std::cout<<"actual Pos: " <<trans.getTrans().x() <<","<<trans.getTrans().y() <<","<<trans.getTrans().z() <<std::endl;

        }
    }
    if (myinteraction->wasStopped() && state == false)
    {
        interActing = false; //unregister interaction
    }
}
void EKU::preFrame()
{
    sensorList.update();
    //Test if button is pressed
    int state = cover->getPointerButton()->getState();
  /*  if (myinteraction->isRunning()) //when interacting the Sphere will be moved
    {
        static osg::Matrix invStartHand;
        static osg::Matrix startPos;
        if (!interActing)
        {
            //remember invStartHand-Matrix, when interaction started and mouse button was pressed
            invStartHand.invert(cover->getPointerMat() * cover->getInvBaseMat());
            startPos = mymtf->getMatrix(); //remember position of sphere, when interaction started
            interActing = true; //register interaction

        }
        else
        {
            //calc the tranformation matrix when interacting is running and mouse button was pressed
            osg::Matrix trans = startPos * invStartHand * (cover->getPointerMat() * cover->getInvBaseMat());

            mymtf->setMatrix(trans);
        }
    }
    if (myinteraction->wasStopped() && state == false)
    {
        interActing = false; //unregister interaction
    }
    */for(const auto &x: allPumps)
        x->preFrame();
}
void EKU::calcPercentageOfCoveredSafetyZones()
{
    std::cout<<"PRIO1 zones covered by 2 cameras: " <<std::endl;
    std::cout<<"PRIO1 zones covered by 1 camera: " <<std::endl;
    std::cout<<"PRIO1 zones not covered: " <<std::endl;
    std::cout<<"PRIO2 zones covered by at least 1 camera: " <<std::endl;
    std::cout<<"PRIO2 zones not covered: " <<std::endl;
}
size_t Pump::counter = 0;
Pump::Pump(osg::ref_ptr<osg::Node> truck,osg::ref_ptr<osg::Node> truckSurface =nullptr, osg::Vec3 pos =osg::Vec3(20,0,0), int rotationZ = 0):position(pos),truck(truck),truckSurfaceBox(truckSurface),rotZ(rotationZ)
{
    Pump::counter ++;
    group = new osg::Group;
    group->setName("DetailedTruck+Cam+Safety"+std::to_string(Pump::counter));
    truck->setName("DetailedTruck"+std::to_string(Pump::counter));
    name = "DetailedTruck+Cam+Safety"+std::to_string(Pump::counter);
    //create safety Zones
    safetyZones.at(0) = new Truck(osg::Vec3(-2.3,0,9),Truck::PRIO2); //safetyZone Dimensions: 2,2,8
    safetyZones.at(1) = new Truck(osg::Vec3(2.3,0,9),Truck::PRIO2);

    //create possible Cam locations
    possibleCamLocations.push_back(new CamPosition(osg::Vec3(1.5,1.6,-0.5)));//to side , height , forward
    possibleCamLocations.push_back(new CamPosition(osg::Vec3(-1.6,1.6,-0.5)));
    auto worldPosition1 = possibleCamLocations[0]->getCamGeode()->getBound().center() * osg::computeLocalToWorld(possibleCamLocations[0]->getCamGeode()->getParentalNodePaths()[0]);
     std::cout<<"before"<<worldPosition1.x()<<"|"<<worldPosition1.y()<<"|"<<worldPosition1.z()<<std::endl;
    //Rotation
    osg::Matrix rotate;
    osg::Quat xRot, yRot;
    xRot.makeRotate(osg::DegreesToRadians(90.0),osg::X_AXIS);
    yRot.makeRotate(osg::DegreesToRadians(270.0+rotZ),osg::Y_AXIS);
    osg::Quat fullRot = yRot*xRot;
    rotate.setRotate(fullRot);
    rotMat = new osg::MatrixTransform();
    rotMat ->setName("Rotation");
    rotMat->setMatrix(rotate);

    rotMat->addChild(truck.get());
    rotMat->addChild(truckSurfaceBox.get());
   for(const auto& x : safetyZones)
        rotMat->addChild(x->getTruckDrawable().get());
    for(const auto& x : possibleCamLocations)
        rotMat->addChild(x->getCamGeode().get());
    rotMat->addChild(truck.get());
    rotMat->addChild(truckSurfaceBox.get());

    auto worldPosition2 = possibleCamLocations[0]->getCamGeode()->getBound().center() * osg::computeLocalToWorld(possibleCamLocations[0]->getCamGeode()->getParentalNodePaths()[0]);
     std::cout<<"afterRot"<<worldPosition2.x()<<"|"<<worldPosition2.y()<<"|"<<worldPosition2.z()<<std::endl;

    //Translation
    transMat= new osg::MatrixTransform();
    transMat->setName("Translation");
    osg::Matrix translate;
    //translate.setTrans(0,0,0);
    translate.setTrans(pos.x(),pos.y(),pos.z());
    transMat->setMatrix(translate);
    transMat->addChild(rotMat.get());

    // Group
   // group = new osg::Group;
   // group->setName("Truck"+std::to_string(Pump::counter));
    group->addChild(transMat.get());

    auto worldPosition3 = possibleCamLocations[0]->getCamGeode()->getBound().center() * osg::computeLocalToWorld(possibleCamLocations[0]->getCamGeode()->getParentalNodePaths()[0]);
     std::cout<<"afterTrans:"<<worldPosition3.x()<<"|"<<worldPosition3.y()<<"|"<<worldPosition3.z()<<std::endl;

     //User Interaction
     myinteraction = new vrui::coTrackerButtonInteraction(vrui::coInteraction::ButtonC, "MoveMode", vrui::coInteraction::Highest);
     interActing = false;
     aSensor = new mySensor(transMat, name, myinteraction);
     sensorList.append(aSensor);
}
Pump::~Pump()
{
   // cover->getObjectsRoot()->removeChild(this->getPumpDrawable().get());
    for (auto it = possibleCamLocations.begin(); it != possibleCamLocations.end(); it++)
    {
        delete *it;
    }
    possibleCamLocations.clear();

    for (auto it = placedCameras.begin(); it != placedCameras.end(); it++)
    {
        delete *it;
    }
    placedCameras.clear();

    for (auto it = safetyZones.begin(); it != safetyZones.end(); it++)
    {
        delete *it;
    }
   // safetyZones.er

}
void EKU::createScene()
{   //add silo
    {
        std::cout<<"Load silo"<<std::endl;
        silo1 = osgDB::readNodeFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/Silo/Model_C0810A007/C0810A007.3ds");
        silo1->setName("Silo1");

        osg::PositionAttitudeTransform* move = new osg::PositionAttitudeTransform();
        move->setPosition( osg::Vec3( -35.0f, 0.0f, 3.f) );
        move->addChild(silo1);
        finalScene->addChild(move);

        osg::PositionAttitudeTransform* move1 = new osg::PositionAttitudeTransform();
        move1->setPosition( osg::Vec3( -35.0f, 5.0f, 3.f) );
        move1->addChild(silo1);
        finalScene->addChild(move1);

        osg::PositionAttitudeTransform* move2 = new osg::PositionAttitudeTransform();
        move2->setPosition( osg::Vec3( -35.0f, 10.0f, 3.f) );
        move2->addChild(silo1);
        finalScene->addChild(move2);

        osg::PositionAttitudeTransform* move3 = new osg::PositionAttitudeTransform();
        move3->setPosition( osg::Vec3( -35.0f, 15.0f, 3.f) );
        move3->addChild(silo1);
        finalScene->addChild(move3);

        silo2 = osgDB::readNodeFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/Silo2/cadnav.com_model/Silo2.3ds");
        silo2->setName("Silo2");
        osg::PositionAttitudeTransform* move4 = new osg::PositionAttitudeTransform();
        move4->setPosition( osg::Vec3( +25.0f, -10.0f, -0.5f) );
        move4->addChild(silo2);
        finalScene->addChild(move4);

        osg::PositionAttitudeTransform* move5 = new osg::PositionAttitudeTransform();
        move5->setPosition( osg::Vec3( +25.0f, -5.0f, -0.5f) );
        move5->addChild(silo2);
        finalScene->addChild(move5);
        std::cout<<"silo loaded"<<std::endl;
 }

    //add container
    {
        std::cout<<"Load container"<<std::endl;
        container = osgDB::readNodeFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/ContainerBig/ContainerCargoN200418.3ds");
        container->setName("Container");

        osg::PositionAttitudeTransform* scale = new osg::PositionAttitudeTransform();
        scale->setScale(osg::Vec3(0.02,0.02,0.02));
        osg::PositionAttitudeTransform* move = new osg::PositionAttitudeTransform();
        move->setPosition( osg::Vec3( -30.0f, 35.0f, 0.f) );
        osg::PositionAttitudeTransform* rotate = new osg::PositionAttitudeTransform();
        osg::Quat zRot;
        zRot.makeRotate(osg::DegreesToRadians(45.0), osg::Z_AXIS);
        rotate->setAttitude(osg::Quat(zRot));
        scale->addChild(container);
        rotate->addChild(scale);
        move->addChild(rotate);
        finalScene->addChild(move);

        osg::PositionAttitudeTransform* move1 = new osg::PositionAttitudeTransform();
        move1->setPosition( osg::Vec3( -30.0f, 40.0f, 0.f) );
        move1->addChild(rotate);
        finalScene->addChild(move1);

        osg::PositionAttitudeTransform* move2 = new osg::PositionAttitudeTransform();
        move2->setPosition( osg::Vec3( -30.0f, 45.0f, 0.f) );
        move2->addChild(rotate);
        finalScene->addChild(move2);

        std::cout<<"Container loaded"<<std::endl;

    }

    cover->getObjectsRoot()->addChild(finalScene.get());

}

EKU::EKU(): ui::Owner("EKUPlugin", cover->ui)
{

    plugin = this;
    fprintf(stderr, "EKUplugin::EKUplugin\n");
    finalScene = new osg::Group;
    finalScene->setName("finalScene");
    createScene();
    // read file

    truck = coVRFileManager::instance()->loadFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/Truck/truck_surface.stl",NULL,finalScene);
    if (!truck.valid())
    {
          osg::notify( osg::FATAL ) << "Unable to load truck data file. Exiting." << std::endl;
    }

    truckSurfaceBox = osgDB::readNodeFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/Truck/Truck_surface_box.osgt");
    if (!truck.valid())
    {
          osg::notify( osg::FATAL ) << "Unable to load truck data file. Exiting." << std::endl;
    }

    // don't show the many vertices of the truck. Only use truck surface
    disactivateDetailedRendering();

    //draw Pumps:
    allPumps.push_back(new Pump(truck,truckSurfaceBox));
    int cnt =0;
    for(int i = 0;i<5;i++)
    {
        osg::Vec3 posOld=allPumps.back()->getPos();;
        osg::Vec3 posNew;
        int rotZ = allPumps.back()->getRot();
        if(cnt % 2 == 0)
        {
             posNew = {posOld.x()*-1,posOld.y(),posOld.z()};
            allPumps.push_back(new Pump(truck,truckSurfaceBox,posNew,180));//180

        }else
        {
             posNew = {posOld.x()*-1,posOld.y()+5,posOld.z()};//+5
             allPumps.push_back(new Pump(truck,truckSurfaceBox,posNew));
        }

        cnt ++;
    }

    for(const auto & x:allPumps)
    {
        cover->getObjectsRoot()->addChild(x->getPumpDrawable().get());

    }



    //all Points to observe from file

  /*  std::vector<osg::Vec3> truckPos;
    FindNamedNode *fnnPointsPRIO1= new FindNamedNode( "pxONE",&truckPos);
    scene->accept(*fnnPointsPRIO1 );
    delete fnnPointsPRIO1;
    for(const auto& x : truckPos)
    {
        trucks.push_back(new Truck(x,Truck::PRIO1));
        priorityList.push_back(Truck::PRIO1);
    }
    FindNamedNode *fnnPointsPRIO2= new FindNamedNode( "pxTWO",&truckPos);
    scene->accept(*fnnPointsPRIO2 );
    delete fnnPointsPRIO2;
    for(const auto& x : truckPos)
    {
        trucks.push_back(new Truck(x,Truck::PRIO2));
        priorityList.push_back(Truck::PRIO2);
    }
*/
    //######################################################################
    for(const auto& x1 : allPumps)
    {
        for(const auto& x2 : x1->safetyZones)
        {
            trucks.push_back(x2);
            priorityList.push_back(Truck::PRIO2);
        }
    }


    osg::Vec3Array* obsPoints = new osg::Vec3Array; //Note: Remove this unecessary
    for(auto x:trucks)
        obsPoints->push_back( x->getPosition());


    //all possible camera locations from file
    std::vector<osg::Vec3> camPos;
 //   FindNamedNode *fnnCam= new FindNamedNode( "cx",&camPos);
 //   scene->accept(*fnnCam );
 //   delete fnnCam;

    for(const auto& x1 : allPumps)
    {
        for(const auto& x2 : x1->possibleCamLocations)
        {
            camPos.push_back(x2->getPosition());
        }
    }

    {   // for each location create a cam with different alpha and beta angles
        std::vector<osg::Vec2> camRots;
        const int userParam =4;//stepsize = PI/userParam
        const int n_alpha = 2*userParam;
        const int n_beta = userParam/2;//+1;
        double alpha =0;
        double beta =0;
        for(int cnt = 0; cnt<n_alpha; cnt++){
            for(int cnt2 = 0; cnt2<n_beta; cnt2++){//stepsize ok?
                osg::Vec2 vec(alpha*M_PI/180, beta*M_PI/180);
                camRots.push_back(vec);
                beta+=180/userParam;
            }
            beta=0;
            alpha+=180/userParam;
        }

        const std::string myString="Cam";
        size_t cnt=0;
        for(const auto& c: camPos)
        {
            for(const auto& x:camRots)
            {
               cnt++;
               cameras.push_back(new Cam(c,x,*obsPoints,myString+std::to_string(cnt)));

            }
        }
    }
    //##################################################### delete this here #####################################
  //  for(const auto& x : cameras)
  //      finalCams.push_back(new CamDrawable(x));

    //Create UI
    EKUMenu  = new ui::Menu("EKU", this);

    //Add Truck
    AddTruck = new ui::Action(EKUMenu , "addTruck");
    AddTruck->setCallback([this](){
        doAddTruck();
    });

    //Add Cam
    AddCam = new ui::Action(EKUMenu , "addCam");
    AddCam->setCallback([this](){
        doAddCam();
    });

    //Remove Truck
    RmvTruck = new ui::Action(EKUMenu , "removeTruck");
    RmvTruck->setCallback([this](){
            doRemoveTruck();
    });

    //Optimize orientation
    OptOrient = new ui::Action(EKUMenu , "OpOrient");
    OptOrient->setCallback([this](){

    });

    //Optimize nbr of cameras

    OptNbrCams = new ui::Action(EKUMenu , "OptNbrCameras");
    OptNbrCams->setCallback([this](){

        for(const auto& x: finalCams)
            x->~CamDrawable();
        finalCams.clear();

        ga =new GA(cameras,priorityList);
        auto finalCamIndex = ga->getfinalCamPos();

        size_t cnt2=0;
        for(auto& x:finalCamIndex)
        {
            if(x==1){
                   finalCams.push_back(new CamDrawable(cameras.at(cnt2)));
               }
           cnt2++;
        }
        for(const auto& x:finalCams)
        {
            cover->getObjectsRoot()->addChild(x->getCamDrawable().get());
            //add User interaction to each final camera
        //   userInteraction.push_back(new mySensor(x->getCamGeode(), x->cam->getName(), myinteraction,x,&trucks,&finalCams));
        }

    });

    //FOV
    FOVRegulator = new ui::Slider(EKUMenu , "Slider1");
    FOVRegulator->setText("FOV");
    FOVRegulator->setBounds(30., 120.);
    FOVRegulator->setValue(60.);
    FOVRegulator->setCallback([this,obsPoints](double value, bool released){
        this->disactivateDetailedRendering();
        for(auto x :finalCams)
        {
          //disactivateDetailedRendering();
          x->updateFOV(value);
          x->cam->calcVisMat(*obsPoints);
          //activateDetailedRendering();
        }
        this-> activateDetailedRendering();
    });

    //Camera visibility
    VisibilityRegulator = new ui::Slider(EKUMenu , "Slider2");
    VisibilityRegulator->setText("Visibility");
    VisibilityRegulator->setBounds(10., 60.);
    VisibilityRegulator->setValue(30.0);
    VisibilityRegulator->setCallback([this,obsPoints](double value, bool released){
        this->disactivateDetailedRendering();
        for(auto x :finalCams)
        {

          x->updateVisibility(value);
          x->cam->calcVisMat(*obsPoints);
        }
        this-> activateDetailedRendering();
    });

    //Make Cameras invisible
    MakeCamsInvisible = new ui::Button(EKUMenu , "CamerasVisible");
    MakeCamsInvisible->setText("CamerasVisible");
    MakeCamsInvisible->setState(true);
    MakeCamsInvisible->setCallback([this](bool state){
        if(!state)
        {
            for(auto x :finalCams)
            {
              x->disactivate();
            }
           // MakeCamsInvisible->setState(false);
        }
        else
        {
            for(auto x :finalCams)
            {
              x->activate();
            }
        }
    });
    //User Interaction
    myinteraction = new vrui::coTrackerButtonInteraction(vrui::coInteraction::AllButtons, "MoveMode", vrui::coInteraction::Medium);
    interActing = false;
    mymtf = new osg::MatrixTransform();
    for(const auto& x:finalCams)
    {
      //  finalScene->addChild(x->getCamDrawable().get());
        //add User interaction to each final camera
    //   userInteraction.push_back(new mySensor(x->getCamGeode(), x->cam->getName(), myinteraction,x,&trucks,&finalCams));
    }
    int cntTrucks =0;
    for(const auto& x:trucks)
    {
        //finalScene->addChild(x->getTruckDrawable().get());
        //add User interaction to each safety zone
    //    userInteraction.push_back(new mySensor(x->getTruckDrawable(),cntTrucks, "Truck", myinteraction,x,&finalCams));
        cntTrucks++;
    }
    // add sensors to sensorList
   // for(const auto& x : userInteraction)
   //     sensorList.append(x);

   // activateDetailedRendering();

    //Write obj file
  //  osgDB::writeNodeFile(*finalScene, "OpenCOVER/plugins/hlrs/EKU/EKU_result.obj");
}

EKU::~EKU()
{
    fprintf(stderr, "BorePlugin::~BorePlugin\n");

}
bool EKU::init()
{

    return true;
}

void EKU::doAddTruck()
{
    allPumps.push_back(new Pump(truck,truckSurfaceBox,allPumps.back()->getPos()+osg::Vec3(20,20,0),30));
    cover->getObjectsRoot()->addChild(allPumps.back()->getPumpDrawable().get());
}


void EKU::doRemoveTruck()
{
    std::cout<<"nbr of Trucks before"<<allPumps.size()<<std::endl;
    if (!trucks.empty())
    {
     //  cover->getObjectsRoot()->removeChild(allPumps.back()->getPumpDrawable().get());
       delete *allPumps.end();
    }


   // if(trucks.size()>0)
   //     trucks.pop_back();

    std::cout<<"nbr of Trucks after"<<allPumps.size()<<std::endl;

  //  cover->getObjectsRoot()->removeChild(allPumps.back()->getPumpDrawable().get())
    /*TOD:
    - delete Trucks from screen
    - when last element is deleted program chrashes (because first is not part of vector?)
    */
}
void EKU::doAddCam()
{

}
void EKU::removeCamDrawable(CamDrawable* cam)
{
    cam->~CamDrawable();
}

COVERPLUGIN(EKU)
