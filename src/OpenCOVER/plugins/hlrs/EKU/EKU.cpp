#include "EKU.h"


#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <string>
#include <numeric>
#include <array>
#include <cstdlib>
#include<osgFX/Outline>

using namespace opencover;

EKU *EKU::plugin = NULL;
void EKU::restrictMovement(osg::Matrix &mat)
{
    coCoord coord;
    coord = mat;
    //only rotation around z !
    coord.hpr[1] = 0;
    coord.hpr[2] = 0;
    coord.makeMat(mat);
}
void Pump::preFrame()
{
    //NOTE: also add Preframe for safetyZones!

    if(!camLeft.expired())
        camLeft.lock()->preFrame();
    if(!camRight.expired())
        camRight.lock()->preFrame();

    sensorList.update();
    //Test if button is pressed
    int state = cover->getPointerButton()->getState();
    if (myinteractionA->isRunning()) //when interacting the Sphere will be moved
    {
        static osg::Matrix invStartHand;
        static osg::Matrix startPos,startPoscamPosinterActor1,startPoscamPosinterActor2,startPosSZ1,startPosSZ2;
        if (!interActingA)
        {
            //remember invStartHand-Matrix, when interaction started and mouse button was pressed
            invStartHand.invert(cover->getPointerMat() * cover->getInvBaseMat());
            startPos = fullMat->getMatrix(); //remember position of truck, when interaction started
            if(!camLeft.expired())
                startPoscamPosinterActor1 = camLeft.lock()->getMatrix();//remember position of cam, when interaction started
            if(!camRight.expired())
                startPoscamPosinterActor2 = camRight.lock()->getMatrix();
            if(!szLeft.expired())
                startPosSZ1=szLeft.lock()->getMatrix();
            if(!szRight.expired())
                startPosSZ2=szRight.lock()->getMatrix();


            interActingA = true; //register interaction
            std::cout<<name<<"start"<<std::endl;
        }
        else if((cover->frameTime() - aSensor->getStartTime())> 0.3)
        {
            //calc the tranformation matrix when interacting is running and mouse button was pressed
            osg::Matrix trans =invStartHand * (cover->getPointerMat() * cover->getInvBaseMat());
            //trans.setTrans(osg::Vec3(0,0,0));//For rotation
            EKU::plugin->restrictMovement(trans);
            trans.setTrans(trans.getTrans().x(),trans.getTrans().y(),0);//set z trans to zero
            trans.orthoNormalize(trans);//remove scale
            osg::Matrix newTrans =  startPos * osg::Matrix::translate(trans.getTrans());//newTrans is translation only
            osg::Matrix transcamPosinterActor1 = startPoscamPosinterActor1 *  osg::Matrix::translate(trans.getTrans());
            osg::Matrix transcamPosinterActor2 = startPoscamPosinterActor2 * osg::Matrix::translate(trans.getTrans());
            osg::Matrix transSZ1 = startPosSZ1 * osg::Matrix::translate(trans.getTrans());
            osg::Matrix transSZ2 = startPosSZ2 * osg::Matrix::translate(trans.getTrans());

           /* For Rotation:
            osg::Matrix newRot = startPos *trans;//newRot is rotation only
            osg::Matrix rotcamPosinterActor1 = startPoscamPosinterActor1 * trans;
            osg::Matrix rotcamPosinterActor2 = startPoscamPosinterActor2 * trans;
            osg::Matrix rotSZ1 = startPosSZ1 * trans;
            osg::Matrix rotSZ2 = startPosSZ2 * trans;

            fullMat->setMatrix(newRot);
            //update possitions of childs:
            possibleCamLocations[0]->setPosition(rotcamPosinterActor1);
            possibleCamLocations[1]->setPosition(rotcamPosinterActor2);
            safetyZones[0]->setMat(rotSZ1);
            safetyZones[1]->setMat(rotSZ2);
*/
            // For Translation:
            fullMat->setMatrix(newTrans);
            //update possitions of childs:
            if(!szLeft.expired())
                szLeft.lock()->setMat(transSZ1);
            if(!szRight.expired())
                szRight.lock()->setMat(transSZ2);
            if(!camLeft.expired())
                camLeft.lock()->setPosition(transcamPosinterActor1);
            if(!camRight.expired())
                camRight.lock()->setPosition(transcamPosinterActor2);


        }
    }
    if (myinteractionA->wasStopped() && state == false)
    {

        interActingA = false; //unregister interaction
       // myinteractionA->cancelPendingActivation();
  //     vrui::coInteractionManager::the()->unregisterInteraction(myinteractionA);
       std::cout<<name<<"stop"<<std::endl;
  //     vrui::coInteractionManager::the()->registerInteraction(myinteractionB);


    }
/*    if (myinteractionB->isRunning()) //when interacting the Sphere will be moved
    {
        static osg::Matrix invStartHand;
        static osg::Matrix startPos;
        static coCoord startPosEuler;
        if (!interActingB)
        {
            //remember invStartHand-Matrix, when interaction started and mouse button was pressed
            invStartHand.invert(cover->getPointerMat() * cover->getInvBaseMat());
            startPos = fullMat->getMatrix(); //remember position of sphere, when interaction started
            startPosEuler = startPos;
            interActingB = true; //register interaction
      //      std::cout<<"Start Pos: " <<startPos.getTrans().x() <<","<<startPos.getTrans().y() <<","<<startPos.getTrans().z() <<std::endl;
            std::cout<<name<<"startB"<<std::endl;
        }
        else
        {
            //calc the tranformation matrix when interacting is running and mouse button was pressed
            osg::Matrix trans = startPos * invStartHand * (cover->getPointerMat() * cover->getInvBaseMat());
            EKU::plugin->restrictMovement(startPosEuler,trans,false,true);
            fullMat->setMatrix(trans);
            //update possitions of childs:
   //         safetyZones[0]->setPosition(trans);
   //         safetyZones[1]->setPosition(trans);
            for(const auto& x:possibleCamLocations)
            {
   //             x->setPosition(trans);
            }
            std::cout<<name<<"continueB"<<std::endl;
        }

    }
    if (myinteractionB->wasStopped() && state == false)
    {
        interActingB = false; //unregister interaction
        //vrui::coInteractionManager::the()->unregisterInteraction(myinteractionB);
        std::cout<<name<<"stopB"<<std::endl;




    }

*/
}

void EKU::preFrame()
{
    sensorList.update();
    for(const auto &x: allPumps)
        x->preFrame();
    for(const auto &x: allCamPositions)
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
Pump::Pump(std::vector<std::shared_ptr<CamPosition>>& allCams,std::vector<std::shared_ptr<SafetyZone>> &allSZ,osg::ref_ptr<osg::Node> truck,osg::ref_ptr<osg::Node> truckSurface =nullptr, osg::ref_ptr<osg::Node> cabine =nullptr, osg::Vec3 pos =osg::Vec3(20,0,0), int rotationZ = 0):allCams(allCams),allSZ(allSZ),position(pos),truck(truck),truckSurfaceBox(truckSurface),rotZ(rotationZ),truckCabine(cabine)
{
    Pump::counter ++;
    group = new osg::Group;
    group->setName("DetailedTruck+Cam+Safety"+std::to_string(Pump::counter));
   // group->setStateSet(UINT_MAX);
  // truck = new osg::Node;
    std::cout<<"stateset"<<    truck->getNodeMask()<<std::endl;

 //   truck->setName("Truck"+std::to_string(Pump::counter));
  //  truckSurfaceBox = new osg::Node;
   // truckSurfaceBox->setName("SurfaceBox"+std::to_string(Pump::counter));
    name = "DetailedTruck+Cam+Safety"+std::to_string(Pump::counter);
    //create safety Zones
    osg::Matrix localSafetyZone1,localSafetyZone2;
    localSafetyZone1.setTrans(osg::Vec3(-2.3,0.1,9));
    localSafetyZone2.setTrans(osg::Vec3(2.3,0.1,9));

   // safetyZones.at(0) = new SafetyZone(localSafetyZone1,SafetyZone::PRIO2,2.0f,2.0f,8.0f); //safetyZone Dimensions: 2,2,8
   // safetyZones.at(1) = new SafetyZone(localSafetyZone2,SafetyZone::PRIO2,2.0f,2.0f,8.0f);
    std::shared_ptr<SafetyZone> sz1 =std::make_shared<SafetyZone>(localSafetyZone1,SafetyZone::PRIO2,2.0f,2.0f,8.0f);
    std::shared_ptr<SafetyZone> sz2 =std::make_shared<SafetyZone>(localSafetyZone2,SafetyZone::PRIO2,2.0f,2.0f,8.0f);
    szLeft = sz1;
    szRight = sz2;
    allSZ.push_back(std::move(sz1));
    allSZ.push_back(std::move(sz2));


    //create possible Cam locations ####Old values of Points:Vec3(1.5,1.6,-0.5) Vec3(-1.6,1.6,-0.5)
    osg::Matrix localInteractor1,localInteractor2;
    osg::Quat rotInteractor;
    rotInteractor.makeRotate(osg::DegreesToRadians(-90.0),osg::X_AXIS);
    localInteractor1.setTrans(osg::Vec3(1.6,1.6,-0.5));
    localInteractor1.setRotate(rotInteractor);
    std::shared_ptr<CamPosition> c1 =std::make_shared<CamPosition>(localInteractor1,this);
    camLeft = c1;
    allCams.push_back(std::move(c1));
   // possibleCamLocations.push_back(new CamPosition(localInteractor1));//to side , height , forward
    localInteractor2.setTrans(osg::Vec3(-1.6,1.6,-0.5));
    localInteractor2.setRotate(rotInteractor);
    std::shared_ptr<CamPosition> c2 =std::make_shared<CamPosition>(localInteractor2,this);
    camRight =c2;
    allCams.push_back(std::move(c2));


   // possibleCamLocations.push_back(new CamPosition(localInteractor2));

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

    group->addChild(truck.get());
   group->addChild(truckSurfaceBox.get());

   //Drivers Cabine
   rotCabine = new osg::MatrixTransform();
   osg::Matrix rotate1;
   osg::Quat xRot1, yRot1;
   xRot1.makeRotate(osg::DegreesToRadians(90.0),osg::Z_AXIS);
   yRot1.makeRotate(osg::DegreesToRadians(90.0),osg::Y_AXIS);
   osg::Quat fullRot1 = yRot1*xRot1;
   rotate1.setRotate(fullRot1);
   rotCabine ->setName("RotCabine");
   rotCabine->setMatrix(rotate1);
   rotCabine->addChild(truckCabine.get());

   transCabine= new osg::MatrixTransform();
   transCabine->setName("TransCabine");
   osg::Matrix translate1;
   translate1.setTrans(osg::Vec3(-0.8,-0.8,-3.3));
   transCabine->setMatrix(translate1);
   transCabine->addChild(rotCabine.get());

   group->addChild(transCabine.get());
   group1 = new osg::Group;
   group1->setName("bothTruckDrawables"+std::to_string(Pump::counter));

   group1->addChild(szLeft.lock()->getSafetyZoneDrawable().get());
   group1->addChild(szRight.lock()->getSafetyZoneDrawable().get());


    group1->addChild(group.get());
    //rotMat->addChild(group1.get());

    //Translation
    transMat= new osg::MatrixTransform();
    transMat->setName("Translation"+std::to_string(Pump::counter));
    osg::Matrix translate;
    //translate.setTrans(0,0,0);
    translate.setTrans(pos.x(),pos.y(),pos.z());
    transMat->setMatrix(translate);
    //transMat->addChild(rotMat.get());

    osg::Matrix full;
    full=rotate*translate;
    fullMat = new osg::MatrixTransform();
    fullMat ->setName("full"+std::to_string(Pump::counter));
    fullMat->setMatrix(full);
    fullMat->addChild(group1.get());

     //User Interaction
     myinteractionA = new vrui::coTrackerButtonInteraction(vrui::coInteraction::ButtonA, "MoveMode", vrui::coInteraction::Highest);

     interActingA = false;

     aSensor = new mySensor(group, name, myinteractionA);

     sensorList.append(aSensor);

     szLeft.lock()->setMat(localSafetyZone1* full);
     szRight.lock()->setMat(localSafetyZone2* full);

     //safetyZones.at(0)->setMat(localSafetyZone1* full);
    //safetyZones.at(1)->setMat(localSafetyZone2* full);

     upperGroup= new osg::MatrixTransform();
     upperGroup->setName("Truck"+std::to_string(Pump::counter));
     upperGroup->addChild(fullMat.get());
    // for(const auto& x : possibleCamLocations)
    upperGroup->addChild(camLeft.lock()->getCamGeode().get());
    upperGroup->addChild(camRight.lock()->getCamGeode().get());
    camLeft.lock()->setPosition(localInteractor1*full);
    camRight.lock()->setPosition(localInteractor2*full);

    // possibleCamLocations.at(0)->setPosition(localInteractor1*full);
    // possibleCamLocations.at(1)->setPosition(localInteractor2*full);

     cover->getObjectsRoot()->addChild(upperGroup.get());
}
Pump::~Pump()
{
    if (sensorList.find(aSensor))
        sensorList.remove();
    delete aSensor;
    std::cout<<"deleted Pump: "<<name<<std::endl;
    upperGroup->getParent(0)->removeChild(upperGroup);
}
/*void EKU::createSafetyZone(float xpos, float ypos, SafetyZone::Priority prio)
{
    float height =2;
    float length =3;
    float width =3;
    for(int y =0;y< 3; y++)
    {
        for(int x =0;x< 2; x++)
        {
            osg::Vec3 pos{xpos+x*1.1f*length,ypos+y*1.11f*width,0.0f};
            safetyZones.push_back(new SafetyZone(pos,prio,length,width,height));
            finalScene->addChild( safetyZones.back()->getSafetyZoneDrawable().get());
            //update PRIO List afterwards !
        }
    }
    cover->getObjectsRoot()->addChild(finalScene.get());

}
*/
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
 /*   {
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
*/    cover->getObjectsRoot()->addChild(finalScene.get());

}

EKU::EKU(): ui::Owner("EKUPlugin", cover->ui)
{
   // Wireframe *test = new Wireframe();
    //cover->getObjectsRoot()->addChild(test->getZoneGeode().get());


   // newSZ= new SZ("test",1,osg::Vec3(0,0,0),osg::Vec3(0,0,0),1.0,1.0);
    //Create user Interation


    osg::Matrix m,m2,mC1,mC2;
    m.setTrans(5.0,15,0);
    m2.setTrans(5.0,20,-15);

    mC2.setTrans(0.0,0.0,0);
    mC2.setTrans(0,0,0);
  //  testCam = new CamPosition(mC1);
   // cover->getObjectsRoot()->addChild(testCam->getCamGeode().get());

   //   allCamPositions.push_back(new CamPosition(mC2));
   //std::shared_ptr<CamPosition> a = std::make_shared<CamPosition>(mC2);

 /*  for(const auto & x:allCamPositions)
   {
    cover->getObjectsRoot()->addChild(x->getCamGeode().get());
    }
*/

    //safetyZones.push_back(new SafetyZone (m,SafetyZone::PRIO2,2.0f,2.0f,2.0f) );
   // safetyZones.push_back(new SafetyZone (m2,SafetyZone::PRIO2,2.0f,2.0f,2.0f) );

 /*   for(const auto & x:safetyZones)
    {
     cover->getObjectsRoot()->addChild(x->getSafetyZoneDrawable().get());
     }
*/

    myinteraction = new vrui::coTrackerButtonInteraction(vrui::coInteraction::ButtonA, "MoveMode", vrui::coInteraction::Medium);
    interActing = false;
    mymtf = new osg::MatrixTransform();

    plugin = this;
    fprintf(stderr, "EKUplugin::EKUplugin\n");
    finalScene = new osg::Group;
    finalScene->setName("finalScene");
    createScene();
    for(const auto& x : userInteraction)
        sensorList.append(x);




    truck = coVRFileManager::instance()->loadFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/Truck/truck_surface.stl",NULL,finalScene);
    if (!truck.valid())
    {
          osg::notify( osg::FATAL ) << "Unable to load truck data file. Exiting." << std::endl;
    }
    truck->setName("Truck");

    truckSurfaceBox = osgDB::readNodeFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/Truck/Truck_surface_box.osgt");
    if (!truck.valid())
    {
          osg::notify( osg::FATAL ) << "Unable to load truck data file. Exiting." << std::endl;
    }
    truckSurfaceBox->setName("Surface");

    truckCabine = osgDB::readNodeFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/Truck/drivers_cab/AssetsvilleTruck/AssetVille.3ds");
    if (!truckCabine.valid())
    {
          osg::notify( osg::FATAL ) << "Unable to load truck data cabine file. Exiting." << std::endl;
    }
    truckCabine->setName("TruckCabine");

    // don't show the many vertices of the truck. Only use truck surface
    disactivateDetailedRendering();



    //draw Pumps:
    std::unique_ptr<Pump> newPump(new Pump(allCamPositions,safetyZones,truck,truckSurfaceBox,truckCabine));
    allPumps.push_back(std::move(newPump));
/*
    int cnt =0;
    for(int i = 0;i<5;i++)
    {
        osg::Vec3 posOld=allPumps.back()->getPos();;
        osg::Vec3 posNew;
        int rotZ = allPumps.back()->getRot();
        if(cnt % 2 == 0)
        {
             posNew = {posOld.x()*-1,posOld.y(),posOld.z()};
              std::unique_ptr<Pump> newPump(new Pump(allCamPositions,safetyZones,truck,truckSurfaceBox,truckCabine,posNew,180));
              allPumps.push_back(std::move(newPump));

        }else
        {
             posNew = {posOld.x()*-1,posOld.y()+5,posOld.z()};//+5
             std::unique_ptr<Pump> newPump(new Pump(allCamPositions,safetyZones ,truck,truckSurfaceBox,truckCabine,posNew));
             allPumps.push_back(std::move(newPump));

        }

        cnt ++;
    }
 */


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

    //Add PRIO1
    AddPRIO1 = new ui::Action(EKUMenu , "addPRIO1");
    AddPRIO1->setCallback([this](){
        //createSafetyZone(-6.0,-5.0,SafetyZone::PRIO1);
    });

    //Add PRIO2
    AddPRIO2 = new ui::Action(EKUMenu , "addPRIO2");
    AddPRIO2->setCallback([this](){
        //createSafetyZone(-6.0,-5.0,SafetyZone::PRIO2);
    });

    //Remove Truck
    RmvTruck = new ui::Action(EKUMenu , "removeTruck");
    RmvTruck->setCallback([this](){
            doRemoveTruck(allPumps.back());
    });

    //Remove Cam
    RmvCam = new ui::Action(EKUMenu , "removeCam");
    RmvCam->setCallback([this](){
            doRemoveCam(allCamPositions.back());
    });



    //CalcVisMat
 /*   CalcVisMat = new ui::Action(EKUMenu , "CalcVisMat");
    CalcVisMat->setCallback([this](){
        for(const auto& x : allPumps)
        {
            for(const auto& x1 : x->possibleCamLocations )
                //x1->allCameras

        }
    });
*/
    //Optimize orientation
    OptOrient = new ui::Action(EKUMenu , "OpOrient");
    OptOrient->setCallback([this](){

    });

    //Optimize nbr of cameras

    OptNbrCams = new ui::Action(EKUMenu , "OptNbrCameras");
    OptNbrCams->setCallback([this](){
        for(const auto& x: allCamPositions)
        {
            //for(const auto& x1 : x->allCameras)
              //  x1->calcVisMat(observationPoints);
        }
/*        for(const auto& x: finalCams)
            x->~CamDrawable();
        finalCams.clear();
        createCamsForEachCamPos();

       // std::array<int,192> finalCamIndex;
        //if (coVRMSController::instance()->isMaster())
        //{
                ga =new GA(cameras,safetyZones);
                auto finalCamIndex = ga->getfinalCamPos();
                delete this->ga;

        //}
        //coVRMSController::instance()->syncData(&finalCamIndex, finalCamIndex.size());
        size_t cnt2=0;
        for(auto& x:finalCamIndex)
        {
            
            if(x==1){
                   finalCams.push_back(new CamDrawable(cameras.at(cnt2)));
               }
           cnt2++;
        }
*/
/*         //this is for GA with std::vector<std::array>
         size_t cnt2=0;
         for(const auto& x:finalCamIndex)
         {
            for(const auto& y : x)
            {
                if(y==1){
                finalCams.push_back(new CamDrawable(cameras.at(cnt2)));
                }
                cnt2++;
            }
         }
*/         //add User interaction to each final camera
/*        for(const auto& x:finalCams)
        {
            cover->getObjectsRoot()->addChild(x->getCamGeode().get());
            userInteraction.push_back(new mySensor(x->getCamGeode(), x->cam->getName(), myinteraction,x,&safetyZones,&finalCams));
        }

        //add User interaction to each safety zone
        int cntsafetyZones =0;
        for(const auto& x:safetyZones)
        {
            userInteraction.push_back(new mySensor(x->getSafetyZoneDrawable(),cntsafetyZones, "SafetyZone", myinteraction,x,&finalCams));
            cntsafetyZones++;
        }
        // add sensors to sensorList
        for(const auto& x : userInteraction)
            sensorList.append(x);
*/
    });

    //FOV
    FOVRegulator = new ui::Slider(EKUMenu , "Slider1");
    FOVRegulator->setText("FOV");
    FOVRegulator->setBounds(30., 120.);
    FOVRegulator->setValue(60.);
    FOVRegulator->setCallback([this](double value, bool released){
        this->disactivateDetailedRendering();
        for(auto x :finalCams)
        {
          x->updateFOV(value);
          //x->cam->calcVisMat(observationPoints);
        }
      //  this-> activateDetailedRendering();

    });

    //Camera visibility
    VisibilityRegulator = new ui::Slider(EKUMenu , "Slider2");
    VisibilityRegulator->setText("Visibility");
    VisibilityRegulator->setBounds(10., 60.);
    VisibilityRegulator->setValue(30.0);
    VisibilityRegulator->setCallback([this](double value, bool released){
        this->disactivateDetailedRendering();
        for(auto x :finalCams)
        {

          x->updateVisibility(value);
         // x->cam->calcVisMat(observationPoints);
        }
      //  this-> activateDetailedRendering();
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

    //Show search space
    MakeCamsInvisible = new ui::Button(EKUMenu , "ShowSearchSpace");
    MakeCamsInvisible->setText("ShowSearchSpace");
    MakeCamsInvisible->setState(false);
    MakeCamsInvisible->setCallback([this](bool state){

       for(const auto &x :allCamPositions)
       {
            x->setSearchSpaceState(state);
            x->setSearchSpaceState(state);
       }
    });

    for(const auto& x:finalCams)
    {
      //  finalScene->addChild(x->getCamDrawable().get());
        //add User interaction to each final camera
    //   userInteraction.push_back(new mySensor(x->getCamGeode(), x->cam->getName(), myinteraction,x,&safetyZones,&finalCams));
    }



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
    if(!allPumps.empty())
    {
        std::unique_ptr<Pump> newPump(new Pump(allCamPositions,safetyZones ,truck,truckSurfaceBox,truckCabine,allPumps.back()->getPos()+osg::Vec3(20,20,0),30));
        allPumps.push_back(std::move(newPump));
    }
    else
    {
        std::unique_ptr<Pump> newPump(new Pump(allCamPositions,safetyZones,truck,truckSurfaceBox,truckCabine));
        allPumps.push_back(std::move(newPump));

    }
}


void EKU::doRemoveTruck(std::unique_ptr<Pump> &truck)
{


    if(!allPumps.empty())
    {
        std::cout<<"nbr of Trucks before"<<allPumps.size()<<std::endl;
        std::cout<<"nbr of Cams before"<<allCamPositions.size()<<std::endl;
        std::cout<<"nbr of SZ before"<<safetyZones.size()<<std::endl;

        //delete cameras from list
        if(!truck->camLeft.expired())
        {    string id = truck->camLeft.lock()->getName();
             allCamPositions.erase(std::remove_if(allCamPositions.begin(),allCamPositions.end(),[&id](std::shared_ptr<CamPosition>const& it){return it->getName() == id;}));
        }
        if(!truck->camRight.expired())
        {   string id = truck->camRight.lock()->getName();
            allCamPositions.erase(std::remove_if(allCamPositions.begin(),allCamPositions.end(),[&id](std::shared_ptr<CamPosition>const& it){return it->getName() == id;}));
        }
        //TODO: delete SafetyZones from list:

        //delete Pump
        allPumps.erase(std::remove_if(allPumps.begin(),allPumps.end(),[&truck](std::unique_ptr<Pump>const& it){return truck == it;}));

        std::cout<<"nbr of Trucks after"<<allPumps.size()<<std::endl;
        std::cout<<"nbr of Cams after"<<allCamPositions.size()<<std::endl;
        std::cout<<"nbr of SZ after"<<safetyZones.size()<<std::endl;
    }
    else
        std::cout<<"No trucks available"<<std::endl;


}
void EKU::doAddCam()
{
    osg::Matrix localInteractor;
   // osg::Quat rotInteractor;
   // rotInteractor.makeRotate(osg::DegreesToRadians(-90.0),osg::X_AXIS);
    localInteractor.setTrans(osg::Vec3(30,-30,5));
   // localInteractor.setRotate(rotInteractor);
    std::shared_ptr<CamPosition> c1 =std::make_shared<CamPosition>(localInteractor);
    allCamPositions.push_back(std::move(c1));
}
void EKU::doRemoveCam(std::shared_ptr<CamPosition> &camera)
{

    if(!allCamPositions.empty())
    {
        std::cout<<"nbr of Cams before"<<allCamPositions.size()<<std::endl;

        allCamPositions.erase(std::remove_if(allCamPositions.begin(),allCamPositions.end(),[&camera](std::shared_ptr<CamPosition>const& it){return camera == it;}));

        std::cout<<"nbr of Cams after"<<allCamPositions.size()<<std::endl;
    }
    else
        std::cout<<"No cameras available"<<std::endl;


}



COVERPLUGIN(EKU)
