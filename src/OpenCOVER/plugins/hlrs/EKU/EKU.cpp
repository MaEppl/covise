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
bool EKU::modifyScene=true;
std::vector<std::shared_ptr<SafetyZone>> EKU::safetyZones;
std::vector<std::shared_ptr<CamPosition>> EKU::allCamPositions;
std::vector<std::unique_ptr<Pump>> EKU::allPumps;

double getZvalueOfSZ()
{
  //  for(const auto& x : saf)

}

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
    if(EKU::modifyScene == true)
    {
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
        /*        if(!szLeft.expired())
                    startPosSZ1=szLeft.lock()->getMatrix();
                if(!szRight.expired())
                    startPosSZ2=szRight.lock()->getMatrix();
        */

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
          //      osg::Matrix transSZ1 = startPosSZ1 * osg::Matrix::translate(trans.getTrans());
          //      osg::Matrix transSZ2 = startPosSZ2 * osg::Matrix::translate(trans.getTrans());

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
            /*   if(!szLeft.expired())
                   szLeft.lock()->setPosition(transSZ1);
               if(!szRight.expired())
                   szRight.lock()->setPosition(transSZ2);
            */    if(!camLeft.expired())
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
    for(const auto &x: safetyZones)
        x->preFrame();

   // newSZ->preFrame();

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
    osg::Quat rotInteractor;
    rotInteractor.makeRotate(osg::DegreesToRadians(-90.0),osg::X_AXIS);
    localSafetyZone1.setTrans(osg::Vec3(-2.2,0.2,8));
    localSafetyZone1.setRotate(rotInteractor);
    localSafetyZone2.setTrans(osg::Vec3(2.2,0.2,8));
    localSafetyZone2.setRotate(rotInteractor);


   // safetyZones.at(0) = new SafetyZone(localSafetyZone1,SafetyZone::PRIO2,2.0f,2.0f,8.0f); //safetyZone Dimensions: 2,2,8
   // safetyZones.at(1) = new SafetyZone(localSafetyZone2,SafetyZone::PRIO2,2.0f,2.0f,8.0f);
 /*   std::shared_ptr<SafetyZone> sz1 =std::make_shared<SafetyZone>(localSafetyZone1,SafetyZone::PRIO1,1.0f,11.0f,2.0f);
    std::shared_ptr<SafetyZone> sz2 =std::make_shared<SafetyZone>(localSafetyZone2,SafetyZone::PRIO1,1.0f,11.0f,2.0f);
    szLeft = sz1;
    szRight = sz2;
    allSZ.push_back(std::move(sz1));
    allSZ.push_back(std::move(sz2));
*/

    //create possible Cam locations ####Old values of Points:Vec3(1.5,1.6,-0.5) Vec3(-1.6,1.6,-0.5)
    osg::Matrix localInteractor1,localInteractor2;
    localInteractor1.setTrans(osg::Vec3(1.6,1.6,-0.5));
    localInteractor1.setRotate(rotInteractor);
    std::shared_ptr<CamPosition> c1 =std::make_shared<CamPosition>(localInteractor1,this);
    camLeft = c1;
    allCams.push_back(std::move(c1));
    //possibleCamLocations.push_back(new CamPosition(localInteractor1));//to side , height , forward
    localInteractor2.setTrans(osg::Vec3(-1.6,1.6,-0.5));
    localInteractor2.setRotate(rotInteractor);
    std::shared_ptr<CamPosition> c2 =std::make_shared<CamPosition>(localInteractor2,this);
    camRight =c2;
    allCams.push_back(std::move(c2));



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



     //safetyZones.at(0)->setMat(localSafetyZone1* full);
    //safetyZones.at(1)->setMat(localSafetyZone2* full);

     upperGroup= new osg::MatrixTransform();
     upperGroup->setName("Truck"+std::to_string(Pump::counter));
     upperGroup->addChild(fullMat.get());
    // for(const auto& x : possibleCamLocations)
     upperGroup->addChild(camLeft.lock()->getCamGeode().get());
     upperGroup->addChild(camRight.lock()->getCamGeode().get());
//    upperGroup->addChild(szLeft.lock()->getSZ());
//    upperGroup->addChild(szRight.lock()->getSZ());
     camLeft.lock()->setPosition(localInteractor1*full);
     camRight.lock()->setPosition(localInteractor2*full);
//    szLeft.lock()->setPosition(localSafetyZone1* full);
//    szRight.lock()->setPosition(localSafetyZone2* full);

     //possibleCamLocations.at(0)->setPosition(localInteractor1*full);
     //possibleCamLocations.at(1)->setPosition(localInteractor2*full);

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
void EKU::createSafetyZone(float xpos, float ypos, SafetyZone::Priority prio)
{
    float height =2;
    float length =3;
    float width =3;
    for(int y =0;y< 3; y++)
    {
        for(int x =0;x< 2; x++)
        {
            osg::Vec3 pos{xpos+x*1.1f*length,ypos+y*1.11f*width,0.0f};
            osg::Matrix m;
            m.setTrans(pos);
            std::shared_ptr<SafetyZone> sz = std::make_shared<SafetyZone>(m,prio,length,width,height);
            safetyZones.push_back(std::move(sz));
            finalScene->addChild( safetyZones.back()->getSafetyZoneDrawable().get());
            //update PRIO List afterwards !
        }
    }
    cover->getObjectsRoot()->addChild(finalScene.get());

}

void EKU::createScene()
{
    //add Christmas Tree
    {
        std::cout<<"Load Christmas Tree"<<std::endl;
        christmasTree = osgDB::readNodeFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/xmas-tree-oil-and-gas-1.snapshot.4/ChristmasTree.ply");
        christmasTree->setName("ChristmasTree");
        osg::PositionAttitudeTransform* scale = new osg::PositionAttitudeTransform();
        scale->setScale(osg::Vec3(0.003,0.003,0.003));
        scale->addChild(christmasTree);
        osg::PositionAttitudeTransform* rotate = new osg::PositionAttitudeTransform();
        osg::Quat zRot;
        zRot.makeRotate(osg::DegreesToRadians(90.0), osg::X_AXIS);
        rotate->setAttitude(osg::Quat(zRot));
        rotate->addChild(scale);
        osg::PositionAttitudeTransform* move = new osg::PositionAttitudeTransform();
        move->setPosition( osg::Vec3( -15.0f, 35.0f, -0.7f) );
        move->addChild(rotate);
        finalScene->addChild(move);

        osg::PositionAttitudeTransform* move1 = new osg::PositionAttitudeTransform();
        move1->setPosition( osg::Vec3( 10.0f, 0.0f, 0.f) );
        move1->addChild(move);
        finalScene->addChild(move1);

        osg::PositionAttitudeTransform* move2 = new osg::PositionAttitudeTransform();
        move2->setPosition( osg::Vec3( 20.0f, 0.0f, 0.f) );
        move2->addChild(move);
        finalScene->addChild(move2);
    }



    //add silo
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

        silo2 = osgDB::readNodeFile("/home/AD.EKUPD.COM/matthias.epple/data/EKU/World/Silo/Model_C0810A007/C0810A007.3ds");
        //"/home/AD.EKUPD.COM/matthias.epple/Downloads/terrain-map-edit-obj/terrain-map-edit.obj"  <-- das hat mal funktioniert !
        silo2->setName("Silo2");
        osg::PositionAttitudeTransform* move4 = new osg::PositionAttitudeTransform();
        move4->setPosition( osg::Vec3( +35.0f, -10.0f, 3.f) );
        move4->addChild(silo2);
        finalScene->addChild(move4);

        osg::PositionAttitudeTransform* move5 = new osg::PositionAttitudeTransform();
        move5->setPosition( osg::Vec3( +35.0f, -5.0f, 3.f) );
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
        scale->setScale(osg::Vec3(0.03,0.03,0.03));
        osg::PositionAttitudeTransform* move = new osg::PositionAttitudeTransform();
        move->setPosition( osg::Vec3( -45.0f, 45.0f, 0.f) );
        osg::PositionAttitudeTransform* rotate = new osg::PositionAttitudeTransform();
        osg::Quat zRot;
        zRot.makeRotate(osg::DegreesToRadians(45.0), osg::Z_AXIS);
        rotate->setAttitude(osg::Quat(zRot));
        scale->addChild(container);
        rotate->addChild(scale);
        move->addChild(rotate);
        finalScene->addChild(move);

        osg::PositionAttitudeTransform* move1 = new osg::PositionAttitudeTransform();
        move1->setPosition( osg::Vec3( -5.0f, 5.0f, 0.f) );
        move1->addChild(move);
        finalScene->addChild(move1);

        osg::PositionAttitudeTransform* move2 = new osg::PositionAttitudeTransform();
        move2->setPosition( osg::Vec3( -5.0f, 5.0f, 0.f) );
        move2->addChild(move);
        finalScene->addChild(move2);

        std::cout<<"Container loaded"<<std::endl;

    }
    cover->getObjectsRoot()->addChild(finalScene.get());
}

EKU::EKU(): ui::Owner("EKUPlugin", cover->ui)
{
    //enable building KDTree structure for all geometries loaded: --> intersection Test is much faster!
    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::Options::BUILD_KDTREES);

    plugin = this;
    fprintf(stderr, "EKUplugin::EKUplugin\n");
    finalScene = new osg::Group;
    finalScene->setName("finalScene");
    createScene();

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

  int cnt =0;
    for(int i = 0;i<11;i++)
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
             posNew = {posOld.x()*-1,posOld.y()+4,posOld.z()};//+5
             std::unique_ptr<Pump> newPump(new Pump(allCamPositions,safetyZones ,truck,truckSurfaceBox,truckCabine,posNew));
             allPumps.push_back(std::move(newPump));

        }

        cnt ++;
    }

    std::unique_ptr<Pump> blender(new Pump(allCamPositions,safetyZones,truck,truckSurfaceBox,truckCabine,osg::Vec3(7,-5,0)));
    allPumps.push_back(std::move(blender));

    doAddPRIO1(osg::Vec3(20,-10,0),40,45,2);



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
        osg::Vec3 pos(0.0,-20.0,0.5);
        doAddPRIO1(pos,10.0,10.0,2.0);
    });

    //Add PRIO2
    AddPRIO2 = new ui::Action(EKUMenu , "addPRIO2");
    AddPRIO2->setCallback([this](){
        osg::Vec3 pos(10.0,-20.0,0.5);
        doAddPRIO2(pos,10.0,10.0,2.0);
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
    //Remove Safety Zone
    RmvSafetyZone = new ui::Action(EKUMenu , "removeSafetyZone");
    RmvSafetyZone->setCallback([this](){
            doRemovePRIOZone(safetyZones.back());
    });
    //Calc VisMat
    calcVisMat = new ui::Action(EKUMenu , "calcVisMat");
    calcVisMat->setCallback([this](){
    /*    std::vector<osg::Vec3> pointsToObserve;
        pointsToObserve.reserve(safetyZones.size());
        for(const auto& x : safetyZones)
            pointsToObserve.push_back(x->getPosition());
*/
        for(const auto& x : allCamPositions)
        {
            x->camDraw->cam->calcVisMat();
            for(const auto& x1: x->allCameras)
                x1->calcVisMat();
        }
    });


    //Menu for Optimization

    Optimize = new ui::Menu(EKUMenu, "Optimize");
    Optimize->setText("Optimize");
    //Optimize orientation
    OptOrient = new ui::Action(EKUMenu, "OpOrient");
    OptOrient->setCallback([this](){
    /*   std::vector<osg::Vec3> pointsToObserve;
        pointsToObserve.reserve(safetyZones.size());
        for(const auto& x : safetyZones)
            pointsToObserve.push_back(x->getPosition());
*/
        for(const auto& x : allCamPositions)
        {
            x->camDraw->cam->calcVisMat();
            for(const auto& x1: x->allCameras)
                x1->calcVisMat();
        }
        //show Points which are currently not visible
        findNotVisiblePoints();


        GA::nbrCamPositions=allCamPositions.size();
        GA::nbrCamsPerCamPosition=allCamPositions.front()->allCameras.size();
        GA::nbrPoints = safetyZones.size();

        ga =new GA(allCamPositions,safetyZones);
        delete this->ga;

        ModifyScene->setState(false);
        modifyScene = false;

        for(const auto& x : safetyZones)
        {
            x->changeInteractorStatus(false);
        }

        for(const auto& x : allCamPositions)
        {
            x->camDraw->cam->calcVisMat();
            for(const auto& x1: x->allCameras)
                x1->calcVisMat();
        }
          //show Points which are currently not visible
          findNotVisiblePoints();
    });

    //Optimize nbr of cameras
    OptNbrCams = new ui::Action(EKUMenu , "OptNbrCameras");
    OptNbrCams->setCallback([this](){
       osg::Matrix m;
       m.setTrans(osg::Vec3(100,0,0));
        allCamPositions.begin()->get()->setPosition(m);
     /*   std::vector<osg::Vec3> pointsToObserve;
        pointsToObserve.reserve(safetyZones.size());
        for(const auto& x : safetyZones)
            pointsToObserve.push_back(x->getPosition());

        for(const auto& x : allCamPositions)
        {
            x->camDraw->cam->calcVisMat();
            for(const auto& x1: x->allCameras)
                x1->calcVisMat();
        }

        std::vector<std::shared_ptr<Cam>> c;
        c.reserve(allCamPositions.size()*allCamPositions.at(1)->allCameras.size());

        for(const auto &x: allCamPositions)
        {
            for (const auto &x1 : x->allCameras)
                c.push_back(x1);
        }
        //if (coVRMSController::instance()->isMaster())
        GA::nbrCamPositions=allCamPositions.size();
        GA::nbrCamsPerCamPosition=allCamPositions.at(1)->allCameras.size();
        GA::nbrPoints = safetyZones.size();
        ga =new GA(c,safetyZones);
        auto finalCamIndex = ga->getfinalCamPos();
        delete this->ga;


        //coVRMSController::instance()->syncData(&finalCamIndex, finalCamIndex.size());
        size_t cnt2=0;
        for(auto& x:finalCamIndex)
        {
            
           // if(x==1){
                   //finalCams.push_back(new CamDrawable(cameras.at(cnt2)));
          //     }
          // cnt2++;
        }

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

    //StopGA
    StopGA = new ui::Action(Optimize , "StopOptimization");
    StopGA->setText("Stop Optimization");
    StopGA->setCallback([this](){
        if(ga != nullptr)
            ga->stopGA();
    });
    //penalty
    penalty = new ui::Slider(Optimize , "penalty");
    penalty->setText("penalty");
    penalty->setBounds(1., 30.);
    penalty->setValue(GA::penalty);
    penalty->setCallback([this](double value, bool released){
        GA::penalty =value;
    });
    //weight for Prio1
    weighting = new ui::Slider(Optimize , "weighting");
    weighting->setText("weigthing PRIO1");
    weighting->setBounds(1., 10.);
    weighting->setValue(GA::weightingPRIO1);
    weighting->setCallback([this](double value, bool released){
        GA::weightingPRIO1=value;
    });


    //FOV
    FOVRegulator = new ui::Slider(EKUMenu , "Slider1");
    FOVRegulator->setText("FOV");
    FOVRegulator->setBounds(30., 120.);
    FOVRegulator->setValue(Cam::fov);
    FOVRegulator->setCallback([this](double value, bool released){
        this->disactivateDetailedRendering();
        for(const auto &x : allCamPositions)
        {
          x->camDraw->updateFOV(value);
          //x->cam->calcVisMat(observationPoints);
        }
      //  if(released)
            //update visMat

        //  this-> activateDetailedRendering();

    });

    //Camera visibility
    VisibilityRegulator = new ui::Slider(EKUMenu , "Slider2");
    VisibilityRegulator->setText("Visibility");
    VisibilityRegulator->setBounds(10., 100.);
    VisibilityRegulator->setValue(Cam::depthView);
    VisibilityRegulator->setCallback([this](double value, bool released){
        this->disactivateDetailedRendering();
        for(const auto& x :allCamPositions)
        {

          x->camDraw->updateVisibility(value);
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
            for(const auto &x :allCamPositions)
            {
              x->camDraw->disactivate();
            }
           // MakeCamsInvisible->setState(false);
        }
        else
        {
            for(const auto &x :allCamPositions)
            {
              x->camDraw->activate();
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

    //Modify scene
    ModifyScene = new ui::Button(EKUMenu , "ModifyScene");
    ModifyScene->setText("ModifyScene");
    ModifyScene->setState(true);
    ModifyScene->setCallback([this](bool state){
       modifyScene = state;
       if(modifyScene == false)
       {
         for(const auto& x : safetyZones)
         {
             x->changeInteractorStatus(false);
         }

         for(const auto& x : allCamPositions)
         {
             x->camDraw->cam->calcVisMat();
             for(const auto& x1: x->allCameras)
                 x1->calcVisMat();
         }
           //show Points which are currently not visible
           findNotVisiblePoints();
       }
       else
       {
           for(const auto& x : safetyZones)
           {
               x->changeInteractorStatus(true);
           }

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

void EKU::findNotVisiblePoints()
{
    if(!allCamPositions.empty())
    {
        std::vector<std::vector<double>> result(safetyZones.size());
        result = allCamPositions.front()->camDraw->cam->visMat;
        for(auto &x:result)
            std::fill(x.begin(), x.end(), 0);

        size_t counter =0;
        for(const auto& x : allCamPositions)
        {
            //init with 0
           // result.at(counter).assign(x->camDraw->cam->visMat.at(counter).size(),0);

            for(const auto& x1: x->camDraw->cam->visMat)
            {
                std::transform(x1.begin(),x1.end(),result.at(counter).begin(),result.at(counter).begin(),std::plus<double>());
                counter ++;

            }
            counter =0;

        }
        //go over all SZ and check if nbr of Cams == PRIO zone!
        counter =0;
        for(const auto& x :result)
        {
            std::vector<double>visMat;
            visMat.reserve(x.size());
            for(const auto&x1 :x)
            {
                if(safetyZones.at(counter)->getPriority()>x1)
                    visMat.push_back(1);
                else
                    visMat.push_back(0);

            }
            safetyZones.at(counter)->pointsVisibleForEnoughCameras(visMat);
            counter++;
        }
    }
    else // if no camera is available, no point is visible
    {
        std::vector<double>visMat;
        for(const auto&x : safetyZones)
        {
            x->pointsVisibleForEnoughCameras(visMat);
        }
    }
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
        // delete SafetyZones from list:
        if(!truck->szLeft.expired())
        {    string id = truck->szLeft.lock()->getName();
             safetyZones.erase(std::remove_if(safetyZones.begin(),safetyZones.end(),[&id](std::shared_ptr<SafetyZone>const& it){return it->getName() == id;}));
        }
        if(!truck->szRight.expired())
        {   string id = truck->szRight.lock()->getName();
            safetyZones.erase(std::remove_if(safetyZones.begin(),safetyZones.end(),[&id](std::shared_ptr<SafetyZone>const& it){return it->getName() == id;}));
        }

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
    osg::Quat rotInteractor;
    rotInteractor.makeRotate(osg::DegreesToRadians(180.0),osg::Z_AXIS);
    localInteractor.setTrans(osg::Vec3(0,-30,5));
    localInteractor.setRotate(rotInteractor);
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

void EKU::doAddPRIO1(osg::Vec3 pos, double l,double w,double h )
{

    osg::Matrix localSafetyZone;
    localSafetyZone.setTrans(pos);
    std::shared_ptr<SafetyZone> sz =std::make_shared<SafetyZone>(localSafetyZone,SafetyZone::PRIO1,l,w,h);
    safetyZones.push_back(std::move(sz));
    cover->getObjectsRoot()->addChild(safetyZones.back()->getSZ());

    std::cout<<"nbr of SZ: "<<safetyZones.size()<<std::endl;


}
void EKU::doAddPRIO2(osg::Vec3 pos,double l,double w,double h)
{
    osg::Matrix localSafetyZone;
    localSafetyZone.setTrans(pos);
    osg::Quat rot;
    rot.makeRotate(osg::DegreesToRadians(-90.0),osg::X_AXIS);
  //  localSafetyZone.setRotate(rot);
    std::shared_ptr<SafetyZone> sz =std::make_shared<SafetyZone>(localSafetyZone,SafetyZone::PRIO2,l,w,h);
    safetyZones.push_back(std::move(sz));
    cover->getObjectsRoot()->addChild(safetyZones.back()->getSZ());

    std::cout<<"nbr of SZ: "<<safetyZones.size()<<std::endl;


}
void EKU::doRemovePRIOZone(std::shared_ptr<SafetyZone>& s)
{
    if(!safetyZones.empty())
    {
        std::cout<<"nbr of SZ before"<<safetyZones.size()<<std::endl;

        safetyZones.erase(std::remove_if(safetyZones.begin(),safetyZones.end(),[&s](std::shared_ptr<SafetyZone>const& it){return s == it;}));

        std::cout<<"nbr of SZ after"<<safetyZones.size()<<std::endl;
    }
    else
        std::cout<<"No SZ available"<<std::endl;

    std::cout<<"nbr of SZ: "<<safetyZones.size()<<std::endl;

}


COVERPLUGIN(EKU)
