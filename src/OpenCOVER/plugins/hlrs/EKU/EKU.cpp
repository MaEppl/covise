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
#include<Cam.h>
using namespace opencover;
bool EKU::modifyScene=true;
bool EKU::deleteObjects=false;
std::vector<std::shared_ptr<SafetyZone>> EKU::safetyZones;
std::vector<std::shared_ptr<CamPosition>> EKU::allCamPositions;
std::vector<std::unique_ptr<Equipment>> EKU::equipment;
std::vector<std::unique_ptr<EquipmentWithCamera>> EKU::equipmentWithCamera;

double getZvalueOfSZ()
{
  //  for(const auto& x : saf)
}

EKU *EKU::plugin = NULL;
void restrictMovement(osg::Matrix &mat)
{
    coCoord coord;
    coord = mat;
    //only rotation around z !
    coord.hpr[1] = 0;
    coord.hpr[2] = 0;
    coord.makeMat(mat);
}


void EKU::preFrame()
{
    for(const auto &x: allCamPositions)
        x->preFrame();

    if(modifyScene)
    {
        for(const auto &x: safetyZones)
            x->preFrame();
        for(const auto &x : equipment)
            x->preFrame();
        for(const auto &x : equipmentWithCamera)
            x->preFrame();
    }

}
void EKU::calcPercentageOfCoveredSafetyZones()
{
    std::cout<<"PRIO1 zones covered by 2 cameras: " <<std::endl;
    std::cout<<"PRIO1 zones covered by 1 camera: " <<std::endl;
    std::cout<<"PRIO1 zones not covered: " <<std::endl;
    std::cout<<"PRIO2 zones covered by at least 1 camera: " <<std::endl;
    std::cout<<"PRIO2 zones not covered: " <<std::endl;
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

    truck = osgDB::readNodeFile(path + "PumpTruck.3ds");
    if (!truck.valid())
    {
          osg::notify( osg::FATAL ) << "Unable to load truck data file. Exiting." << std::endl;
    }
    truck->setName("Pump Truck");
    //draw Pumps:
    osg::Matrix pos;
    pos.setTrans(11,0,0);
    doAddTruck(pos);
    int cnt =0;
    for(int i = 0;i<13;i++)
    {
        osg::Vec3 posOld=equipmentWithCamera.back()->getPosition().getTrans();
        osg::Vec3 posNew;
        if(cnt % 2 == 0)
        {
              posNew = {posOld.x()*-1,posOld.y(),posOld.z()};
              osg::Matrix matrixNew;
              matrixNew.setTrans(posNew);
              osg::Quat q;
              q.makeRotate(osg::DegreesToRadians(180.0),osg::Z_AXIS);
              matrixNew.setRotate(q);

              doAddTruck(matrixNew);
        }else
        {
            posNew = {posOld.x()*-1,posOld.y()+4.5f,posOld.z()};//+5
            osg::Matrix matrixNew;
            matrixNew.setTrans(posNew);

            doAddTruck(matrixNew);

        }

        cnt ++;
    }

    //add manifold
    {
        manifold = osgDB::readNodeFile(path +"manifold_simplified.3ds");
        if (!manifold.valid())
        {
              osg::notify( osg::FATAL ) << "Unable to load Manifold data file. Exiting." << std::endl;
        }
        std::string name = "Manifold";
        manifold->setName(name);
        osg::Quat q;
        q.makeRotate(osg::DegreesToRadians(90.0),osg::Z_AXIS);
        osg::Matrix m;
        m.setTrans(0,10,0);
        m.setRotate(q);
        std::unique_ptr<Equipment> manifoldNew(new Equipment(name,m,manifold));
        equipment.push_back(std::move(manifoldNew));

    }
    //add blender
    {
        blender = osgDB::readNodeFile(path +"Blender.3ds");
        if (!truck.valid())
        {
              osg::notify( osg::FATAL ) << "Unable to load Data Van file. Exiting." << std::endl;
        }
        std::string name = "Blender";
        blender->setName(name);
        std::vector<osg::Matrix> vecOfCameras;
        osg::Matrix matBlender;
        matBlender.setTrans(0,-7,0);
        osg::Matrix cam1,cam2;
        cam1.setTrans(7,1.5,2.5);
        cam2.setTrans(-3,1.5,2.5);
        osg::Quat rotY,rotZ,rotX,rotLast1,rotLast2;
        rotY.makeRotate(osg::DegreesToRadians(180.0),osg::Z_AXIS);
        cam1.setRotate(rotY);
        cam2.setRotate(rotY);
        vecOfCameras.push_back(cam1);
        vecOfCameras.push_back(cam2);
        std::unique_ptr<EquipmentWithCamera> blenderDraw(new EquipmentWithCamera(name,matBlender,blender,vecOfCameras));
        equipmentWithCamera.push_back(std::move(blenderDraw));
    }
    //add Data Van
    {
        dataVan = osgDB::readNodeFile(path +"dataVan.3ds");
        std::string name = "Data Van";
        dataVan->setName(name);
        std::vector<osg::Matrix> vecOfCameras;
        osg::Matrix matDataVan;
        matDataVan.setTrans(10,50,0);
        osg::Matrix cam1,cam2;
        cam1.setTrans(0,-1.5,3.2);
        cam2.setTrans(-4,-1.5,3.2);
        osg::Quat rotY;
        rotY.makeRotate(osg::DegreesToRadians(-20.0),osg::Z_AXIS);
        matDataVan.setRotate(rotY);
       // cam1.setRotate(rotY);
       // cam2.setRotate(rotY);
        vecOfCameras.push_back(cam1);
        vecOfCameras.push_back(cam2);
        std::unique_ptr<EquipmentWithCamera> dataVanDraw(new EquipmentWithCamera(name,matDataVan,dataVan,vecOfCameras));
        equipmentWithCamera.push_back(std::move(dataVanDraw));
    }

    //add Christmas Tree
    {

        christmasTree = osgDB::readNodeFile(path +"ChristmasTree.ply");
        if (!christmasTree.valid())
        {
              osg::notify( osg::FATAL ) << "Unable to load ChristmasTree data file. Exiting." << std::endl;
        }
        std::string name = "Christmas Tree";
        christmasTree->setName(name);
        osg::Quat q;
        q.makeRotate(osg::DegreesToRadians(0.0),osg::Z_AXIS);
        osg::Matrix m;
        m.setTrans(0,40,0);
        m.setRotate(q);
        std::unique_ptr<Equipment> christmasTreeNew(new Equipment(name,m,christmasTree));
        equipment.push_back(std::move(christmasTreeNew));
        m.setTrans(-10,40,0);
        std::unique_ptr<Equipment> christmasTreeNew1(new Equipment(name+"1",m,christmasTree));
        equipment.push_back(std::move(christmasTreeNew1));
        m.setTrans(10,40,0);
        std::unique_ptr<Equipment> christmasTreeNew2(new Equipment(name+"2",m,christmasTree));
        equipment.push_back(std::move(christmasTreeNew2));
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

        silo2 = osgDB::readNodeFile(path+"silo/C0810A007.3ds");
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
     /*   std::cout<<"Load container"<<std::endl;
        container = osgDB::readNodeFile(path + "container.3ds");
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


        std::cout<<"Container loaded"<<std::endl;
*/
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

    //Create UI
    EKUMenu  = new ui::Menu("EKU", this);

    //Add Truck
    AddTruck = new ui::Action(EKUMenu , "addTruck");
    AddTruck->setCallback([this](){
        osg::Matrix pos;
        pos.setTrans(50,0,0);
        doAddTruck(pos);
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
            doRemoveTruck(equipmentWithCamera.back());
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
            x->updateVisibleCam();
            x->createCamsInSearchSpace();
            std::cout<<x->getName()<<"nbrCams:"<<x->allCameras.size() ;
            for(const auto &x1:x->allCameras)
            {
               std::cout<<" prio1: "<< x1->getVisMatPrio1().size();
               std::cout<<" prio2: "<< x1->getVisMatPrio2().size()<<std::endl;
            }
            std::cout<<" "<<std::endl;
        }

        //show Points which are currently not visible
       // findNotVisiblePoints();

        std::vector<osg::Matrix> finalCamMatrixes;

        /*Optimization is only done on master. Problem with random generator and multithreading on Slaves
        ->get different results on each slave!
        */
        if(coVRMSController::instance()->isMaster())
        {

            GA::nbrCamPositions=allCamPositions.size();
            GA::nbrPoints = safetyZones.size();

            ga =new GA(allCamPositions,safetyZones);

            std::vector<std::shared_ptr<Cam>>result = ga->getfinalCamPos();
            for(const auto &x : result)
            {
                finalCamMatrixes.push_back(x->getMatrix());
            }

            delete this->ga;

         }
        if(!coVRMSController::instance()->isMaster())
        {
            finalCamMatrixes.resize(allCamPositions.size());
        }

        coVRMSController::instance()->syncData(finalCamMatrixes.data(),sizeof(osg::Matrix)*allCamPositions.size());

        {// test if sync is successfull between Master and Slave
        /*    if(!coVRMSController::instance()->isMaster())
            {
                std::string name = "resultsMatrix_"+std::to_string(coVRMSController::instance()->getID())+".txt";

                std::ofstream  output_file;
                output_file.open(name);
                for(const auto& x : allCamPositions)
                {
                    osg::Matrix m = x->getMatrix();
                    coCoord euler =m;
                    output_file<<"x"<<euler.xyz[0]<<"\t"<<"y"<<euler.xyz[1]<<"\t"<<"z"<<euler.xyz[2]<<"\t"<<"xr"<<euler.hpr[0]<<"\t"<<"yr"<<euler.hpr[1]<<"\t"<<"zr"<<euler.hpr[2]<<"\t"<<"\n";

                }
                output_file.close();
            }
            if(!coVRMSController::instance()->isMaster())
            {
                std::string name = "resultsMatrix_Master";

                std::ofstream  output_file;
                output_file.open(name);
                for(const auto& x : allCamPositions)
                {
                    osg::Matrix m = x->getMatrix();

                    coCoord euler =m;
                    output_file<<"x"<<euler.xyz[0]<<"\t"<<"y"<<euler.xyz[1]<<"\t"<<"z"<<euler.xyz[2]<<"\t"<<"xr"<<euler.hpr[0]<<"\t"<<"yr"<<euler.hpr[1]<<"\t"<<"zr"<<euler.hpr[2]<<"\t"<<"\n";
                }
                output_file.close();
            }
        */}


        int count=0;
        for(const auto &x : finalCamMatrixes)
        {
            allCamPositions.at(count)->setPosition(x);
            count ++;
        }

        ModifyScene->setState(false);
        modifyScene = false;

        for(const auto& x : safetyZones)
        {
            x->changeInteractorStatus(false);
        }

    /*    for(const auto& x : allCamPositions)
        {
            x->updateVisibleCam();
        //    x->camDraw->cam->calcVisMat();
        //    for(const auto& x1: x->allCameras)
        //        x1->calcVisMat();
        }
     */     //show Points which are currently not visible
          findNotVisiblePoints();
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


    //Menu for Camera
    Camera = new ui::Menu(EKUMenu, "Camera");
    Camera->setText("Camera");

    //Size of shown camera
    ShowRealSize = new ui::Button(Camera , "ShowRealSize");
    ShowRealSize->setText("Real Camera FOV");
    ShowRealSize->setState(false);
    ShowRealSize->setCallback([this](bool state){

       for(const auto &x :allCamPositions)
       {
           if(!state)
           {
                x->camDraw->_showRealSize = false;
                x->camDraw->resetSize();
            }
           else
            {
               x->camDraw->_showRealSize = true;
               x->camDraw->showRealSize();
            }
       }
    });
    //Show search space
    ShowSearchSpace = new ui::Button(Camera , "ShowSearchSpace");
    ShowSearchSpace->setText("ShowSearchSpace");
    ShowSearchSpace->setState(false);
    ShowSearchSpace->setCallback([this](bool state){

       for(const auto &x :allCamPositions)
       {
            x->setSearchSpaceState(state);
            x->setSearchSpaceState(state);
       }
    });

    //Camera visibility
    VisibilityRegulator = new ui::Slider(Camera , "Slider2");
    VisibilityRegulator->setText("Visibility");
    VisibilityRegulator->setBounds(10., 100.);
    VisibilityRegulator->setValue(Cam::depthView);
    VisibilityRegulator->setCallback([this](double value, bool released){
       // this->disactivateDetailedRendering();
        for(const auto& x :allCamPositions)
        {

          x->camDraw->updateVisibility(value);
        }
      //  this-> activateDetailedRendering();
    });

    //FOV
    FOVRegulator = new ui::Slider(Camera , "Slider1");
    FOVRegulator->setText("FOV");
    FOVRegulator->setBounds(30., 120.);
    FOVRegulator->setValue(Cam::fov);
    FOVRegulator->setCallback([this](double value, bool released){
       // this->disactivateDetailedRendering();
        for(const auto &x : allCamPositions)
        {
          x->camDraw->updateFOV(value);
          //x->cam->calcVisMat(observationPoints);
        }
      //  if(released)
            //update visMat

        //  this-> activateDetailedRendering();

    });


 /*  //Make Cameras invisible
    MakeCamsInvisible = new ui::Button(EKUMenu , "CamerasVisible");
    MakeCamsInvisible->setText("Cameras Visible");
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
*/

    //Calc VisMat
    calcVisMat = new ui::Action(Camera , "calcVisMat");
    calcVisMat->setCallback([this](){
    /*    std::vector<osg::Vec3> pointsToObserve;
        pointsToObserve.reserve(safetyZones.size());
        for(const auto& x : safetyZones)
            pointsToObserve.push_back(x->getPosition());
*/
        for(const auto& x : allCamPositions)
        {
            x->updateVisibleCam();
          /*  x->camDraw->cam->calcVisMat();
            for(const auto& x1: x->allCameras)
                x1->calcVisMat();
                */
        }
        findNotVisiblePoints();
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
             x->updateVisibleCam();
             x->createCamsInSearchSpace();

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

    //Delete Selected Objects
    Delete = new ui::Button(EKUMenu , "Delete");
    Delete->setText("Delete Selected Objects");
    Delete->setState(false);
    Delete->setCallback([this](bool state){
        deleteObjects = state;
    });
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

void EKU::doAddTruck(osg::Matrix pos )
{

    std::vector<osg::Matrix> vecOfCameras;
    osg::Matrix matTruck;
    osg::Matrix cam1,cam2;
    cam1.setTrans(10,1.6,2.5);
    cam2.setTrans(10,-1.6,2.5);
    osg::Quat rotY,rotZ,rotX,rotLast1,rotLast2;
    rotY.makeRotate(osg::DegreesToRadians(-90.0),osg::Z_AXIS);
    rotZ.makeRotate(osg::DegreesToRadians(90.0),osg::X_AXIS);
    rotX.makeRotate(osg::DegreesToRadians(-15.0),osg::Y_AXIS);
    rotLast1.makeRotate(osg::DegreesToRadians(10.0),osg::Z_AXIS);
    rotLast2.makeRotate(osg::DegreesToRadians(-10.0),osg::Z_AXIS);
    cam1.setRotate(rotY*rotZ*rotX*rotLast2);
    cam2.setRotate(rotY*rotZ*rotX*rotLast1);
    vecOfCameras.push_back(cam1);
    vecOfCameras.push_back(cam2);
    std::string name ="Pump";
    std::unique_ptr<EquipmentWithCamera> pumpTruck(new EquipmentWithCamera(name,pos,truck,vecOfCameras));
    equipmentWithCamera.push_back(std::move(pumpTruck));
}


void EKU::doRemoveTruck(std::unique_ptr<EquipmentWithCamera> &truck)
{

/*
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

*/
}
void EKU::doAddCam()
{
    osg::Matrix localInteractor;
    osg::Quat rotInteractor,rotInteractor2;
    rotInteractor.makeRotate(osg::DegreesToRadians(180.0),osg::Z_AXIS);
   // rotInteractor2.makeRotate(osg::DegreesToRadians(45.0),osg::X_AXIS);

  //  localInteractor.setTrans(osg::Vec3(0,-30,5));
    localInteractor.setRotate(rotInteractor*rotInteractor2);
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
