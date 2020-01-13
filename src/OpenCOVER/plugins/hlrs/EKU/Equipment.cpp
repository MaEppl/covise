 /* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#include <iostream>
#include <math.h>
#include <numeric>

#include <Equipment.h>
using namespace opencover;

Equipment::Equipment(std::string name,osg::Matrix position,osg::ref_ptr<osg::Node> node):name(name)
{

    matrix = new osg::MatrixTransform();
    matrix->setName(node->getName()+ "-Transform");
    matrix->setMatrix(position);
    matrix->addChild(node.get());

    outline = new osgFX::Outline;
    outline->setName(node->getName()+ "-Outline");
    outline->setWidth(8);
    outline->setColor(osg::Vec4(1,1,0,1));
    outline->addChild(matrix.get());


    switchNode = new osg::Switch();
    switchNode->setName(node->getName()+ "Switch");
    switchNode->addChild(matrix.get(),true);
    switchNode->addChild(outline.get(),false);

    group = new osg::Group();
    group->setName(node->getName() + "Group");
    group->addChild(switchNode.get());

    cover->getObjectsRoot()->addChild(group.get());

    //User Interaction
    myinteractionA = new vrui::coTrackerButtonInteraction(vrui::coInteraction::ButtonA, "MoveMode", vrui::coInteraction::Highest);
    interActingA = false;
    aSensor = new mySensor(switchNode, name, myinteractionA);
    sensorList.append(aSensor);

}

void Equipment::preFrame()
{
    sensorList.update();
    //Test if button is pressed
    int state = cover->getPointerButton()->getState();
    if (myinteractionA->isRunning()) //when interacting the Sphere will be moved
    {

        static osg::Matrix invStartHand;
        static osg::Matrix startPos;
        if (!interActingA)
        {
            showOutline(true);
            //remember invStartHand-Matrix, when interaction started and mouse button was pressed
            invStartHand.invert(cover->getPointerMat() * cover->getInvBaseMat());
            startPos = matrix->getMatrix(); //remember position when interaction started
            interActingA = true; //register interaction
        }
        else if((cover->frameTime() - aSensor->getStartTime())> 0.3)
        {
            //calc the tranformation matrix when interacting is running and mouse button was pressed
            osg::Matrix trans =invStartHand * (cover->getPointerMat() * cover->getInvBaseMat());
            //trans.setTrans(osg::Vec3(0,0,0));//For rotation
            restrictMovement(trans);
            trans.setTrans(trans.getTrans().x(),trans.getTrans().y(),0);//set z trans to zero
            trans.orthoNormalize(trans);//remove scale
            osg::Matrix newTrans =  startPos * osg::Matrix::translate(trans.getTrans());//newTrans is translation only

           /* For Rotation:
            osg::Matrix newRot = startPos *trans;//newRot is rotation only
            osg::Matrix rotcamPosinterActor1 = startPoscamPosinterActor1 * trans;
            osg::Matrix rotcamPosinterActor2 = startPoscamPosinterActor2 * trans;
            osg::Matrix rotSZ1 = startPosSZ1 * trans;
            osg::Matrix rotSZ2 = startPosSZ2 * trans;

            fullMat->setMatrix(newRot);

*/
            // For Translation:
            matrix->setMatrix(newTrans);
        }
    }
    if (myinteractionA->wasStopped() && state == false)
    {

        interActingA = false; //unregister interaction
        showOutline(false);

    }
}

void Equipment::showOutline(bool status)
{
    switchNode->setChildValue(outline.get(),status);
    switchNode->setChildValue(matrix.get(),!status);

}

EquipmentWithCamera::EquipmentWithCamera(std::string name,osg::Matrix position,osg::ref_ptr<osg::Node> node,std::vector<osg::Matrix> camMatrixes)
    :Equipment(name,position,node)
{
    // create cameras
    for(const auto& x: camMatrixes)
    {
        std::shared_ptr<CamPosition> c_s =std::make_shared<CamPosition>(x,this);

        //std::shared_ptr<CamPosition> c_s =std::make_shared<CamPosition>(x*position,this);
        std::weak_ptr<CamPosition> c_w = c_s;
        cameraPositions.push_back(std::move(c_w));
        EKU::allCamPositions.push_back(std::move(c_s));
    }
    for(const auto& x:cameraPositions)
    {
        group->addChild(x.lock()->getCamGeode().get());
        x.lock()->setPosition(x.lock()->getMatrix()*matrix.get()->getMatrix());
    }

}

void EquipmentWithCamera::preFrame()
{
    if(EKU::modifyScene == true)
    {
        sensorList.update();
        //Test if button is pressed
        int state = cover->getPointerButton()->getState();
        if (myinteractionA->isRunning()) //when interacting the Sphere will be moved
        {
            showOutline(true);
            static osg::Matrix invStartHand;
            static osg::Matrix startPos;
            static std::vector<osg::Matrix> startPosCameras;
            startPosCameras.resize(cameraPositions.size());
            if (!interActingA)
            {
                startPosCameras.clear();
                //remember invStartHand-Matrix, when interaction started and mouse button was pressed
                invStartHand.invert(cover->getPointerMat() * cover->getInvBaseMat());
                startPos = matrix->getMatrix(); //remember position of truck, when interaction started

                //remember position of cam, when interaction started
                int count =0;
                for(const auto& x:cameraPositions )
                {
                    if(!x.expired())
                    {
                        osg::Matrix start = x.lock()->getMatrix();
                        startPosCameras.push_back(start);
                    }
                    count++;
                }
                interActingA = true; //register interaction
            }
            else if((cover->frameTime() - aSensor->getStartTime())> 0.3)
            {
                //calc the tranformation matrix when interacting is running and mouse button was pressed
                osg::Matrix trans =invStartHand * (cover->getPointerMat() * cover->getInvBaseMat());
                //trans.setTrans(osg::Vec3(0,0,0));//For rotation
                restrictMovement(trans);
                trans.setTrans(trans.getTrans().x(),trans.getTrans().y(),0);//set z trans to zero
                trans.orthoNormalize(trans);//remove scale

                osg::Matrix newTrans =  startPos * osg::Matrix::translate(trans.getTrans());//newTrans is translation only
                std::vector<osg::Matrix> transInteractor;
                for(const auto & x:startPosCameras)
                {
                    osg::Matrix transCam = x * osg::Matrix::translate(trans.getTrans());
                    transInteractor.push_back(transCam);
                }
             //   osg::Matrix transcamPosinterActor1 = startPoscamPosinterActor1 *  osg::Matrix::translate(trans.getTrans());
             //   osg::Matrix transcamPosinterActor2 = startPoscamPosinterActor2 * osg::Matrix::translate(trans.getTrans());

               /* For Rotation:
                osg::Matrix newRot = startPos *trans;//newRot is rotation only
                osg::Matrix rotcamPosinterActor1 = startPoscamPosinterActor1 * trans;
                osg::Matrix rotcamPosinterActor2 = startPoscamPosinterActor2 * trans;


                fullMat->setMatrix(newRot);
                //update possitions of childs:
                possibleCamLocations[0]->setPosition(rotcamPosinterActor1);
                possibleCamLocations[1]->setPosition(rotcamPosinterActor2);

    */
                // For Translation:
                matrix->setMatrix(newTrans);
                //update possitions of childs:
                int count2 =0;
                for(const auto& x:cameraPositions )
                {
                    if(!x.expired())
                    {
                        x.lock()->setPosition(transInteractor.at(count2));
                    }
                    count2++;
                }
            }
        }
        if (myinteractionA->wasStopped() && state == false)
        {
            interActingA = false; //unregister interaction
            showOutline(false);


        }
    }
    else if(EKU::deleteObjects == true)
    {

    }

}
