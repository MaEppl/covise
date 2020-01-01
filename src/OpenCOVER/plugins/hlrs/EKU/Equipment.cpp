 /* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#include <iostream>
#include <math.h>
#include <numeric>

#include <Equipment.h>
using namespace opencover;

Equipment::Equipment(std::string name,osg::Matrix position,osg::ref_ptr<osg::Node> node):name(name),pos(position)
{

    matrix = new osg::MatrixTransform();
    matrix->setName("Transform");
    matrix->setMatrix(pos);
    matrix->addChild(node.get());

    outline = new osgFX::Outline;
    outline->setName("outline");
    outline->setWidth(8);
    outline->setColor(osg::Vec4(1,1,0,1));
    outline->addChild(matrix.get());


    switchNode = new osg::Switch();
    switchNode->setName("switch");
    switchNode->addChild(matrix.get(),true);
    switchNode->addChild(outline.get(),false);

    cover->getObjectsRoot()->addChild(switchNode.get());

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




