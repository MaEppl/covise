#include <Sensor.h>
#include <cover/RenderObject.h>
#include <EKU.h>

mySensor::mySensor(osg::Node *node, std::string name, vrui::coTrackerButtonInteraction *_interactionA, osg::ShapeDrawable *cSphDr)
    : coPickSensor(node)
{
    sensorName = name;
    isActive = false;
    _interA = _interactionA;
    shapDr = cSphDr;
}

mySensor::mySensor(osg::Node *node, std::string name, vrui::coTrackerButtonInteraction *_interactionA, CamDrawable *camDraw,std::vector<SafetyZone*> *observationPoints,std::vector<CamDrawable*> *cams)
    : observationPoints(observationPoints), cams(cams), coPickSensor(node)
{
    sensorName = name;
    isActive = false;
    _interA = _interactionA;
    camDr = camDraw;
}

mySensor::mySensor(osg::Node *node,int pos, std::string name,vrui::coTrackerButtonInteraction *_interactionA, SafetyZone *safetyDraw ,std::vector<CamDrawable*> *cams)
    :pos(pos),cams(cams),coPickSensor(node)
{
    sensorName = name;
    isActive = false;
    _interA = _interactionA;
    safetyDr = safetyDraw;
}
mySensor::mySensor(osg::Node *node, std::string name,vrui::coTrackerButtonInteraction *_interactionA):coPickSensor(node)
{
    sensorName = name;
    isActive = false;
    _interA = _interactionA;
}
mySensor::~mySensor()
{
}

//-----------------------------------------------------------
void mySensor::activate()
{
    isActive = true;
    cout << "---Activate--" << sensorName << endl;
    vrui::coInteractionManager::the()->registerInteraction(_interA);
    if(shapDr != nullptr)
        shapDr->setColor(osg::Vec4(1., 1., 0., 1.0f));
    else if(camDr != nullptr)
    {
        size_t cnt=0;
        //camDr->updateColor();
        for(const auto& x: camDr->cam->visMat)
        {
            if(x==1)
            {
                observationPoints->at(cnt)->updateColor();
            }
        cnt++;
        }
        for(const auto& x:*cams)
        {    if(x != camDr)
                x->disactivate();
        }
    }
    else if(safetyDr != nullptr)
    {
        safetyDr->updateColor();
        for( const auto x:*cams)
        {
            if(x->cam->visMat[pos]==1)
            {
                x->activate();
            }

        }

    }
}

//-----------------------------------------------------------
void mySensor::disactivate()
{
    cout << "---Disactivate--" << sensorName << endl;
    isActive = false;
    vrui::coInteractionManager::the()->unregisterInteraction(_interA);
    if(shapDr != nullptr)
        shapDr->setColor(osg::Vec4(1., 0., 0., 1.0f));
    else if(camDr != nullptr)
    {
        size_t cnt=0;
        //camDr->resetColor();
        for(const auto& x: camDr->cam->visMat)
        {
            if(x==1)
            {
                observationPoints->at(cnt)->resetColor();
            }
        cnt++;
        }
        for(const auto& x:*cams)
        {    if(x != camDr)
                x->activate();
        }
    }
    else if(safetyDr != nullptr)
    {
        safetyDr->resetColor();
        for( const auto x:*cams)
        {
            if(x->cam->visMat[pos]==1)
            {
                x->disactivate();
            }

        }

    }
 /*   else
    {
        EKU::plugin ->updateObservationPointPosition();
        for(auto x :EKU::plugin->finalCams)
        {
          x->cam->calcVisMat((EKU::plugin)->*observationPoints);
        }
    }
*/

}

//-----------------------------------------------------------

std::string mySensor::getSensorName()
{
    return sensorName;
}

//-----------------------------------------------------------
bool mySensor::isSensorActive()
{
    if (isActive)
        return true;
    else
        return false;
}

//-----------------------------------------------------------
