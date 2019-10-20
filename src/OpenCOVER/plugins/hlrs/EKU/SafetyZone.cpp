#include <SafetyZone.h>

using namespace opencover;
size_t SafetyZone:: count = 0;

SafetyZone::SafetyZone(osg::Vec3 pos, Priority priority, float length = 2, float width =2, float height = 8):pos(pos),priority(priority),length(length),width(width),height(height)
{

    count++;
    fprintf(stderr, "new SafetyZone\n");
    zone = new osg::Box(pos,length,width,height);
    safetyZoneDrawable = new osg::ShapeDrawable(zone,hint.get());
    name = "Zone "+std::to_string(SafetyZone::count);

    // Declare a instance of the geode class:
    safetyZoneGeode = new osg::Geode();
    safetyZoneGeode->setName("SafetyZone" +std::to_string(SafetyZone::count));
    osg::StateSet *stateset = safetyZoneGeode->getOrCreateStateSet();
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    stateset->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
    osg::Vec4 _color;
    if(priority == PRIO1)
        _color.set(1.0, 0.0, 0.0, 0.5);
    else if(priority == PRIO2)
         _color.set(1.0, 0.0, 1.0, 0.5);
    safetyZoneDrawable->setColor(_color);
    safetyZoneDrawable->setUseDisplayList(false);
    osg::StateSet *mystateSet = safetyZoneGeode->getOrCreateStateSet();
    setStateSet(mystateSet);

    // Add the unit cube drawable to the geode:
    safetyZoneGeode->addDrawable(safetyZoneDrawable);


    text = new osgText::Text;
  //  text->setName("Text");
  //  text->setText("SafetyZone"+std::to_string(SafetyZone::count));
    //text->setColor()
  //  text->setCharacterSize(4);
  //  text->setPosition(SafetyZone->getCenter());

    //SafetyZoneGeode->addChild(text.get());

    //User Interaction
  /*  myinteraction = new vrui::coTrackerButtonInteraction(vrui::coInteraction::AllButtons, "MoveMode", vrui::coInteraction::Medium);
    interActing = false;
    aSensor = new mySensor(SafetyZoneGeode, name, myinteraction);
    sensorList.append(aSensor);
    */
}

void SafetyZone::setStateSet(osg::StateSet *stateSet)
{
    osg::Material *material = new osg::Material();
    material->setColorMode(osg::Material::DIFFUSE);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  /*  osg::LightModel *defaultLm;
    defaultLm = new osg::LightModel();
    defaultLm->setLocalViewer(true);
    defaultLm->setTwoSided(true);
    defaultLm->setColorControl(osg::LightModel::SINGLE_COLOR);
    */stateSet->setAttributeAndModes(material, osg::StateAttribute::ON);
  //  stateSet->setAttributeAndModes(defaultLm, osg::StateAttribute::ON);
}
SafetyZone::~SafetyZone()
{
    fprintf(stderr, "Removed SafetyZone\n");
}

bool::SafetyZone::destroy()
{
    //is this function necessary or do it in Destructor?
    //free memory space???
    //call desctuctor???
    cover->getObjectsRoot()->removeChild(safetyZoneGeode.get());
    return true;
}

void SafetyZone::updateColor()
{
    safetyZoneDrawable->setColor(osg::Vec4(1., 1., 0., 0.5f));
}

void SafetyZone::resetColor()
{
    if(priority == PRIO1)
        safetyZoneDrawable->setColor(osg::Vec4(1.0, 0.0, 0.0, 0.5));
    else if(priority == PRIO2)
         safetyZoneDrawable->setColor(osg::Vec4(1.0, 0.0, 1.0, 0.5));
}
void SafetyZone::updatePosInWorld()
{
    pos = safetyZoneGeode->getBound().center() * osg::computeLocalToWorld(safetyZoneGeode->getParentalNodePaths()[0]) / 1000;
   // std::cout<<"POS in World:"<<name<<pos.x()<<" "<<pos.y()<<" "<<pos.z()<<std::endl;

}
