#include <SafetyZone.h>

using namespace opencover;

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

/*SafetyZone::SafetyZone(osg::Vec3 pos, Priority priority, float length = 2, float width =2, float height = 8):pos(pos),priority(priority),length(length),width(width),height(height)
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
  //  myinteraction = new vrui::coTrackerButtonInteraction(vrui::coInteraction::AllButtons, "MoveMode", vrui::coInteraction::Medium);
  //  interActing = false;
  //  aSensor = new mySensor(SafetyZoneGeode, name, myinteraction);
  //  sensorList.append(aSensor);

}
*/
SafetyZone::SafetyZone(osg::Matrix m, Priority priority, float length = 2, float width =2, float height = 8):mat(m),priority(priority),length(length),width(width),height(height)
{

    count++;
    fprintf(stderr, "new SafetyZone\n");
    zone = new osg::Box(m.getTrans(),length,width,height);
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
    safetyZoneGeode->getParent(0)->removeChild(safetyZoneGeode.get());
    std::cout<<"Removed Safety Zone"<<std::endl;
}

bool::SafetyZone::destroy()
{
    //is this function necessary or do it in Destructor?
    //free memory space???
    //call desctuctor???
    safetyZoneGeode->getParent(0)->removeChild(safetyZoneGeode);
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
    scaleVec = Vec3(imgWidth / scale_, depthView / scale_, imgHeight / scale_); // urspruenglich 800 x -600
    scaleVec = Matrix::transform3x3(scaleVec, localMat);
    scaleVec += localVec;

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

Wireframe::Wireframe()
{
    createGeometry();

}

void Wireframe::createGeometry()
{
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(0.0f, 0.0f, 10.0f));
    vertices->push_back(osg::Vec3(1.0f, 0.0f, 10.0f));
    vertices->push_back(osg::Vec3(1.0f, 1.0f, 10.0f));
    vertices->push_back(osg::Vec3(0.0f, 1.0f, 10.0f));

    normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));

    colors = new osg::Vec4Array; colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));

    //ow create a geometry object, where the description of our square will be stored, which will be rendered. Pass an array of vertices to this geometry.
    quad = new osg::Geometry;
    quad->setVertexArray(vertices.get());
    //Transmitting an array of normals, we inform the engine that a single normal will be used for all vertices, indicating the method of linking ("binding") the normals BIND_OVAERALL
    quad->setNormalArray(normals.get());
    quad->setNormalBinding(osg::Geometry::BIND_OVERALL);
    // Now we create a set of primitives for geometry. We indicate that square (GL_QUADS) faces should be generated from the array of vertices, taking the vertex with index 0 as the first vertex, and the total number of vertices will be 4
    quad->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    wireframe = new osg::Geode;
    wireframe->addDrawable(quad.get());
}
















