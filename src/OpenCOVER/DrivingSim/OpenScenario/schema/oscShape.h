/* This file is part of COVISE.

You can use it under the terms of the GNU Lesser General Public License
version 2.1 or later, see lgpl - 2.1.txt.

* License: LGPL 2 + */


#ifndef OSCSHAPE_H
#define OSCSHAPE_H

#include "oscExport.h"
#include "oscObjectBase.h"
#include "oscObjectVariable.h"

#include "oscVariables.h"
#include "schema/oscPolyline.h"
#include "schema/oscClothoid.h"
#include "schema/oscSpline.h"

namespace OpenScenario
{
class OPENSCENARIOEXPORT oscShape : public oscObjectBase
{
public:
    oscShape()
    {
        OSC_ADD_MEMBER(reference);
        OSC_OBJECT_ADD_MEMBER(Polyline, "oscPolyline");
        OSC_OBJECT_ADD_MEMBER(Clothoid, "oscClothoid");
        OSC_OBJECT_ADD_MEMBER(Spline, "oscSpline");
    };
    oscDouble reference;
    oscPolylineMember Polyline;
    oscClothoidMember Clothoid;
    oscSplineMember Spline;

};

typedef oscObjectVariable<oscShape *> oscShapeMember;


}

#endif //OSCSHAPE_H
