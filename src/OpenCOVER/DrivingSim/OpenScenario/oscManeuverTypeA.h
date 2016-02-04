/* This file is part of COVISE.

You can use it under the terms of the GNU Lesser General Public License
version 2.1 or later, see lgpl-2.1.txt.

* License: LGPL 2+ */

#ifndef OSC_MANEUVER_TYPE_A_H
#define OSC_MANEUVER_TYPE_A_H

#include "oscExport.h"
#include "oscObjectBase.h"
#include "oscObjectVariable.h"

#include "oscVariables.h"
#include "oscFileHeader.h"
#include "oscParameterListTypeA.h"
#include "oscEvents.h"


namespace OpenScenario {

/// \class This class represents a generic OpenScenario Object
class OPENSCENARIOEXPORT oscManeuverTypeA: public oscObjectBase
{
public:
    oscManeuverTypeA()
    {
        OSC_OBJECT_ADD_MEMBER(fileHeader, "oscFileHeader");
        OSC_ADD_MEMBER(name);
        OSC_OBJECT_ADD_MEMBER(parameterList, "oscParameterListTypeA");
        OSC_OBJECT_ADD_MEMBER(events, "oscEvents");
    };

    oscFileHeaderMember fileHeader;
    oscString name;
    oscParameterListTypeAArrayMember parameterList;
    oscEventsArrayMember events;
};

typedef oscObjectVariable<oscManeuverTypeA *>oscManeuverTypeAMember;

}

#endif //OSC_MANEUVER_TYPE_A_H