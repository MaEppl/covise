/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef VRUI_HIT_H
#define VRUI_HIT_H

#include <util/coTypes.h>
#include <util/coVector.h>
#include <vector>
#include <OpenVRUI/sginterface/vruiNode.h>

namespace vrui
{
typedef  std::vector<vruiNode *> NodePath;
using covise::coVector;

class vruiNode;

class OPENVRUIEXPORT vruiHit
{

public:
    vruiHit()
    {
    }

    virtual ~vruiHit();

    virtual coVector &getLocalIntersectionPoint() const = 0;
    virtual coVector &getWorldIntersectionPoint() const = 0;
    virtual coVector &getWorldIntersectionNormal() const = 0;
    virtual bool isMouseHit() const = 0;

    virtual vruiNode *getNode() = 0;
    virtual NodePath *getNodePath() {return NULL;}
};
}
#endif
