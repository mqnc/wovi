#ifndef IKRAY_H
#define IKRAY_H

#include "defs.h"
#include "scl_custom.h"
#include <sstream>

////////////////////////////// class for GPU computation of IK probe rays

class IKRay{

public:
    IKRay();
    void compute(void *endParameters,
            void *rayEnds,
            void *rayColors,
            void* startPos,
            void *startOri,
            void *invTool,
            void* nsparam,
            int8_t config,
            void* targets,
            void *targetOris,
            uint32_t nrays,
            uint32_t nsteps,
            bool autoElbow,
            bool doublePrecision,
            void *debugOutput,
            uint32_t debugIndex);

private:

    sclHard* hardware;
    sclSoft softwareSingle;
    sclSoft softwareDouble;
    sclSoft software2Single;
    sclSoft software2Double;

};

#endif
