
#ifndef ACCMAP_H
#define ACCMAP_H

#include "defs.h"
#include "scl_custom.h"
#include <sstream>

////////////////////////////// class for GPU computation of IK samples

class AccMap{ // accessibility map (reachability and capability already taken)

public:
    AccMap();
    void compute(int16_t error[],
            int16_t intervals[],
            float cost[],
            void* targets,
            uint8_t configs[],
            void* invTool,
            uint32_t nSamples,
            bool doublePrecision);

private:

    sclHard* hardware;
    sclSoft softwareSingle;
    sclSoft softwareDouble;

};

#endif
