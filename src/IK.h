#ifndef IK_H
#define IK_H

#include <stdint.h>
#include "simpleCL.h"
#include "scl_custom.h"

class IK{

public:
    IK();
    void compute(
            int8_t* error,
            int16_t* joints,
            void* target,
            void* nsparam,
            int8_t* config,
            int32_t n,
            bool doublePrecision);

private:

    sclHard* hardware;
    sclSoft softwareSingle;
    sclSoft softwareDouble;
};

#endif // IK_H
