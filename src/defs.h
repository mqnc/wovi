
#ifndef DEFS_H
#define DEFS_H

#define DOUBLE_PRECISION
#define GPU_LOCAL_SIZE 64

#ifdef DOUBLE_PRECISION
    #define FLOAT double
#else
    #define FLOAT float
#endif

#endif
