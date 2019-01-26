
#include "scl_custom.h"

sclSoft sclGetCLSoftwareFromString(char* source, char* name, sclHard hardware ){
    sclSoft software;
    sprintf( software.kernelName, "%s", name);
    software.program = _sclCreateProgram( source, hardware.context );
    _sclBuildProgram( software.program, hardware.device, name );
    software.kernel = _sclCreateKernel( software );
    return software;
}
