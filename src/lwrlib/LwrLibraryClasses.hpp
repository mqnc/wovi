#ifndef LWR_LIBRARY_CLASSES_HPP
#define LWR_LIBRARY_CLASSES_HPP

//#define LWR_LIBRARY_USE_KDL

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <vector>

using namespace Eigen;

typedef enum{
    LWR_OK = 0,
    LWR_WARNING = 1<<0,
    LWR_ERROR = 1<<1,
    LWR_JOINTLIMIT = 1<<2,
    LWR_TARGET_TOO_FAR = 1<<3,
    LWR_TARGET_TOO_CLOSE = 1<<4,
    LWR_CLOSE_TO_SINGULARITY = 1<<5,
    LWR_SINGULARITY = 1<<6,
    LWR_NO_SOLUTION_FOR_ELBOW = 1<<7
} LwrErrorMsg;

class LwrJoints{
public:
    double j[7];

    LwrJoints();
    LwrJoints(double j1, double j2, double j3, double j4, double j5, double j6, double j7);

    double& operator[](int index){return j[index];}
    double operator()(int index) const{return j[index];} // read only access for const arrays

    void setJoints(double values[]);
    void setJoints(std::vector<double> &values);
    void setJoints(double j1, double j2, double j3, double j4, double j5, double j6, double j7);
    void setJoints(Matrix <double,7,1> &values);
    void getJoints(double values[]);
    void getJoints(std::vector<double> &values);
    void getJoints(Matrix <double,7,1> &values);
};

class LwrFrame{
public:
    Matrix3d ori;
    Vector3d pos;

    LwrFrame();

    void setOri(double values[], bool rowwise=false);
    void setOri(std::vector<double> &values, bool rowwise=false);
    void getOri(double values[], bool rowwise=false);
    void getOri(std::vector<double> &values, bool rowwise=false);

    void setQuat(double w, double x,double y, double z);
    void setQuat(double values[]);
    void setQuat(std::vector<double> &values);
    void getQuat(double &w, double &x,double &y, double &z);
    void getQuat(double values[]);
    void getQuat(std::vector<double> &values);

    void setPos(double x, double y, double z);
    void setPos(double values[]);
    void setPos(std::vector<double> &values);
    void getPos(double &x, double &y, double &z);
    void getPos(double values[]);
    void getPos(std::vector<double> &values);

#ifdef LWR_LIBRARY_USE_KDL
    void setFrame(KDL::Frame &T);
#endif //LWR_LIBRARY_USE_KDL

	LwrFrame inverse();
};

class LwrTwist{
public:
    Vector3d trans;
    Vector3d rot;

    LwrTwist(){trans=Vector3d(0,0,0); rot=Vector3d(0,0,0);};

    void setTrans(double vx, double vy, double vz);
    void setTrans(double values[]);
    void setTrans(std::vector<double> &values);
    void getTrans(double &vx, double &vy, double &vz);
    void getTrans(double values[]);
    void getTrans(std::vector<double> &values);

    void setRot(double vx, double vy, double vz);
    void setRot(double values[]);
    void setRot(std::vector<double> &values);
    void getRot(double &vx, double &vy, double &vz);
    void getRot(double values[]);
    void getRot(std::vector<double> &values);

    void computeFrom(LwrFrame &pose1, LwrFrame &pose2, double dt);
    LwrFrame applyTo(LwrFrame &pose1, double dt);
};

class LwrXCart{
public:
    LwrFrame pose;
    double nsparam;
    int config;

    LwrXCart(){nsparam=0; config=0;};
};

class LwrXTwist{
public:
    LwrTwist twist;
    double dnsparamdt;

    LwrXTwist(){dnsparamdt=0;};
};

class LwrElbowInterval{
public:
	bool valid;
	double lower;
	double upper;

	LwrElbowInterval(){valid=false; lower=0; upper=0;};
	void setBorders(double low, double up){lower=low; upper=up; valid=true;}
	void invalidate(){lower=0; upper=0; valid=false;}
};

// is not zeroed upon initialization!
typedef Matrix <double, 7, 7> LwrXJacobian;

LwrFrame operator*(LwrFrame T1, LwrFrame T2);
Vector3d operator*(LwrFrame T, Vector3d v);

LwrErrorMsg operator|(LwrErrorMsg e1, LwrErrorMsg e2);
LwrErrorMsg operator&(LwrErrorMsg e1, LwrErrorMsg e2);

// screen output functions
std::ostream& operator<<(std::ostream& stream, LwrJoints const& val);
std::ostream& operator<<(std::ostream& stream, LwrFrame const& val);
std::ostream& operator<<(std::ostream& stream, LwrTwist const& val);
std::ostream& operator<<(std::ostream& stream, LwrXCart const& val);
std::ostream& operator<<(std::ostream& stream, LwrXTwist const& val);
std::ostream& operator<<(std::ostream& stream, LwrElbowInterval const& val);

#endif // LWR_LIBRARY_CLASSES_HPP
