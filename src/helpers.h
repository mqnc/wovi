#ifndef HELPERS_H
#define HELPERS_H

#include "defs.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <Eigen/Core>
#include "lwrlib/LwrLibrary.hpp"
#include "marchingcubes.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <keyboard/Key.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;

#define RANDPOS ((double)rand()/RAND_MAX)
#define RAND ((double)rand()/RAND_MAX*2.0-1.0)
#define RANDDEGPOS ((double)rand()/RAND_MAX/180.0*M_PI)
#define RANDDEG (((double)rand()/RAND_MAX*2.0-1.0)/180.0*M_PI)
#define DEG (M_PI/180.0)

LwrFrame dh(double d, double theta, double a, double alpha);
void fk(LwrFrame jointPositions[], LwrJoints& jointAngles);
Matrix3d camcsys(double camyaw, double campitch, bool snap);
vector<Quaterniond> generateOris();
Matrix3d snapOri(Matrix3d src, vector<Quaterniond> &list);
string int2str (int a);
void pubcsys(Vector3d pos, Matrix3d ori, const char* name);
void pubcsys(Vector3d pos, Vector4d xyzw, const char* name);
visualization_msgs::Marker initializeMarker(int id, int type, int nPoints);
geometry_msgs::Point positionFromEigen(Vector3d v);
geometry_msgs::Quaternion orientationFromEigen(Vector4d v);
std_msgs::ColorRGBA colorFromEigen(Vector4d v);
std_msgs::ColorRGBA jointColor(int joint, double value, Vector4d deflt);
double elbowCost(const LwrJoints &joints, const LwrJoints &jointsBefore);
double elbowCost2(const LwrJoints &joints, const LwrJoints &jointsBefore);
void marchCubes(visualization_msgs::Marker* pmarker, int nTriangles,
                float voxelData[],
                float x0, float y0, float z0, float dx, float dy, float dz,
                int nx, int ny, int nz, float iso, float alpha);
void oclFromFrame(FLOAT pdst[], LwrFrame* psrc);

#endif
