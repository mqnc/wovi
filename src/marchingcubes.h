
// source: http://paulbourke.net/geometry/polygonise/

#ifndef MARCHINGCUBES_H
#define MARCHINGCUBES_H

#include <Eigen/Core>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

using namespace Eigen;

/*
   Linearly interpolate the position where an isosurface cuts
   an edge between two vertices, each with their own scalar value
*/
Vector3d vertexInterp(double isolevel, Vector4d p1, Vector4d p2);

/*
   Given a grid cell and an isolevel, calculate the triangular
   facets required to represent the isosurface through the cell.
   Return the number of triangular facets, the array "triangles"
   will be loaded up with the vertices at most 5 triangular facets.
    0 will be returned if the grid cell is either totally above
   of totally below the isolevel.
*/
int marchingCube(Vector4d *grid, double isolevel, geometry_msgs::Point *triangles);

#endif
