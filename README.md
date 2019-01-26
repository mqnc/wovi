# On-the-Fly Workspace Visualization for Redundant Manipulators

![Preview](/misc/preview.jpg)

Source code for the workspace visualization algorithm presented in this PhD thesis  
[https://publikationen.bibliothek.kit.edu/1000065120](https://publikationen.bibliothek.kit.edu/1000065120)

## About

Due to public request by countless people (one), the source code of this project was disclosed.

In its current state, it is probably absolutely unusable. The thesis was handed in in 2016 and the software was designed to run on whatever ROS version was recent back then. The code was written by an engineer (not a software designer) without the intention of revealing it to anyone.

The plan is to make it usable now so the world of robotics can benefit from it.

## Abstract

Within this thesis, different concepts for on-the-fly visualization of workspace boundaries for 7-DoF manipulators have been investigated. It is explored, which possibilities parallelization and the utilization of modern graphics cards can bring to workspace visualization through on-line computation of positioning limits for the current situation. The higher goal is to employ intuitive visualization as a substitute for the natural human feeling for reachability that humans possess for their own arms.

The workspace is considered as the set of poses that can be accessed by the TCP, taking into account robot joint limits, link lengths and singularities. The novel distinction between the directly and the generally accessible workspace is made: The directly accessible workspace contains all points that are reachable for the tool center point from the current position in a linear motion or a rotation about a constant axis without reconfiguration. The generally accessible workspace contains all reachable poses, even if a reconfiguration is necessary to reach them.

The generally accessible workspace is obtained using rasterization of the Cartesian space and assessing the reachability of every node by analytical inverse kinematics computation. The directly accessible workspace is obtained via simultaneous virtual robot motion in all directions.

In order to reduce the complexity of this six-dimensional problem, it is split into the consideration of translation limits while keeping the orientation constant and rotation limits while keeping the TCP position constant.

Different visualization concepts are devised, based on the considerations above. The motion boundaries for the current robot pose are displayed as semi transparent barriers within a virtual environment.

All investigated concepts are exemplarily implemented for the DLR/KUKA LBR IV manipulator and consider its redundant kinematic structure using a novel cost function concept. The usability and perspicuity of the devised visualization concepts have been assessed in a user study. While the translational components received _good_ to _excellent_ ratings, the rotational components scored average but still proved to be effective in the tested scenario.

Possible applications for the proposed concepts are visual support for the manual operation of manipulators, fast workspace analyses in time-critical scenarios, assistance for robot or target placement and repositioning, interactive workspace exploration for design and comparison of robots and tools and determination of the motion tolerance around a trajectory.

## Dependencies

The program uses [simple-opencl](https://github.com/morousg/simple-opencl) (included, no longer maintained) and the visualization is done in [RViz](http://wiki.ros.org/rviz).

## License

Since simple-opencl is licensed under GPLv3, this project has to follow. Maybe we can kick it out and use a less restrictive license.

## Thanks to

[Achille Verheye](https://github.com/Achllle) for the interest in this project!  
[Prof. Kröger](https://www.ipr.kit.edu/mitarbeiter_2538.php) for the permission to disclose the source!
