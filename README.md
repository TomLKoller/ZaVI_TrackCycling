# ZaVI_TrackCycling
Contains a batch estimator that estimates the pose of a bike in a track race with an IMU as the only sensor. It uses the 3D map of the track and vehicle constraints. See the publication: State observability through prior Knowledge: Tracking Track Cyclers with Inertial Sensors.

The repository is intended to reproduce the results of the aforementioned publication. Please contact the author if you have any trouble to run it.

Requirements:
1. C++17 compiler 
1. Ceres Solver 
1. Boost
1. Eigen 
1. Cmake 
1. OpenSceneGraph
1. Glog (Google Logging)
1. libconfig 

The Code was evaluated on a Ubuntu 18.04 LTS Machine

Compile it with -O3 and -march=native. Even with optimizations a hundred iterations run for an hour.

Start the Code from the base SixdaysCode directory. Otherwise it wont find the required files

Set the Trial number in the Main Config File
