# frasier_openrave

## Installation

Dependencies:
* [ROS Melodic](http://wiki.ros.org/melodic/Installation)
* [HSR Software](https://docs.hsr.io/manual_en/index.html)
* [OpenRAVE](http://openrave.org)
* [TrajOpt](http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/)
* [Gurobi](http://www.gurobi.com/)

**OpenRAVE Installation**   
Install dependencies:
* Please follow the instructions [here](https://github.com/RIVeR-Lab/openrave).
* Add the following to `.bashrc`:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)
```
**TrajOpt Installation**   
Install dependencies:
```
sudo apt install libopenscenegraph-dev libeigen3-dev
```
Trajopt uses Gurobi for the optimization. Install Gurobi by following the instructions [here](https://www.gurobi.com/registration/download-reg). You can get a free acadamic license if you have .edu email.
1. `git clone https://github.com/tkelestemur/trajopt.git`
2. `cd trajopt && mkdir build && cd build`  
3. `cmake .. -DGUROBI_LIBRARY=/opt/gurobi801/linux64/lib/libgurobi80.so`
4. `make -j4`
5. Add the following to `.bashrc`:
```
export TRAJOPT_HOME=/path/to/trajopt/folder
```

**Package Installation**   
Install dependencies:
```
sudo apt install ros-kinetic-ecl
```
1. `cd catkin_ws/src`
2. `git clone  https://github.com/tkelestemur/frasier_openrave.git`
3. `catkin build`


## TODO:
- [ ] visualize collison objects in rviz
- [ ] redundant ik solver from openrave
- [ ] point cloud updater using ROS
- [x] cannot set goal angle for base_t_joint
- [x] fix viewer (change to osg viewer)
- [x] IK using trajopt
- [x] planning using trajopt
