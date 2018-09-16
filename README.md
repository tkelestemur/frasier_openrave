# frasier_openrave

## Installation

Dependencies:
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
* [HSR Software](https://docs.hsr.io/manual_en/index.html)
* [OpenRAVE](http://openrave.org)
* [TrajOpt](http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/)
* [Gurobi](http://www.gurobi.com/)

**OpenRAVE Installation**   
Install dependencies:
```
sudo apt install cmake g++ git ipython minizip
python-dev python-h5py python-numpy python-scipy python-sympy qt4-dev-tools
```
```
sudo apt install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev
libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev   
libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev
libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev
```
Build OpenRAVE from source:   
1. `git clone  https://github.com/rdiankov/openrave.git`
2. `cd openrave && mkdir build && cd build`  
3. `cmake .. -DOPENRAVE_PLUGIN_QTOSGRAVE=OFF`
4. `make -j5`
5. `sudo make install`   
6. Add the following to `.bashrc`:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)
```
Note: OSG is turned off because OpenRAVE requires OSG 3.4 which is creating problems for TrajOpt compiliation.
The package uses QtCoin for the GUI.

**TrajOpt Installation**   
Install dependencies:
```
sudo apt install libopenscenegraph-dev libeigen3-dev
```
Trajopt uses Gurobi for the optimization. Install Gurobi by following the instructions [here](https://www.gurobi.com/registration/download-reg). You can get a free acadamic license if you have .edu email.
1. `git clone  git@github.com:tkelestemur/trajopt.git`
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
- [x] cannot set goal angle for base_t_joint
- [x] fix viewer (change to osg viewer)
- [ ] redundant ik solver from openrave
- [ ] point cloud updater using ROS
- [x] IK using trajopt
- [x] planning using trajopt
