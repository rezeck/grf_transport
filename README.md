<h1 align="center">Cooperative Object Transportation using Gibbs Random Fields</h1>
<p align="center">2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2021)</p>


<p align="center">
<a href="https://www.youtube.com/watch?v=hrkJKL3W3pQ"><img src="https://img.youtube.com/vi/hrkJKL3W3pQ/0.jpg" width="500"></a></p>

<p align="center">
This paper presents a novel methodology that allows a swarm of robots to perform a cooperative transportation task. Our approach consists of modeling the swarm as a Gibbs Random Field (GRF), taking advantage of this framework's locality properties. By setting appropriate potential functions, robots can dynamically navigate, form groups, and perform cooperative transportation in a completely decentralized fashion. Moreover, these behaviors emerge from the local interactions without the need for explicit communication or coordination. To evaluate our methodology, we perform a series of simulations and proof-of-concept experiments in different scenarios. Our results show that the method is scalable, adaptable, and robust to failures and changes in the environment.</p>


<h2 align="left">Citation</h2>

```
@inproceedings{rezeck2021collective,
  author={Rezeck, Paulo and Assunção, Renato M. and Chaimowicz, Luiz},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Cooperative Object Transportation using Gibbs Random Fields}, 
  year={2021},
  volume={},
  number={},
  pages={9131-9138},
  doi={10.1109/IROS51168.2021.9635928}
}
```
---

<h2 align="left">Usage</h2>
<h3 align="left">Prerequisites</h3>

- Ubuntu 18.04/20.04
- ROS Melodic/Noetic
- Gazebo 11
- [HeRo common package](https://verlab.github.io/hero_common)

<h3 align="left">Installation</h3>
<h4 align="left"><b>Option 1:</b> For Docker users</h4>

- Download this repository (1 of 2):
```sh
git clone https://github.com/verlab/grf_colletive_transport.git /
&& cd grf_colletive_transport
```
- Build docker image (2 of 2):
```sh
docker-compose build \
&& docker images
```
```
REPOSITORY                   TAG               IMAGE ID       CREATED         SIZE
grf_swarm_tranport           v1.0              f18898a6cb7a   6 days ago      3.89GB
```

<h4 align="left"><b>Option 2:</b> System installation</h4>

- Access your ROS workspace directory (1 of 5):
```sh
CATKIN_DIR=~/catkin_ws
cd $CATKIN_DIR
```
- Install dependecies (2 of 5):
```sh
apt-get install -y \
      qt5-default \
      python3-pyqt5 \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-usb-cam \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-urdfdom-py \
      ros-${ROS_DISTRO}-rosserial \
      ros-${ROS_DISTRO}-rosserial-server \
      ros-${ROS_DISTRO}-urdf \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-rviz \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-gazebo-plugins \
      ros-${ROS_DISTRO}-gazebo-ros-control
```
- Install gazebo_ros  (3 of 5):
```sh
    cd $CATKIN_DIR/src \
    && git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git \
    && cd gazebo_ros_pkgs \
    && git checkout b0ed38f9ecedbe929340f5e8b0aa7a457248e015 #branch before tf_prefix deprecation decision \ 
    && cd $CATKIN_DIR \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make"
```

- Install hero common package (4 of 5):
```sh
    cd $CATKIN_DIR/src \
    && git clone --depth 1 --branch noetic-devel https://github.com/verlab/hero_common.git \
    && cd $CATKIN_DIR \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make"
```

- Install this GRF transport package (5 of 5):
```sh
  cd $CATKIN_DIR/src /
  && git clone https://github.com/verlab/grf_colletive_transport.git /
  && cd $CATKIN_DIR /
  && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make"
```


<h3 align="left">Execution</h3>

---

## Usage

- How to run the algorithm proposed in this work?

### Launching HeRo gazebo
```sh
$ roslaunch hero_gazebo gazebo_bringup.launch 
```
<p align="center">
  <img width="500" src="resources/gazebo_hero.png">
</p>

### Start our code
- Use the script ```spawn_multi_robots_launch.py``` to create a ```.launch``` file with a pre-set environment.
```sh
$ python spawn_multi_robots_launch.py --help
usage: spawn_multi_robots_launch.py [-h] [--robots ROBOTS] [--groups GROUPS]
                                    [--sensing SENSING]
                                    [--worldsize WORLDSIZE]
                                    [--safezone SAFEZONE] [--dt DT]
                                    [--mass MASS] [--vmax VMAX] [--seed SEED]
```
- For example:
```sh
$ python3 spawn_multi_robots_launch.py --robots 80 --groups 4 > swarm.launch
```
- Now, launch the file using ros and then the robots should start spawning on Gazebo.
```sh
$ roslaunch grf_transport swarm.launch
```
- Initial our transport controller
```sh
$ rosrun grf_transport grf_rl_transport_node
```

<p align="center">
  <img width="500" src="resources/heros.png">
</p>



<h2>Related Publications</h2>

- hero
- grf segregation
- grf pattern formation
