<!-- <h1 align="center">Cooperative Object Transportation using Gibbs Random Fields</h1>
<p align="center">2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2021)</p> -->


<!-- <p align="center">
<a href="https://www.youtube.com/watch?v=hrkJKL3W3pQ"><img src="https://img.youtube.com/vi/hrkJKL3W3pQ/0.jpg" width="500"></a></p> -->

<p align="center">
<iframe width="560" height="315" src="https://www.youtube.com/embed/hrkJKL3W3pQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe></p>

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
    && git checkout b0ed38f9ecedbe929340f5e8b0aa7a457248e015 \ #branch before tf_prefix deprecation decision
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

- First, start Gazebo simulator:
```sh
roslaunch hero_gazebo gazebo_bringup.launch
```
<p align="center">
<img src="https://user-images.githubusercontent.com/14208261/222988971-ec5c9e56-2743-4e45-a0bb-23ce0301ce65.png" width="600"></p>

- Then, spawn the objects:
```sh
roslaunch grf_transport spawn_object.launch
```
<p align="center">
<img src="https://user-images.githubusercontent.com/14208261/222989074-b8048ee4-7d25-4df4-a5c5-8872401348f1.png" width="600"></p>

- Now, spawn the robots and the environment ( and unpause Gazebo):
```sh
roslaunch grf_transport spawn_robots.launch
```
<p align="center">
<img src="https://user-images.githubusercontent.com/14208261/222989868-cadc0e69-bb57-4669-9c36-dc4ead95db17.png" width="600"></p>

- Start GRF controller:
```sh
roslaunch grf_transport grf_controller.launch
```
<p align="center">
<img src="https://user-images.githubusercontent.com/14208261/222990076-e2ead94e-3396-4ee1-8c3b-4c67cd272ea6.png" width="600"></p>

- RViz launch (for debug):
```sh
roscd grf_transport/config/  \
&& rosrun rviz rviz -d config.rviz
```
<p align="center">
<img src="https://user-images.githubusercontent.com/14208261/222990175-a25ac7cf-0a09-4f02-bd56-8b4b6ed749fe.png" width="600"></p>


<h3 align="left">Setup</h3>
- How change the number of robots?

```sh
roslaunch hero_gazebo gazebo_wizard.launch
```
<p align="center">
<img src="https://user-images.githubusercontent.com/14208261/222992920-4bea83ab-e3af-447f-9f0c-121eb1d1de6a.png" width="600"></p>

Also set the number of robots in the grf controller ```param.yaml``` file:

```
# Number of robots in the environment
robots: 30
.
.
.
```

- How to change the object?

Edit the launch ```spawn_object.launch``` and select the object you want to spawn:

```
<!-- Select a object shape: "rectangular_prism, triangular_prism, polygonal_prism" -->
    <arg name="object_shape" default="rectangular_prism" />
    <param name="object_shape" value="$(arg object_shape)" />
```

---

<h2>Related Publications</h2>

- Rezeck, P., Azpurua, H., Correa, M. F., & Chaimowicz, L. (2022). Hero 2.0: A low-cost robot for swarm robotics research. arXiv preprint arXiv:2202.12391.

- P. Rezeck and L. Chaimowicz, "Chemistry-Inspired Pattern Formation With Robotic Swarms," in IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 9137-9144, Oct. 2022, doi: 10.1109/LRA.2022.3190638.

- P. Rezeck, R. M. Assunção and L. Chaimowicz, "Flocking-Segregative Swarming Behaviors using Gibbs Random Fields," 2021 IEEE International Conference on Robotics and Automation (ICRA), Xi'an, China, 2021, pp. 8757-8763, doi: 10.1109/ICRA48506.2021.9561412.

