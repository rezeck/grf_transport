#pragma once

#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <random>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <sys/time.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <gazebo_msgs/SetLightProperties.h>
#include <gazebo_msgs/ModelStates.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include <omp.h>

#define SHOW_TARGET_VEL_RVIZ
#define SHOW_OBSTACLES_RVIZ
#define SHOW_OBJECT_RVIZ
#define SHOW_GRADIENT_OBJECT_RVIZ
// #define SHOW_NEIGHBORNS_RVIZ

#define OBJECT_SIZE 0.39704 // (cm)
#define OBJECT_SX 1.259320
#define OBJECT_SY 1.007450

class Vector2
{
public:
    Vector2() : x(0.0), y(0.0){};
    Vector2(double x_, double y_) : x(x_), y(y_){};
    double x;
    double y;
};

class Robot
{
public:
    Vector2 position;
    Vector2 velocity;
    double id;
    double type;
    double theta;
};

class Body
{
public:
    std::string name;
    Vector2 cm_position;
    geometry_msgs::Quaternion cm_orientation;
    std::vector<Vector2> global_corners;
    std::vector<Vector2> local_corners;
    double roll, pitch, yaw;
    bool is_obstacle = false;
};


class Controller
{
public:
    // WiseRobot Constructor
    Controller(ros::NodeHandle *nodehandle);

    int robots;
    int groups;
    double sensing;
    double worldsize;
    double safezone;
    double mass;
    double vmax;
    double dt;

    int metric_v;

    std::string worldfile;
    std::string logginfile;
    std::ofstream logfile;

    std::vector<Robot> states;
    std::vector<Body> bodies_state;
    boost::mutex mutex;

    void update(long iterations);

private:
    ros::NodeHandle nh_; // ROS Node Handle

    std_msgs::ColorRGBA getColorByType(uint8_t type);

    std::vector<ros::Publisher> r_cmdvel_;
    void r_pose_cb(const nav_msgs::OdometryConstPtr &msg, const std::string &topic, const int &id);
    std::vector<ros::Subscriber> r_pose_;
    std::vector<geometry_msgs::Pose2D> global_poses;
    std::vector<geometry_msgs::Twist> global_velocities;
    std::vector<ros::ServiceClient> color_service;
    ros::Subscriber gz_model_poses_;
    void gz_poses_cb(const gazebo_msgs::ModelStatesConstPtr &msg);
    ros::Publisher object_points_pub;
    ros::Publisher object_gradient_pub;

    ros::Publisher show_target_vel_rviz;
    ros::Publisher show_neighborns_rviz;
    ros::Publisher show_obstacles_rviz;
    ros::Publisher show_objects_rviz;
    ros::Publisher show_gradient_object_rviz;

    double kineticEnergy(double v, double m);
    double coulombBuckinghamPotential(double r, double eplson, double eplson0, double r0, double alpha, double q1, double q2);
    double fof_Us(Robot r_i, Vector2 v);
    double fof_Ut(Robot r_i, Vector2 v);
    double fof_Ust(Robot r_i, Vector2 v, std::vector<Robot> states_t);
    double euclidean(Vector2 a, Vector2 b);
    std::vector<Vector2> getObstaclesPoints(double sensing, Robot r);
    std::vector<Vector2> getObjectPoints(double sensing, Robot r);
    bool getIntersection(double sensing, Vector2 circle, Vector2 p1, Vector2 p2, Vector2 &o1, Vector2 &o2);
    Vector2 checkSegment(Vector2 v, Vector2 v0, Vector2 v1);
    bool onSegment(Vector2 p, Vector2 q, Vector2 r);
    int orientation(Vector2 p, Vector2 q, Vector2 r);
    bool doIntersect(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2);
    int doIntersectWithObstacle(Vector2 p1, Vector2 q1, std::vector<Vector2> obstacle);
    bool getSegmentIntersection(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2, Vector2 &out);

    std::vector<std::vector<Robot>> getAllRobotsNeighborns(std::vector<Robot> agents);
    Vector2 saturation(Vector2 v, double norm);
    Vector2 metropolisHastings(Robot r_i, std::vector<Robot> states_t);
};
