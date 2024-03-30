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
#include <std_srvs/Empty.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
// #include <gazebo_msgs/SetLightProperties.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>

#include <xmlrpcpp/XmlRpcValue.h> 

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include <omp.h>
#define USE_OPENMP_

// #define OBJECT_SX 0.8
// #define OBJECT_SY 0.8

// #define OBJECT_SX 2.4
// #define OBJECT_SY 2.4

// #define OBJECT_SX 1.259320
// #define OBJECT_SY 1.007450

// #define OBJECT_SX 2.51864
// #define OBJECT_SY 2.0149

#define RECTANGLE_PRISM "rectangular_prism"
#define TRIANGLE_PRIM "triangular_prism"
#define POLY_PRISM "polygonal_prism"
// Choose one above
#define TARGET_OBJECT RECTANGLE_PRISM
// #define OBJECT_POS_X 0.0
// #define OBJECT_POS_Y 0.0

#define ROBOT_COLOR_STATE

// #define EXPERIMENT_MODE
// #define ENABLE_FAILURES
// #define MAX_ROBOTS_FAILS 4

// #define PUBLISH_OBJECT_STATE

// #define OBJECT_SX 1.57415
// #define OBJECT_SY 1.2593125

// #define OBJECT_SX 1.88898
// #define OBJECT_SY 1.511175

#define LASER_RESOLUTION 0.05

class Vector2
{
public:
    Vector2() : x(0.0), y(0.0){};
    Vector2(double x_, double y_) : x(x_), y(y_){};
    double x;
    double y;
};

class LineStruct
{
public:
    Vector2 start;
    Vector2 end;

    LineStruct() : start(Vector2()), end(Vector2()) {}
    LineStruct(Vector2 start_, Vector2 end_) : start(start_), end(end_) {}
};

class Robot
{
public:
    Vector2 position;
    Vector2 velocity;
    double id;
    double type;
    double theta;
    int state;
    bool is_dead;
};

class Body
{
public:
    std::string name;
    std::string type;
    Vector2 cm_position;
    geometry_msgs::Quaternion cm_orientation;
    std::vector<LineStruct> global_lines;
    std::vector<LineStruct> local_lines;    
    double roll, pitch, yaw;
    bool is_obstacle = false;
};

class Controller
{
public:
    // WiseRobot Constructor
    Controller(ros::NodeHandle *nodehandle);

    std::ofstream outputFile;

    // Robot parameters
    int robots = 10;
    int groups = 0;
    double sensing = 0.5;
    double safezone = 0.15;
    double mass = 5.0;
    double vmax = 0.3;
    double dt = 0.01;

    // Environment parameters
    std::string environment_name = "arena_1";
    double worldsize = 3.8;

    // Object parameters
    std::string object_shape = "rectangular_prism";
    double scale_x = 1.0;
    double scale_y = 1.0;

    
    int num_died_robots = 0;
    int metric_v;

    bool is_running = true;

    std::string worldfile;
    std::string logginfile;
    std::ofstream logfile;

    std::vector<Robot> states;
    std::vector<Body> bodies_state;
    std::vector<std::string> targets_name;
    int targets_id;
    Vector2 target;
    boost::mutex mutex;

    void update(long iterations);
    void resetSimulation();

private:
    /* ROS Node Handle */
    ros::NodeHandle nh_;

    /* Topics and Services */
    std::vector<ros::Publisher> r_cmdvel_;
    std::string cmd_vel_topic = "/cmd_vel";
    std::vector<ros::Subscriber> r_pose_;
    std::string odom_topic =  "/odom";
    ros::Subscriber gz_model_poses_;
    std::vector<ros::Publisher> r_cmdcolor_;
    std::string led_topic =  "/led";

    /* Topics Callbacks */
    void r_pose_cb(const nav_msgs::OdometryConstPtr &msg, const std::string &topic, const int &id);
    void gz_poses_cb(const gazebo_msgs::ModelStatesConstPtr &msg);

    /* RViz debug topics */
    ros::Publisher pub_target_vel_markers;
    std::string pub_target_vel_markers_topic = "/show_target_vel_rviz";
    bool pub_target_vel_markers_rviz = false;
    ros::Publisher pub_neighborns_markers;
    std::string pub_neighborns_markers_topic = "/show_neighborns_rviz";
    bool pub_neighborns_markers_rviz = false;
    ros::Publisher pub_obstacles_markers;
    std::string pub_obstacles_markers_topic = "/show_obstacles_rviz";
    bool pub_obstacles_markers_rviz = false;
    ros::Publisher pub_objects_markers;
    std::string pub_objects_markers_topic = "/show_objects_rviz";
    bool pub_objects_markers_rviz = true;
    ros::Publisher pub_gradient_object;
    std::string pub_gradient_object_topic = "/show_gradient_object_rviz";
    bool pub_gradient_object_rviz = false;
    ros::Publisher pub_object_state;
    std::string pub_object_state_topic = "/object_state";
    bool pub_object_state_rviz = true;
// #ifdef EXPERIMENT_MODE
    // ros::Publisher publish_goal_state;
    // Publish object pose

    std_msgs::ColorRGBA getColorByType(uint8_t type);
    void setRobotColor(Robot robot, int colorId);

    std::vector<geometry_msgs::Pose2D> global_poses;
    std::vector<geometry_msgs::Twist> global_velocities;

    double kineticEnergy(double v, double m);
    double coulombBuckinghamPotential(double r, double eplson, double eplson0, double r0, double alpha, double q1, double q2);
    double fof_Us(Robot r_i, Vector2 v, std::vector<Vector2> obstacles);
    double fof_Ut(Robot r_i, Vector2 v, std::vector<Vector2> objects);
    double fof_Ust(Robot r_i, Vector2 v, std::vector<Robot> states_t);
    double euclidean(Vector2 a, Vector2 b);
    std::vector<Vector2> getObstaclesPoints(double sensing, Robot r);
    std::vector<Vector2> getObjectPoints(double sensing, Robot r);
    bool getIntersection(double sensing, Vector2 circle, Vector2 p1, Vector2 p2, Vector2 &o1, Vector2 &o2);
    Vector2 checkSegment(Vector2 v, Vector2 v0, Vector2 v1);
    bool onSegment(Vector2 p, Vector2 q, Vector2 r);
    int orientation(Vector2 p, Vector2 q, Vector2 r);
    bool doIntersect(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2);
    bool doIntersectWithObstacle(Vector2 p1, Vector2 q1);
    bool getSegmentIntersection(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2, Vector2 &out);
    Vector2 getClosestIntersectionPoint(LineStruct line, std::string body_type);

    double targetOcclusion(Robot robot, std::vector<Vector2> objects);
    bool goCWise(Robot robot, std::vector<Vector2> objects);

    std::vector<std::vector<Robot>> getAllRobotsNeighborns(std::vector<Robot> agents);
    Vector2 saturation(Vector2 v, double norm);
    Vector2 metropolisHastings(Robot r_i, std::vector<Robot> states_t);

    double consensusVelocity();
    int clusterNumber(std::vector<Robot> states);
};
