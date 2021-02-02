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
#include <std_msgs/ColorRGBA.h>
#include <gazebo_msgs/SetLightProperties.h>
#include <gazebo_msgs/ModelStates.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"


#include <omp.h>

// #define SHOW_VELOCITY
// #define SHOW_SENSING
// #define SHOW_OBSTACLES
// #define SAVE_FIGURES
// #define VISUALIZATION
// #define LOGGING

// #define V_CONTROL

#define OBJECT_SIZE 0.39704 // (cm)
#define OBJECT_SX 1.259320
#define OBJECT_SY 1.007450

class Vector2{
public:
	double x;
	double y;
};

class Robot{
public:
	Vector2 position;
	Vector2 velocity;
	double id;
	double type;
};

class Object{
    public:
    std::string name;
    geometry_msgs::Point cm_position;
    geometry_msgs::Quaternion cm_orientation;
    std::vector<geometry_msgs::Point> contact_points;
    double roll, pitch, yaw;
};


class ClusterMetric
{
public:
    ClusterMetric(double robots, double groups, double threshold);
    double robots, groups, threshold;
    int compute(std::vector<Robot> states);
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
    std::vector<Vector2> obstacles;
    Object object_state_;
    boost::mutex mutex;



    #ifdef V_CONTROL
    std::vector<Vector2> ver_points;
    std::vector<Vector2> lab_points;
    #endif

	void update(long iterations);
    bool draw(int step);


private:
    ros::NodeHandle nh_; // ROS Node Handle

    int countCollisions();
    double meanDist();
    ros::Publisher mDist_pub, consensusVel_pub, mClu_pub;
    double consensusVel();
    std_msgs::ColorRGBA getColorByType(uint8_t type);

    std::vector<ros::Publisher> r_cmdvel_;
    void r_pose_cb(const nav_msgs::OdometryConstPtr &msg, const std::string &topic, const int &id);
    std::vector<ros::Subscriber> r_pose_;
    std::vector<geometry_msgs::Pose2D> global_poses;
    std::vector <geometry_msgs::Twist> global_velocities;
    std::vector <ros::Publisher> target_velocities_pub;
    std::vector <ros::ServiceClient> color_service;

    double kineticEnergy(double v, double m);
    double coulombBuckinghamPotential(double r, double eplson, double eplson0, double r0, double alpha, double q1, double q2);
    double fof_Us(Robot r_i, Vector2 v);
    double fof_Ust(Robot r_i, Vector2 v, std::vector<Robot> states_t);
    double euclidean(Vector2 a, Vector2 b);
    std::vector<Vector2> getObstaclesPoints(double sensing, Vector2 p);
    bool getIntersection(double sensing, Vector2 circle, Vector2 p1, Vector2 p2, Vector2& o1, Vector2& o2);
    std::vector<std::vector<Robot>> getAllRobotsNeighborns(std::vector<Robot> agents);
    Vector2 saturation(Vector2 v, double norm);
    Vector2 metropolisHastings(Robot r_i, std::vector<Robot> states_t);
};


