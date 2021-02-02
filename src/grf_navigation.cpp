#include "grf_navigation.h"

ClusterMetric::ClusterMetric(double robots_, double groups_, double threshold_) : robots(robots_), groups(groups_), threshold(threshold_) {}

int ClusterMetric::compute(std::vector<Robot> states)
{
    std::vector<std::vector<Robot>> clusters;

    while (states.size())
    {
        std::vector<Robot> cluster;
        cluster.push_back(states[0]);
        states.erase(states.begin());
        bool founded = true;
        while (founded)
        {
            founded = false;
            for (int i = 0; i < cluster.size(); i++)
            {
                for (int j = 0; j < states.size(); j++)
                {
                    if (cluster[i].type != states[j].type)
                    {
                        continue;
                    }
                    double dx = cluster[i].position.x - states[j].position.x;
                    double dy = cluster[i].position.y - states[j].position.y;
                    double dist = sqrt(dx * dx + dy * dy);
                    if (dist > this->threshold)
                    {
                        continue;
                    }
                    cluster.push_back(states[j]);
                    states.erase(states.begin() + j);
                    founded = true;
                    break;
                }
            }
        }
        clusters.push_back(cluster);
    }

    return (int)clusters.size();
}

Controller::Controller(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{ // constructor
    this->sensing = 0.50;
    ros::param::get("/sensing", this->sensing);
    this->safezone = 0.15;
    ros::param::get("/safezone", this->safezone);
    this->mass = 5.0;
    ros::param::get("/mass", this->mass);
    this->vmax = 0.30;
    ros::param::get("/vmax", this->vmax);
    this->dt = 0.01;
    ros::param::get("/dt", this->dt);
    this->worldsize = 3.80;
    ros::param::get("/worldsize", this->worldsize);

    this->robots = 10;
    ros::param::get("/robots", this->robots);
    this->groups = 2;
    ros::param::get("/groups", this->groups);

    for (int i = 0; i < this->robots; i++)
    {
        // Topics name
        std::string robot_name = "/hero_" + boost::lexical_cast<std::string>(i);
        ROS_INFO("Starting robot: %s", robot_name.c_str());
        std::string cmd_topic = robot_name + "/cmd_vel";
        std::string pose_topic = robot_name + "/odom";
        std::string target_topic = robot_name + "/target";
        std::string color_topic = robot_name + "/hat_color";
        // Topics
        this->r_cmdvel_.push_back(nh_.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 1));
        this->r_pose_.push_back(nh_.subscribe<nav_msgs::Odometry>(pose_topic.c_str(), 1, boost::bind(&Controller::r_pose_cb, this, _1, pose_topic, i)));
        this->color_service.push_back(nh_.serviceClient<gazebo_msgs::SetLightProperties>(color_topic.c_str()));
        // this->color_service[i].waitForExistence()
        int type_ = 0;
        ros::param::get(robot_name + "/type", type_);
        std_msgs::ColorRGBA color = this->getColorByType((uint8_t)type_);
        gazebo_msgs::SetLightProperties gz_color;
        gz_color.request.diffuse.a = color.a / 255.0;
        gz_color.request.diffuse.r = color.r / 255.0;
        gz_color.request.diffuse.g = color.g / 255.0;
        gz_color.request.diffuse.b = color.b / 255.0;
        this->color_service[i].call(gz_color);
        geometry_msgs::Pose2D p;
        geometry_msgs::Twist t;
        this->global_velocities.push_back(t);
        this->global_poses.push_back(p);
        this->target_velocities_pub.push_back(nh_.advertise<visualization_msgs::Marker>(target_topic.c_str(), 1));

        // Get initial state of the robots
        Robot r;
        // Get initial velocities
        r.velocity.x = this->vmax;
        r.velocity.y = this->vmax;
        r.type = type_;
        r.id = i;
        ROS_INFO("Type: %f", r.type);
        this->states.push_back(r);
    }

    this->mDist_pub = nh_.advertise<std_msgs::Float32>("/mDist", 1); //, consensusVel_pub, mClu_pub;
    this->consensusVel_pub = nh_.advertise<std_msgs::Float32>("/consensusVel", 1);
    this->mClu_pub = nh_.advertise<std_msgs::Float32>("/mClu", 1);
}

std_msgs::ColorRGBA Controller::getColorByType(uint8_t type)
{
    std_msgs::ColorRGBA color;
    color.a = 255.0;
    switch (type)
    { // BGR
    case 0:
        color.r = 128;
        color.g = 0;
        color.b = 0;
        break; // maroon
    case 1:
        color.r = 47;
        color.g = 79;
        color.b = 79;
        break; // dark slate gray
    case 2:
        color.r = 138;
        color.g = 43;
        color.b = 226;
        break; // blue violet
    case 3:
        color.r = 199;
        color.g = 21;
        color.b = 133;
        break; // medium violet red
    case 4:
        color.r = 144;
        color.g = 238;
        color.b = 144;
        break; // light green
    case 5:
        color.r = 255;
        color.g = 215;
        color.b = 0;
        break; // gold
    case 6:
        color.r = 218;
        color.g = 165;
        color.b = 32;
        break; // golden rod
    case 7:
        color.r = 189;
        color.g = 183;
        color.b = 107;
        break; // dark khaki
    case 8:
        color.r = 128;
        color.g = 128;
        color.b = 0;
        break; // olive
    case 9:
        color.r = 154;
        color.g = 205;
        color.b = 50;
        break; // yellow green
    case 10:
        color.r = 107;
        color.g = 142;
        color.b = 35;
        break; // olive drab
    case 11:
        color.r = 127;
        color.g = 255;
        color.b = 0;
        break; // chart reuse
    case 12:
        color.r = 0;
        color.g = 100;
        color.b = 0;
        break; // dark green
    case 13:
        color.r = 255;
        color.g = 140;
        color.b = 0;
        break; // dark orange
    case 14:
        color.r = 46;
        color.g = 139;
        color.b = 87;
        break; // sea green
    case 15:
        color.r = 102;
        color.g = 205;
        color.b = 170;
        break; // medium aqua marine
    case 16:
        color.r = 220;
        color.g = 20;
        color.b = 60;
        break; // crimson
    case 17:
        color.r = 0;
        color.g = 139;
        color.b = 139;
        break; // dark cyan
    case 18:
        color.r = 0;
        color.g = 255;
        color.b = 255;
        break; // cyan
    case 19:
        color.r = 70;
        color.g = 130;
        color.b = 180;
        break; // steel blue
    case 20:
        color.r = 100;
        color.g = 149;
        color.b = 237;
        break; // corn flower blue
    case 21:
        color.r = 30;
        color.g = 144;
        color.b = 255;
        break; // dodger blue
    case 22:
        color.r = 0;
        color.g = 0;
        color.b = 128;
        break; // navy
    case 23:
        color.r = 240;
        color.g = 128;
        color.b = 128;
        break; // light coral
    case 24:
        color.r = 75;
        color.g = 0;
        color.b = 130;
        break; // indigo
    case 25:
        color.r = 139;
        color.g = 0;
        color.b = 139;
        break; // dark magenta
    case 26:
        color.r = 238;
        color.g = 130;
        color.b = 238;
        break; // violet
    case 27:
        color.r = 255;
        color.g = 160;
        color.b = 122;
        break; // light salmon
    case 28:
        color.r = 255;
        color.g = 105;
        color.b = 180;
        break; // hot pink
    case 29:
        color.r = 112;
        color.g = 128;
        color.b = 144;
        break; // slate gray
    default:
        color.r = 0;
        color.g = 128;
        color.b = 240;
        break; // black
    }
    return color;
}

int Controller::countCollisions()
{
    int collisions = 0;

    for (int i = 0; i < this->robots; i++)
    {
        for (int j = 0; j < this->robots; j++)
        {
            if (i == j)
                continue;
            double dist = this->euclidean(this->states[i].position, this->states[j].position);
            if (dist <= 0.14)
            {
                collisions++;
            }
        }
    }
    return collisions / 2;
}

double Controller::meanDist()
{
    double overallMeanDist = 0;
    double counter = 0;

    for (int i = 0; i < this->robots; i++)
    {
        for (int j = 0; j < this->robots; j++)
        {
            if ((i == j) || (this->states[i].type != this->states[j].type))
                continue;
            double dist = this->euclidean(this->states[i].position, this->states[j].position);
            overallMeanDist += dist;
            counter++;
        }
    }
    return overallMeanDist / counter;
}

double Controller::consensusVel()
{
    double overallVelNorm = 0;
    double counter = 0;

    for (int i = 0; i < this->robots; i++)
    {
        for (int j = 0; j < this->robots; j++)
        {
            if ((i == j) || (this->states[i].type != this->states[j].type))
                continue;
            double dist = this->euclidean(this->states[i].velocity, this->states[j].velocity);
            overallVelNorm += dist;
            counter++;
        }
    }
    return overallVelNorm / counter;
}

/* Callbacks */
void Controller::r_pose_cb(const nav_msgs::OdometryConstPtr &msg, const std::string &topic, const int &id)
{
    // ROS_INFO("Robot %d getting poses callback", id);
    this->global_poses[id].x = msg->pose.pose.position.x;
    this->global_poses[id].y = msg->pose.pose.position.y;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    this->global_poses[id].theta = yaw;
    this->global_velocities[id].linear.x = msg->twist.twist.linear.x;
    this->global_velocities[id].linear.y = msg->twist.twist.linear.y;
}

bool Controller::draw(int step)
{
    // Create board
    cv::Mat board(700, 700, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Scalar color;
    cv::rectangle(board, cv::Point(50, 50), cv::Point(650, 650), cv::Scalar(0, 0, 0), 4, 8);

    cv::putText(board, std::to_string(step), cv::Point(board.cols / 2, 35), cv::FONT_HERSHEY_DUPLEX,
                0.6, CV_RGB(0, 0, 0), 1);
    // cv::putText(board, "Clusters: " + std::to_string(this->metric_v), cv::Point(50, 35), cv::FONT_HERSHEY_DUPLEX,
    //         0.6, CV_RGB(0, 0, 0), 1);

    float c = 300.0 / 2.0;

#ifdef SHOW_VELOCITY
    for (int i = 0; i < this->robots; i++)
    {
        Vector2 vel;
        vel = this->saturation(this->states[i].velocity, 0.3);
        vel.x = 350 + c * (this->states[i].position.x + vel.x);
        vel.y = 350 - c * (this->states[i].position.y + vel.y);
        cv::arrowedLine(board, cv::Point(350 + c * this->states[i].position.x, 350 - c * this->states[i].position.y), cv::Point(vel.x, vel.y), cv::Scalar(220, 220, 220), 2, 8);
    }
#endif

#ifdef SHOW_SENSING
    for (int i = 0; i < this->robots; i++)
    {
        cv::circle(board, cv::Point(350 + c * this->states[i].position.x, 350 - c * this->states[i].position.y), c * this->sensing, cv::Scalar(240, 240, 240), 1, 8);
    }
#endif

    for (int i = 0; i < this->robots; i++)
    {
        switch ((int)this->states[i].type)
        { // BGR
        case 0:
            color = cv::Scalar(128, 0, 0);
            break; // maroon
        case 1:
            color = cv::Scalar(47, 79, 79);
            break; // dark slate gray
        case 2:
            color = cv::Scalar(138, 43, 226);
            break; // blue violet
        case 3:
            color = cv::Scalar(199, 21, 133);
            break; // medium violet red
        case 4:
            color = cv::Scalar(144, 238, 144);
            break; // light green
        case 5:
            color = cv::Scalar(255, 215, 0);
            break; // gold
        case 6:
            color = cv::Scalar(218, 165, 32);
            break; // golden rod
        case 7:
            color = cv::Scalar(189, 183, 107);
            break; // dark khaki
        case 8:
            color = cv::Scalar(128, 128, 0);
            break; // olive
        case 9:
            color = cv::Scalar(154, 205, 50);
            break; // yellow green
        case 10:
            color = cv::Scalar(107, 142, 35);
            break; // olive drab
        case 11:
            color = cv::Scalar(127, 255, 0);
            break; // chart reuse
        case 12:
            color = cv::Scalar(0, 100, 0);
            break; // dark green
        case 13:
            color = cv::Scalar(255, 140, 0);
            break; // dark orange
        case 14:
            color = cv::Scalar(46, 139, 87);
            break; // sea green
        case 15:
            color = cv::Scalar(102, 205, 170);
            break; // medium aqua marine
        case 16:
            color = cv::Scalar(220, 20, 60);
            break; // crimson
        case 17:
            color = cv::Scalar(0, 139, 139);
            break; // dark cyan
        case 18:
            color = cv::Scalar(0, 255, 255);
            break; // cyan
        case 19:
            color = cv::Scalar(70, 130, 180);
            break; // steel blue
        case 20:
            color = cv::Scalar(100, 149, 237);
            break; // corn flower blue
        case 21:
            color = cv::Scalar(30, 144, 255);
            break; // dodger blue
        case 22:
            color = cv::Scalar(0, 0, 128);
            break; // navy
        case 23:
            color = cv::Scalar(240, 128, 128);
            break; // light coral
        case 24:
            color = cv::Scalar(75, 0, 130);
            break; // indigo
        case 25:
            color = cv::Scalar(139, 0, 139);
            break; // dark magenta
        case 26:
            color = cv::Scalar(238, 130, 238);
            break; // violet
        case 27:
            color = cv::Scalar(255, 160, 122);
            break; // light salmon
        case 28:
            color = cv::Scalar(255, 105, 180);
            break; // hot pink
        case 29:
            color = cv::Scalar(112, 128, 144);
            break; // slate gray
        default:
            color = cv::Scalar(0, 0, 0);
            break; // black
        }
        std::swap(color[0], color[2]);
        cv::circle(board, cv::Point(350 + c * this->states[i].position.x, 350 - c * this->states[i].position.y), c * 0.03, color, -1, 8);
    }

#ifdef SHOW_OBSTACLES
    for (int i = 0; i < this->obstacles.size(); i++)
    {
        cv::circle(board, cv::Point(350 + c * this->obstacles[i].x, 350 - c * this->obstacles[i].y), c * 0.01, cv::Scalar(0, 0, 255), -1, 8);
    }
    this->obstacles.clear();
#endif

    cv::imshow("GRF-Transport", board);
#ifdef SAVE_FIGURES
    char filenanme[17];
    std::sprintf(filenanme, "image_%06d.png", step);
    cv::imwrite(filenanme, board);
#endif
    return (cv::waitKey(1) != 27);
}

double Controller::kineticEnergy(double v, double m)
{
    return 0.5 * v * m;
}

double Controller::coulombBuckinghamPotential(double r, double eplson, double eplson0, double r0, double alpha, double q1, double q2)
{
    // Compute the Coulomb-Buckingham Potential
    return eplson * ((6.0 / (alpha - 6.0)) * exp(alpha) * (1.0 - r / r0) - (alpha / (alpha - 6.0)) * std::pow(r0 / r, 6)) + (q1 * q2) / (4.0 * M_PI * eplson0 * r);
}

double Controller::fof_Us(Robot r_i, Vector2 v)
{
    // Simulated (kinematic model of the robot) the motion of the robot using the sampled velocity
    r_i.position.x = r_i.position.x + v.x * this->dt;
    r_i.position.y = r_i.position.y + v.y * this->dt;

    // Get potential for the sampled velocity
    double Us = 0.0f;
    // for each obstacles point in the world (same when using a laser)
    std::vector<Vector2> obstacles = this->getObstaclesPoints(this->safezone, r_i.position);
    // ROS_INFO("Number of obstacles: %d", obstacles.size());
    for (int i = 0; i < obstacles.size(); ++i)
    {
#ifdef SHOW_OBSTACLES
        // this->mutex.lock();
        this->obstacles.push_back(obstacles[i]);
// this->mutex.unlock();
#endif
        // ROS_INFO("Robot: %f O(%f, %f)", r_i.id, obstacles[i].x, obstacles[i].y);
        // ROS_INFO("Number of obstacles: %f %f", obstacles[i].x, obstacles[i].y);
        // lets compute the distance to the obstacle (we only use the distance)
        double dist = this->euclidean(r_i.position, obstacles[i]);
        // if (dist <= (this->safezone*2.0)){
        Us += this->coulombBuckinghamPotential(dist / 2.0, 0.04, 0.04, 0.8, 1.0, 16.0, 1.0);
        // }
    }

    return Us;
}

double Controller::fof_Ust(Robot r_i, Vector2 v, std::vector<Robot> states_t)
{
    // Simulated (kinematic model of the robot) the motion of the robot using the sampled velocity
    r_i.position.x = r_i.position.x + v.x * this->dt;
    r_i.position.y = r_i.position.y + v.y * this->dt;
    // Get the sum of the relative velocity of all my neighbor and they mass
    Vector2 group_vrel;
    double group_mass = this->mass;
    // Get the pairwise potential for the sampled velocity
    double Ust = 0.0f;
    // for each neighborn in current state
    // #ifdef _OPENMP
    // #pragma omp parallel for
    // #endif
    for (int i = 0; i < states_t.size(); ++i)
    {
        Vector2 n_p;
        n_p.x = states_t[i].position.x + states_t[i].velocity.x * this->dt;
        n_p.y = states_t[i].position.y + states_t[i].velocity.y * this->dt;
        double dist = this->euclidean(r_i.position, n_p);
        // double dist = this->euclidean(r_i.position, states_t[i].position);
        // Indicator function f: 1 -> same type, f: -1 -> otherwise
        double I = 2.0 * (int)(r_i.type == states_t[i].type) - 1.0;

        // Don`t be to reative to robot from different type
        if ((I < 0) && (dist > this->safezone))
        {
            continue;
        }
        if ((I > 0) && (dist > this->sensing))
        {
            return 10000;
        }

        Ust += this->coulombBuckinghamPotential(dist * 1.1, 0.04, 0.04, 0.8, 1.0, 16.0 * I, -1.0);

        // Get the sum of the relative velocity of all my neighbor and they mass
        if (I > 0 && (dist < this->sensing) && (dist > this->safezone))
        {
            group_vrel.x += (states_t[i].velocity.x - v.x);
            group_vrel.y += (states_t[i].velocity.y - v.y);
            group_mass += this->mass;
        }
    }
    // Now compute the kinetic Energy using relative velocity
    group_vrel = this->saturation(group_vrel, 1.0);
    double group_speed = (group_vrel.x * group_vrel.x + group_vrel.y * group_vrel.y) + 1.0e-9;
    double my_speed = (v.x * v.x + v.y * v.y);
    // double group_speed = sqrt(group_vrel.x*group_vrel.x + group_vrel.y*group_vrel.y) + 1.0e-9;
    // double my_speed = sqrt(v.x*v.x + v.y*v.y);

    Ust += this->kineticEnergy(group_speed, group_mass) + this->kineticEnergy(group_mass, this->vmax - my_speed);
    // Ust += this->kineticEnergy(group_speed, group_mass) - this->kineticEnergy(group_mass, my_speed);
    return Ust;
}

bool Controller::getIntersection(double r, Vector2 circle, Vector2 p1, Vector2 p2, Vector2 &o1, Vector2 &o2)
{
    // Convert p1 and p2 to be relative to circle; circle -> (0,0)
    p1.x -= circle.x;
    p1.y -= circle.y;

    p2.x -= circle.x;
    p2.y -= circle.y;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dr = sqrt(dx * dx + dy * dy);
    double D = p1.x * p2.y - p2.x * p1.y;
    double disc = (r * r) * (dr * dr) - (D * D);
    double sgn_dy = (dy >= 0) - (dy < 0);

    if (disc > 0.0)
    {
        o1.x = (D * dy + sgn_dy * dx * sqrt(disc)) / (dr * dr);
        o2.x = (D * dy - sgn_dy * dx * sqrt(disc)) / (dr * dr);
        o1.y = (-D * dx + fabs(dy) * sqrt(disc)) / (dr * dr);
        o2.y = (-D * dx - fabs(dy) * sqrt(disc)) / (dr * dr);

        o1.x += circle.x;
        o2.x += circle.x;
        o1.y += circle.y;
        o2.y += circle.y;
        return true;
    }
    return false;
}

std::vector<std::vector<Robot>> Controller::getAllRobotsNeighborns(std::vector<Robot> agents)
{
    std::vector<std::vector<Robot>> neighbors;
    // #pragma omp parallel for
    for (int i = 0; i < this->robots; ++i)
    {
        std::vector<Robot> ri;
        neighbors.push_back(ri);
    }

    // #pragma omp parallel for
    for (int i = 0; i < (this->robots - 1); ++i)
    {
        for (int j = i + 1; j < this->robots; ++j)
        {
            double dist = this->euclidean(agents[i].position, agents[j].position);
            if (dist <= this->sensing)
            {
                neighbors[j].push_back(agents[i]);
                neighbors[i].push_back(agents[j]);
                // ROS_INFO("Robots %d and %d are neighborns.", i, j);
            }
        }
    }
    return neighbors;
}

std::vector<Vector2> Controller::getObstaclesPoints(double sensing, Vector2 p)
{
    /* World
             p3 o------o p4
                   |          |                            
                   |          |
             p1 o------o p2
    */
    Vector2 p1;
    p1.x = -this->worldsize / 2;
    p1.y = -this->worldsize / 2;
    Vector2 p2;
    p2.x = this->worldsize / 2;
    p2.y = -this->worldsize / 2;
    Vector2 p3;
    p3.x = -this->worldsize / 2;
    p3.y = this->worldsize / 2;
    Vector2 p4;
    p4.x = this->worldsize / 2;
    p4.y = this->worldsize / 2;

    std::vector<Vector2> obstacles;

    Vector2 out1, out2;
    double res = 0.1;
    if (this->getIntersection(sensing, p, p1, p2, out1, out2))
    {
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("12: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.x, out2.x) + res; i < std::max(out1.x, out2.x); i += res)
        {
            Vector2 a;
            a.x = i;
            a.y = out1.y;
            obstacles.push_back(a);
        }
    }
    if (this->getIntersection(sensing, p, p1, p3, out1, out2))
    {
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("13: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.y, out2.y) + res; i < std::max(out1.y, out2.y); i += res)
        {
            Vector2 a;
            a.x = out1.x;
            a.y = i;
            obstacles.push_back(a);
        }
    }
    if (this->getIntersection(sensing, p, p2, p4, out1, out2))
    {
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("24: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.y, out2.y) + res; i < std::max(out1.y, out2.y); i += res)
        {
            Vector2 a;
            a.x = out1.x;
            a.y = i;
            obstacles.push_back(a);
        }
    }
    if (this->getIntersection(sensing, p, p3, p4, out1, out2))
    {
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("34: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = (std::min(out1.x, out2.x) + res); i < std::max(out1.x, out2.x); i += res)
        {
            Vector2 a;
            a.x = i;
            a.y = out1.y;
            obstacles.push_back(a);
        }
    }
    return obstacles;
}

double Controller::euclidean(Vector2 a, Vector2 b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy) + 1.0e-9;
}

Vector2 Controller::saturation(Vector2 v, double norm)
{
    Vector2 vnorm;
    double r = fabs(v.y / v.x);
    if ((r >= 1.0) && (fabs(v.y) > norm))
    {
        vnorm.y = (v.y * norm) / fabs(v.y);
        vnorm.x = (v.x * norm) / (r * fabs(v.x));
    }
    else if ((r < 1.0) && (fabs(v.x) > norm))
    {
        vnorm.x = (v.x * norm) / fabs(v.x);
        vnorm.y = (v.y * norm * r) / (fabs(v.y));
    }
    else
    {
        vnorm.x = v.x;
        vnorm.y = v.y;
    }
    return vnorm;
}

Vector2 Controller::metropolisHastings(Robot r_i, std::vector<Robot> states_t)
{
    std::vector<Vector2> mcmc_chain;
    mcmc_chain.push_back(r_i.velocity);

    std::vector<double> chain_potential;
    chain_potential.push_back(this->fof_Us(r_i, r_i.velocity) +
                              this->fof_Ust(r_i, r_i.velocity, states_t));

    // Random Seed for normal distribution
    unsigned seed_x = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator_x(seed_x);
    unsigned seed_y = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator_y(seed_y);

    for (int i = 0; i < 100; i++)
    {
        // Get the last potential simulated
        Vector2 last_v;
        last_v.x = mcmc_chain.back().x;
        last_v.y = mcmc_chain.back().y;
        double last_U = chain_potential.back();
        // Get a sample of velocity considering normal distribution
        // std::normal_distribution<double> norm_vx(r_i.type - r_i.position.x, 0.2);
        // std::normal_distribution<double> norm_vy(r_i.type - r_i.position.y, 0.2);
        std::normal_distribution<double> norm_vx(r_i.velocity.x, 0.1);
        std::normal_distribution<double> norm_vy(r_i.velocity.y, 0.1);
        Vector2 sampled_vel;
        sampled_vel.x = norm_vx(generator_x);
        sampled_vel.y = norm_vy(generator_y);
        sampled_vel = this->saturation(sampled_vel, this->vmax);
        // Compute the potential for the sampled velocity
        double U = this->fof_Us(r_i, sampled_vel) + this->fof_Ust(r_i, sampled_vel, states_t);
        // Accept the sampled velocity over the gibbs distribuition sampling
        double dE = U - last_U;
        double grf = exp(-dE);
        double r = ((double)rand() / ((double)(RAND_MAX) + (double)(1)));
        if ((dE < 0.0) || (r < grf))
        {
            mcmc_chain.push_back(sampled_vel);
            chain_potential.push_back(U);
        }
        else
        {
            mcmc_chain.push_back(last_v);
            chain_potential.push_back(last_U);
        }
    }
    // Discard first half of MCMC chain and thin out the rest.
    Vector2 new_velocity;
    new_velocity.x = 0.0;
    new_velocity.y = 0.0;
    double n = 0.0;
    int burnin = (int)(0.6 * mcmc_chain.size());
    for (int i = burnin; i < mcmc_chain.size(); i++)
    {
        new_velocity.x += mcmc_chain[i].x;
        new_velocity.y += mcmc_chain[i].y;
        n += 1.0;
    }
    new_velocity.x = new_velocity.x / n;
    new_velocity.y = new_velocity.y / n;
    new_velocity = this->saturation(new_velocity, this->vmax);
    r_i.velocity = new_velocity;
    r_i.position.x += new_velocity.x * this->dt;
    r_i.position.y += new_velocity.y * this->dt;
    this->states[(int)r_i.id] = r_i;
    return new_velocity;
}

void Controller::update(long iterations)
{
    for (int i = 0; i < this->robots; i++)
    {
        this->states[i].position.x = this->global_poses[i].x;
        this->states[i].position.y = this->global_poses[i].y;
    }
    std::vector<Robot> states_t;
    states_t = this->states;
    std::vector<std::vector<Robot>> neighbors = this->getAllRobotsNeighborns(this->states);

    // #pragma omp parallel for ordered schedule(dynamic)
    // #pragma omp parallel for
    for (int i = 0; i < this->robots; ++i)
    {
        Vector2 new_velocity = this->metropolisHastings(states_t[i], neighbors[i]);
    }

    /* Command goto (x,y) */
    for (int i = 0; i < this->robots; ++i)
    {
        geometry_msgs::Twist v;
        visualization_msgs::Marker m;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = "/hero_" + boost::lexical_cast<std::string>(i) + "/odom";
        m.id = i;
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        m.color.a = 1.0;
        m.pose.position.x = this->global_poses[i].x;
        m.pose.position.y = this->global_poses[i].y;
        double yaw_ = atan2(this->states[i].position.y - this->global_poses[i].y, this->states[i].position.x - this->global_poses[i].x);
        tf::Quaternion q;
        q.setEuler(0, 0, yaw_);
        m.pose.orientation.x = q.getX();
        m.pose.orientation.y = q.getY();
        m.pose.orientation.z = q.getZ();
        m.pose.orientation.w = q.getW();
        m.scale.x = sqrt(this->states[i].velocity.y * this->states[i].velocity.y + this->states[i].velocity.x * this->states[i].velocity.x);
        m.scale.y = 0.02;
        m.scale.z = 0.02;
        this->target_velocities_pub[i].publish(m);

#define FORWARD_CONTROLLER
#ifdef FORWARD_CONTROLLER
        double theta_ = atan2(this->states[i].position.y - this->global_poses[i].y, this->states[i].position.x - this->global_poses[i].x);
        double err_ = theta_ - this->global_poses[i].theta;
        // ROS_INFO("theta_: %f, theta: %f, err: %f", theta_, this->global_poses[i].theta, err_);
        double velx = sqrt(this->states[i].velocity.y * this->states[i].velocity.y + this->states[i].velocity.x * this->states[i].velocity.x);

        if (err_ > M_PI)
            err_ = -(2.0 * M_PI - err_);
        else if (err_ < -M_PI)
            err_ = 2.0 * M_PI + err_;

        if (abs(err_) > M_PI / 18)
        {
            v.linear.x = std::min(velx, 0.02);
            v.angular.z = 1.2 * err_;
        }
        else
        {
            v.linear.x = std::min(velx, 0.10);
            v.angular.z = 0.6 * err_;
        }
#else
        double theta_ = atan2(this->states[i].position.y - this->global_poses[i].y, this->states[i].position.x - this->global_poses[i].x);
        double r_theta_f = this->global_poses[i].theta;
        double r_theta_b = r_theta_f > 0 ? r_theta_f - M_PI : M_PI + r_theta_f;
        double err_f = theta_ - r_theta_f;
        double err_b = theta_ - r_theta_b;
        double velx = sqrt(this->states[i].velocity.y * this->states[i].velocity.y + this->states[i].velocity.x * this->states[i].velocity.x);
        // ROS_INFO("velx: %f, err_f: %f, err_b: %f", velx, err_f, err_b);
        if (abs(err_f) < abs(err_b))
        {
            v.linear.x = std::min(velx, 0.10);
            v.angular.z = 1.8 * err_f;
        }
        else
        {
            v.linear.x = -std::min(velx, 0.10);
            v.angular.z = -1.8 * err_b;
        }
        // ROS_INFO("theta_: %f, theta: %f, err: %f", theta_, this->global_poses[i].theta, err);
#endif

        this->r_cmdvel_[i].publish(v);
    }

    ClusterMetric metric(this->robots, this->groups, 0.3);
    this->metric_v = metric.compute(this->states);
    std_msgs::Float32 clu;
    clu.data = this->metric_v;
    this->mClu_pub.publish(clu);

    std_msgs::Float32 mdist;
    mdist.data = this->meanDist();
    this->mDist_pub.publish(mdist);

    std_msgs::Float32 convel;
    convel.data = this->consensusVel();
    this->consensusVel_pub.publish(convel);
}

int main(int argc, char **argv)
{
    srand(time(0));
    // ROS setups:
    ros::init(argc, argv, "grf_controller", ros::init_options::AnonymousName); // node name

    ros::NodeHandle nh("~"); // create a node handle; need to pass this to the class constructor

    ROS_INFO("[Main] Instantiating an object of type Controller");
    Controller control(&nh);

    ros::Rate rate(30);
    uint64_t iterations = 0;
    while (ros::ok())
    {
        control.update(0);
#ifdef VISUALIZATION
        control.draw(iterations);
#endif
        iterations++;
        ros::spinOnce();
        rate.sleep();
    }

    // long iterations = 0;
    // bool cvok = true;
    // int porcente = 0;
    // long max_it = 20000;
    // do{
    //     auto start = std::chrono::high_resolution_clock::now();
    //     control.update(iterations);
    //     #ifdef VISUALIZATION
    //         cvok = control.draw(iterations);
    //     #endif
    //     if (!(iterations % (int)(max_it * 0.1))){
    //         // auto stop = std::chrono::high_resolution_clock::now();
    //         // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //         // ROS_INFO("Update %f ms", (double)duration.count()/1000.0);
    //         ROS_INFO("Processing: %d", porcente*10);
    //         porcente += 1;
    //     }
    //     iterations += 1;
    // }while(ros::ok() && (iterations < max_it) && cvok);
    // ROS_INFO("Processing: %d", porcente*10);
    // #ifdef VISUALIZATION
    //     cv::waitKey(0);
    // #endif

    // ros::spin();

    return 0;
}