#ifndef MP_CONTROLLER_NODE_H
#define MP_CONTROLLER_NODE_H

#include <sstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_srvs/Empty.h>

using namespace std;
enum TrackState {   
    StopState=0,
    RotateState=1, 
    MoveState=2
};

class MPControllerNode
{
public:

    MPControllerNode(ros::NodeHandle &nh);
    void track();
    void track0();
    void setTrajectory();

private:
    void odomCallback(const nav_msgs::Odometry odom_msg);
    void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);

    void generateTrajectory();
    void turn2Target(double targetOrientation);
    void move2Target(cv::Point2d targetPosition);
    void move2TargetWithmaxVelocity(cv::Point2d targetPosition);

    ros::NodeHandle nh_;

    ros::Subscriber rviz_sub;
    ros::Subscriber odom_sub;

    ros::Publisher target_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher path_pub;
    ros::Publisher cmd_vel_pub;

    nav_msgs::Path traj_msg;
    nav_msgs::Path path_msg;
    geometry_msgs::Twist tw_msg;
    visualization_msgs::Marker target_msg;
    string header_frame_id;

    double odom_x,odom_y;
    double odom_yaw;    //odometry orientation (yaw)
    double odom_v, odom_w;    //odometry linear and angular speeds

    //Global variables for MP
    double maxAngularRate;
    double maxVelocity;
    
    int horizon_n;
    double control_period;//It Should be consistent with rate of ros
    double max_w;
    double min_w;
    double max_v;
    double min_v;

    bool forcedToStop;
    double angleTolerance;
    double distanceTolerance;

    bool hasTrajectory;
    vector<cv::Point2d> traj_pts;

    vector<cv::Point3d> mv_trajectory;
    TrackState me_trackState;
    int mn_targetID;
    cv::Point2d moveOrgin;
    bool isTrackStarted;

    cv::Point2d target_prev;
    cv::Point2d target_pt;
    cv::Point2d path_angle;
    
};
#endif MP_CONTROLLER_NODE_H


