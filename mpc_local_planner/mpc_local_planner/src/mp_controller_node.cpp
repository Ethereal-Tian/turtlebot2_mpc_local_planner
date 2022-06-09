#include <ros/ros.h>

#include <Eigen/Core>

#include <mpc_local_planner/controller.h>
#include <mpc_local_planner/mpc_local_planner_ros.h>
#include <mpc_local_planner/utils/publisher.h>
#include <teb_local_planner/obstacles.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <memory>

#include "mp_controller_node.h"

namespace mpc = mpc_local_planner;


double computeGradient(cv::Point2d &pt1, cv::Point2d &pt2) {
    //direction vector pt1->pt2
    double deltaX = pt2.x - pt1.x;
    double deltaY = pt2.y - pt1.y;
    double grad = atan2(deltaY,deltaX);
    return grad;
}

MPControllerNode::MPControllerNode(ros::NodeHandle &nh){
    nh_=nh;

    odom_sub = nh_.subscribe("/odom", 10, &MPControllerNode::odomCallback, this);
    rviz_sub = nh.subscribe("/clicked_point", 100 ,&MPControllerNode::rvizCallBack,this);

    target_pub = nh.advertise<visualization_msgs::Marker>("target_point", 10);
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    trajectory_pub = nh_.advertise<nav_msgs::Path>("/cmd_trajectory", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("/robot_path", 1);

    control_period = 0.1; 
    horizon_n = 5; 

    angleTolerance=0.01;
    distanceTolerance=0.001;

    maxVelocity = 2;
    maxAngularRate = M_PI;
    max_w = maxAngularRate; min_w = -maxAngularRate; 
    max_v = maxVelocity; min_v = -maxVelocity; 

    forcedToStop=true;

    mv_trajectory.clear();

    isTrackStarted=false;
    moveOrgin=cv::Point2d(0,0);

    hasTrajectory=false;
    mn_targetID=1;
    me_trackState=RotateState;

    header_frame_id = "map";
//  header_frame_id = "odom";

    traj_msg.header.frame_id = header_frame_id; 
    traj_msg.header.stamp = ros::Time::now();
        
    path_msg.header.frame_id = header_frame_id;
    path_msg.header.stamp = ros::Time::now();

    target_msg.header.frame_id = header_frame_id;
    target_msg.header.stamp=ros::Time(0);
    target_msg.type = target_msg.POINTS;
    target_msg.action =target_msg.ADD;
    target_msg.pose.orientation.w =1.0;
    target_msg.scale.x=0.3; 
    target_msg.scale.y=0.3; 
    target_msg.color.r = 255.0/255.0;
    target_msg.color.g = 0.0/255.0;
    target_msg.color.b = 0.0/255.0;
    target_msg.color.a=1.0;
    target_msg.lifetime = ros::Duration();
}

void MPControllerNode::generateTrajectory(){
    mv_trajectory.clear();
    mv_trajectory.push_back(cv::Point3d(0,0,0));//Orgin

    if(0){// y-line
        mv_trajectory.push_back(cv::Point3d(0,0,0));//Orgin
        mv_trajectory.push_back(cv::Point3d(0,5,0));
    }

    if(1){// rectangle
        double len_tmp=1;
        // mv_trajectory.push_back(cv::Point3d(0,0,0));//Orgin
        mv_trajectory.push_back(cv::Point3d(len_tmp,0,0));
        mv_trajectory.push_back(cv::Point3d(len_tmp,len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(0,len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(0,0,0));
    }

    if(0){// pentagon 
        double len_tmp=5;
        double sin_36=sin(36.0/180*M_PI);
        double cos_36=cos(36.0/180*M_PI);
        double sin_72=sin(72.0/180*M_PI);
        double cos_72=cos(72.0/180*M_PI);
        mv_trajectory.push_back(cv::Point3d(0,0,0));//Orgin
        mv_trajectory.push_back(cv::Point3d(cos_36*len_tmp,sin_36*len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(2*cos_36*cos_72*len_tmp,2*cos_36*sin_72*len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(-2*cos_36*cos_72*len_tmp,2*cos_36*sin_72*len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(-cos_36*len_tmp,sin_36*len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(0,0,0));//Orgin
    }

    if(0){// hexagon 
        double len_tmp=5;
        double sin_60=sin(M_PI/3);
        mv_trajectory.push_back(cv::Point3d(0,0,0));//Orgin
        mv_trajectory.push_back(cv::Point3d(0,len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(sin_60*len_tmp,1.5*len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(2*sin_60*len_tmp,len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(2*sin_60*len_tmp,0,0));
        mv_trajectory.push_back(cv::Point3d(sin_60*len_tmp,-0.5*len_tmp,0));
        mv_trajectory.push_back(cv::Point3d(0,0,0));//Orgin
    }

    if(0){// pentagram 
        double len_tmp=5;
        double sin_36=sin(36.0/180*M_PI);
        double cos_36=cos(36.0/180*M_PI);
        double sin_72=sin(72.0/180*M_PI);
        double cos_72=cos(72.0/180*M_PI);
        mv_trajectory.push_back(cv::Point3d(0,0,0));//0
        mv_trajectory.push_back(cv::Point3d(2*cos_36*cos_72*len_tmp,2*cos_36*sin_72*len_tmp,0));//2
        mv_trajectory.push_back(cv::Point3d(-cos_36*len_tmp,sin_36*len_tmp,0));//4
        mv_trajectory.push_back(cv::Point3d(cos_36*len_tmp,sin_36*len_tmp,0));//1
        mv_trajectory.push_back(cv::Point3d(-2*cos_36*cos_72*len_tmp,2*cos_36*sin_72*len_tmp,0));//3
        mv_trajectory.push_back(cv::Point3d(0,0,0));//0
    }
}

void MPControllerNode::setTrajectory(){
    if (!hasTrajectory){
        hasTrajectory=true;
        generateTrajectory();

        for(int i=0;i<mv_trajectory.size();i++){
            cout<<i<<" :"<<mv_trajectory[i]<<endl;
            geometry_msgs::PoseStamped pose_tmp;
            pose_tmp.pose.position.x = mv_trajectory[i].x;
            pose_tmp.pose.position.y = mv_trajectory[i].y;
            traj_msg.poses.push_back(pose_tmp);
        }
    }
}


void MPControllerNode::rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 
    geometry_msgs::Point point_tmp;  
    point_tmp.x = msg->point.x;
    point_tmp.y = msg->point.y;
    point_tmp.z = msg->point.z;
    target_msg.points.push_back(point_tmp);
    target_pub.publish(target_msg);

    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.pose.position.x = msg->point.x;
    pose_tmp.pose.position.y = msg->point.y;
    traj_msg.poses.push_back(pose_tmp);

    cout<<"Insert Target "<<cv::Point3d(msg->point.x,msg->point.y,0)<<endl;

    mv_trajectory.push_back(cv::Point3d(msg->point.x,msg->point.y,0));
}

void MPControllerNode::odomCallback(const nav_msgs::Odometry odom_msg) {

    odom_x=odom_msg.pose.pose.position.x;
    odom_y=odom_msg.pose.pose.position.y;
//  tf::Point odom_pos;    //odometry position (x, y, z)
//  tf::pointMsgToTF(odom_msg.pose.pose.position, odom_pos);
//  odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);
//  odom_yaw = acos(odom_msg.pose.pose.orientation.w)*2;
    odom_yaw = atan(odom_msg.pose.pose.orientation.z/odom_msg.pose.pose.orientation.w)*2;
    if(false){
            std::cout<<cv::Point2d(odom_msg.pose.pose.orientation.w,odom_msg.pose.pose.orientation.z)
             <<"@ tf="<<tf::getYaw(odom_msg.pose.pose.orientation)<<" "
             <<" tan="<<2*atan(odom_msg.pose.pose.orientation.z/odom_msg.pose.pose.orientation.w)<<" "
             <<" cos="<<2*acos(odom_msg.pose.pose.orientation.w)<<std::endl;
    }
    //update observed linear and angular speeds (real speeds published from simulation)
    odom_v = odom_msg.twist.twist.linear.x;
    odom_w = odom_msg.twist.twist.angular.z;
    //display on terminal screen
    //ROS_INFO("Position: (%f, %f); Yaw: %f", odom_x, odom_y, odom_yaw);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(odom_x, odom_y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, odom_yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

void MPControllerNode::turn2Target(double targetOrientation){
    {
        std::vector<teb_local_planner::PoseSE2> via_points;

        teb_local_planner::PoseSE2 x0(odom_x,odom_y,odom_yaw);
        teb_local_planner::PoseSE2 xf(odom_x,odom_y,targetOrientation);

        double scale_ctrl = horizon_n * control_period * maxAngularRate / abs(targetOrientation - odom_yaw);
        if(scale_ctrl < 1.0){
            double yaw_f = odom_yaw + scale_ctrl * (targetOrientation - odom_yaw);
            xf = teb_local_planner::PoseSE2(odom_x,odom_y,yaw_f);
        }

        for(int i = 0;i<=horizon_n;i++){
            double yaw_i =  odom_yaw + (targetOrientation - odom_yaw)* i / horizon_n ;
            via_points.emplace_back(odom_x,odom_y,yaw_i);
        }

        mpc_local_planner::Controller controller;
        teb_local_planner::ObstContainer obstacles;
        teb_local_planner::RobotFootprintModelPtr robot_model = mpc_local_planner::MpcLocalPlannerROS::getRobotFootprintFromParamServer(nh_);
        
        if (!controller.configure(nh_, obstacles, robot_model, via_points))
        {
            ROS_ERROR("Controller configuration failed.");
            return;
        }

        corbo::TimeSeries::Ptr x_seq = std::make_shared<corbo::TimeSeries>();
        corbo::TimeSeries::Ptr u_seq = std::make_shared<corbo::TimeSeries>();

        bool success = false;
        success = controller.step(x0, xf, tw_msg, control_period, ros::Time::now(), u_seq, x_seq);
        for (int i = 0; i < x_seq->getTimeDimension(); ++i)
        {
            double xxi = x_seq->getValuesMap(i)[0];
            double yyi = x_seq->getValuesMap(i)[1];
            double thetai = x_seq->getValuesMap(i)[2];
            // std::cout<<cv::Point3d(xxi,yyi,thetai)<<" ";
        }
        for (int i = 0; i < u_seq->getTimeDimension(); ++i)
        {
            double xxi = u_seq->getValuesMap(i)[0];
            double yyi = u_seq->getValuesMap(i)[1];
            double thetai = u_seq->getValuesMap(i)[2];
            // std::cout<<cv::Point3d(xxi,yyi,thetai)<<" ";
        }
        std::cout<<"R tw "<<tw_msg.linear.x<<"    "<<tw_msg.angular.z<<std::endl;
        std::cout<<"R st  "<<odom_yaw<<"    "<<targetOrientation<<std::endl;
    }

    cmd_vel_pub.publish(tw_msg);

    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.pose.position.x=odom_x; 
    pose_tmp.pose.position.y=odom_y; 
    path_msg.poses.push_back(pose_tmp);
    path_pub.publish(path_msg);
    trajectory_pub.publish(traj_msg);

    if(abs(targetOrientation-odom_yaw)<angleTolerance){
        cout<<"targetOrientation "<<targetOrientation<<" odom_yaw"<<odom_yaw<<endl;
        me_trackState=MoveState;
        moveOrgin=cv::Point2d(odom_x,odom_y);
        if(true){
            cout<<endl<<"========================================================================"<<endl;
            cout<<"Move TO THE "<<mn_targetID<<"th TARGET !"
                <<cv::Point3d(odom_x,odom_y,odom_yaw)<<"==>"<<mv_trajectory[mn_targetID]<<endl;
            cout<<"========================================================================"<<endl<<endl;
        }
    }
}

void MPControllerNode::move2Target(cv::Point2d targetPosition){
    //Error calculation
    cv::Point2d pose_curr;
    pose_curr.x = odom_x;
    pose_curr.y = odom_y;

    double  dx = targetPosition.x - odom_x;
    double  dy = targetPosition.y - odom_y;

    {
        double targetYaw = computeGradient(pose_curr, targetPosition);
        

        std::vector<teb_local_planner::PoseSE2> via_points;

        teb_local_planner::PoseSE2 x0(odom_x,odom_y,odom_yaw);
        teb_local_planner::PoseSE2 xf(targetPosition.x,targetPosition.y,targetYaw);

        double scale_ctrl = horizon_n * control_period * maxVelocity / sqrt(dx * dx + dy * dy);
        if(scale_ctrl < 1.0){
            double x_f = odom_x + scale_ctrl * (targetPosition.x - odom_x);
            double y_f = odom_y + scale_ctrl * (targetPosition.y - odom_y);
            xf = teb_local_planner::PoseSE2(x_f, y_f, targetYaw);
        }

        for(int i = 0;i<=horizon_n;i++){
            double xi = odom_x + (targetPosition.x - odom_x)* i / horizon_n ;
            double yi = odom_y + (targetPosition.y - odom_y) * i / horizon_n;
            via_points.emplace_back(xi,yi,targetYaw);
        }

        mpc_local_planner::Controller controller;
        teb_local_planner::ObstContainer obstacles;
        teb_local_planner::RobotFootprintModelPtr robot_model = mpc_local_planner::MpcLocalPlannerROS::getRobotFootprintFromParamServer(nh_);
        
        if (!controller.configure(nh_, obstacles, robot_model, via_points))
        {
            ROS_ERROR("Controller configuration failed.");
            return;
        }

        corbo::TimeSeries::Ptr x_seq = std::make_shared<corbo::TimeSeries>();
        corbo::TimeSeries::Ptr u_seq = std::make_shared<corbo::TimeSeries>();

        bool success = false;
        success = controller.step(x0, xf, tw_msg, control_period, ros::Time::now(), u_seq, x_seq);


        for (int i = 0; i < x_seq->getTimeDimension(); ++i)
        {
            double xxi = x_seq->getValuesMap(i)[0];
            double yyi = x_seq->getValuesMap(i)[1];
            double thetai = x_seq->getValuesMap(i)[2];
            // std::cout<<cv::Point3d(xxi,yyi,thetai)<<" ";
        }
        for (int i = 0; i < u_seq->getTimeDimension(); ++i)
        {
            double xxi = u_seq->getValuesMap(i)[0];
            double yyi = u_seq->getValuesMap(i)[1];
            double thetai = u_seq->getValuesMap(i)[2];
            std::cout<<cv::Point3d(xxi,yyi,thetai)<<" ";
        }

        std::cout<<"T tw "<<tw_msg.linear.x<<"    "<<tw_msg.angular.z<<std::endl;
        std::cout<<"T st  "<<cv::Point3d(odom_x,odom_y,odom_yaw)<<"    "<<cv::Point3d(targetPosition.x,targetPosition.y,targetYaw)<<std::endl;
    }

    if(!forcedToStop){
        double distToTarget=sqrt((targetPosition.x-odom_x)*(targetPosition.x-odom_x)+(targetPosition.y-odom_y)*(targetPosition.y-odom_y));
        if(distToTarget<distanceTolerance){
            me_trackState=RotateState;
            mn_targetID++;
            moveOrgin=cv::Point2d(odom_x,odom_y);
            if(true){
                cout<<endl<<"========================================================================"<<endl;
                cout<<"ROTATE TO THE "<<mn_targetID<<"th TARGET !"
                    <<cv::Point3d(odom_x,odom_y,odom_yaw)<<"==>"<<mv_trajectory[mn_targetID]<<endl; 
                cout<<"========================================================================"<<endl<<endl;
            }
        }
    }else{
        if(tw_msg.linear.x<=0){
            tw_msg.linear.x = 0;
            me_trackState=RotateState;
            mn_targetID++;
            moveOrgin=cv::Point2d(odom_x,odom_y);
            if(true){
                cout<<endl<<"========================================================================"<<endl;
                cout<<"ROTATE TO THE "<<mn_targetID<<"th TARGET !"
                    <<cv::Point3d(odom_x,odom_y,odom_yaw)<<"==>"<<mv_trajectory[mn_targetID]<<endl; 
                cout<<"========================================================================"<<endl<<endl;
            }
        }
    }

    cmd_vel_pub.publish(tw_msg);

    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.pose.position.x=odom_x; 
    pose_tmp.pose.position.y=odom_y; 
    path_msg.poses.push_back(pose_tmp);
    path_pub.publish(path_msg);
    trajectory_pub.publish(traj_msg);
}

void MPControllerNode::track(){

    if(mn_targetID>=mv_trajectory.size()){
        cout<<"mv_trajectory.size() = "<<mv_trajectory.size()<<endl;

    //   me_trackState=StopState;
        return;
    }

    if (!isTrackStarted){
        isTrackStarted=true;
        moveOrgin=cv::Point2d(odom_x,odom_y);
        if(true){
            cout<<endl<<"========================================================================"<<endl;
            cout<<"ROTATE TO THE "<<mn_targetID<<"th TARGET !"
                <<cv::Point3d(odom_x,odom_y,odom_yaw)<<"==>"<<mv_trajectory[mn_targetID]<<endl; 
            cout<<"========================================================================"<<endl<<endl;
        }
    }
    // cout<<"mn_targetID "<<mn_targetID<<", "<<"me_trackState"<<me_trackState<<endl;

    cv::Point2d targetPoint=cv::Point2d(mv_trajectory[mn_targetID].x,mv_trajectory[mn_targetID].y);
    if(me_trackState==RotateState){
        double targetAngle = computeGradient(moveOrgin, targetPoint);
        turn2Target(targetAngle);
    }else if(me_trackState==MoveState){
        move2Target(targetPoint);
    }
    return;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "MP_Controller_Node");
    ros::NodeHandle nh("~");

    std::cout<<"MP_Controller_Node"<<std::endl;
    MPControllerNode* pmpNode = new MPControllerNode(nh);
    pmpNode->setTrajectory();

    ros::Rate loop_rate(10); // ros spins 10 frames per second
    while (ros::ok()) {
        pmpNode->track();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/*

class TestMpcOptimNode
{
 public:
    TestMpcOptimNode() = default;

    void start(ros::NodeHandle& nh,teb_local_planner::PoseSE2 &x0,teb_local_planner::PoseSE2 &xf);

 protected:
    void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
    void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
    void GenerateViaPoint();

    teb_local_planner::ObstContainer _obstacles;
    std::vector<teb_local_planner::PoseSE2> _via_points;
    std::vector<teb_local_planner::PoseSE2> _all_track;
};

void TestMpcOptimNode::start(ros::NodeHandle& nh,teb_local_planner::PoseSE2 &x0,teb_local_planner::PoseSE2 &xf)
{
    std::string map_frame = "map";

    // setup callback for clicked points (in rviz) that are considered as via-points
    ros::Subscriber clicked_points_sub = nh.subscribe("/clicked_point", 5, &TestMpcOptimNode::CB_clicked_points, this);

    // setup callback for via-points (callback overwrites previously set via-points)
    ros::Subscriber via_points_sub = nh.subscribe("via_points", 1, &TestMpcOptimNode::CB_via_points, this);

    // configure via-points 
    GenerateViaPoint();

    // Setup robot shape model
    teb_local_planner::RobotFootprintModelPtr robot_model = mpc_local_planner::MpcLocalPlannerROS::getRobotFootprintFromParamServer(nh);

    mpc_local_planner::Controller controller;
    if (!controller.configure(nh, _obstacles, robot_model, _via_points))
    {
        ROS_ERROR("Controller configuration failed.");
        return;
    }

    mpc::Publisher publisher(nh, controller.getRobotDynamics(), map_frame);



    corbo::TimeSeries::Ptr x_seq = std::make_shared<corbo::TimeSeries>();
    corbo::TimeSeries::Ptr u_seq = std::make_shared<corbo::TimeSeries>();

    geometry_msgs::Twist vel;

    bool success = false;

    ros::Rate rate(20);
    while (ros::ok())
    {
        success = controller.step(x0, xf, vel, rate.expectedCycleTime().toSec(), ros::Time::now(), u_seq, x_seq);

        if (success)
            publisher.publishLocalPlan(*x_seq);
        else
            ROS_ERROR("OCP solving failed.");

        publisher.publishRobotFootprintModel(x0, *robot_model);
        publisher.publishViaPoints(_via_points);
        ros::spinOnce();
        rate.sleep();
    }
}

void TestMpcOptimNode::CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
    // we assume for simplicity that the fixed frame is already the map/planning frame
    // consider clicked points as via-points
    _via_points.emplace_back(point_msg->point.x, point_msg->point.y, 0.0);
    ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
}

void TestMpcOptimNode::CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
    ROS_INFO_ONCE("Via-points received. This message is printed once.");
    _via_points.clear();
    for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
    {
        _via_points.emplace_back(pose.pose.position.x, pose.pose.position.y, 0);
    }
}

void TestMpcOptimNode::GenerateViaPoint()
{
    ROS_INFO_ONCE("Generate via-points.");
    _via_points.clear();
    // _via_points.emplace_back(0,0, 0);
    // _via_points.emplace_back(1,1, 0);
    // _via_points.emplace_back(2,1, 0);
    // _via_points.emplace_back(3,1, 0);
    // _via_points.emplace_back(4,1, 0);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_optim_node");
    ros::NodeHandle n("~");

    // teb_local_planner::PoseSE2 x0(0, 0, 0);
    // teb_local_planner::PoseSE2 xf(0, 0, M_PI);

    teb_local_planner::PoseSE2 x0(0, 0, M_PI/4.0);
    teb_local_planner::PoseSE2 xf(2, 2, M_PI/4.0);

    TestMpcOptimNode mpc_test;
    mpc_test.start(n,x0,xf);

    return 0;
}
*/