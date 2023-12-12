#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose2D.h>
//
#include <math.h>
#include <angles/angles.h>
//
//
class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
        gazebo_msgs::ModelStates getHumanPoseFromGazebo(gazebo_msgs::ModelStates msg);
        gazebo_msgs::ModelStates humanDetectionFromGazebo(geometry_msgs::Pose2D cur_rpos, sensor_msgs::PointCloud cloud_laser,gazebo_msgs::ModelStates msg);

     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;
        sensor_msgs::PointCloud cloud_laser_;

        ros::Publisher point_cloud_publisher_;
        ros::Publisher detected_human_pub_;
        ros::Subscriber scan_sub_;
        ros::Subscriber model_states_sub_;
        geometry_msgs::Pose2D cur_rpos_;
        gazebo_msgs::ModelStates human_states_;

        ros::Publisher laserpoint_pub_;
};
//
visualization_msgs::Marker createMarkerPoint(float_t x, float_t y, float_t z, int id_point, int id_zone)
{
    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "laser_pointcloud_" << id_zone;
    //
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = id_point; // this number should be different
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5); // the marker will be deleted after 0.5s
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.02;

    marker.color.a = 0.9;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 1;
    return marker;
}
//
double convertQuaternionToAngle(geometry_msgs::Quaternion quat_in){
    tf::Quaternion quat;
    double dummy;
    double cur_ptheta;
    tf::quaternionMsgToTF(quat_in, quat);
    tf::Matrix3x3 mat(quat);
    mat.getRPY(dummy, dummy, cur_ptheta);

    return cur_ptheta;
}
//
My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Filter::scanCallback, this);
        model_states_sub_ = node_.subscribe("/gazebo/model_states", 100, &My_Filter::modelStatesCallback, this);

        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("/mybot_description/laser_pointcloud", 10, false);
        laserpoint_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/mybot_description/laser_pointcloud_vis",0);
        detected_human_pub_ = node_.advertise<gazebo_msgs::ModelStates>("/mybot_description/detected_human",10);
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}
//
void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    ROS_INFO("Laser data: size = %lu; angle_min = %f; angle_max = %f; angle_increment = %f;range_min = %f;range_max = %f",
             scan->ranges.size(),scan->angle_min,scan->angle_max,scan->angle_increment,scan->range_min,scan->range_max);

    if(!tfListener_.waitForTransform(scan->header.frame_id,"/map",scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
            ros::Duration(1.0))){
         return;
    }
    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("/map",*scan, cloud,tfListener_);


    ROS_INFO("Size laser data: %lu; size pointcloud: %lu", scan->ranges.size(), cloud.points.size());
    //point_cloud_publisher_.publish(cloud); // publish to "mybot_approaching_pose"
    cloud_laser_ = cloud;

    /*
    for(int i=0; i<cloud.points.size();i++){
        geometry_msgs::Point32 point = cloud.points[i];
        marker = createMarkerPoint(point.x, point.y, 0.5, i, 0);
        marker_array.markers.push_back(marker);
    }
    laserpoint_pub_.publish(marker_array);
    marker_array.markers.clear();
    */
}
//
//
gazebo_msgs::ModelStates My_Filter::getHumanPoseFromGazebo(gazebo_msgs::ModelStates msg){

    gazebo_msgs::ModelStates human_states;

    for(int i =0; i<msg.name.size(); i++){
        if((msg.name[i] == "p0")||(msg.name[i] == "p1")||(msg.name[i] == "p2")||(msg.name[i] == "p3")||(msg.name[i] == "p4")||(msg.name[i] == "p5")
          ||(msg.name[i] == "p6")||(msg.name[i] == "p7")||(msg.name[i] == "p8")||(msg.name[i] == "p9")||(msg.name[i] == "p10")||(msg.name[i] == "p11")
          ||(msg.name[i] == "p12")||(msg.name[i] == "p13")||(msg.name[i] == "p14")||(msg.name[i] == "p15")||(msg.name[i] == "p16")||(msg.name[i] == "p17")||(msg.name[i] == "p18"))
        {
            human_states.name.push_back(msg.name[i]);
            human_states.pose.push_back(msg.pose[i]);
            human_states.twist.push_back(msg.twist[i]);
        }
    }
    ROS_INFO("Number of Human: %lu", human_states.name.size());

    return human_states;
}
//
gazebo_msgs::ModelStates My_Filter::humanDetectionFromGazebo(geometry_msgs::Pose2D cur_rpos, sensor_msgs::PointCloud cloud_laser,gazebo_msgs::ModelStates msg){


    gazebo_msgs::ModelStates human_states;
    double max_dis = 8.0; double max_dxy = 0.4;
    for(int i=0; i<msg.name.size(); i++){
        double dx = msg.pose[i].position.x - cur_rpos.x;
        double dy = msg.pose[i].position.y - cur_rpos.y;
        double d = sqrt(dx*dx + dy*dy);
        double ang = atan2(dy,dx);
         if(d<max_dis){// maximum distance of laser data
            double diff = angles::shortest_angular_distance(cur_rpos.theta, ang);
            //if(fabs(diff)<M_PI/2){
            if(fabs(diff)<2.09){ // 240^o
                bool flag = false;
                for (unsigned int j=0; j<cloud_laser.points.size();j++){
                    geometry_msgs::Point32 point = cloud_laser.points[j];
                    double dx1 = point.x - cur_rpos.x;
                    double dy1 = point.y - cur_rpos.y;
                    double d1 = sqrt(dx1*dx1 + dy1*dy1);
                    double ang1 = atan2(dy1,dx1);
                    double diff1 = angles::shortest_angular_distance(ang,ang1);
                    if(fabs(diff1)<M_PI/18){
                        if(fabs(d-d1)<max_dxy){
                            flag = true;
                        }
                    }
                }
                if(flag){
                    human_states.name.push_back(msg.name[i]);
                    human_states.pose.push_back(msg.pose[i]);
                    human_states.twist.push_back(msg.twist[i]);
                }
            }
        }
    }
    ROS_INFO("Number of Human after filtering by distance and angle: %lu", human_states.name.size());

    return human_states;
}
//
void My_Filter::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg){

    gazebo_msgs::ModelStates human_states, human_states1;
    gazebo_msgs::ModelState human_state, robot_state;
    double cur_rx, cur_ry, cur_rtheta;
    for(int i =0; i<msg->name.size(); i++){
        if(msg->name[i] == "robot_model"){
            robot_state.model_name = msg->name[i]; robot_state.pose = msg->pose[i]; robot_state.twist = msg->twist[i];
            cur_rtheta = convertQuaternionToAngle(msg->pose[i].orientation);
            cur_rx = msg->pose[i].position.x; cur_ry = msg->pose[i].position.y;
            cur_rpos_.x = cur_rx; cur_rpos_.y = cur_ry; cur_rpos_.theta = cur_rtheta;
            //ROS_INFO("Robot pose (x,y,theta) = (%f, %f, %f)", cur_rx, cur_ry, cur_rtheta);
            break;
        }
    }
    //
    human_states = My_Filter::getHumanPoseFromGazebo(*msg);
    human_states1 = humanDetectionFromGazebo(cur_rpos_, cloud_laser_, human_states);
    detected_human_pub_.publish(human_states1);
}
//
//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Convert_laser_pointcloud_and_detect_human");

    My_Filter filter;

    ros::spin();

    return 0;
}
