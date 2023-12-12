#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
//
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
//
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
//
#include "circle_fitting/mystuff.h"
#include "circle_fitting/data.h"
#include "circle_fitting/circle.h"
#include "circle_fitting/Utilities.cpp"
#include "circle_fitting/CircleFitByTaubin.cpp"
#include "circle_fitting/CircleFitByPratt.cpp"
#include "circle_fitting/CircleFitByKasa.cpp"
#include "circle_fitting/CircleFitByHyper.cpp"
#include <time.h>
//
#include <math.h>
#include <angles/angles.h>
// people_msgs
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
//
#include <mybot_description/GetApproachingPose.h>
//
#include <iostream>
//
ros::Publisher human_pose_pub, approaching_pub, app_pose_vis_pub, app_pose_pub;
ros::Publisher collision_ind_pub, orientation_ind_pub, interaction_ind_pub, motion_ind_pub;
ros::Publisher detected_human_vis_pub;
ros::ServiceServer app_pose_service;

nav_msgs::OccupancyGrid local_costmap_;
nav_msgs::Odometry robot_odom_;
geometry_msgs::Pose2D cur_rpos_;
geometry_msgs::Pose2D app_pose_;
sensor_msgs::PointCloud cloud_laser_;
std_msgs::Int8 goal_index_;
FILE * resultFile  = fopen("real_robot_results.csv","w");
gazebo_msgs::ModelStates human_states_, detected_human_states_;
std::vector <geometry_msgs::Pose2D> vhgroup_;
double moving_x = 5.5; // for moving people
bool start_moving_flag = false;
//
double cur_hx, cur_hy, cur_htheta;
// Select the potential approaching areas
std::vector<std::vector <geometry_msgs::Point> > selectPotentialApproachingAreas2D(std::vector <geometry_msgs::Point> approaching_areas, unsigned char minpoints){
    geometry_msgs::Point p1, p2;
    std::vector <geometry_msgs::Point> tmp1, tmp2;
    std::vector<std::vector <geometry_msgs::Point> > potential_ap_2d;
    unsigned int index=1;
    unsigned int npoint= 1;
    // separate the area
    for(int i=1;i<approaching_areas.size();i++){
        p1 = approaching_areas[i-1];
        p2 = approaching_areas[i];
        if((p2.z-p1.z)>1){
        //if(((p2.z-p1.z)>2)&&((p2.z-p1.z)<200-2)){
            if(npoint>minpoints){
                // add the center point to the end
                tmp1.push_back(tmp1[int(npoint/2)]);
                potential_ap_2d.push_back(tmp1);
            }
            npoint = 1;
            tmp1.clear();
            index++;
            p2.z = index;
            tmp1.push_back(p2);
        }else{
            p2.z = index;
            tmp1.push_back(p2);
            npoint++;
        }
    }
    // Add the last area
    if(npoint>minpoints){
        // add the center point to the end
        tmp1.push_back(tmp1[int(npoint/2)]);
        potential_ap_2d.push_back(tmp1);
    }
    ROS_INFO("Size of potential_ap_2d with one area: %lu", potential_ap_2d.size());
    ROS_INFO("Number of approaching areas: %d", index);
    return potential_ap_2d;
}

//
bool checkObstacles(nav_msgs::OccupancyGrid local_costmap, double xc, double yc)
{
    bool flag=true;
    double step = local_costmap.info.resolution;
    double x_localmap = local_costmap.info.origin.position.x;
    double y_localmap = local_costmap.info.origin.position.y;
    unsigned int size_localmap = local_costmap.data.size();

    double col_localmap = local_costmap.info.width;
    double row_localmap = local_costmap.info.height;
    //ROS_INFO("Local map info:%f,%f,%f,%d,%f,%f",step,x_localmap,y_localmap,size_localmap,col_localmap,row_localmap);

    unsigned int x = (int)((xc-x_localmap)/step);
    unsigned int y = (int)((yc-y_localmap)/step);

    if((x>=col_localmap)||(y>=col_localmap))
    {
        flag = true;
    }else{
        unsigned int index = x + y*col_localmap;
        //ROS_INFO("x = %d, y = %d, index = %d, data = %d", x, y, index, local_costmap.data[index]);
        if(local_costmap.data[index]>0){
            flag = true;
            //ROS_INFO("x = %d, y = %d, index = %d, data = %d", x, y, index, local_costmap.data[index]);
        }
        else{
            flag = false;
            //ROS_INFO("x = %d, y = %d, index = %d, data = %d", x, y, index, local_costmap.data[index]);
        }
    }
    return flag;
}
//
std::vector <geometry_msgs::Point> estimateApproachingAreas(double xc,double yc,double rc, double r_step, unsigned int npoints,std_msgs::Int8 goal_index){

    int8_t ind;
    ind = goal_index.data;
    geometry_msgs::Point approaching_point;
    std::vector <geometry_msgs::Point> approaching_areas;
    double xa=0, ya=0;
    for(int i=0;i<npoints;i++){
        if((ind==2)||(ind==16)||(ind==19)||(ind==22)){ // use this because of the approaching filter not very good at this time :)
            if(ind==19){//[-3pi/4-pi/4]
                int j = i-3*(npoints/4);
                xa = xc + (r_step+rc)*cos(j*M_PIl/(npoints));
                ya = yc + (r_step+rc)*sin(j*M_PIl/(npoints));
            }else{//[-pi-pi]
                int j = i-npoints/2;
                xa = xc + (r_step+rc)*cos(j*M_PIl/(npoints));
                ya = yc + (r_step+rc)*sin(j*M_PIl/(npoints));
            }
        }else{//[0-2pi]
            xa = xc + (r_step+rc)*cos(i*M_PIl/(0.5*npoints));
            ya = yc + (r_step+rc)*sin(i*M_PIl/(0.5*npoints));
        }

        bool flag = checkObstacles(local_costmap_, xa, ya);
        if(!flag){
            approaching_point.x = xa; approaching_point.y = ya; approaching_point.z = i;
            approaching_areas.push_back(approaching_point);
            //ROS_INFO("flag = %d, id_point=%d",flag, id_point);
        }
    }
    return approaching_areas;
}
//
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
std::vector <geometry_msgs::Point> subFilterApproachingAreasByFiewOfView(std::vector <geometry_msgs::Point> approaching_areas,
                                                                         geometry_msgs::Quaternion quat_in,geometry_msgs::Point hp){

    std::vector <geometry_msgs::Point> approaching_areas_fov;
    //double ang = convertQuaternionToAngle(human_states.pose[i].orientation);
    //geometry_msgs::Point hp = human_states.pose[i].position;
    double ang = convertQuaternionToAngle(quat_in)-M_PI_2;
    for(int j=0;j<approaching_areas.size();j++){
        geometry_msgs::Point point = approaching_areas[j];
        double dx1 = point.x - hp.x;
        double dy1 = point.y - hp.y;
        double ang1 = atan2(dy1,dx1);
        double diff1 = angles::shortest_angular_distance(ang,ang1);
        if(fabs(diff1)<M_PI/2.5){
            approaching_areas_fov.push_back(point);
        }
    }
    return approaching_areas_fov;
}
//
std::vector <geometry_msgs::Point> filterApproachingAreasByFiewOfView(std::vector <geometry_msgs::Point> approaching_areas,
                                                                      gazebo_msgs::ModelStates human_states, std_msgs::Int8 goal_index ){
    int8_t ind;
    ind = goal_index.data;
    std::vector <geometry_msgs::Point> approaching_areas_fov;

    for(int i = 0; i<human_states.name.size(); i++){
        if((ind==0)){
            if(human_states.name[i] == "p0"){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
        }
        if(ind==0){
            if((human_states.name[i] == "p1")||(human_states.name[i] == "p2")){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
        }
        if(ind==0){
            if((human_states.name[i] == "p0")||(human_states.name[i] == "p1")||(human_states.name[i] == "p2")){ // close loop
                //approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas_fov = approaching_areas;// because field of view
            }
        }
    }
    return approaching_areas_fov;
}
//

//
std::vector <geometry_msgs::Point> selectedApproachingArea(std::vector<std::vector <geometry_msgs::Point> > approaching_areas_2d, geometry_msgs::Pose2D cur_rpos){

    geometry_msgs::Point tmp;
    std::vector <geometry_msgs::Point> approaching_areas, selected_app_area;
    double max_dis = 1000000;
    int index = 1000000;
    //if(approaching_areas_2d.size()>1){
        for(int i=0;i<approaching_areas_2d.size();i++){
            approaching_areas = approaching_areas_2d[i];
            tmp = approaching_areas.back();// get the last element
            double dis = sqrt((cur_rpos.x-tmp.x)*(cur_rpos.x-tmp.x)+(cur_rpos.y-tmp.y)*(cur_rpos.y-tmp.y));
            if(dis<max_dis){
                max_dis = dis;
                index = i;
                selected_app_area=approaching_areas;
            }
        }
    //}
        return selected_app_area;
}
//
geometry_msgs::Pose2D calculateApproachingPose(std::vector <geometry_msgs::Point> selected_app_area, geometry_msgs::Pose2D center_area){

    geometry_msgs::Pose2D app_pose;
    geometry_msgs::Point tmp;
    tmp = selected_app_area.back();
    app_pose.x = tmp.x;
    app_pose.y = tmp.y;
    double angle = atan2((center_area.y-app_pose.y),(center_area.x-app_pose.x));
    app_pose.theta = angle;
    ROS_INFO("Approaching pose of robot: x= %f, y= %f, theta = %f", app_pose.x, app_pose.y, app_pose.theta);

    return app_pose;
}
//
//
void approachingPoseVisualize(geometry_msgs::Pose2D app_pose, int n_goal)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    std::stringstream ss;
    ss << "approaching_pose_" << n_goal;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = n_goal; // this number should be different
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    //
    p.x = app_pose.x;
    p.y = app_pose.y;
    p.z = 0;
    marker.points.push_back(p);
    //
    p.x = app_pose.x + 0.5*cos(app_pose.theta);
    p.y = app_pose.y + 0.5*sin(app_pose.theta);
    p.z = 0;
    marker.points.push_back(p);
    app_pose_vis_pub.publish(marker);
    marker.points.clear();

}
//
visualization_msgs::Marker createMarkerPoint(float_t x, float_t y, float_t z, int id_point, int id_zone)
{
    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "approaching_area_" << id_zone;
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
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.02;

    marker.color.a = 0.9;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 1;
    return marker;
}
//
visualization_msgs::Marker createMarkerHumanBody(float_t x, float_t y, float_t z, int id_point, int id_zone){

    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "human_body_" << id_zone;
    //
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = id_point; // this number should be different
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5); // the marker will be deleted after 0.5s
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 2*z;

    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    return marker;
}
//
double collisionIndex(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
    double dx = x-x0, dy = y-y0;
    double h = sqrt(dx*dx+dy*dy);
    double angle = atan2(dy,dx);
    double mx = cos(angle-skew) * h;
    double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * varx),
           f2 = pow(my, 2.0)/(2.0 * vary);
    return A * exp(-(f1 + f2));
}
//
double orientationIndex(double x, double y, double theta, double x0, double y0, double theta0){
    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    double ang_rh = atan2(dy,dx);
    double ang_hr = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_rh,theta0);
    double varphi = angles::shortest_angular_distance(ang_hr,theta);
    double tmp = (2+ cos(beta) + cos(varphi))/4;

    return tmp;
}
//
double interactionIndex(double x, double y, double theta, double x0, double y0, double theta0){

    double tmp = 0.5;

    return tmp;
}
//
double motionIndex(double x, double y, double theta, double x0, double y0, double theta0){

    double tmp = 0.2;

    return tmp;
}
//
std::vector <double> estimateHumanComfortableSafetyIndices(geometry_msgs::Pose2D cur_rpos, gazebo_msgs::ModelStates msg){
    std::vector <double> indices;
    double x0 = cur_rpos.x;
    double y0 = cur_rpos.y;
    double theta0 = cur_rpos.theta;
    double max_collision = 0; double max_interaction = 0;
    double max_orientation = 0; double max_motion = 0;

    for(int i=0; i<msg.name.size(); i++){
        double x = msg.pose[i].position.x;
        double y = msg.pose[i].position.y;
        // calculate the collision index
        double tmp = collisionIndex(x,y,x0,y0,1,0.45,0.45,0);
        if(tmp>max_collision){
            max_collision = tmp;
        }
        // calculate the orientation index
        geometry_msgs::Quaternion quat_in = msg.pose[i].orientation;
        double theta = convertQuaternionToAngle(quat_in)-M_PI_2;
        double tmp1 = orientationIndex(x,y,theta,x0,y0,theta0);
        if(tmp1>max_orientation){
            max_orientation = tmp1;
        }
        // calculate interaction index
        double tmp2 = interactionIndex(x,y,theta,x0,y0,theta0);
        if(tmp2>max_interaction){
            max_interaction = tmp2;
        }
        // calculate interaction index
        double tmp3 = motionIndex(x,y,theta,x0,y0,theta0);
        if(tmp3>max_motion){
            max_motion = tmp3;
        }
    }
    indices.push_back(max_collision);
    indices.push_back(max_orientation);
    indices.push_back(max_interaction);
    indices.push_back(max_motion);

    return indices;
}
//
geometry_msgs::Pose2D selectApproachingHumans(gazebo_msgs::ModelStates human_states, std_msgs::Int8 goal_index){

    geometry_msgs::Pose2D pose;
    std::vector <geometry_msgs::Pose2D> vpose;
    geometry_msgs::Point p0, p1;
    Circle circle;

    int8_t ind;
    geometry_msgs::Pose2D shgroup; //single human
    shgroup.theta = 0;
    ind = goal_index.data;
    int k =0;
    //
    ROS_INFO("Select approaching humans");
    for(int i = 0; i<human_states.name.size(); i++){
            if(human_states.name[i] == "p0"){
                shgroup.x = human_states.pose[i].position.x;
                shgroup.y = human_states.pose[i].position.y;
                shgroup.theta = 1.1;
            }
            if((human_states.name[i] == "p1")||(human_states.name[i] == "p2")){
                if(human_states.name[i] == "p1"){p0=human_states.pose[i].position;}
                if(human_states.name[i] == "p2"){p1=human_states.pose[i].position;}
                k = k+ 1;
                if(k==2){
                    double xg = (p0.x+p1.x)/2; double yg = (p0.y+p1.y)/2;
                    shgroup.x = xg;//
                    shgroup.y = yg;//
                    shgroup.theta = 0.9; //
                }
            }
    }
    return shgroup;
}
/* */
void localCostmapCallback(const nav_msgs::OccupancyGridConstPtr & local_costmap)
{

    local_costmap_ = *local_costmap;

    double step = local_costmap->info.resolution;
    double x_localmap = local_costmap->info.origin.position.x;
    double y_localmap = local_costmap->info.origin.position.y;
    unsigned int value_localmap = local_costmap->data.size();
    double xsize_localmap = local_costmap->info.height;
    double ysize_localmap = local_costmap->info.width;
    //ROS_INFO("Local map info:%f,%f,%f,%d,%f,%f",step,x_localmap,y_localmap,value_localmap,xsize_localmap,ysize_localmap);

    //for(int i =0; i<local_costmap->data.size(); i++)
    //{
    //    ROS_INFO("%d=%d,",i, local_costmap->data[i]);
    //}
}
//
void goalIndexCallback(const std_msgs::Int8::ConstPtr &msg){
    goal_index_ = *msg;
    start_moving_flag = true;

}
//
bool getApproachingPose(mybot_description::GetApproachingPose::Request &req,mybot_description::GetApproachingPose::Response &res){

    res.index = goal_index_;
    res.app_pose = app_pose_;

    return true;
}
// get human group from matlab
void humanGroupFromMatlabCallback(const geometry_msgs::PoseArray::ConstPtr &msg){
    geometry_msgs::Pose2D pose;
    std::vector <geometry_msgs::Pose2D> vpose;
    for(int i=0; i<msg->poses.size(); i++){
        geometry_msgs::Pose p = msg->poses[i];
        pose.x = p.position.x;
        pose.y = p.position.y;
        pose.theta = p.position.z;
        vpose.push_back(pose);
    }
    vhgroup_ = vpose;
}
// from  "mybot_convert_laser_pointcloud"
void detectedHumanCallback(const gazebo_msgs::ModelStates::ConstPtr &msg){

    gazebo_msgs::ModelStates human_states;
    gazebo_msgs::ModelState human_state, robot_state;
    geometry_msgs::Pose2D app_pose, center_area;
    people_msgs::People people;
    people_msgs::Person person;
    visualization_msgs::Marker marker, hmarker;
    visualization_msgs::MarkerArray approaching_marker, hmarker_array;

    ROS_INFO("Detected humans: %lu", msg->name.size());
    detected_human_states_ = *msg;
    human_states = *msg;
    //
    // Human safety and comfort indices
    std_msgs::Float32 collision_index, orientation_index,interaction_index, motion_index;
    std::vector<double> indices= estimateHumanComfortableSafetyIndices(cur_rpos_, human_states);
    collision_index.data = indices[0]; orientation_index.data = indices[1] ; interaction_index.data = indices[2]; motion_index.data = indices[3];
    collision_ind_pub.publish(collision_index);
    orientation_ind_pub.publish(orientation_index);
    interaction_ind_pub.publish(interaction_index);
    motion_ind_pub.publish(motion_index);
    // people
    for(int i =0; i<human_states.name.size(); i++){
        //
        double cur_ptheta;
        cur_ptheta = convertQuaternionToAngle(human_states.pose[i].orientation);

        //ROS_INFO("cur_ptheta_p%d=%f",k,cur_ptheta);
        person.position.x = human_states.pose[i].position.x;
        person.position.y = human_states.pose[i].position.y;
        person.position.z = 0;
        // moving people
        if((human_states.name[i] == "p10")||(human_states.name[i] == "p11")){
            person.velocity.x = 0.8*cos(cur_ptheta);// 0.25
            person.velocity.y = 0.8*sin(cur_ptheta);
            person.velocity.z = 0;
        }
        else{ // stationary people
            person.velocity.x = 0.12*cos(cur_ptheta);
            person.velocity.y = 0.12*sin(cur_ptheta);
            person.velocity.z = 0;
        }
        // sitting people
        //if((human_states.name[i] == "p8")||(human_states.name[i] == "p9")||(human_states.name[i] == "p14")){
        if((human_states.name[i] == "p8")||(human_states.name[i] == "p14")){
            person.reliability = 0;
        }
        else{
            person.reliability = 1;
        }
        person.name = "people_info";
        people.people.push_back(person);
        hmarker = createMarkerHumanBody(person.position.x, person.position.y, 0.6, i, 9999);
        hmarker_array.markers.push_back(hmarker);
        // object poeple
        int object_people_flag = true;
        if(object_people_flag){
            if((human_states.name[i] == "p8")||(human_states.name[i] == "p15")||(human_states.name[i] == "p16")){
                //
                double cur_ptheta;
                cur_ptheta = convertQuaternionToAngle(human_states.pose[i].orientation);
                person.position.x = human_states.pose[i].position.x;
                person.position.y = human_states.pose[i].position.y;
                person.position.z = 0;

                person.velocity.x = cos(cur_ptheta);
                person.velocity.y = sin(cur_ptheta);
                person.velocity.z = 0;
                // distance from human to object
                person.reliability = 2.0;
                person.name = "object_people_info";
                people.people.push_back(person);
            }
        }
    }
    // Find the o-space of a group
    ROS_INFO("Estimate human group");
    geometry_msgs::Pose2D hgroup, shgroup;
    //
    shgroup = selectApproachingHumans(human_states, goal_index_);
    std::vector <geometry_msgs::Point> approaching_areas, approaching_areas1, approaching_areas2;
    std::vector <geometry_msgs::Point> selected_app_area;
    std::vector<std::vector <geometry_msgs::Point> > approaching_areas_2d;
    geometry_msgs::Point  approaching_point, p1;
    //
    double r_step = 0.1; // use to increase the radius of the approaching area
    bool approaching_flag = true;
    //if((approaching_flag)&&(!vhgroup.empty())){
            //hgroup = vhgroup[0];
    if((approaching_flag)&&(shgroup.theta!=0)){
            hgroup = shgroup;
            unsigned char n_rstep = 2;// control the step of rstep
            while((approaching_areas_2d.empty())&&(n_rstep<6)){
                approaching_areas = estimateApproachingAreas(hgroup.x, hgroup.y, hgroup.theta, r_step*n_rstep,200,goal_index_);
                //approaching_areas = filterApproachingAreasByFiewOfView(approaching_areas, human_states, goal_index_);
                // filter the area with small number of points
                approaching_areas_2d = selectPotentialApproachingAreas2D(approaching_areas,9);
                ROS_INFO("Size of 2d vector: %lu", approaching_areas_2d.size());

                //ROS_INFO("Number of rstep: %d", n_rstep);
                n_rstep++;
            }
            //
            if(!approaching_areas_2d.empty()){
                selected_app_area = selectedApproachingArea(approaching_areas_2d, cur_rpos_);
                app_pose = calculateApproachingPose(selected_app_area, hgroup);
                app_pose_ = app_pose;
                approachingPoseVisualize(app_pose, 99999);
                //
                int id_point=0;
                int id_approaching =0;
                //approaching_areas2 = selected_app_area;
                for(int m=0;m<approaching_areas_2d.size();m++){
                    approaching_areas2 = approaching_areas_2d[m];
                    for(int n=0;n<approaching_areas2.size()-1;n++){// (approaching_areas2.size()-1) because the end element is the center point
                        p1 = approaching_areas2[n];
                        marker=createMarkerPoint(p1.x,p1.y,0,id_point,id_approaching);
                        approaching_marker.markers.push_back(marker);
                        //ROS_INFO("Aproaching areas:x= %f,y= %f,index= %f,nrstep = %d",p1.x, p1.y, p1.z,n_rstep);
                        id_point++;
                    }
                }
            }else{ROS_INFO("Could not find the suitable approaching pose :(");}
    }
    // group people
    int group_people_flag = true;
    if(group_people_flag){
        for(int i=0;i<vhgroup_.size();i++){
            geometry_msgs::Pose2D hgroup_info = vhgroup_[i];
            person.position.x = hgroup_info.x;
            person.position.y = hgroup_info.y;
            person.position.z = 0;

            person.velocity.x = 0;
            person.velocity.y = 0;
            person.velocity.z = 0;
            // radius of the human group
            person.reliability = hgroup_info.theta;
            person.name = "group_people_info";
            people.people.push_back(person);
        }
    }
    //
    people.header.frame_id = "map"; // global map
    //people.header.seq = human_body.header.seq;
    //people.header.stamp = human_body.header.stamp;
    // pulish
    app_pose_pub.publish(app_pose_);
    human_pose_pub.publish(people);
    approaching_pub.publish(approaching_marker);
    approaching_marker.markers.clear();
    detected_human_vis_pub.publish(hmarker_array);
    //
    //human_state = human_states_.pose; // P0
    double vrl = sqrt(robot_state.twist.linear.x*robot_state.twist.linear.x + robot_state.twist.linear.y*robot_state.twist.linear.y);// linear velocity
    double vra = sqrt(robot_state.twist.angular.x*robot_state.twist.angular.x + robot_state.twist.angular.y*robot_state.twist.angular.y); // angular velocity
    resultFile = fopen("result_file_real_robot.csv","a");
    fprintf(resultFile,"%f %f %f %f %f %f %f %f %f\n",cur_rpos_.x,cur_rpos_.y,cur_rpos_.theta,vrl,vra,collision_index.data,orientation_index.data,interaction_index.data, motion_index.data); //Write validity value to file
    fclose(resultFile);
}
//
void robotOdomCallback(const nav_msgs::Odometry::ConstPtr & msg){
    robot_odom_ = *msg;
    double cur_rx, cur_ry, cur_rtheta, cur_rvx, cur_rvy, cur_rv;
    cur_rx = robot_odom_.pose.pose.position.x;
    cur_ry = robot_odom_.pose.pose.position.y;
    cur_rtheta = convertQuaternionToAngle(robot_odom_.pose.pose.orientation);

    cur_rvx = robot_odom_.twist.twist.linear.x;
    cur_rvy = robot_odom_.twist.twist.linear.y;
    cur_rv = sqrt(cur_rvx*cur_rvx + cur_rvy*cur_rvy);

    cur_rpos_.x = cur_rx; cur_rpos_.y = cur_ry; cur_rpos_.theta = cur_rtheta;

    ROS_INFO("Robot states: (xr = %f, yr = %f, thetar = %f, vr = %f)", cur_rx, cur_ry, cur_rtheta, cur_rv);
}
//
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "approaching_pose_real_robot_node");
    ros::NodeHandle nh;
    app_pose_service = nh.advertiseService("/get_approaching_pose", getApproachingPose);
    human_pose_pub = nh.advertise<people_msgs::People>("/human_information",5);
    detected_human_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/detected_human_vis",0);
    approaching_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/approaching_areas",0);
    app_pose_vis_pub = nh.advertise<visualization_msgs::Marker>("mybot_description/approaching_pose_vis",0);
    app_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/mybot_description/approaching_pose",5);
    collision_ind_pub = nh.advertise<std_msgs::Float32>("/Collision_index",5);
    orientation_ind_pub = nh.advertise<std_msgs::Float32>("/Orientation_index",5);
    interaction_ind_pub = nh.advertise<std_msgs::Float32>("/Interacion_index",5);
    motion_ind_pub = nh.advertise<std_msgs::Float32>("/Motion_index",5);
    ros::Subscriber local_cosmap_sub = nh.subscribe("/move_base/local_costmap/costmap",5,&localCostmapCallback);
    ros::Subscriber goal_index_sub = nh.subscribe("/send_goal_pose/goal_index",10,&goalIndexCallback);
    ros::Subscriber human_group_from_matlab_sub = nh.subscribe("/from_matlab/human_group",10,&humanGroupFromMatlabCallback);
    ros::Subscriber detected_human_sub = nh.subscribe("/mybot_description/detected_human",10,&detectedHumanCallback);
    ros::Subscriber robot_odom_sub = nh.subscribe ("/odom",10,&robotOdomCallback);

    //
    //ros::Rate rate(30);
    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
   return 0;
}



