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
ros::Publisher model_state_pub, human_pose_pub, approaching_pub, app_pose_vis_pub, app_pose_pub;
ros::Publisher collision_ind_pub, pub_sii_tp, pub_sii_tc,  direction_ind_pub, pub_sdi_td, interaction_ind_pub,pub_sgi_tg, motion_ind_pub;
ros::Publisher detected_human_vis_pub;
ros::ServiceServer app_pose_service;

nav_msgs::OccupancyGrid local_costmap_;
geometry_msgs::Pose2D cur_rpos_;
geometry_msgs::Pose2D app_pose_, app_pose_old_, app_pose_old1_;
sensor_msgs::PointCloud cloud_laser_;
std_msgs::Int8 goal_index_;
FILE * resultFile  = fopen("simulation_robot_results_hlobs.csv","w");
//FILE * resultFile  = fopen("simulation_robot_results_dsz.csv","w");
//FILE * resultFile  = fopen("simulation_robot_results_dsz_appose.csv","w");
gazebo_msgs::ModelState robot_state_;
gazebo_msgs::ModelStates human_states_, detected_human_states_;
std::vector <geometry_msgs::Pose2D> vhgroup_;
double moving_x = 5.5; // for moving people
double moving_xh = -12.8, moving_yh = 0.3, moving_xr = -12.4, moving_yr = 0.7; // for moving robot and new moving people
bool start_moving_flag = false;
//
std_msgs::Float32 sdi_td_;// using for the threshold of the social direction index
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
/* the old version of "selectPotentialApproachingAreas2D"
std::vector <geometry_msgs::Point> selectPotentialApproachingAreas(std::vector <geometry_msgs::Point> approaching_areas){
    geometry_msgs::Point p1, p2;
    std::vector <geometry_msgs::Point> potential_ap, tmp1, tmp2;
    unsigned int index=1;
    unsigned int index1=1;
    unsigned int npoint= 0;
    // separate the area
    for(int i=1;i<approaching_areas.size();i++){
        p1 = approaching_areas[i-1];
        p2 = approaching_areas[i];
        if((p2.z-p1.z)>1){
            index++;
            p2.z = index;
            tmp1.push_back(p2);
        }else{
            p2.z = index;
            tmp1.push_back(p2);
        }
    }
    // if there is only one area
    if(index==1){
        ROS_INFO("Size of potential_ap with one area: %lu", tmp1.size());
        // add the center point into the end of vector
        potential_ap=tmp1;
    }
    // filter the area with minimum number of points
    for(int i=1;i<tmp1.size();i++){
        p1 = tmp1[i-1];
        p2 = tmp1[i];
        if((p2.z-p1.z)==0){
            npoint++;
            p2.z = index1;
            tmp2.push_back(p2);
        }else{
            if(npoint>5){// choose the area with the number of point >5
                //potential_ap.push_back(tmp2);
                for(int j=0;j<tmp2.size();j++){
                    potential_ap.push_back(tmp2[j]);
                }
                // add the center point into the end of vector
                //potential_ap.push_back(tmp2[int(npoint/2)]);
            }
            tmp2.clear();
            npoint=0;
            index1++;
            p2.z = index1;
            tmp2.push_back(p2);
        }
    }
    //ROS_INFO("Number of approaching area before and after filter: %d and %d", index, index1);
    return potential_ap;
}
*/
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
        if((ind==2)||(ind==18)||(ind==21)||(ind==24)){ // use this because of the approaching filter not very good at this time :)
            if(ind==21){//[-3pi/4-pi/4]
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
    //ROS_INFO("Me no chu mai khong debug duoc la sao: %lu", approaching_areas.size());
    for(int i = 0; i<human_states.name.size(); i++){
        //if(ind==0){
        if((ind==0)||(ind==1)){
            if((human_states.name[i] == "p0")||(human_states.name[i] == "p1")){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        if(ind==5){
            if((human_states.name[i] == "p2")||(human_states.name[i] == "p3")||(human_states.name[i] == "p4")){
                //approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas_fov = approaching_areas;// because field of view
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        if(ind==2){
            if((human_states.name[i] == "p5")||(human_states.name[i] == "p6")||(human_states.name[i] == "p7")){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        if(ind==8){
            if(human_states.name[i] == "p8"){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        if(ind==12){
            if((human_states.name[i] == "p15")||(human_states.name[i] == "p16")){
                ROS_INFO("Buc minh qua:p15p16 ind = %d",goal_index_.data);
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        //if((ind==14)||(ind==15)){
        if((ind==15)){
            if(human_states.name[i] == "p17"){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                //ROS_INFO("Buc minh qua:p17 ind = %d, inputsize = %lu, outputsize =%lu ",goal_index_.data,approaching_areas.size(),approaching_areas_fov.size());
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        if(ind==18){
            if(human_states.name[i] == "p9"){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        if(ind==21){
            if((human_states.name[i] == "p10")||(human_states.name[i] == "p11")){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        if(ind==24){
            if((human_states.name[i] == "p12")||(human_states.name[i] == "p13")){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
        }
        if(ind==27){
            if(human_states.name[i] == "p14"){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
                approaching_areas = approaching_areas_fov;
            }
            //approaching_areas = approaching_areas_fov;
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
gazebo_msgs::ModelStates getHumanPoseFromGazebo(gazebo_msgs::ModelStates msg){

    gazebo_msgs::ModelStates human_states;

    for(int i =0; i<msg.name.size(); i++){
        if((msg.name[i] == "p0")||(msg.name[i] == "p1")||(msg.name[i] == "p2")||(msg.name[i] == "p3")||(msg.name[i] == "p4")||(msg.name[i] == "p5")
          ||(msg.name[i] == "p6")||(msg.name[i] == "p7")||(msg.name[i] == "p8")||(msg.name[i] == "p9")||(msg.name[i] == "p10")||(msg.name[i] == "p11")
          ||(msg.name[i] == "p12")||(msg.name[i] == "p13")||(msg.name[i] == "p14")||(msg.name[i] == "p15")||(msg.name[i] == "p16")||(msg.name[i] == "p17"))
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
gazebo_msgs::ModelStates humanDetectionFromGazebo(geometry_msgs::Pose2D cur_rpos, sensor_msgs::PointCloud cloud_laser,gazebo_msgs::ModelStates msg){


    gazebo_msgs::ModelStates human_states;
    double max_dis = 8.0; double max_dxy = 0.4;
    for(int i=0; i<msg.name.size(); i++){
        double dx = msg.pose[i].position.x - cur_rpos.x;
        double dy = msg.pose[i].position.y - cur_rpos.y;
        double d = sqrt(dx*dx + dy*dy);
        double ang = atan2(dy,dx);
         if(d<max_dis){// maximum distance of laser data
            double diff = angles::shortest_angular_distance(cur_rpos.theta, ang);
            if(fabs(diff)<M_PI/2){
            //if(fabs(diff)<2.09){ // 240^o
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
double collisionIndex(double x, double y, double x0, double y0, double A, double sigmax, double sigmay, double skew){
    double dx = x-x0, dy = y-y0;
    double h = sqrt(dx*dx+dy*dy);
    double angle = atan2(dy,dx);
    double mx = cos(angle-skew) * h;
    double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * sigmax * sigmax),
           f2 = pow(my, 2.0)/(2.0 * sigmay * sigmay);
    return A * exp(-(f1 + f2));
}
//
double orientationIndex(double x, double y, double theta, double x0, double y0, double theta0){
    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    double ang_rh = atan2(-dy,-dx);
    double ang_hr = atan2(dy,dx);
    double beta = angles::shortest_angular_distance(ang_rh,theta);
    double varphi = angles::shortest_angular_distance(ang_hr,theta0);
    //double tmp = (2+ cos(beta) + cos(varphi))/4;
    double tmp = 999999;
    //if(dis<6.5)
    if(dis<2.5)
        tmp = (2+ cos(beta) + cos(varphi))/(4*dis);
        //tmp = (1+ cos(varphi))/(2*dis);

    return tmp;
}
//
double interactionIndex(double x, double y, double x0, double y0, double A, double sigmax, double sigmay, double skew){
    double dx = x-x0, dy = y-y0;
    double h = sqrt(dx*dx+dy*dy);
    double angle = atan2(dy,dx);
    double mx = cos(angle-skew) * h;
    double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * sigmax * sigmax),
           f2 = pow(my, 2.0)/(2.0 * sigmay * sigmay);
    return A * exp(-(f1 + f2));
}
//
double motionIndex(double x, double y, double theta, double vr, double x0, double y0, double theta0, double vh){

    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    double ang_hr = atan2(dy,dx);
    double ang_rh = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_rh,theta);
    double varphi = angles::shortest_angular_distance(ang_hr,theta0);
    //double tmp = (2+ vr*cos(beta) + vh*cos(varphi))/(4*dis);
    double tmp = (1+ cos(varphi))/(2*dis);

    return tmp;
}
// collision
double motionIndex1(double x, double y, double theta, double vr, double x0, double y0, double theta0, double vh){

    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    double ang_hr = atan2(dy,dx);
    double ang_rh = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_rh,theta);
    double varphi = angles::shortest_angular_distance(ang_hr,theta0);
    //double tmp = (2+  cos(beta)/(1+exp(-vr)) + cos(varphi)/(1+exp(-vh)))/(4*dis);
    double tmp;
    if(dis<2.5)
        tmp = (2+ cos(beta) + cos(varphi))/4;

    return tmp;
}
// direction
double motionIndex2(double x, double y, double theta, double vr, double x0, double y0, double theta0, double vh){

    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    //double ang_hr = atan2(dy,dx);
    double ang_rh = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_rh,theta);
    //double varphi = angles::shortest_angular_distance(ang_hr,theta0);
    double tmp = (1+ vr*cos(beta))/(2*dis);

    return tmp;
}
// interaction
double motionIndex3(double x, double y, double theta, double vr, double x0, double y0, double theta0, double vh){

    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    //double ang_hr = atan2(dy,dx);
    double ang_rh = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_rh,theta);
    //double varphi = angles::shortest_angular_distance(ang_hr,theta0);
    double tmp = (1+  cos(beta)/(1+exp(-vr)) )/(2*dis);

    return tmp;
}
double motionIndex4(double x, double y, double theta, double vr, double x0, double y0, double theta0, double vh){

    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    double ang_hr = atan2(dy,dx);
    double ang_rh = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_rh,theta);
    double varphi = angles::shortest_angular_distance(ang_hr,theta0);
    double vrh = vr*cos(beta) + vh*cos(varphi);
    double tmp = 0.5 +1/(2+exp(-0.5*vrh/dis));

    return tmp;
}
double motionIndex5(double x, double y, double theta, double vr, double x0, double y0, double theta0, double vh){

    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    //double ang_hr = atan2(dy,dx);
    double ang_rh = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_rh,theta);
    //double varphi = angles::shortest_angular_distance(ang_hr,theta0);
    double tmp = (vr*cos(beta))/(dis);

    return tmp;
}
//
std::vector <double> estimateHumanComfortableSafetyIndices(geometry_msgs::Pose2D cur_rpos, gazebo_msgs::ModelStates msg){
    std::vector <double> indices;
    int8_t ind = goal_index_.data;
    double x = cur_rpos.x;
    double y = cur_rpos.y;
    double theta = cur_rpos.theta;
    double max_collision = 0; double max_interaction = 0;
    double min_orientation = 999999; double max_motion = 0; double max_motion1 = 0;
    double max_motion2 = 0; double max_motion3 = 0; double max_motion4 = 0; double max_motion5 = 0;
    double vrl = sqrt(robot_state_.twist.linear.x*robot_state_.twist.linear.x + robot_state_.twist.linear.y*robot_state_.twist.linear.y);// linear velocity

    for(int i=0; i<msg.name.size(); i++){
        double x0 = msg.pose[i].position.x;
        double y0 = msg.pose[i].position.y;
        double vh = sqrt(msg.twist[i].linear.x*msg.twist[i].linear.x+msg.twist[i].linear.y*msg.twist[i].linear.y);
        // calculate the collision index
        double sigmaxy = 0.8652/2.0;//0.8652; 2.4
        double tmp ;
        if((msg.name[i] == "p15")||(msg.name[i] == "p16"))
            tmp = collisionIndex(x,y,x0-0.3,y0-0.0,1,sigmaxy,sigmaxy,0);
            //tmp = collisionIndex(x,y,x0-0.0,y0-0.0,1,sigmaxy,sigmaxy,0);
        else
            tmp = collisionIndex(x,y,x0,y0,1,sigmaxy,sigmaxy,0);

        if(tmp>max_collision){
            max_collision = tmp;
        }
        // calculate the orientation index
        geometry_msgs::Quaternion quat_in = msg.pose[i].orientation;
        double theta0 = convertQuaternionToAngle(quat_in)-M_PI_2;
        double tmp1 = 999999;
        if((ind==1)||(ind==2)){
            if((msg.name[i] == "p0")||(msg.name[i] == "p1")){
                tmp1 = orientationIndex(x,y,theta,x0,y0,theta0);
            }
        }else{
            tmp1 = orientationIndex(x,y,theta,x0,y0,theta0);
        }

        if(tmp1<min_orientation){
            min_orientation = tmp1;
        }
        // calculate interaction index
        double tmp2 = motionIndex(x,y,theta,vrl,x0,y0,theta0,vh);
        if(tmp2>max_motion){
            max_motion = tmp2;
        }
        double tmpm1 = motionIndex1(x,y,theta,vrl,x0,y0,theta0,vh);
        if(tmpm1>max_motion1){
            max_motion1 = tmpm1;
        }
        double tmpm2 = motionIndex2(x,y,theta,vrl,x0,y0,theta0,vh);
        if(tmpm2>max_motion2){
            max_motion2 = tmpm2;
        }
        double tmpm3 = motionIndex3(x,y,theta,vrl,x0,y0,theta0,vh);
        if(tmpm3>max_motion3){
            max_motion3 = tmpm3;
        }
        double tmpm4 = motionIndex4(x,y,theta,vrl,x0,y0,theta0,vh);
        if(tmpm4>max_motion4){
            max_motion4 = tmpm4;
        }
        double tmpm5 = motionIndex5(x,y,theta,vrl,x0,y0,theta0,vh);
        if(tmpm5>max_motion5){
            max_motion5 = tmpm5;
        }
    }
    if(min_orientation==999999){min_orientation=0;}
    // interaction space
    for(int i=0;i<vhgroup_.size();i++){
        geometry_msgs::Pose2D hgroup_info = vhgroup_[i];
        double x0 = hgroup_info.x;
        double y0 = hgroup_info.y;
        // radius of the human group
        double rg;
        if(ind>11&ind<=14){
            rg = 0.8; x0=-13.2;y0=-5.8;}
        else
            rg = hgroup_info.theta;
        // calculate interaction index
        double tmp3 = interactionIndex(x,y,x0,y0,1,rg/1.8,rg/1.8,0);
        if(tmp3>max_interaction){
            max_interaction = tmp3;
        }
    }

    indices.push_back(max_collision);
    indices.push_back(min_orientation);
    indices.push_back(max_interaction);
    indices.push_back(max_motion);indices.push_back(max_motion1);indices.push_back(max_motion2);
    indices.push_back(max_motion3);indices.push_back(max_motion4);indices.push_back(max_motion5);

    return indices;
}
//
std::vector <geometry_msgs::Pose2D> humanGroupDetection(gazebo_msgs::ModelStates human_states){

    geometry_msgs::Pose2D pose;
    std::vector <geometry_msgs::Pose2D> vpose;
    Circle circle;
    reals BEDataX[3]= {0,0,0};reals BEDataX1[3]= {0,0,0};reals BEDataX2[2]= {0,0};
    reals BEDataX3[2]= {0,0};reals BEDataX4[2]= {0,0};reals BEDataX5[3]= {0,0,0};
    reals BEDataY[3]= {0,0,0}; reals BEDataY1[3]= {0,0,0}; reals BEDataY2[2]= {0,0};
    reals BEDataY3[2]= {0,0}; reals BEDataY4[2]= {0,0}; reals BEDataY5[3]= {0,0,0};

    int k =0; int k1 =0; int k2 =0; int k3 =0; int k4 =0; int k5 =0;
    for(int i = 0; i<human_states.name.size(); i++){
        if((human_states.name[i] == "p2")||(human_states.name[i] == "p3")||(human_states.name[i] == "p4")){
            BEDataX[k] = human_states.pose[i].position.x;
            BEDataY[k] = human_states.pose[i].position.y;
            k = k+ 1;
            if(k==3){
                Data data1(3,BEDataX,BEDataY);
                //Circle circle;
                cout.precision(7);
                circle = CircleFitByTaubin (data1);
                //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
                vpose.push_back(pose);
            }
        }
        if((human_states.name[i] == "p5")||(human_states.name[i] == "p6")||(human_states.name[i] == "p7")){
            BEDataX1[k1] = human_states.pose[i].position.x;
            BEDataY1[k1] = human_states.pose[i].position.y;
            k1 = k1+ 1;
            if(k1==3){
                Data data1(3,BEDataX1,BEDataY1);
                //Circle circle;
                cout.precision(7);
                circle = CircleFitByTaubin (data1);
                //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
                vpose.push_back(pose);
            }
        }
        /*
        if((human_states.name[i] == "p10")||(human_states.name[i] == "p11")){
            BEDataX1[k2] = human_states.pose[i].position.x;
            BEDataY1[k2] = human_states.pose[i].position.y;
            k2 = k2+ 1;
            if(k2==2){
                Data data1(2,BEDataX2,BEDataY2);
                //Circle circle;
                cout.precision(7);
                circle = CircleFitByTaubin (data1);
                cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
                vpose.push_back(pose);
            }
        }
        if((human_states.name[i] == "p12")||(human_states.name[i] == "p13")){
            BEDataX1[k3] = human_states.pose[i].position.x;
            BEDataY1[k3] = human_states.pose[i].position.y;
            k3 = k3+ 1;
            if(k3==2){
                Data data1(2,BEDataX3,BEDataY3);
                //Circle circle;
                cout.precision(7);
                circle = CircleFitByTaubin (data1);
                cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
                vpose.push_back(pose);
            }
        }

        if((human_states.name[i] == "p0")||(human_states.name[i] == "p1")){
            BEDataX1[k4] = human_states.pose[i].position.x;
            BEDataY1[k4] = human_states.pose[i].position.y;
            k4 = k4+ 1;
            if(k4==2){
                Data data1(2,BEDataX4,BEDataY4);
                //Circle circle;
                cout.precision(7);
                circle = CircleFitByTaubin (data1);
                //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
                vpose.push_back(pose);
            }
        }
        if((human_states.name[i] == "p15")||(human_states.name[i] == "p16")){
            BEDataX1[k5] = human_states.pose[i].position.x;
            BEDataY1[k5] = human_states.pose[i].position.y;
            k5 = k5+ 1;
            if(k5==2){
                Data data1(2,BEDataX5,BEDataY5);
                //Circle circle;
                cout.precision(7);
                circle = CircleFitByTaubin (data1);
                //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
                vpose.push_back(pose);
            }
        }
        */
    }
    return vpose;
}
//
geometry_msgs::Pose2D selectApproachingHumans(gazebo_msgs::ModelStates human_states, std_msgs::Int8 goal_index){

    geometry_msgs::Pose2D pose;
    std::vector <geometry_msgs::Pose2D> vpose;
    geometry_msgs::Point p0, p1;
    Circle circle;
    reals BEDataX[3]= {0,0,0};reals BEDataX1[3]= {0,0,0};reals BEDataX2[2]= {0,0};
    reals BEDataX3[2]= {0,0};reals BEDataX4[2]= {0,0};reals BEDataX5[3]= {0,0,0};
    reals BEDataY[3]= {0,0,0}; reals BEDataY1[3]= {0,0,0}; reals BEDataY2[2]= {0,0};
    reals BEDataY3[2]= {0,0}; reals BEDataY4[2]= {0,0}; reals BEDataY5[3]= {0,0,0};
    int8_t ind;
    geometry_msgs::Pose2D shgroup; //single human
    shgroup.theta = 0;
    ind = goal_index.data;
    int k =0; int k1 =0; int k2 =0; int k3 =0; int k4 =0; int k5 =0;
    for(int i = 0; i<human_states.name.size(); i++){
        //if(ind==0){
        if((ind==0)||(ind==1)){
            if((human_states.name[i] == "p0")||(human_states.name[i] == "p1")){
                if(human_states.name[i] == "p0"){p0=human_states.pose[i].position;}
                if(human_states.name[i] == "p1"){p1=human_states.pose[i].position;}
                k = k+ 1;
                if(k==2){
                    double xg = (p0.x+p1.x)/2; double yg = (p0.y+p1.y)/2;
                    shgroup.x = xg-0.35;//5.0
                    shgroup.y = yg;//3.9
                    shgroup.theta = 0.8661; //1.0
                }
            }
        }
        if(ind==5){
            if((human_states.name[i] == "p2")||(human_states.name[i] == "p3")||(human_states.name[i] == "p4")){
                BEDataX[k1] = human_states.pose[i].position.x;
                BEDataY[k1] = human_states.pose[i].position.y;
                k1 = k1+ 1;
                if(k1==3){
                    Data data1(3,BEDataX,BEDataY);
                    cout.precision(7);
                    circle = CircleFitByTaubin (data1);
                    //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                    shgroup.x = circle.a; shgroup.y = circle.b; shgroup.theta = circle.r+0.0;//0.2

                }
            }
        }
        if(ind==2){
            if((human_states.name[i] == "p5")||(human_states.name[i] == "p6")||(human_states.name[i] == "p7")){
                BEDataX1[k2] = human_states.pose[i].position.x;
                BEDataY1[k2] = human_states.pose[i].position.y;
                k2 = k2+ 1;
                if(k2==3){
                    Data data1(3,BEDataX1,BEDataY1);
                    cout.precision(7);
                    circle = CircleFitByTaubin (data1);
                    shgroup.x = circle.a; shgroup.y = circle.b; shgroup.theta = circle.r-0.15;//0
                }
            }
        }
        if(ind==8){
            if(human_states.name[i] == "p8"){
                shgroup.x = human_states.pose[i].position.x;
                shgroup.y = human_states.pose[i].position.y;
                shgroup.theta = 1.1;
            }
        }
        if(ind==12){
            if((human_states.name[i] == "p15")||(human_states.name[i] == "p16")){
                k3 = k3+ 1;
                if(k3==2){
                    shgroup.x = -13.2;//-13.5
                    shgroup.y = -5.6;//-5.5
                    shgroup.theta = 0.9;
                }
            }
        }
        //if((ind==14)||(ind==15)){
        if((ind==15)){
            if(human_states.name[i] == "p17"){
                shgroup.x = human_states.pose[i].position.x;
                shgroup.y = human_states.pose[i].position.y;
                shgroup.theta = 1.0;
            }
        }
        if(ind==18){
            if(human_states.name[i] == "p9"){
                shgroup.x = human_states.pose[i].position.x;
                shgroup.y = human_states.pose[i].position.y;
                shgroup.theta = 1.1;//1.1
            }
        }
        if(ind==21){
            if((human_states.name[i] == "p10")||(human_states.name[i] == "p11")){
                k4 = k4+ 1;
                if(k4==2){
                    shgroup.x = -9.0;
                    shgroup.y = 9.2;
                    shgroup.theta = 0.85;
                }
            }
        }
        if(ind==24){
            if((human_states.name[i] == "p12")||(human_states.name[i] == "p13")){
                k5 = k5+ 1;
                if(k5==2){
                    shgroup.x = -16.0;
                    shgroup.y = 11.5;
                    shgroup.theta = 0.9;
                }
            }
        }
        if(ind==27){
            if(human_states.name[i] == "p14"){
                shgroup.x = human_states.pose[i].position.x;
                shgroup.y = human_states.pose[i].position.y;
                shgroup.theta = 1.0;// 1.1
            }
        }
    }
    return shgroup;
}
//
geometry_msgs::Pose2D predictApproachingPoseOfMovingHumans(geometry_msgs::Pose2D pose)
{
    geometry_msgs::Pose2D app_pose;
    app_pose = pose;
    //goal_index_;  //cur_rpos_;
    double x = cur_rpos_.x, y = cur_rpos_.y, theta = cur_rpos_.theta;
    double x0 = pose.x, y0 = pose.y, theta0 =M_PI;

    if(goal_index_.data ==15){
        theta0 = -M_PI/4;}

    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    double ang_ar = atan2(dy,dx);
    double ang_ra = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_ra,theta);
    double varphi = angles::shortest_angular_distance(ang_ar,theta0);
    double min_disr = dis*sin(varphi);
    double min_disa = dis*cos(beta);
    double vr = 0.2, va = 0.2;
    double factor = 1.6;
    double dis_app = factor*min_disr*va/vr; // dis_app >= min_disr*vp/vr
    ROS_INFO("Predict approaching distance: %f", dis_app);
    bool flag_dsz_appose = false;
    if(flag_dsz_appose){
        if(goal_index_.data==1){
            if(min_disr>1.2){
                app_pose.x = app_pose.x - dis_app; // predict
                app_pose_old_ = app_pose;
            }else{
                app_pose = app_pose_old_;
            }
        }
        /**/
        if(goal_index_.data==15){
            if(min_disr>1.6){
                app_pose.x = app_pose.x + dis_app/(1.0*1.4142); // predict
                app_pose.y = app_pose.y - dis_app/(1.0*1.4142); // predict
                app_pose_old1_ = app_pose;
            }else{
                app_pose = app_pose_old1_;
            }
        }
    }
    return app_pose;
}
//
void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg){

    visualization_msgs::Marker marker, hmarker;
    visualization_msgs::MarkerArray approaching_marker, hmarker_array;
    geometry_msgs::Pose2D app_pose, center_area;
    gazebo_msgs::ModelStates human_states, human_states1;
    gazebo_msgs::ModelState human_state;

    // Find the humans state

    people_msgs::People people;
    people_msgs::Person person;
    //
    ROS_INFO("Goal index: %d", goal_index_.data);
    // robot pose
    double cur_rx, cur_ry, cur_rtheta;
    for(int i =0; i<msg->name.size(); i++){
        if(msg->name[i] == "robot_model"){
            robot_state_.model_name = msg->name[i]; robot_state_.pose = msg->pose[i]; robot_state_.twist = msg->twist[i];
            cur_rtheta = convertQuaternionToAngle(msg->pose[i].orientation);
            cur_rx = msg->pose[i].position.x; cur_ry = msg->pose[i].position.y;
            cur_rpos_.x = cur_rx; cur_rpos_.y = cur_ry; cur_rpos_.theta = cur_rtheta;
            //ROS_INFO("Robot pose (x,y,theta) = (%f, %f, %f)", cur_rx, cur_ry, cur_rtheta);
            //ROS_INFO("Robot vel (lx,ly,lz,ax,ay,az) = (%f, %f, %f, %f, %f, %f, %f, %f)",msg->twist[i].linear.x,msg->twist[i].linear.y,
            //         msg->twist[i].linear.z,msg->twist[i].angular.x,msg->twist[i].angular.y,msg->twist[i].angular.z);
            break;
        }
    }
    //
    for(int i =0; i<msg->name.size(); i++)
    {
        if((msg->name[i] == "p0")&&(start_moving_flag))
        {
            //ROS_INFO("P0 pose (x,y,theta) ith %d = (%f, %f, %f)",i, msg->pose[i].position.x,msg->pose[i].position.y,msg->pose[i].position.z);
            human_state.pose = msg->pose[i];
            human_state.twist = msg->twist[i];
            human_state.reference_frame = "world";
            human_state.model_name = "p0";
            if((human_state.pose.position.x>-16.5)&&(goal_index_.data<7)){//goal_index_.data<8
                moving_x = human_state.pose.position.x - 0.01;
                human_state.pose.position.x = moving_x;// 0.2 m/s
            }else{
                human_state.pose.position.x = 5.5;// 0.2 m/s
            }
            model_state_pub.publish(human_state);
            //human_state.model_name = "p1";
            //model_state_pub.publish(human_state);
        }
        /* */
        if((msg->name[i] == "p1")&&(start_moving_flag))
        {
            //ROS_INFO("P0 pose (x,y,theta) ith %d = (%f, %f, %f)",i, msg->pose[i].position.x,msg->pose[i].position.y,msg->pose[i].position.z);
            human_state.pose = msg->pose[i];
            human_state.twist = msg->twist[i];
            human_state.reference_frame = "world";
            human_state.model_name = "p1";

            if((human_state.pose.position.x>-16.5)&&(goal_index_.data<7)){//goal_index_.data<8
                //human_state.pose.position.x = human_state.pose.position.x - 0.02;// 0.2 m/s
                human_state.pose.position.x = moving_x;// 0.2 m/s
            }else{
                human_state.pose.position.x = 5.5;// 0.2 m/s
            }

            model_state_pub.publish(human_state);
        }
        //
        if((msg->name[i] == "p17")&&(start_moving_flag))
        {
            human_state.pose = msg->pose[i];
            human_state.twist = msg->twist[i];
            human_state.reference_frame = "world";
            human_state.model_name = "p17";
            if((human_state.pose.position.x<-8.2)&&(goal_index_.data>14)){
                moving_xh = human_state.pose.position.x + 0.01;
                human_state.pose.position.x = moving_xh;// 0.2 m/s
                moving_yh = human_state.pose.position.y - 0.01;
                human_state.pose.position.y = moving_yh;// 0.2 m/s
            }else{
                human_state.pose.position.x = moving_xh;// 0.2 m/s
                human_state.pose.position.y = moving_yh;// 0.2 m/s

            }
            model_state_pub.publish(human_state);
            //human_state.model_name = "p1";
            //model_state_pub.publish(human_state);
        }
        if((msg->name[i] == "asimo")&&(start_moving_flag))
        {
            human_state.pose = msg->pose[i];
            human_state.twist = msg->twist[i];
            human_state.reference_frame = "world";
            human_state.model_name = "asimo";
            if((human_state.pose.position.x<-8.2+0.4)&&(goal_index_.data>14)){
                human_state.pose.position.x = moving_xh+0.4;// 0.2 m/s
                human_state.pose.position.y = moving_yh+0.4;// 0.2 m/s
            }else{
                human_state.pose.position.x = moving_xh+0.4;// 0.2 m/s
                human_state.pose.position.y = moving_yh+0.4;// 0.2 m/s
            }
            model_state_pub.publish(human_state);
            //human_state.model_name = "p1";
            //model_state_pub.publish(human_state);
        }
    }
    // Laser data in point cloud data
    ROS_INFO("Size of cloud_laser: %lu",cloud_laser_.points.size());
    //
    human_states = getHumanPoseFromGazebo(*msg);
    human_states_ = human_states;
    //human_states1 = humanDetectionFromGazebo(cur_rpos_, cloud_laser_, human_states);
    //human_states = human_states1;
    human_states = detected_human_states_;

    // Human safety and comfort indices
    std_msgs::Float32 collision_index,sii_tp, sii_tc, direction_index, interaction_index, sgi_tg;
    std_msgs::Float32 robot_velocity, motion_index, motion_index1, motion_index2, motion_index3, motion_index4, motion_index5;

    std::vector<double> indices= estimateHumanComfortableSafetyIndices(cur_rpos_, human_states);
    collision_index.data = indices[0]; direction_index.data = indices[1] ;
    interaction_index.data = indices[2];
    motion_index.data = indices[3];motion_index1.data = indices[4];motion_index2.data = indices[5];motion_index3.data = indices[6];motion_index4.data = indices[7];motion_index5.data = indices[8];
    double vr = sqrt(robot_state_.twist.linear.x * robot_state_.twist.linear.x + robot_state_.twist.linear.y * robot_state_.twist.linear.y);
    robot_velocity.data = vr;

    collision_ind_pub.publish(collision_index); sii_tp.data = 0.54; pub_sii_tp.publish(sii_tp); sii_tc.data = 0.14; pub_sii_tc.publish(sii_tc);
    direction_ind_pub.publish(direction_index); sdi_td_.data = 0.45; pub_sdi_td.publish(sdi_td_);
    interaction_ind_pub.publish(interaction_index); sgi_tg.data = 0.14; pub_sgi_tg.publish(sgi_tg);
    motion_ind_pub.publish(robot_velocity);//motion_index2

    //ollision_ind_pub.publish(motion_index1);
    //direction_ind_pub.publish(motion_index2);
    //interaction_ind_pub.publish(motion_index3);
    //motion_ind_pub.publish(motion_index);
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
        if((human_states.name[i] == "p0")||(human_states.name[i] == "p1")||(human_states.name[i] == "p17")){
            if(human_states.name[i] == "p17"){
                if(goal_index_.data>14){ // when p17 is moving
                    person.velocity.x = 0.8*cos(cur_ptheta);// 0.25
                    person.velocity.y = 0.8*sin(cur_ptheta);
                    person.velocity.z = 0;
                }else{
                    person.velocity.x = 0.12*cos(cur_ptheta);
                    person.velocity.y = 0.12*sin(cur_ptheta);
                    person.velocity.z = 0;
                }
            }
            else{
                person.velocity.x = 0.8*cos(cur_ptheta);// 0.25
                person.velocity.y = 0.8*sin(cur_ptheta);
                person.velocity.z = 0;
            }

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
            person.velocity.x = 0.18*cos(cur_ptheta);
            person.velocity.y = 0.18*sin(cur_ptheta);
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
                // distance from human to object
                if(human_states.name[i] == "p8"){
                    person.reliability = 1.9;
                }else{
                    person.reliability = 2.2;
                }
                person.velocity.z = 0;
                person.velocity.x = 1*cos(cur_ptheta);
                person.velocity.y = 1*sin(cur_ptheta);
                person.name = "object_people_info";
                people.people.push_back(person);
            }
        }

    }
    // Find the o-space of a group
    ROS_INFO("Estimate human group");
    geometry_msgs::Pose2D hgroup, shgroup;

    //vhgroup_ = humanGroupDetection(human_states);
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
                ROS_INFO("Size of approaching areas: filtered dsz = %lu",approaching_areas.size());
                approaching_areas = filterApproachingAreasByFiewOfView(approaching_areas, human_states, goal_index_);
                // filter the area with small number of points
                //approaching_areas1 = selectPotentialApproachingAreas(approaching_areas);
                approaching_areas_2d = selectPotentialApproachingAreas2D(approaching_areas,9);
                ROS_INFO("Size of approaching areas: filtered fov = %lu, filtered numberofpoint = %lu", approaching_areas.size(), approaching_areas_2d.size());

                //ROS_INFO("Number of rstep: %d", n_rstep);
                n_rstep++;
            }
            //
            if(!approaching_areas_2d.empty()){
                selected_app_area = selectedApproachingArea(approaching_areas_2d, cur_rpos_);
                app_pose = calculateApproachingPose(selected_app_area, hgroup);
                app_pose = predictApproachingPoseOfMovingHumans(app_pose);

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
    double vrl = sqrt(robot_state_.twist.linear.x*robot_state_.twist.linear.x + robot_state_.twist.linear.y*robot_state_.twist.linear.y);// linear velocity
    double vra = sqrt(robot_state_.twist.angular.x*robot_state_.twist.angular.x + robot_state_.twist.angular.y*robot_state_.twist.angular.y); // angular velocity
    resultFile = fopen("simulation_robot_results_hlobs.csv","a");
    //resultFile = fopen("simulation_robot_results_dsz.csv","a");
    //resultFile = fopen("simulation_robot_results_dsz_appose.csv","a");
    fprintf(resultFile,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",cur_rpos_.x,cur_rpos_.y,cur_rpos_.theta,vrl,vra,collision_index.data,direction_index.data,interaction_index.data,motion_index1.data,motion_index.data,motion_index2.data,motion_index3.data,motion_index4.data,motion_index5.data); //Write validity value to file
    fclose(resultFile);
}
//
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
// get data from "mybot_convert_laser_pointcloud"
// this is data in point cloud type of the laser data
/*
void laserPointCloudCallback(const sensor_msgs::PointCloud::ConstPtr & cloud){
    cloud_laser_ = *cloud;

}
*/
//
void goalIndexCallback(const std_msgs::Int8::ConstPtr &msg){
    goal_index_ = *msg;
    if(msg->data>0){
        start_moving_flag = true;
    }

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
    detected_human_states_ = *msg;
    ROS_INFO("Detected humans: %lu", msg->name.size());

}
//
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "approaching_pose_node");
    ros::NodeHandle nh;
    app_pose_service = nh.advertiseService("/get_approaching_pose", getApproachingPose);
    model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    human_pose_pub = nh.advertise<people_msgs::People>("/human_information",5);
    detected_human_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/detected_human_vis",0);
    approaching_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/approaching_areas",0);
    app_pose_vis_pub = nh.advertise<visualization_msgs::Marker>("mybot_description/approaching_pose_vis",0);
    app_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/mybot_description/approaching_pose",5);

    collision_ind_pub = nh.advertise<std_msgs::Float32>("/SII",5);
    pub_sii_tc = nh.advertise<std_msgs::Float32>("/Tc",5);
    pub_sii_tp = nh.advertise<std_msgs::Float32>("/Tp",5);
    direction_ind_pub = nh.advertise<std_msgs::Float32>("/SDI",5);
    pub_sdi_td = nh.advertise<std_msgs::Float32>("/Td",5);
    interaction_ind_pub = nh.advertise<std_msgs::Float32>("/SGI",5);
    pub_sgi_tg = nh.advertise<std_msgs::Float32>("/Tg",5);
    motion_ind_pub = nh.advertise<std_msgs::Float32>("/Vr",5);

    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 100, &modelStatesCallback);
    ros::Subscriber local_cosmap_sub = nh.subscribe("/move_base/local_costmap/costmap",5,&localCostmapCallback);
    //ros::Subscriber laser_data_sub = nh.subscribe("/mybot_description/laser_pointcloud",10,&laserPointCloudCallback);
    ros::Subscriber goal_index_sub = nh.subscribe("/send_goal_pose/goal_index",10,&goalIndexCallback);
    ros::Subscriber human_group_from_matlab_sub = nh.subscribe("/from_matlab/human_group",1,&humanGroupFromMatlabCallback);//10
    ros::Subscriber detected_human_sub = nh.subscribe("/mybot_description/detected_human",10,&detectedHumanCallback);

    //

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

