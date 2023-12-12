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
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
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
ros::Publisher model_state_pub, human_pose_pub, approaching_pub, app_pose_vis_pub, app_pose_pub, human_direction_pub;
ros::Publisher collision_ind_pub, pub_sii_tc, pub_sii_tp, direction_ind_pub, pub_sdi_td, interaction_ind_pub, pub_sgi_tg, motion_ind_pub;
ros::Publisher detected_human_vis_pub;
ros::ServiceServer app_pose_service;

nav_msgs::OccupancyGrid local_costmap_;
nav_msgs::Odometry robot_odom_;

geometry_msgs::Pose2D cur_rpos_;
geometry_msgs::Pose2D app_pose_, app_pose_old_, app_pose_old1_;
sensor_msgs::PointCloud cloud_laser_;
std_msgs::Int8 goal_index_;
FILE * resultFile  = fopen("real_robot_results.csv","w");
gazebo_msgs::ModelState robot_state_;
gazebo_msgs::ModelStates human_states_, detected_human_states_, detected_human_states_old_;
std::vector <geometry_msgs::Pose2D> vhgroup_, vhgroup_old_;
double moving_x = 5.5; // for moving people
double moving_xh = -12.8, moving_yh = 0.3, moving_xr = -12.4, moving_yr = 0.7; // for moving robot and new moving people
bool start_moving_flag = false;
bool stop_human_data = false;
//
double cur_hx, cur_hy, cur_htheta;
//
visualization_msgs::Marker createVector(float_t xpos, float_t ypos, float_t zpos, float_t vx, float_t vy, int id_point, std::string ns, int colorindex)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    //std::stringstream ss;
    //ss << ns;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(0.5);
    //marker.ns = ss.str();
    marker.ns = ns;
    marker.id = id_point; // this number should be different
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
    switch(colorindex){
        case 1:// red
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            break;
        case 2:// blue
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            break;
        case 3: // green
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            break;
        case 4: // cyan
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            break;
        case 5: // magenta
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            break;
        default: // yellow
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = .0;
    }

    //
    p.x = xpos; p.y = ypos; p.z = zpos;
    marker.points.push_back(p);
    //
    p.x = xpos + vx; p.y = ypos + vy;  p.z = zpos;
    marker.points.push_back(p);

    return marker;
}
//
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
                                                                         geometry_msgs::Quaternion quat_in,geometry_msgs::Point hp,
                                                                         geometry_msgs::Twist tw)
{

    std::vector <geometry_msgs::Point> approaching_areas_fov;

    double cur_ptheta;
    double vpx = tw.linear.x;
    double vpy = tw.linear.y;
    double cur_angleo_tmp = convertQuaternionToAngle(quat_in);// angle using orientation
    double cur_angleo = -M_PI_2 - angles::normalize_angle(cur_angleo_tmp);
    double cur_anglev = atan2(vpy, vpx);// angle using velocity
    double vp = sqrt(vpx*vpx + vpy*vpy);
    if(vp<0.2) cur_ptheta = cur_angleo;
    else cur_ptheta = cur_anglev;

    //double ang = convertQuaternionToAngle(quat_in)-M_PI_2;
    double ang = cur_ptheta;
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
                                                                      gazebo_msgs::ModelStates human_states, geometry_msgs::Pose2D shgroup){

    std::vector <geometry_msgs::Point> approaching_areas_fov;
    if(shgroup.theta==1.09999){// single peron
        for(int i=0; i<human_states.name.size(); i++){
            if(human_states.name[i] == "p0"){
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position,human_states.twist[i]);
                approaching_areas = approaching_areas_fov;
            }
        }
    }else{
        for(int i=0; i<human_states.name.size(); i++){
            approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position,human_states.twist[i]);
            approaching_areas = approaching_areas_fov;
        }
    }
    return approaching_areas_fov;
}
//
geometry_msgs::Pose2D selectApproachingHumans(gazebo_msgs::ModelStates human_states, std::vector <geometry_msgs::Pose2D> vhgroup){
    // always approach a first person. If only one group then approach a group
    geometry_msgs::Point p;
    geometry_msgs::Pose2D shgroup; //single human
    shgroup.theta = 0;
    if(!vhgroup.empty())
    {
        shgroup = vhgroup[0];
        /* */
        if(shgroup.theta ==0.25555)// single person
            shgroup.theta = 1.09999;

    }
    return shgroup;
}
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
    marker.header.frame_id = "map";//map base_footprint
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
    if(dis<6.5)
        tmp = (2+ cos(beta) + cos(varphi))/(4*dis);


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
    double tmp = 999999;
    if(dis<6.5)
        tmp = (1+ cos(varphi))/(2*dis);

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
    double tmp = (2+  cos(beta)/(1+exp(-vr)) + cos(varphi)/(1+exp(-vh)))/(4*dis);

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
        double tmp1 = orientationIndex(x,y,theta,x0,y0,theta0);
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
        double rg = hgroup_info.theta;
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
geometry_msgs::Pose2D predictApproachingPoseOfMovingHumans(geometry_msgs::Pose2D pose)
{
    geometry_msgs::Pose2D app_pose;
    app_pose = pose;
    //goal_index_;  //cur_rpos_;
    double x = cur_rpos_.x, y = cur_rpos_.y, theta = cur_rpos_.theta;
    double x0 = pose.x, y0 = pose.y, theta0 =0;//M_PI

    double dx = x-x0, dy = y-y0;
    double dis = sqrt(dx*dx+dy*dy);
    double ang_ar = atan2(dy,dx);
    double ang_ra = atan2(-dy,-dx);
    double beta = angles::shortest_angular_distance(ang_ra,theta);
    double varphi = angles::shortest_angular_distance(ang_ar,theta0);
    double min_disr = dis*sin(varphi);
    double min_disa = dis*cos(beta);
    double vr = 0.2, va = 0.2;
    double factor = 1.0;//1.6
    double dis_app = factor*min_disr*va/vr; // dis_app >= min_disr*vp/vr
    ROS_INFO("Predict approaching distance: %f", dis_app);
    bool flag_moving = false;
    if(flag_moving){
        if(min_disr>0.5){//1.2
            app_pose.x = app_pose.x - dis_app; // predict
            app_pose_old_ = app_pose;
        }else{
            app_pose = app_pose_old_;
        }

    }
    return app_pose;
}
//
void publishHumanComfortableSafetyIndices(){

    gazebo_msgs::ModelStates human_states;
    gazebo_msgs::ModelState h0,h1,h2,h3;
    double th0,th1,th2,th3;
    human_states = detected_human_states_;
    // prepair to record the human data
    if(human_states.name.size()==1){
        h0.pose=human_states.pose[0]; h0.twist = human_states.twist[0];
        th0 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h0.pose.orientation));
    }else if(human_states.name.size()==2){
        h0.pose=human_states.pose[0]; h0.twist = human_states.twist[0];
        th0 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h0.pose.orientation));
        h1.pose=human_states.pose[1]; h1.twist = human_states.twist[1];
        th1 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h1.pose.orientation));
    }else if(human_states.name.size()==3){
        h0.pose=human_states.pose[0]; h0.twist = human_states.twist[0];
        th0 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h0.pose.orientation));
        h1.pose=human_states.pose[1]; h1.twist = human_states.twist[1];
        th1 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h1.pose.orientation));
        h2.pose=human_states.pose[2]; h2.twist = human_states.twist[2];
        th2 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h2.pose.orientation));
    }else if (human_states.name.size()==3) {
        h0.pose=human_states.pose[0]; h0.twist = human_states.twist[0];
        th0 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h0.pose.orientation));
        h1.pose=human_states.pose[1]; h1.twist = human_states.twist[1];
        th1 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h1.pose.orientation));
        h2.pose=human_states.pose[2]; h2.twist = human_states.twist[2];
        th2 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h2.pose.orientation));
        h3.pose=human_states.pose[3]; h3.twist = human_states.twist[3];
        th3 = -M_PI_2 - angles::normalize_angle(convertQuaternionToAngle(h3.pose.orientation));
    }else{
        th0=0; th1=0; th2=0; th3=0;
    }

    //
    std_msgs::Float32 collision_index, sii_tp, sii_tc, direction_index, sdi_td, interaction_index, sgi_tg;
    std_msgs::Float32 robot_velocity, motion_index, motion_index1, motion_index2, motion_index3, motion_index4, motion_index5;
    std::vector<double> indices= estimateHumanComfortableSafetyIndices(cur_rpos_, human_states);
    collision_index.data = indices[0]; direction_index.data = indices[1] ;
    interaction_index.data = indices[2];
    motion_index.data = indices[3];motion_index1.data = indices[4];motion_index2.data = indices[5];motion_index3.data = indices[6];motion_index4.data = indices[7];motion_index5.data = indices[8];
    double vr = sqrt(robot_state_.twist.linear.x * robot_state_.twist.linear.x + robot_state_.twist.linear.y * robot_state_.twist.linear.y);
    robot_velocity.data = vr;

    collision_ind_pub.publish(collision_index); sii_tp.data = 0.54; pub_sii_tp.publish(sii_tp); sii_tc.data = 0.14; pub_sii_tc.publish(sii_tc);
    direction_ind_pub.publish(direction_index); sdi_td.data = 1, pub_sdi_td.publish(sdi_td);
    interaction_ind_pub.publish(interaction_index); sgi_tg.data = 0.14; pub_sgi_tg.publish(sgi_tg);
    motion_ind_pub.publish(robot_velocity);//motion_index2
    //
    //human_state = human_states_.pose; // P0
    double vrl = sqrt(robot_state_.twist.linear.x*robot_state_.twist.linear.x + robot_state_.twist.linear.y*robot_state_.twist.linear.y);// linear velocity
    double vra = sqrt(robot_state_.twist.angular.x*robot_state_.twist.angular.x + robot_state_.twist.angular.y*robot_state_.twist.angular.y); // angular velocity
    resultFile = fopen("real_robot_results.csv","a");
    fprintf(resultFile,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",cur_rpos_.x,cur_rpos_.y,
            cur_rpos_.theta,vrl,vra,collision_index.data,direction_index.data,interaction_index.data,motion_index.data,
            h0.pose.position.x,h0.pose.position.y,th0,h0.twist.linear.x,h0.twist.linear.y,
            h1.pose.position.x,h1.pose.position.y,th1,h1.twist.linear.x,h1.twist.linear.y,
            h2.pose.position.x,h2.pose.position.y,th2,h2.twist.linear.x,h2.twist.linear.y,
            h3.pose.position.x,h3.pose.position.y,th3,h3.twist.linear.x,h3.twist.linear.y); //Write validity value to file
    fclose(resultFile);
}
//
bool humanObjectInteraction(double xp, double yp, double thetap, double xobj, double yobj){

    bool hoi_flag = false;
    double dx = xobj - xp;
    double dy = yobj - yp;
    double ang = atan2(dy,dx);
    double dis = sqrt(dx*dx+dy*dy);
    if(dis<2.8){
        double diff = angles::shortest_angular_distance(thetap,ang);
        if(fabs(diff)<M_PI/10){
            hoi_flag = true;
        }
    }
    return hoi_flag;
}
//
void humanInformationExtraction(){

    tf::TransformListener tf_listener;
    visualization_msgs::Marker marker, hmarker, hdmarker;
    visualization_msgs::MarkerArray hmarker_array, hdmarker_array;
    gazebo_msgs::ModelStates human_states, human_states1;
    gazebo_msgs::ModelState human_state;

    // Find the humans state
    people_msgs::People people;
    people_msgs::Person person;
    //
    human_states = detected_human_states_;
    // people
    for(int i =0; i<human_states.name.size(); i++){
        //
        tf::StampedTransform transform;
        geometry_msgs::PointStamped inp, outp;
        //
        inp.header.frame_id = "base_footprint";
        inp.point.x = human_states.pose[i].position.x;
        inp.point.y = human_states.pose[i].position.y;
        inp.point.z = 0;
        //outp = inp;
        /* */
        try{
            //tf_listener.transformPoint("map", inp, outp);
            tf_listener.transformPoint("base_footprint", inp, outp);
          }
          catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"base_footprint\" to \"map\": %s", ex.what());
          }

        double cur_ptheta;
        double vpx = human_states.twist[i].linear.x;
        double vpy = human_states.twist[i].linear.y;

        //double cur_angleo = convertQuaternionToAngle(human_states.pose[i].orientation)-M_PI_2;// angle using orientation
        double cur_angleo_tmp = convertQuaternionToAngle(human_states.pose[i].orientation);// angle using orientation
        double cur_angleo = -M_PI_2 - angles::normalize_angle(cur_angleo_tmp);
        double cur_anglev = atan2(vpy, vpx);// angle using velocity
        double vp = sqrt(vpx*vpx + vpy*vpy);
        bool laser_data = false;
        double hvel_threshold = 0.2;
        if(vp<hvel_threshold) cur_ptheta = cur_angleo;
        else cur_ptheta = cur_anglev;

        ROS_INFO("cur_ptheta_p %d: %f or %f or %f or %f",i,vp,cur_ptheta, cur_anglev,cur_angleo);
        person.position.x = outp.point.x;
        person.position.y = outp.point.y;
        person.position.z = human_states.pose[i].position.z;
       // sitting people
        if(human_states.twist[i].angular.x){// = 1 sitting otherwise
            ROS_INFO("Sitting person P_%d===================================================================",i);
            person.reliability = 0;
            person.velocity.x = 0.18*cos(cur_ptheta+M_PI_2);
            person.velocity.y = 0.18*sin(cur_ptheta+M_PI_2);
            person.velocity.z = 0;
        }
        else{
            person.reliability = 1;
            person.velocity.z = 0;
            if(vp<hvel_threshold){// standing
                if(laser_data){
                    person.velocity.x = 0.0*cos(cur_ptheta+M_PI_2);// laser data
                    person.velocity.y = 0.0*sin(cur_ptheta+M_PI_2);// laser data
                }else{
                    person.velocity.x = 0.12*cos(cur_ptheta+M_PI_2);// kinect data
                    person.velocity.y = 0.12*sin(cur_ptheta+M_PI_2);// kinect data
                }
            }else{ // moving
                person.velocity.x = vp*cos(cur_ptheta+M_PI_2);// vp
                person.velocity.y = vp*sin(cur_ptheta+M_PI_2);
            }
        }
        person.name = "people_info";
        people.people.push_back(person);
        hmarker = createMarkerHumanBody(person.position.x, person.position.y, 0.6, i, 9999);
        hmarker_array.markers.push_back(hmarker);
        hdmarker = createVector(person.position.x,person.position.y,person.position.z+0.2,0.5*cos(cur_ptheta),0.5*sin(cur_ptheta),i,"Human_direction",4);
        hdmarker_array.markers.push_back(hdmarker);
        // object poeple
        int object_people_flag = true;
        double xobj = 3.6; double yobj = -1.2;
        if(object_people_flag){
            if((human_states.name[i] == "p0")||(human_states.name[i] == "p1")){
                bool hoi_flag1 = humanObjectInteraction(person.position.x,person.position.y,cur_ptheta,xobj,yobj);
                if(hoi_flag1){
                    // distance from human to object
                    ROS_INFO("Human object interaction ***************");
                    if(human_states.name[i] == "p0"){
                        person.reliability = 1.9;
                    }else{
                        person.reliability = 2.0;
                    }
                    double hoi_angle = atan2(yobj-person.position.y, xobj-person.position.x);
                    person.velocity.z = 0;
                    person.velocity.x = 1*cos(hoi_angle+M_PI_2);//cur_ptheta
                    person.velocity.y = 1*sin(hoi_angle+M_PI_2);//cur_ptheta
                    person.name = "object_people_info";
                    people.people.push_back(person);
                }
            }
        }
    }
    // Find the o-space of a group
    ROS_INFO("Estimate human group");
    // group people
    int group_people_flag = true;
    //if((group_people_flag)&&(!stop_human_data)){
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
    people.header.frame_id = "map"; // map //base_footprint
    //people.header.frame_id = "base_footprint";

    //people.header.seq = human_body.header.seq;
    //people.header.stamp = human_body.header.stamp;
    // pulish
    human_pose_pub.publish(people);
    detected_human_vis_pub.publish(hmarker_array);
    human_direction_pub.publish(hdmarker_array);
    //

}

//
void approachingPoseEstimation(){
    //
    geometry_msgs::Pose2D hgroup, shgroup;
    gazebo_msgs::ModelStates human_states;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray approaching_marker;
    geometry_msgs::Pose2D app_pose, center_area;
    //
    human_states = detected_human_states_;
    //
    shgroup = selectApproachingHumans(human_states, vhgroup_);
    std::vector <geometry_msgs::Point> approaching_areas, approaching_areas1, approaching_areas2;
    std::vector <geometry_msgs::Point> selected_app_area;
    std::vector<std::vector <geometry_msgs::Point> > approaching_areas_2d;
    geometry_msgs::Point  approaching_point, p1;
    //
    double r_step = 0.2; // use to increase the radius of the approaching area
    bool approaching_flag = true;
    //
    if((approaching_flag)&&(shgroup.theta!=0)){
            hgroup = shgroup;
            unsigned char n_rstep = 1;// control the step of rstep
            while((approaching_areas_2d.empty())&&(n_rstep<6)){
                approaching_areas = estimateApproachingAreas(hgroup.x, hgroup.y, hgroup.theta, r_step*n_rstep,200,goal_index_);
                ROS_INFO("Size of approaching areas: filtered dsz = %lu",approaching_areas.size());
                approaching_areas = filterApproachingAreasByFiewOfView(approaching_areas, human_states, shgroup);
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
    //
    app_pose_pub.publish(app_pose_);
    approaching_pub.publish(approaching_marker);
    approaching_marker.markers.clear();
}
//
// data from human_skeleton package
void detectedHumanCallback(const gazebo_msgs::ModelStates::ConstPtr &msg){

    if(!stop_human_data){
        detected_human_states_ = *msg;
        approachingPoseEstimation();
        detected_human_states_old_ = detected_human_states_;
        ROS_INFO("Detected humans: %lu", msg->name.size());
    }else{
        detected_human_states_ = detected_human_states_old_;
        vhgroup_ = vhgroup_old_;
        ROS_INFO("Stop update humans data: %lu and %lu", detected_human_states_.name.size(),vhgroup_.size());
    }
    //humanInformationExtraction();
    //publishHumanComfortableSafetyIndices();
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
    if(msg->data>0){
        start_moving_flag = true;
    }
}
//
bool getApproachingPose(mybot_description::GetApproachingPose::Request &req,mybot_description::GetApproachingPose::Response &res){

    res.index = goal_index_;
    res.app_pose = app_pose_;
    stop_human_data = true;// people do not move or change pose

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
    if(!stop_human_data){
        vhgroup_ = vpose;
        vhgroup_old_ = vhgroup_;
    }

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

    // Extract human infor and send it to the navigation package
    humanInformationExtraction();
    publishHumanComfortableSafetyIndices();

}
//
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "approaching_pose_real_robot_node");
    ros::NodeHandle nh;
    //
    app_pose_service = nh.advertiseService("/get_approaching_pose", getApproachingPose);
    // Transfer to navigation package
    human_pose_pub = nh.advertise<people_msgs::People>("/human_information",5);
    // Visualize
    detected_human_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/detected_human_vis",0);
    approaching_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/approaching_areas",0);
    human_direction_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/human_direction",0);
    app_pose_vis_pub = nh.advertise<visualization_msgs::Marker>("mybot_description/approaching_pose_vis",0);
    // Publish approaching pose
    app_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/mybot_description/approaching_pose",5);
    // Human comfortable indices
    collision_ind_pub = nh.advertise<std_msgs::Float32>("/SII",5);
    pub_sii_tc = nh.advertise<std_msgs::Float32>("/Tc",5);
    pub_sii_tp = nh.advertise<std_msgs::Float32>("/Tp",5);
    direction_ind_pub = nh.advertise<std_msgs::Float32>("/SDI",5);
    pub_sdi_td = nh.advertise<std_msgs::Float32>("/Td",5);
    interaction_ind_pub = nh.advertise<std_msgs::Float32>("/SGI",5);
    pub_sgi_tg = nh.advertise<std_msgs::Float32>("/Tg",5);
    motion_ind_pub = nh.advertise<std_msgs::Float32>("/Vr",5);

    //
    //ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 100, &modelStatesCallback);
    ros::Subscriber local_cosmap_sub = nh.subscribe("/move_base/local_costmap/costmap",5,&localCostmapCallback);
    ros::Subscriber goal_index_sub = nh.subscribe("/send_goal_pose/goal_index",10,&goalIndexCallback);
    ros::Subscriber human_group_from_matlab_sub = nh.subscribe("/from_matlab/human_group",10,&humanGroupFromMatlabCallback);
    ros::Subscriber detected_human_sub = nh.subscribe("/mybot_description/detected_human",10,&detectedHumanCallback);
    ros::Subscriber robot_odom_sub = nh.subscribe ("/odom",10,&robotOdomCallback);

    //ros::Rate rate(30);
    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
   return 0;
}

