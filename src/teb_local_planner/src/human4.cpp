/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>
//#include <ros/console.h>
//#include <ros/assert.h>

#include <hrvo/hrvo.h>
#include <hrvo/Vector2.h>
//#include <hrvo/Clusters.h>

#include <animated_marker_msgs/AnimatedMarker.h>
#include <animated_marker_msgs/AnimatedMarkerArray.h>
#include <math.h>

//#include <costmap_converter/ObstacleMsg.h>
//#include <costmap_converter/ObstacleArrayMsg.h>


using namespace teb_local_planner;
using namespace hrvo;

animated_marker_msgs::AnimatedMarkerArray human_marker_array,goal_marker_array;
ros::Publisher detected_human_vis_pub,detected_human_pub,detected_goal_pub,detected_goal_vis_pub;
costmap_converter::ObstacleArrayMsg human_arr, goal_arr;
std::vector<Vector2> human_pos,goal_pos;
std::vector<double> human_agle_factor,goal_agle_factor;
TebConfig cfg_;
double foot_speed=0.7;

Eigen::Quaternionf rpy2Quaternion(double roll, double pitch, double yaw)
{
    Eigen::Quaternionf r_m = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
                            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    return r_m.normalized();
}
//
animated_marker_msgs::AnimatedMarker humanVirtualUsingVel(int id, Vector2 pos, Vector2 vel, std::string name){

    animated_marker_msgs::AnimatedMarker marker;
        marker.mesh_use_embedded_materials = true;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.lifetime = ros::Duration(1);
        std::stringstream ss;
        ss << name << id;
        marker.ns = ss.str();
//        marker.ns = "human";
        marker.id = id;
        marker.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
        marker.action = 0;  // add or modify
        marker.pose.position.x = pos.getX();
        marker.pose.position.y = pos.getY();

        marker.animation_speed = sqrt(vel.getX()*vel.getX()+vel.getY()*vel.getY());
        double theta = atan2 ( vel.getY(), vel.getX() );
        Eigen::Quaternionf q = rpy2Quaternion(M_PI / 2.0, theta + (M_PI / 2.0), 0.0);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.mesh_resource = "package://animated_marker_tutorial/meshes/animated_walking_man.mesh";
        const double person_scale6 = 1.8 / 8.5 * 1.8;  // TODO - move these magic numbers to a config file
        marker.scale.x = person_scale6; marker.scale.y = person_scale6; marker.scale.z = person_scale6;
        if ( id % 2 == 0 ){
            marker.color.r=0.15; marker.color.g=0.9; marker.color.b=0.9;marker.color.a=1;
        }else{
            if(id % 3 ==0){
                marker.color.r=0.9; marker.color.g=0.9; marker.color.b=0.15;marker.color.a=1;
            }else{
                marker.color.r=0.9; marker.color.g=0.15; marker.color.b=0.9;marker.color.a=1;
            }
        }
    return marker;
}

void setHuman(Vector2 h_pos, int id, double k)
{

    Vector2 vel,pos;
    vel.setX(foot_speed*cos(k));
    vel.setY(foot_speed*sin(k));//M_PI
    pos = h_pos;
    human_marker_array.markers.push_back(humanVirtualUsingVel(id, pos, vel,"human_obst_"));

    costmap_converter::ObstacleMsg obst_;
    geometry_msgs::Point32 new_pt;

    obst_.header.frame_id = "odom";
    obst_.header.stamp = ros::Time();
    obst_.id = 0;

    double theta = atan2 ( vel.getY(), vel.getX() );
    Eigen::Quaternionf q = rpy2Quaternion(M_PI / 2.0, theta + (M_PI / 2.0), 0.0);
    obst_.orientation.x = q.x();
    obst_.orientation.y = q.y();
    obst_.orientation.z = k;//q.z();
    obst_.orientation.w = q.w();

    new_pt.x = h_pos.getX();
    new_pt.y = h_pos.getY();
    new_pt.z = 0;
    obst_.polygon.points.push_back(new_pt);
    human_arr.obstacles.push_back(obst_);
}

// =============== Main function =================
int main( int argc, char** argv )
{
    ros::init(argc, argv, "human1");
    ros::NodeHandle n("~");
    cfg_.loadRosParamFromNodeHandle(n);

    detected_human_vis_pub = n.advertise<animated_marker_msgs::AnimatedMarkerArray>("/human1/human1_vis_pub",1);
    detected_human_pub = n.advertise<costmap_converter::ObstacleArrayMsg>("/test_optim_node/obstacles", 1);

    Vector2 h1,h2;
    double xh1 = -5.0, xh2 = -5.0;
    bool revse_ = true;
    int count_walk = 0;

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
      human_arr.obstacles.clear();
      human_marker_array.markers.clear();
      human_pos.clear();
      human_agle_factor.clear();
      foot_speed = 0.7;

//      human_agle_factor.push_back(-M_PI_2);
//      human_agle_factor.push_back(M_PI_2);
//M_PI

//      h1.setX(-1);h1.setY(yh1);
//      h2.setX(0);h2.setY(yh2);
      h1.setX(xh1);h1.setY(1.0);
      h2.setX(xh2);h2.setY(0.2);

      human_pos.push_back(h1);
      human_pos.push_back(h2);

//      yh1 = yh1 - 0.04;yh2 = yh2 + 0.04;
//      if (yh1 < -3.5)
//      {
//          foot_speed = 0.01;
//          yh1 = -3.5;
//          count_delay ++;
//          if (count_delay > 100)
//          {
//            yh1 = 3.5;
//            yh2 = -3.5;
//            yg1 = -3.5;
//            count_delay = 0;
//      robot_pos_reset = true;
//          }
//      }
//      if (yh2 > 3.5)
//      {
//          yh2 = 3.5;
//      }

      if (revse_)
      {
          count_walk++;
        human_agle_factor.push_back(0);
        human_agle_factor.push_back(0);

        if  (count_walk < 130)
        {
            xh1 = xh1 + 0.04;
            xh2 = xh2 + 0.04;
            foot_speed = 0.7;
        }
        else {
            xh1 = xh1 + 0.02;
            xh2 = xh2 + 0.02;
            foot_speed = 0.7;
        }


        if (xh1 > 6.0)
        {
            xh1 = -5.0, xh2 = -5.0;
            count_walk = 0;
//          revse_ = false;
        }
//        if (xh2 > 4.5)
//        {
//          xh2 = -2.5;
//        }
      }
//      else {
//          count_walk = 0;
//        human_agle_factor.push_back(M_PI);
//        human_agle_factor.push_back(M_PI);

//        xh1 = xh1 - 0.04;
//        xh2 = xh2 - 0.04;
//        foot_speed = 0.7;

//        if (xh1 < -5.0)
//        {
//          revse_ = true;
//        }
//      }



      for (int i=0;i < human_pos.size();++i) {
        setHuman(human_pos[i], i,human_agle_factor[i]);
      }

      detected_human_vis_pub.publish(human_marker_array);
      detected_human_pub.publish(human_arr);


      ros::spinOnce();
      loop_rate.sleep();
    }
    ros::spin();

    return 0;
}
