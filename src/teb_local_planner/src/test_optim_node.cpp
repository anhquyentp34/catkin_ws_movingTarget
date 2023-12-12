/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
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
#include <teb_local_planner/homotopy_class_planner.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <tf/transform_broadcaster.h>
#include <hrvo/hrvo.h>
#include <hrvo/Vector2.h>
#include <animated_marker_msgs/AnimatedMarker.h>
#include <animated_marker_msgs/AnimatedMarkerArray.h>
#include <std_msgs/Float64.h>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes
using namespace hrvo;

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub,custom_goal_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CB_customGoal(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);

//define new variables
geometry_msgs::Twist *start_vel;
TebOptimalPlannerPtr best_teb_optim;
visualization_msgs::Marker createVector(float_t xpos, float_t ypos, float_t zpos, float_t vx, float_t vy, int id_point);
ros::Publisher goal_arrow_pub,detected_goal_vis_pub,Public_SII,Public_SGI;
double foot_speed=0.7;
std::vector<Vector2> goal_pos;
std::vector<double> goal_agle_factor;
animated_marker_msgs::AnimatedMarkerArray human_marker_array,goal_marker_array;
costmap_converter::ObstacleArrayMsg goal_arr;
FILE * resultFile_;
int write_count = 0;
bool check_goal_reset = false;
int human_name = 0;

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

void goal_Vis(Vector2 h_pos, int id, double k)
{
    Vector2 vel,pos;
    vel.setX(foot_speed*cos(k));
    vel.setY(foot_speed*sin(k));//M_PI
    pos = h_pos;
    goal_marker_array.markers.push_back(humanVirtualUsingVel(id, pos, vel,"goal_obst_"));
    detected_goal_vis_pub.publish(goal_marker_array);

    costmap_converter::ObstacleMsg obst_;
    geometry_msgs::Point32 new_pt;

    obst_.header.frame_id = "odom";
    obst_.header.stamp = ros::Time();
    obst_.id = 0;

    new_pt.x = h_pos.getX();
    new_pt.y = h_pos.getY();
    new_pt.z = 0;

    double theta = atan2 ( vel.getY(), vel.getX() );
    Eigen::Quaternionf q = rpy2Quaternion(M_PI / 2.0, theta + (M_PI / 2.0), 0.0);
    obst_.orientation.x = q.x();
    obst_.orientation.y = q.y();
    obst_.orientation.z = q.z();
    obst_.orientation.w = q.w();

    obst_.polygon.points.push_back(new_pt);
    goal_arr.obstacles.push_back(obst_);
}
// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
  tf::TransformBroadcaster g_transformBroadcaster;
  
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);

  n.getParam("human", human_name);

  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
//  custom_goal_sub = n.subscribe("goal1", 1, CB_customGoal);
  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);
  
  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);
  
  // setup callback for via-points (callback overwrites previously set via-points)
  via_points_sub = n.subscribe("via_points", 1, CB_via_points);

  // interactive marker server for simulated dynamic obstacles
  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

//  obst_vector.push_back( boost::make_shared<PointObstacle>(-3,1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(8,2) );
//  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,1.5,1,-1.5) ); //90 deg
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,0,-1,0) ); //180 deg
//  obst_vector.push_back( boost::make_shared<PointObstacle>(-1.5,-0.5) );

  // Dynamic obstacles
//  Eigen::Vector2d vel (0.1, -0.3);
//  obst_vector.at(0)->setCentroidVelocity(vel);
//  vel = Eigen::Vector2d(-0.3, -0.2);
//  obst_vector.at(1)->setCentroidVelocity(vel);

  /*
  PolygonObstacle* polyobst = new PolygonObstacle;
  polyobst->pushBackVertex(1, -1);
  polyobst->pushBackVertex(0, 1);
  polyobst->pushBackVertex(1, 1);
  polyobst->pushBackVertex(2, 1);
 
  polyobst->finalizePolygon();
  obst_vector.emplace_back(polyobst);
  */
  
  for (unsigned int i=0; i<obst_vector.size(); ++i)
  {
    // setup callbacks for setting obstacle velocities
    std::string topic = "/test_optim_node/obstacle_" + std::to_string(i) + "/cmd_vel";
    obst_vel_subs.push_back(n.subscribe<geometry_msgs::Twist>(topic, 1, boost::bind(&CB_setObstacleVelocity, _1, i)));

    //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);
    // Add interactive markers for all point obstacles
    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
    if (pobst)
    {
      CreateInteractiveMarker(pobst->x(),pobst->y(),i, config.map_frame, &marker_server, &CB_obstacle_marker);
    }
  }
  marker_server.applyChanges();
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n);
  
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
  {
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
  }
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
  
  no_fixed_obstacles = obst_vector.size();


  start_vel = new (geometry_msgs::Twist);
//  robot_pose_ = new (geometry_msgs::Pose);

  global_start_vel = new (geometry_msgs::Twist);
  global_robot_pose_ = new (geometry_msgs::Pose);

  global_robot_pose_->position.x = -4.5;
  global_robot_pose_->position.y = -1.0;
  global_robot_pose_->orientation.z = 0;

  goal_arrow_pub = n.advertise<visualization_msgs::Marker>( "/test_optim_node/goal_arrow", 10 );

  Vector2 g1;
//  double yg1 = 1.9,xg1=4.5;
  double yg1 = 1.9,xg1=-3.0;
  goal_agle_factor.push_back(0);
  detected_goal_vis_pub = n.advertise<animated_marker_msgs::AnimatedMarkerArray>("/test_optim_node/goal_vis_pub",1);
  ros::Publisher Public_SII = n.advertise<std_msgs::Float64>("/social_teb/Public_SII", 1);
  ros::Publisher Public_SGI = n.advertise<std_msgs::Float64>("/social_teb/Public_SGI", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
      if (config.goal_tolerance.reset_robot)
      {
          yg1 = 1.9;
//          xg1 = 4.5;
          xg1 = -3.0;
          global_robot_pose_->position.x = -5.0;
          global_robot_pose_->position.y = -1;
          global_robot_pose_->orientation.z = 0;

          write_count++;
          check_goal_reset = true;
      }
//    if (goal_pos_reset)
//    {
//        yg1 = 1.9;
//        xg1 = 4.5;
//        global_robot_pose_->position.x = -4.5;
//        global_robot_pose_->position.y = -2;
//        global_robot_pose_->orientation.z = 0;



//    }

    std_msgs::Float64 maxSII;
    std_msgs::Float64 maxSGI;

    tf::Transform g_currentPose;
    g_currentPose.getOrigin().setX(global_robot_pose_->position.x);
    g_currentPose.getOrigin().setY(global_robot_pose_->position.y);
    double theta = global_robot_pose_->orientation.z;
    g_currentPose.setRotation( tf::createQuaternionFromRPY(0, 0, theta));
    g_transformBroadcaster.sendTransform( tf::StampedTransform(g_currentPose, ros::Time::now(), "odom", "base_footprint") );

    goal_marker_array.markers.clear();
    goal_pos.clear();
    goal_agle_factor.clear();
    goal_arr.obstacles.clear();


    g1.setX(xg1);g1.setY(yg1);
    goal_pos.push_back(g1);


    if (!goal_pos_pause)
    {
//        xg1 = xg1 - 0.035;
        xg1 = xg1 + 0.035;
        foot_speed = 0.7;
    }
    else
    {
        foot_speed = 0.01;
    }

    if (xg1 > 4.5)
    {
        xg1 = -3.0;//-4.5
    }
    for (int i=0;i < goal_pos.size();++i) {
      goal_Vis(goal_pos[i], i,goal_agle_factor[i]);
    }

    maxSII.data = global_SII;
    maxSGI.data = global_SGI;
    Public_SII.publish(maxSII);
    Public_SGI.publish(maxSGI);

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}

visualization_msgs::Marker createVector(float_t xpos, float_t ypos, float_t zpos, float_t vx, float_t vy, int id_point)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    std::stringstream ss;
    ss << "goal_ARROW";

    marker.header.frame_id = "odom";//world
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = id_point; // this number should be different
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = xpos;
    marker.pose.position.y = ypos;
    marker.pose.position.z = zpos;

    double theta = atan2 ( vy, vx );
    Eigen::Quaternionf q = rpy2Quaternion(M_PI / 2.0, theta, 0.0);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 0.1;

    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    //
//    p.x = xpos; p.y = ypos; p.z = zpos;
//    marker.points.push_back(p);
    //
//    p.x = xpos + q.x(); p.y = ypos + q.y();  p.z = zpos;
//    marker.points.push_back(p);

    return marker;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
    PoseSE2 start,goal1,goal2;
    std::vector<PoseSE2> goal_vector;
    visualization_msgs::Marker marker1, marker2;

    start.x() = global_robot_pose_->position.x;
    start.y() = global_robot_pose_->position.y;
    start.theta() = global_robot_pose_->orientation.z;
    start_vel = global_start_vel;

    goal_vector.resize(goal_pos.size() * 2);
    for (int i = 0; i < goal_pos.size(); ++i)
    {
        goal_vector.at(i).x() = goal_pos[i].getX() + 1;
        goal_vector.at(i).y() = goal_pos[i].getY() - 0.7;
        goal_vector.at(i).theta() = 3 * M_PI_4;
        marker1 = createVector(goal_vector.at(i).x(), goal_vector.at(i).y(), 0, cos(3 * M_PI_4), sin(3 * M_PI_4), i);

        goal_vector.at(i+1).x() = goal_pos[i].getX() + 1;
        goal_vector.at(i+1).y() = goal_pos[i].getY() + 0.7;
        goal_vector.at(i+1).theta() = 5*M_PI_4;
        marker2 = createVector(goal_vector.at(i+1).x(), goal_vector.at(i+1).y(), 0, cos(5*M_PI_4), sin(5*M_PI_4), i+1);
    }
    goal_arrow_pub.publish(marker1);
    goal_arrow_pub.publish(marker2);
    planner->plan(start, goal_vector);
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e)
{
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  
  if (index>=no_fixed_obstacles) 
    return;
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);

  
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
//    printf("obst_msg->obstacles.at(i).polygon.points.size()=%lu",obst_msg->obstacles.at(i).polygon.points.size());
//    printf("------------ \n");
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
    {
      if (obst_msg->obstacles.at(i).radius == 0) 
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
                                                            obst_msg->obstacles.at(i).radius )));
      }
      double th0 = obst_msg->obstacles.at(0).orientation.z;// tf::getYaw(obst_msg->obstacles.at(0).orientation);
      double th1 = obst_msg->obstacles.at(1).orientation.z;//tf::getYaw(obst_msg->obstacles.at(1).orientation);
      double thr = global_robot_pose_->orientation.z;//tf::getYaw(global_robot_pose_->orientation);


      if (!config.goal_tolerance.reset_robot && check_goal_reset)
      {
          char filename_[200];
//          sprintf( filename_, "/home/hoangbay/Desktop/Ghi_file/simulation_%d.csv", human_name );
          sprintf( filename_, "/home/hoangbay/Desktop/Ghi_file/simulation_%d.csv", write_count );
          resultFile_ = fopen(filename_,"a");
          fprintf(resultFile_,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",global_robot_pose_->position.x,global_robot_pose_->position.y,thr, global_start_vel->linear.x,
                global_SII,global_SGI,
                obst_msg->obstacles.at(0).polygon.points[0].x,obst_msg->obstacles.at(0).polygon.points[0].y,th0, obst_msg->obstacles.at(0).velocities.twist.linear.x,
                obst_msg->obstacles.at(1).polygon.points[0].x,obst_msg->obstacles.at(1).polygon.points[0].y,th1, obst_msg->obstacles.at(1).velocities.twist.linear.x,
                goal_pos[0].getX(),goal_pos[0].getY(),M_PI,obst_msg->obstacles.at(1).velocities.twist.linear.x);
          fclose(resultFile_);
      }
    }
    else if (obst_msg->obstacles.at(i).polygon.points.empty())
    {
      ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
      continue;
    }
    else
    {
      ROS_WARN("size > 1...................");
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                  obst_msg->obstacles.at(i).polygon.points[j].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if(!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }

  //human goals goal_arr
  for (size_t i = 0; i < goal_arr.obstacles.size(); ++i)
  {
      obst_vector.push_back(ObstaclePtr(new PointObstacle( goal_arr.obstacles.at(i).polygon.points.front().x,
                                                         goal_arr.obstacles.at(i).polygon.points.front().y )));
  }
}

void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint<=0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel (twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}
