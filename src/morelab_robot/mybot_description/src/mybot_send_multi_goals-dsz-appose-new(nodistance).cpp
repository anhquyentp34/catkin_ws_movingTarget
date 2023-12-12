// This program is used to send the goal to the robot.
// The goal is in local map
#include <iostream>
#include <vector>
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
#include <mybot_description/GetApproachingPose.h>
//
#define N_GOALS 29
#define PI 3.139601
#define PI2 1.569799
#define PI4 0.784899
//
ros::Publisher goal_pose_pub, goal_index_pub;
geometry_msgs::Pose2D new_goal_;
unsigned int counter_ = 0;
std_msgs::Int8 update_index_old;
geometry_msgs::Pose update_goal_pose_old;
bool subgoal_flag = false;
//
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// N_GOALS x [x,y,w]
unsigned int approching_index [] ={0, 1, 3, 8, 12, 16, 19, 22, 25};
double_t landmarks_pose[N_GOALS][3] = {{0.5,-0.5,-0.9849},{2.0,-3.0,0.01},{3.8,-5.5,-PI2},{3.0,-8.0,PI},{3.8,-5.5,PI2},{3.8,-4.0,PI2},{3.5,-1.5,PI4},{-3.2,-4.0,PI},{-3.2,-1.0,PI2},{-5.0,1.0,PI2},
                                       {-3.2,-1.0,-PI2},{-3.2,-4.0,PI},{-8.5,-4.0,PI},{-13.0,-7.0,PI2},{-11.0,-6.0,PI2},{-13.0,-2.5,PI2},{-11.8,1.2,PI2},{-11.8,6.6,PI2},{-12.8,6.6,PI},{-14.0,8.0,-3*PI4},
                                       {-12.8,6.6,0.01},{-10.8,6.6,0.01},{-8.0,8.5,PI4},{-10.8,6.6,PI},{-12.8,9.7,PI},{-14.0,11.5,PI},{-12.8,9.7,0.01},{-10.8,12.0,0.01},
                                       {-9.0,12.0,PI4}};//{{0.5,-1.0,-0.7849} or {0.5,-1.5,-0.9849}
std::vector<geometry_msgs::Pose> landmarks(N_GOALS);
// init the goal pose
void initLandmarks()
{
    geometry_msgs::Pose goal_pose;
    for(unsigned int i =0; i<landmarks.size();i++)
    {
        goal_pose.position.x = landmarks_pose[i][0];
        goal_pose.position.y = landmarks_pose[i][1];
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,landmarks_pose[i][2]);
        goal_pose.orientation = quat;
        landmarks[i] = goal_pose;
    }
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
void goalPoseVisualize(geometry_msgs::Pose goal_pose, int n_goal)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    std::stringstream ss;
    ss << "goal_pose_" << n_goal;

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
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //
    p.x = goal_pose.position.x;
    p.y = goal_pose.position.y;
    p.z = 0;
    marker.points.push_back(p);
    //
    double ang = convertQuaternionToAngle(goal_pose.orientation);
    p.x = goal_pose.position.x + 0.4*cos(ang);
    p.y = goal_pose.position.y + 0.4*sin(ang);
    p.z = 0;
    marker.points.push_back(p);
    goal_pose_pub.publish(marker);
    marker.points.clear();

}

// update landmarks from the new goals
bool updateLandmarks()
{
}
//
/*
void newGoalCallback(const geometry_msgs::Pose2D::ConstPtr &msg){
    ROS_INFO("Got a new approaching pose");
    new_goal_ = *msg;
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = new_goal_.x;
    goal_pose.position.y = new_goal_.y;
    goal_pose.orientation.w = new_goal_.theta;
    landmarks[counter_+1] = goal_pose;
}
*/
//
void sleepWhenApproachingHumans(double nsecond, unsigned int goal_index){
    //if((goal_index==3)||(goal_index==6)||(goal_index==9)||(goal_index==13)||(goal_index==17)||(goal_index==20)||(goal_index==23)||(goal_index==26))
    if((goal_index==2)||(goal_index==4)||(goal_index==7)||(goal_index==10)||(goal_index==14)||(goal_index==16)||(goal_index==20)||(goal_index==23)||(goal_index==26)||(goal_index==29))
    {
        if((goal_index==2)){
            ros::Duration(nsecond+2).sleep();// sleep nscond
        }else{
            ros::Duration(nsecond).sleep();// sleep nscond
        }
    }
}
//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mybot_send_multi_goals_node");
  ros::NodeHandle nh;
  goal_pose_pub = nh.advertise<visualization_msgs::Marker>("send_goal_pose/goal_pose",0);
  goal_index_pub = nh.advertise<std_msgs::Int8>("/send_goal_pose/goal_index",100);
  //ros::Subscriber new_goal_sub = nh.subscribe("/mybot_description/approaching_pose",5,&newGoalCallback);
  ros::ServiceClient app_pose_client = nh.serviceClient<mybot_description::GetApproachingPose>("/get_approaching_pose");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient action_goal("move_base", true);
  //
  initLandmarks();
  //wait for the action server to come up
  while(!action_goal.waitForServer(ros::Duration(10.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  //
  //initLandmarks();
  //ROS_INFO("Number of goals: %d",landmarks.size());
  //
  unsigned int counter = 0;
  while(counter<N_GOALS)
  {
      mybot_description::GetApproachingPose srv;

      if(app_pose_client.call(srv)){
          std_msgs::Int8 update_index;
          geometry_msgs::Pose update_goal_pose;
          update_goal_pose.position.x = srv.response.app_pose.x;
          update_goal_pose.position.y = srv.response.app_pose.y;
          //update_goal_pose.orientation.w = srv.response.app_pose.theta;
          geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,srv.response.app_pose.theta);
          update_goal_pose.orientation = quat;

          update_index = srv.response.index;
          ROS_INFO("Receive a new goal [%d] (x, y, w) = (%f, %f, %f)",update_index.data,update_goal_pose.position.x,
                   update_goal_pose.position.y,srv.response.app_pose.theta);
          //if(update_goal_pose.position.x!=update_goal_pose_old.position.x){
          if((update_goal_pose.position.x!=update_goal_pose_old.position.x)||(update_goal_pose.position.y!=update_goal_pose_old.position.y)){
              if((counter==1)||(counter==2)||(counter==15)){
                landmarks[update_index.data] = update_goal_pose;// for moving humans
                ROS_INFO("Update new goal [%d] (x, y, w) = (%f, %f, %f)",update_index.data,update_goal_pose.position.x,
                       update_goal_pose.position.y,srv.response.app_pose.theta);
              }else{
                landmarks[update_index.data+1] = update_goal_pose;
                ROS_INFO("Update new goal [%d] (x, y, w) = (%f, %f, %f)",update_index.data+1,update_goal_pose.position.x,
                         update_goal_pose.position.y,srv.response.app_pose.theta);
              }
                update_goal_pose_old = update_goal_pose;
                ROS_INFO("Asign a new goal");
          }
      }else{
          ROS_ERROR("Failed to call service approaching pose");
      }

      counter_ = counter;
      updateLandmarks();
      move_base_msgs::MoveBaseGoal next_goal;
      geometry_msgs::Pose goal_pose;
      goal_pose = landmarks[counter];
      goalPoseVisualize(goal_pose, 1);
      std_msgs::Int8 goal_ind;
      goal_ind.data = counter;
      goal_index_pub.publish(goal_ind);

      //we'll send a goal to the robot to move 1 meter forward
      //goal.target_pose.header.frame_id = "base_link"; // use "base_link" this one for local map and "map" for global map
      next_goal.target_pose.header.frame_id = "map";
      next_goal.target_pose.header.stamp = ros::Time::now();
      next_goal.target_pose.pose = goal_pose;

      ROS_INFO("Sending goal [%d] (x, y, w) = (%f, %f, %f)",counter,next_goal.target_pose.pose.position.x,next_goal.target_pose.pose.position.y, next_goal.target_pose.pose.orientation.w);
      action_goal.sendGoal(next_goal);
      if((counter==1)||(counter==15)){// moving human
          bool flag = action_goal.waitForResult(ros::Duration(1.0));
          ROS_INFO("Wait for result: %d", flag);
          //if((flag)||(next_goal.target_pose.pose.position.x<0.0)){
          if(flag){
            subgoal_flag = false;
            action_goal.cancelGoal();
          }else{
            subgoal_flag = true;
          }
      }else{
        action_goal.waitForResult();
        subgoal_flag = false;
      }
      if(action_goal.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("The robot aproached the goal [%d]",counter);
      }
      else
      {
        ROS_INFO("The robot failed to move to the goal for some reason");
      }
      /* Select the next landmark*/
      //ros::Duration(1).sleep();// sleep 2 second
      //sleepWhenApproachingHumans(6, counter);// sleep 1 second
      if(!subgoal_flag)
        counter++;

      sleepWhenApproachingHumans(10, counter);// sleep 1 second
  }

  return 0;
}

