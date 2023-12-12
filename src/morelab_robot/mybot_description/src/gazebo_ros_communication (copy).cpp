#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/tf.h>
// people_msgs
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
//
#include <iostream>
//
ros::Publisher model_states_pub, human_pose_pub;
//
double cur_hx, cur_hy, cur_htheta;
//
void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    // Find the humans state
    tf::Quaternion quat;
    double dummy;
    gazebo_msgs::ModelState & human_state;
    human_state = msg;

    people_msgs::People people;
    people_msgs::Person person;
    // robot pose
    double cur_rx, cur_ry, cur_rtheta;
    for(int i =0; i<msg->name.size(); i++)
    {
        if(msg->name[i] == "robot_model")
        {
            tf::quaternionMsgToTF(msg->pose[i].orientation, quat);
            tf::Matrix3x3 mat(quat);
            mat.getRPY(dummy, dummy, cur_rtheta);
            cur_rx = msg->pose[i].position.x;
            cur_ry = msg->pose[i].position.y;
            ROS_INFO("Robot pose (x,y,theta) = (%f, %f, %f)", cur_rx, cur_ry, cur_rtheta);
        }
    }
    //
    int k = 0;
    for(int i =0; i<msg->name.size(); i++)
    {
        if((msg->name[i] == "p0")||(msg->name[i] == "p1")||(msg->name[i] == "p2")||(msg->name[i] == "p3")||(msg->name[i] == "p4")||(msg->name[i] == "p5")
          ||(msg->name[i] == "p6")||(msg->name[i] == "p7")||(msg->name[i] == "p8")||(msg->name[i] == "p9")||(msg->name[i] == "p10")||(msg->name[i] == "p11"))
        {
            k = k+ 1;
            person.position.x = msg->pose[i].position.x;
            person.position.y = msg->pose[i].position.y;
            person.position.z = 0;

            person.velocity.x = 0;
            person.velocity.y = 0;
            person.velocity.z = 0;

            person.reliability = 0.5;
            people.people.push_back(person);
            if(msg->name[i] == "p0"){
                ROS_INFO("P0 pose (x,y,theta) = (%f, %f, %f)",person.position.x,person.position.y,person.position.y);
            }
        }
    }
    people.header.frame_id = "map"; // global map
    //people.header.seq = human_body.header.seq;
    //people.header.stamp = human_body.header.stamp;
    // pulish
    human_pose_pub.publish(people);
}
//
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "gazebo_ros_communication_node");
    ros::NodeHandle nh;
    model_states_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
    human_pose_pub = nh.advertise<people_msgs::People>("/human_information",5);
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 1, &modelStatesCallback);
    //
    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
   return 0;
}
