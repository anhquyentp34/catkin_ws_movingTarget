#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
//#include <gazebo_plugins/gazebo_ros_camera.h>
//#include <gazebo_ros/PhysicsConfig.h>
//#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <tf/tf.h>
#include <math.h>
// people_msgs
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
//
#include <iostream>
//
ros::Publisher model_state_pub, human_pose_pub;
//
double cur_hx, cur_hy, cur_htheta;

//
void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    // Find the humans state
    tf::Quaternion quat;
    double dummy;
    gazebo_msgs::ModelState human_state;


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
            //ROS_INFO("Robot pose (x,y,theta) = (%f, %f, %f)", cur_rx, cur_ry, cur_rtheta);
            break;
        }
    }
    // people
    int k = 0;
    for(int i =0; i<msg->name.size(); i++)
    {
        if((msg->name[i] == "p0")||(msg->name[i] == "p1")||(msg->name[i] == "p2")||(msg->name[i] == "p3")||(msg->name[i] == "p4")||(msg->name[i] == "p5")
          ||(msg->name[i] == "p6")||(msg->name[i] == "p7")||(msg->name[i] == "p8")||(msg->name[i] == "p9")||(msg->name[i] == "p10")||(msg->name[i] == "p11")
          ||(msg->name[i] == "p12")||(msg->name[i] == "p13")||(msg->name[i] == "p14")||(msg->name[i] == "p15")||(msg->name[i] == "p16"))
        {
             k = k+ 1;
            //
            tf::quaternionMsgToTF(msg->pose[i].orientation, quat);
            tf::Matrix3x3 mat(quat);
            double cur_ptheta;
            mat.getRPY(dummy, dummy, cur_ptheta);
            //ROS_INFO("cur_ptheta_p%d=%f",k,cur_ptheta);
            //
            person.position.x = msg->pose[i].position.x;
            person.position.y = msg->pose[i].position.y;
            person.position.z = 0;
            // moving people
            if((msg->name[i] == "p0")||(msg->name[i] == "p1")){
                person.velocity.x = 0.25*cos(cur_ptheta);
                person.velocity.y = 0.25*sin(cur_ptheta);
                person.velocity.z = 0;
            }
            else{
                person.velocity.x = 0.1*cos(cur_ptheta);
                person.velocity.y = 0.1*sin(cur_ptheta);
                person.velocity.z = 0;
            }

            // sitting people
            if((msg->name[i] == "p8")||(msg->name[i] == "p9")||(msg->name[i] == "p14")){

                person.reliability = 0;
            }
            else{
                person.reliability = 1;
            }
            person.name = "people_info";
            people.people.push_back(person);


        }

        for(int i =0; i<msg->name.size(); i++)
        {
            if(msg->name[i] == "p0")
            {
                //ROS_INFO("P0 pose (x,y,theta) ith %d = (%f, %f, %f)",i, msg->pose[i].position.x,msg->pose[i].position.y,msg->pose[i].position.z);
                human_state.pose.position.x = msg->pose[i].position.x + 0.01;// 0.2 m/s
                //human_state.pose.position.y = msg->pose[i].position.y;
                //geometry_msgs::Pose new_pose.orientation = tf::createQuaternionFromRPY(0, 0, yaw)
                human_state.pose.orientation = msg->pose[i].orientation;
                human_state.reference_frame = "link";
                human_state.model_name = msg->name[i]; //"p0";
                //model_state_pub.publish(human_state);
                break;
            }
        }
    }
    // group people
    int group_people_flag = true;
    if(group_people_flag){
        person.position.x = 4.25;
        person.position.y = 0;
        person.position.z = 0;

        person.velocity.x = 0;
        person.velocity.y = 0;
        person.velocity.z = 0;
        // radius of the human group
        person.reliability = 1.25;
        person.name = "group_people_info";
        people.people.push_back(person);
    }
    // object poeple
    int object_people_flag = true;
    if(object_people_flag){
        person.position.x = -4.8;
        person.position.y = 2.8;
        //person.position.x = -5.8;
        //person.position.y = 1.8;
        person.position.z = 0;

        person.velocity.x = cos(-M_PI_4l);
        person.velocity.y = sin(-M_PI_4l);
        person.velocity.z = 0;
        // distance from human to object
        person.reliability = 2.0;
        person.name = "object_people_info";
        people.people.push_back(person);
    }
    //
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
    model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
    human_pose_pub = nh.advertise<people_msgs::People>("/human_information",5);
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 100, &modelStatesCallback);
    //
    ros::Rate rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
   return 0;
}
