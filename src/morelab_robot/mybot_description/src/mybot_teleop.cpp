#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
class RobotTeleop
{
public:
    RobotTeleop();
    void keyLoop();
    void watchdog();
private:
    ros::NodeHandle nh_,ph_;
    double linear_x, linear_y, angular_z ;
    ros::Time first_publish_;
    ros::Time last_publish_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber pose_sub_, collision_flag ;
    double robot_ox;
    double robot_oz;
    double robot_oy;
    void publish(double, double, double);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    double roll, pitch, yaw;
    boost::mutex publish_mutex_;
    // bool inCollision ;
};
RobotTeleop::RobotTeleop():
    ph_("~"),
    linear_x(0),
    linear_y(0),
    angular_z(0),
    l_scale_(2.0),// 1.0
    a_scale_(2.0) // 1.0
{
    ph_.param("scale_angular", a_scale_, a_scale_);
    ph_.param("scale_linear", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pose_sub_ = nh_.subscribe("/mavros/vision_pose/pose" ,1, &RobotTeleop::poseCallback, this );
    // collision_flag = nh_.subscribe<std_msgs::Bool>("/collision_flag" , 1, &SlaveController::get_inCollision , this);
}
void RobotTeleop::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion q( msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::cout << "pitch" << pitch << std::endl ;
    std::cout << "roll" << roll << std::endl ;
    std::cout << "yaw" << yaw << std::endl ;
    std::cout << "yaw in degrees: " << yaw * 180 / 3.14 << std::endl ;
}
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlebot_teleop");
    RobotTeleop turtlebot_teleop;
    ros::NodeHandle n;
    signal(SIGINT,quit);
    boost::thread my_thread(boost::bind(&RobotTeleop::keyLoop, &turtlebot_teleop));
    ros::Timer timer = n.createTimer(ros::Duration(10.0), boost::bind(&RobotTeleop::watchdog, &turtlebot_teleop));
    ros::spin();
    my_thread.interrupt() ;
    my_thread.join() ;
    return(0);
}
void RobotTeleop::watchdog()
{
    boost::mutex::scoped_lock lock(publish_mutex_);
    if ((ros::Time::now() > last_publish_ + ros::Duration(10.0)) &&
    (ros::Time::now() > first_publish_ + ros::Duration(10.0)))
    publish(0, 0, 0);
}
void RobotTeleop::keyLoop()
{
    char c;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the quadrotor.");
    while (ros::ok())
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        linear_x=linear_y=0;
        ROS_DEBUG("value: 0x%02X\n", c);
        switch(c)
        {
        case KEYCODE_L:
            ROS_DEBUG("LEFT");
            //angular_z = 0.0;
            angular_z = 0.1;
            linear_x = 0.0;
            linear_y = 0.0;
            //linear_y = 0.1;
            break;
        case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            //angular_z = 0.0;
            angular_z = -0.1;
            linear_x = 0.0;
            linear_y = 0.0;
            //linear_y = -0.1;
            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            linear_y = 0.0;
            angular_z = 0.0;
            linear_x = 0.1;
            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            linear_x = -0.1;
            linear_y = 0.0;
            angular_z = 0.0;
            break;
        case KEYCODE_Q:
            ROS_DEBUG("Emergancy");
            linear_x = 0.0;
            linear_y = 0.0;
            angular_z = 0.0;
            break;
        }
        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) {
            first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        publish(linear_y, linear_x, angular_z);
    }
    return;
}
void RobotTeleop::publish(double linear_y, double linear_x , double angular_z)
{
    geometry_msgs::Twist vel;
    vel.linear.x = linear_x*cos(yaw ) - linear_y * sin(yaw) ;
    vel.linear.y = linear_x*sin (yaw) + linear_y * cos(yaw) ;
    vel.angular.z = angular_z;
    std::cout << "vel.linear.x = " << vel.linear.x << std::endl ;
    std::cout << "vel.linear.y = " << vel.linear.y<< std::endl ;
    std::cout << "vel.angular.z = " << angular_z<< std::endl ;
    vel_pub_.publish(vel);
    return;
}
