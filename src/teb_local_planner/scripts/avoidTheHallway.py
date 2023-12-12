#!/usr/bin/env python
import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from teb_local_planner.msg import FeedbackMsg
from matplotlib import pylab

import numpy
from std_msgs.msg import String, Float64, Float64MultiArray
from threading import Thread
import threading
from animated_marker_msgs.msg import AnimatedMarkerArray
from animated_marker_tutorial import createAnimatedPersonMarker

def feedback_pose_obstacle(data):
    flag = 0
#    print("data.data[7] = " + str(data.data[7]))
#    print("velocity_obstacle = " + str(data.data[1]))
    publish_obstacle_msg(data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5],data.data[6],data.data[7],
    data.data[8],data.data[9],data.data[10],data.data[11])

t = 2.0
t1 = 0
t2 = 0
t3 = 0
k_transition_step = 0
def publish_obstacle_msg(obstacle_yl,obstacle_yr,obstacle_frequency,transition_step, orientP_x, orientP_y,
multiple_obstacles, global_set_origin,obstacle_pause,global_two_obstacles, reverse_o,reset_origin):
  global flag
  global t,t1,t2,t3,k_transition_step
  flag = 0
  if (k_transition_step == 0):
      k_transition_step = 1
      transition_step = 0

  vel_x = orientP_x
  vel_y = orientP_y
  range_y = 6.0
  yaw0 = math.atan2(vel_y, vel_x)
  q = tf.transformations.quaternion_from_euler(0,0,yaw0)

  obstacle_msg = ObstacleArrayMsg()
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
  if (global_set_origin == 1):
      obstacle_yl = 0.1
      obstacle_yr = -0.1
  # Add point obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[0].id = 40
  obstacle_msg.obstacles[0].polygon.points = [Point32()]
  obstacle_msg.obstacles[0].polygon.points[0].x = 0
  obstacle_msg.obstacles[0].polygon.points[0].y = -2.5
  obstacle_msg.obstacles[0].polygon.points[0].z = 0
  obstacle_msg.obstacles[0].orientation = Quaternion(*q)

  # Add line 1 obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[1].id = 50
  line_start1 = Point32()
  line_start1.x = -5
  line_start1.y = 1
  line_end1 = Point32()
  line_end1.x = -1
  line_end1.y = 1
  #line_end.y = -4
  obstacle_msg.obstacles[1].polygon.points = [line_start1, line_end1]

  # Add line 2 obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[2].id = 60
  line_start2 = Point32()
  line_start2.x = -5
  line_start2.y = -1
  line_end2 = Point32()
  line_end2.x = -1
  line_end2.y = -1
  #line_end.y = -4
  obstacle_msg.obstacles[2].polygon.points = [line_start2, line_end2]

  # Add line 3 obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[3].id = 70
  line_start3 = Point32()
  line_start3.x = 1
  line_start3.y = 1
  line_end3 = Point32()
  line_end3.x = 5
  line_end3.y = 1
  obstacle_msg.obstacles[3].polygon.points = [line_start3, line_end3]
  # Add line 4 obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[4].id = 80
  line_start4 = Point32()
  line_start4.x = 1
  line_start4.y = -1
  line_end4 = Point32()
  line_end4.x = 5
  line_end4.y = -1
  obstacle_msg.obstacles[4].polygon.points = [line_start4, line_end4]
  # Add line 5 obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[5].id = 80
  line_start5 = Point32()
  line_start5.x = -1
  line_start5.y = 3
  line_end5 = Point32()
  line_end5.x = -1
  line_end5.y = 0.95
  obstacle_msg.obstacles[5].polygon.points = [line_start5, line_end5]
  # Add line 6 obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[6].id = 90
  line_start6 = Point32()
  line_start6.x = 1
  line_start6.y = 3
  line_end6 = Point32()
  line_end6.x = 1
  line_end6.y = 0.95
  obstacle_msg.obstacles[6].polygon.points = [line_start6, line_end6]
  # Add line 7 obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[7].id = 100
  line_start7 = Point32()
  line_start7.x = -1
  line_start7.y = -0.95
  line_end7 = Point32()
  line_end7.x = -1
  line_end7.y = -3
  obstacle_msg.obstacles[7].polygon.points = [line_start7, line_end7]
  # Add line 8 obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[8].id = 110
  line_start8 = Point32()
  line_start8.x = 1
  line_start8.y = -0.95
  line_end8 = Point32()
  line_end8.x = 1
  line_end8.y = -3
  obstacle_msg.obstacles[8].polygon.points = [line_start8, line_end8]

  r = rospy.Rate(obstacle_frequency) # 10hz
  while not rospy.is_shutdown():
    if (flag == 1):
        return
    if (obstacle_pause == 1):
        return

    if (reverse_o == 0):
        if (reset_origin == 1):
            t1 = 0
        t3 = 0
        obstacle_msg.obstacles[0].polygon.points[0].y = -2.5 + t1
        t1 = t1 + transition_step
        if (t1 > 5):
            t1 = 5
            t2 = t2 + 1
        if (t2 > 10):
            t1 = 0
            t2 = 0

    else:
        if (reset_origin == 1):
            t3 = 0
        t1 = 0
        obstacle_msg.obstacles[0].polygon.points[0].y = 2.5 + t3
        t3 = t3 - transition_step
        if (t3 < -5):
            t3 = -5
            t2 = t2 + 1
        if (t2 > 10):
            t3 = 0
            t2 = 0
    # Publish animations
    markerArray = AnimatedMarkerArray()
    x00 = obstacle_msg.obstacles[0].polygon.points[0].x
    y00 = obstacle_msg.obstacles[0].polygon.points[0].y
    markerArray.markers.append( createAnimatedPersonMarker(0, pos=(x00,y00), thetaDeg=yaw0*180.0/math.pi + 90,  animationSpeed=0.3, color=(0.15, 0.9, 0.9)) )

    pub_animation.publish(markerArray)

    delta_t = 1 / obstacle_frequency
    vel_h_x = transition_step / delta_t
    obstacle_msg.obstacles[0].velocities.twist.linear.x = vel_h_x
    obstacle_msg.obstacles[0].velocities.twist.linear.y = 0
    obstacle_msg.obstacles[0].velocities.twist.linear.z = 0

    print("obstacle_msg.obstacles[0].polygon.points[0].y = " + str(obstacle_msg.obstacles[0].polygon.points[0].y))
    print("vh = " + str(vel_h_x))
    pub.publish(obstacle_msg)

    r.sleep()
    flag = 1


def listener():
    global pub
    global pub_animation
    global abc
    rospy.init_node("test_obstacle_msg")

    #subscriber
    topic_pose_obstacle = "/test_optim_node/Public_pose_obstacle"
    rospy.Subscriber(topic_pose_obstacle, Float64MultiArray, feedback_pose_obstacle, queue_size = 10)
    #publishs
    pub = rospy.Publisher('/test_optim_node/obstacles', ObstacleArrayMsg, queue_size=1)
    pub_animation = rospy.Publisher("/test_optim_node/animated_markers", AnimatedMarkerArray, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
