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
def publish_obstacle_msg(obstacle_yl,obstacle_yr,obstacle_frequency,transition_step, orientP_x, orientP_y,
multiple_obstacles, global_set_origin,obstacle_pause,global_two_obstacles, reverse_o,reset_origin):
  global flag
  global t,t1,t2,t3
  flag = 0

  # Add point obstacle 1
  y_0 = -3.0
  vel_x = orientP_x
  vel_y = orientP_y
  range_y = 6.0
  yaw0 = math.atan2(vel_y, vel_x)
  q = tf.transformations.quaternion_from_euler(0,0,yaw0)

#  print("multiple_obstacles = " + str(multiple_obstacles))
#  print("obstacle_pause = " + str(obstacle_pause))

  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
  if (global_set_origin == 1):
      obstacle_yl = 0.1
      obstacle_yr = -0.1
  # Add point obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[0].id = 10
  obstacle_msg.obstacles[0].polygon.points = [Point32()]
  obstacle_msg.obstacles[0].polygon.points[0].x = 0
  obstacle_msg.obstacles[0].polygon.points[0].y = obstacle_yl
  obstacle_msg.obstacles[0].polygon.points[0].z = 0
  obstacle_msg.obstacles[0].orientation = Quaternion(*q)

  if (multiple_obstacles == 1):
      obstacle_msg.obstacles.append(ObstacleMsg())
      obstacle_msg.obstacles[1].id = 1
      obstacle_msg.obstacles[1].polygon.points = [Point32()]
      obstacle_msg.obstacles[1].polygon.points[0].x = 0
      obstacle_msg.obstacles[1].polygon.points[0].y = obstacle_yr
      obstacle_msg.obstacles[1].polygon.points[0].z = 0
      obstacle_msg.obstacles[1].orientation = Quaternion(*q)

      if (global_two_obstacles == 1.0):
          obstacle_msg.obstacles.append(ObstacleMsg())
          obstacle_msg.obstacles[2].id = 2
          obstacle_msg.obstacles[2].polygon.points = [Point32()]
          obstacle_msg.obstacles[2].polygon.points[0].x = 0 #(obstacle_xl + obstacle_xr) / 2
          obstacle_msg.obstacles[2].polygon.points[0].y = (obstacle_yl + obstacle_yr) / 2
          obstacle_msg.obstacles[2].polygon.points[0].z = 0
          obstacle_msg.obstacles[2].orientation = Quaternion(*q)

  r = rospy.Rate(obstacle_frequency) # 10hz
  print("global_two_obstacles = " + str(global_two_obstacles))
  while not rospy.is_shutdown():
    if (flag == 1):
        return
    if (obstacle_pause == 1):
        return
    # Vary y component of the point obstacle
#    obstacle_msg.obstacles[0].polygon.points[0].x = 3 * math.sin(math.pi * t)

#    t = t + transition_step
    if (reverse_o == 0):
        if (reset_origin == 1):
            t1 = 0
        t3 = 0
        obstacle_msg.obstacles[0].polygon.points[0].x = -2.5 + t1
#        obstacle_msg.obstacles[0].velocities.twist.linear.x = 2.234
        if (multiple_obstacles == 1):
            obstacle_msg.obstacles[1].polygon.points[0].x = -2.5 + t1
            if (global_two_obstacles == 1.0):
                obstacle_msg.obstacles[2].polygon.points[0].y = (obstacle_msg.obstacles[0].polygon.points[0].y + obstacle_msg.obstacles[1].polygon.points[0].y) / 2
                obstacle_msg.obstacles[2].polygon.points[0].x = -2.5 + t1
        t1 = t1 + transition_step
        print("t1 = " + str(t1))
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
        obstacle_msg.obstacles[0].polygon.points[0].x = 2.5 + t3
#        obstacle_msg.obstacles[0].velocities.twist.linear.x = 1.234
        if (multiple_obstacles == 1):
            obstacle_msg.obstacles[1].polygon.points[0].x = 2.5 + t3
            if (global_two_obstacles == 1.0):
                obstacle_msg.obstacles[2].polygon.points[0].y = (obstacle_msg.obstacles[0].polygon.points[0].y + obstacle_msg.obstacles[1].polygon.points[0].y) / 2
                obstacle_msg.obstacles[2].polygon.points[0].x = 2.5 + t3
        t3 = t3 - transition_step
        print("t3 = " + str(t3))
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
    markerArray.markers.append( createAnimatedPersonMarker(0, pos=(x00,y00), thetaDeg=yaw0*180.0/math.pi,  animationSpeed=0.3, color=(0.15, 0.9, 0.9)) )
    if (multiple_obstacles == 1):
        x01 = obstacle_msg.obstacles[1].polygon.points[0].x
        y01 = obstacle_msg.obstacles[1].polygon.points[0].y
        markerArray.markers.append( createAnimatedPersonMarker(1, pos=(x01,y01), thetaDeg=yaw0*180.0/math.pi, animationSpeed=0.3, color=(0.9, 0.9, 0.15)) )

#        x02 = obstacle_msg.obstacles[2].polygon.points[0].x
#        y02 = obstacle_msg.obstacles[2].polygon.points[0].y
#        markerArray.markers.append( createAnimatedPersonMarker(2, pos=(x02,y02), thetaDeg=yaw0*180.0/math.pi, animationSpeed=0.3, color=(0.3, 0.3, 0.15)) )
    pub_animation.publish(markerArray)

    delta_t = 1/obstacle_frequency
    vel_h_x = transition_step / delta_t
    obstacle_msg.obstacles[0].velocities.twist.linear.x = vel_h_x
    print("vel_h_x = " + str(vel_h_x))

    pub.publish(obstacle_msg)

    r.sleep()
    flag = 1


def listener():
    global pub
    global pub_animation
    global abc
    rospy.init_node("test_obstacle_msg")
    #publishs
    pub = rospy.Publisher('/test_optim_node/obstacles', ObstacleArrayMsg, queue_size=1)
    pub_animation = rospy.Publisher("/test_optim_node/animated_markers", AnimatedMarkerArray, queue_size=1)
    #subscriber
    topic_pose_obstacle = "/test_optim_node/Public_pose_obstacle"
    rospy.Subscriber(topic_pose_obstacle, Float64MultiArray, feedback_pose_obstacle, queue_size = 10)

    rospy.spin()


if __name__ == '__main__':
#    t2 = threading.Thread(target=sigmoidPlt, args=(getdata,))
#    t2.start()
    listener()


#    try:


#  except rospy.ROSInterruptException:
#    pass
