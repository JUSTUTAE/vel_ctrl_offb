#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cmath>
#include <queue>
#include <mutex>
#include <numeric>
#include <experimental/filesystem>

#include <nav_msgs/Path.h> 
#include <geometry_msgs/PointStamped.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

double kp=0.7; //ku=1.4
double ki=0.4; //0.4
double kd=0.0;

















