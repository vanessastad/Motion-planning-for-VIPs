
#ifndef SIMPLE_LOCAL_PLANNER_ROS_H_
#define SIMPLE_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>

// time
#include <time.h>

//files
#include <fstream>
#include <iostream>

// msgs
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes  TODO do I need?
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// other
#include <array>
#include <vector>

// definitions
#define PI 3.14159265
#define D2R 0.0174532925 // = 3.14159265/180

using namespace std;

struct pos
{
  double x, y, az;
};

class SimplePlannerROS : public nav_core::BaseLocalPlanner
{

public:
  SimplePlannerROS(); //Default constructor

  SimplePlannerROS(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros); // Constructs the ros wrapper

  ~SimplePlannerROS();


  /////////////////////////////////////////  OVERRIDDEN CLASSES from interface nav_core::BaseLocalPlanner /////////////////////////////////////////////////////////////////////////////////

  void initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros); // Initializes the local planner

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan); // Set the plan the controller is following; also reset Simple-planner

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel); // Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base

  bool isGoalReached(); // Check if the goal pose has been achieved

private:
  //Pointer to external objects (do NOT delete object)
  costmap_2d::Costmap2DROS *costmap_ros_;
  tf::TransformListener *tf_;

  // Topics & Services
  ros::Subscriber amcl_sub;
  ros::Publisher path_pub;

  // Data
  pos now;    // present frame
  pos next;   // next frame
  pos nError; // error between present and next frames
  double distance;
  int length;                                   // number of frames in the global plan
  int count;                                    // keeps track of the number for the next frame in the global plan
  std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
  geometry_msgs::Twist cmd;                     // contains the velocity

  visualization_msgs::Marker points;
  ofstream file;
  bool hasStarted;

  double average;
  int num;
  double average2, average3;
  int num2, howmanytimes;
  int p, hmt;
  double minus;
  double haha, haha2;
  double beforee;

  //measuring
  double stopTime, startTime;
  double beginning2, ending2;
  bool firstTime, number1;
  double pathLength;
  double four;

  // Flags
  bool goal_reached_;
  bool initialized_;

  ///////////////////////////////////////// SETTING FUNCTIONS ///////////////////////////////////////////////////////////////////

  void setVel(); // Function that sets linear speed

  void setRot(); // Function that sets angular speed

  void setVelZ(); // Function that sets linear and angular speed to zero

  void setNowError(); // Calculates the error between the next and present frame

  void setNext(); // Uses count to set the next goal frame

  ///////////////////////////////////////// AUXILIAR FUNCTIONS ///////////////////////////////////////////////////////////////////

  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg); // Function called whenever a new amcl msg is published on that topic

  double getYaw(geometry_msgs::PoseWithCovarianceStamped msg); // Calculates the Yaw angle from the position message that amcl sent

  void pathVisualization();
};

#endif