#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <bits/stdc++.h>
#include <boost/mpl/push_front.hpp>
#include <curl/curl.h>

// include ros libraries
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// for global path planner interface
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <set>

using namespace std;
using std::string;
using namespace Eigen;

// Node structure
struct node
{

    int cx;         // x coordinate
    int cy;         // y coordinate
    int id;         // node id
    float CostNode; // cost node
    int flag;       // @0 : qnearest
                    // @1 : qnear;
                    // @2 : start/goal
                    // @-1: POI
                    // @-2: multiPoi
};

// Point Of Interest structure
struct poi
{

    int x;         // x coordinate
    int y;         // y coordinate
    string nameId; // POI name
};

class RRTstar : public nav_core::BaseGlobalPlanner
{

public:
    // Image dimension
    int rows;
    int cols;

    // Half bounding box dimension
    int dimBox;

    //   vector<vector<int>> corners;    // Corners array

    vector<node> nodes;        // Nodes array
    int n_nodes = 0;           // Goal and start node numbers
    vector<node> nodes_parent; // Nodes parent information

    vector<node> nodes_final;
    vector<node> nodes_parent_final;

    vector<int> goalsPriority; // Priority vector
    vector<vector<int>> multiGoals;

    float cost_final;

    node qstart;         // qstart node
    vector<node> Qgoals; // goal nodes

    vector<poi> POIs; // POIs array

    vector<vector<node>> multiPOIs; // Multiple POIs vector

    vector<float> CostToCome;

    bool block = false; // Boolean variable used to stop the execution in case of problem

    bool plotting = true; // Boolean variable used to enable plotting abilitazione del plottaggio delle immagini

    // Ideal angle values
    int minAngle = 30;
    int maxAngle = 60;

    double eta = 80.0; // Distance value used in Steer (default: 100)

    int samples;

    int maxSamples = 500; //200 5000;					// Maximum samples number

    int w_length = 1; // Arcs length weight

    int w_degRot = 1; // Amplitute rotation weight

    int w_numRot = 1; // Number rotation weight

    int numRot; // Number rotation counter

    bool initialized_;
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;
    float originX;
    float originY;
    float resolution;
    double step_size_, min_dist_from_robot_;
    int width;
    int height;

    int value;
    int mapSize;
    vector<vector<bool>> occupancyGridMap;

    bool printTime;
    double start_global, end_global;

    string frame_id_;
    ros::Publisher plan_pub_;

    RRTstar(); // Initializes the planner attributes with default values

    RRTstar(std::string name, costmap_2d::Costmap2DROS *costmap_ros); // It is used to initialize the costmap, that is the map that will be used for planning (costmap_ros), and the name of the planner (name).

    /////////////////////////////////////////  OVERRIDDEN CLASSESfrom interface nav_core::BaseGlobalPlanner /////////////////////////////////////////////////////////////////////////////////

    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros); //  initializes the costmap, that is the map that will be used for planning (costmap_ros), and the name of the planner (name).

    bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan); //

    ///////////////////////////////////////// RRTstar FUNCTIONS ///////////////////////////////////////////////////////////////////

    node sampleFree(); // Samples in free space

    vector<node> nearest(node qrand); // Finds the nearest node (belonging to the tree) of the sample found

    node steer(node qnearest, node qrand); // Checks if the sample and the nearest node are closer than eta, otherwise moves the sample closer to the nearest

    vector<node> near(node qnew, float r); // Looks for other nodes around the sample

    vector<int> findPath(node start, node dest, float *c); // Provides the final path

    ///////////////////////////////////////// COST FUNCTIONS  ///////////////////////////////////////////////////////////////////

    node nodesCost(node q1); // Calculates the cost of the node based on the safety of it

    float costCurve(Vector2d a, Vector2d b); // Calculates the amplitude of the rotation

    float costMin(node q1, node q2); // Calculates the total cost to move from q1 to q2

    void compareCost(vector<node> Qnear, node qnew, node qnearest); // Based on the cost, update the tree

    ////////////////////////////////////////////////////// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////////////

    bool isValid(int x, int y); // Checks if the node is on an obstacle or outside the image

    float dist(float x1, float y1, float x2, float y2); // Calculates the distance between two point

    void addNode(int x, int y, int SoG); // Saves start and goals nodes in global variables

    // void assign_dimBox();                                       // Based on the image information it sets a dimbox value

    void addPOIs(int x, int y, string nameId); // Adds POIs in a global variable

    void checkPOIs(); // Checks if there are multiple POIs and update Qgoals and multiPOIs

    vector<node> convert(vector<int> goals); // Convert id into node

    vector<vector<int>> findMultiPOIs(vector<int> aG); // Provides a matrix containing the possible combination of multiPOIs

    vector<vector<int>> comb(); // Calculates possible combinations

    bool checkValid(vector<int> c, vector<vector<int>> C); // Checks if the found combination is already found or not

    vector<int> update(vector<int> c); // In the case the combination was already found it is updated

    vector<int> sortGoals(vector<int> g, int n); // Sorts goal nodes based on the given priority (It's necessary to set a priority value for each goal)

    void findPermutations(vector<int> a, int n); // int a[]     // Calculates the possible permutations

    int factorial(int n); // Calculates the factorial for the number n

    vector<int> algorithmManager(); // Manages the presence of multiPOIs and/or priorities

    bool obstacleFree(node q1, node q2, int d); // Checks if two nodes can be linked safetly

    float totalLen(vector<int> path); // Calculates the total length of the final path

    int encode(int x, int y); // Based on x, y coordinate it founds a different node id

    int numRotations(vector<int> p); // Calculates the number of rotation of the chosen path

    void coordinate(vector<int> p); // Calculates the coordinates of the final path

    bool isCoordinateInBounds(float x, float y); // Check if the position is inside the map

    bool isFree(int i, int j); // Check if the position is free

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path); // Plot the path on rviz
};