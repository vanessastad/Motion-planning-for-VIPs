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
    int flag;       // @0 : node
                    // @1 : start/goal;
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

// Path structure (individual)
struct path
{
    vector<node> path; // id of nodes
    float score;       // Fitness score
};

// Graph structure
struct graph
{

    vector<int> seq;           // step necessary to find a path between start and goal
    vector<node> id_son;       // son nodes
    vector<vector<node>> id_p; // partent nodes
};

class GA : public nav_core::BaseGlobalPlanner
{

public:
    // Image dimension
    int rows;
    int cols;

    // Image scale: 1 [px] = scale [cm]

    // Half bounding box dimension
    int dimBox;

    vector<node> nodes; // Nodes array
    int n_nodes;        // Goal and start node numbers

    vector<node> nodes_final; // This vector contains the information of the nodes belonging to the final path

    vector<int> goalsPriority; // Priority vector
    vector<vector<int>> multiGoals;

    float cost_final;

    node qstart;         // qstart node
    vector<node> Qgoals; // goal nodes

    graph Graph; // Graph object

    vector<poi> POIs; // POIs array

    vector<vector<node>> multiPOIs; // Multiple POIs vector

    bool block; // Boolean variable used to stop the execution in case of problem

    bool plotting = true; // Boolean variable used to enable plotting abilitazione del plottaggio delle immagini

    // Ideal angle values
    int minAngle = 30;
    int maxAngle = 60;

    int w_length = 1; //1; // Arcs length weight

    int w_degRot = 1; // Amplitute rotation weight

    int w_numRot = 1; // Number rotation weight

    int numRot; // Number rotation counter

    int Max = 0; // Variable used to normalize the distance in the cost function

    float probability; // crossover/mutation probability

    int TotGenerations = 100; // Number of generation we want to create before to interrupt the research

    int numIndividuals = 80; // Numer of individuals in each generation

    int m = 2; // 5 Maximum number of connection for each node

    float finalScore; // Final path fitness score

    bool error;

    vector<int> firstPath;
    vector<node> firstPath_nodes;

    vector<int> firstPath_final;
    vector<node> firstPath_nodes_final;

    bool initialized_;
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;

    float originX;
    float originY;
    float resolution;
    double step_size_, min_dist_from_robot_;
    //base_local_planner::WorldModel* world_model_;
    int width;
    int height;

    int mapSize;
    vector<vector<bool>> occupancyGridMap;

    int numGenerations;

    bool printTime;
    double start_global, end_global;

    string frame_id_;
    ros::Publisher plan_pub_;

    GA(); // Initializes the planner attributes with default values

    GA(std::string name, costmap_2d::Costmap2DROS *costmap_ros); // It is used to initialize the costmap, that is the map that will be used for planning (costmap_ros), and the name of the planner (name).

    /////////////////////////////////////////  OVERRIDDEN CLASSESfrom interface nav_core::BaseGlobalPlanner /////////////////////////////////////////////////////////////////////////////////

    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros); //  initializes the costmap, that is the map that will be used for planning (costmap_ros), and the name of the planner (name).

    bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan); //

    ///////////////////////////////////////// GA FUNCTIONS ///////////////////////////////////////////////////////////////////

    vector<path> initialPopulation(node start, node goal, int numIndividuals); // This function create the first generation

    graph createGraph(node q1, node q2); // This function builds a graph based on which we extract different individuals of our first generation

    vector<node> sortDist(vector<node> last_added_n, int lim, node qold, node q); // It sorts nodes from the closest to the further respect to q and check if q and those nodes are obstacleFee

    node randomNode(); // It provide a random valid node

    node findFurther(node q1, node q2); // It provides the further valid node respect to qold on the line q-qold

    vector<path> fitness(vector<path> population); // A score is attributed to each path based on the cost of the path

    vector<path> Sort(vector<path> population); // The paths are sorted based on the fitness score

    vector<path> memory(vector<path> sortedPopulation, vector<path> pathParents, int sizeOffspring); // It saves paths from the old generation to bring them in the new one

    vector<path> select(vector<path> population); // Selection of paths to use as parents in crossovermutation()

    vector<path> crossoverMutation(vector<path> pathParents, int numPopulation); // Based on a selector it executed crossover and/or mutation

    vector<int> findPath(node start, node dest, float *c); // A path is found executing the GA steps

    ///////////////////////////////////////// COST FUNCTIONS  ///////////////////////////////////////////////////////////////////

    node nodesCost(node q1); // Calculates the cost of the node based on the safety of it

    float costCurve(Vector2d a, Vector2d b); // Calculates the amplitude of the rotation

    float costMin(vector<node> p); // Calculates the total cost of a path

    ////////////////////////////////////////////////////// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////////////

    bool isValid(int x, int y); // Checks if the node is on an obstacle or outside the image

    float dist(int x1, int y1, int x2, int y2); // Calculates the distance between two point

    void max_dist(vector<path> population); // It provides the maximization term Max

    void addNode(int x, int y, int SoG); // Saves start and goals nodes in global variables

    void addPOIs(int x, int y, string nameId); // Adds POIs in a global variable

    void checkPOIs(); // Checks if there are multiple POIs and update Qgoals and multiPOIs

    vector<node> convert(vector<int> goals); // Convert id into node

    vector<node> convertReturn(vector<float> vect);

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