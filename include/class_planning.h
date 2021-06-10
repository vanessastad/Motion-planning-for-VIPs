#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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
#include "ros/ros.h"
#include <curl/curl.h>
#include <algorithm>

#define MAX_ADJ 40          // Maximum number of adjacencies for each node

using namespace cv;
using namespace std;
using namespace Eigen;

// Structure with characteristic parameters useful in Dijkstra algorithm
struct param{
    
    int id;                 // node identifier  
    float CostTot;          // node cost-to-come
    float heuristicCost;    // herusistic cost
    int parent;             // parent id 
    
};

// Node structure
struct node{
       
    int cx;             // x coordinate
    int cy;             // y coordinate
    int lx;             // x-lenght cell
    int ly;             // x-lenght cell
    float CostNode;     // cost node
    int vertex[8];
    
    int flag;           // @0 : cell barycenter node;
                        // @1 : cell side node;
                        // @2 : special point (start/goal)
};

// POI structure (Point Of Interest)
struct poi{
       
    int x;             // x coordinate
    int y;             // y coordinate
    string nameId;     // POI name

};


// Each coordinate is expressed in pixel

class class_planning{

    public:
             
        char* name;                 // Location variable fot the image we want to load 
                
        Mat src, src_gray, clast, subImage;   // Image boc
        // Image dimension
        int rows;
        int cols; 
        
        // This variable is used to choose the kind of adjacents
        // 0 := horizontal and vertical
        // 1 := diagonal 
        // 2 := between each node connectable with a segment dimBox-width
        int moreAdj;
        
        // Image scale: 1 [px] = scale [cm]
        int scala; 
        // Half bounding box dimension
        int dimBox;
        
        // Detection parameters
        int blockSize = 2;
        int apertureSize = 3;
        double k = 0.1;
        int thresh = 151;
        int max_thresh = 255;

        char* source_window = "Source image";
        char* corners_window = "Corners detected";

        vector<vector<int>> corners;    // Corners array
        int n_cor;                      // Vertex number
        
        vector<vector<int>> vert_poly;  // Each line contains vertex coordinates (each polygon has to have 4 vertexes)
        int n_poly;                     // Number of polygons
        
        vector<node> nodes;             // Nodes array
        int n_nodes;                    // Number of nodes

        // start and goal nodes are insert at the end of the vector
        
        int start;                      //  start id
        vector<int> goals;              //  goal id 
        vector<int> goalsPriority;      // Priorities vector
        vector<vector<int>> multiGoals;
        
        bool POIs = false;              // It is true if there are POIs data         
        vector<poi> POIS;               // POIs vector
        vector<int> multiPOIs;          // Vector use to sotre multiPOIs
                                        // Codific: 1° element says how many times a POI is repeated ( n+1 times), other elements list
                                        // indexes of this repetition;
                                        // from n+2 position there can be a new repeated POI  
        
        vector<vector<int>> adj;        // Adjacencies vector
        MatrixXf A;                     // Adjacencies matrix
        
        vector<vector<int>> neighbor;   // Neighbor (cell) vector
        
        vector<vector<float>> cost;     // Cost vector
        
        bool block = false;             // Boolean used to stop the execution in case of problem in reaching the objective
        
        bool plotting;                  // Boolean used to enable plotting
        
        int flag = 2;                   // Cells division type: @0:=vertical; @1:=horizontal; @2:=double
        


        float cost_final;


        // Ideal angle values
        int minAngle;                   
        int maxAngle;
        
        

        int w_length;                   // Arcs length weight

        int w_degRot;                   // Amplitute rotation weight

        int w_numRot;                   // Number rotation weight 

        int numRot = 0;                 // Number rotation counter 
        
        

        float Max;                      // Value used to normalized length arc cost

    class_planning(){}
    
    
    
    /////////////////////////////////////////  INITIALIZATION FUNCTION  /////////////////////////////////////////////////////////////////////////////////
    
    void init(ros::NodeHandle node);                                // Parameters initialization through launch file
    

    /////////////////////////////////////////  CORNERS DETECTION FUNCTIONS  ///////////////////////////////////////////////////////////////////

    vector<vector<int>> cornerHarris_fun();                         // Corner detection
    
    vector<vector<int>> deleteCorners(vector<vector<int>> c);       // Delete useless corners 

    void correctCorners(vector<vector<int>> aux);                   // Corner detection improvement 
    
    void trackLines();                                              // Tracking line manager 

    void trackLinesV();                                             // Vertical tracking
    
    void trackLinesO();                                             // Horizontal tracking
    
    
    /////////////////////////////// CELL AND NODES FUNCTIONS ////////////////////////////////////////////////
        
    void poly2node();                                               // Polygon barycenter becomes a graph node
    
    void edge2node();                                               // The center of each side of cells becomes a graph node
    
    void adjacencies();                                             // Adjacencies calculation

    void moreAdjacencies();                                         // Extension of adjacencies to each connectable nodes

    vector<int> reMerge(vector<int> ind, int minDim);               // Small cell merge
        
    void divideCell(vector<int> ind, int maxDim, vector<int> f, int* nV); // Big cell division
    
    void archsCost();                                               // Length arc cost
        
    void nodesCost();                                               // Safety node cost, it depends on how close the node is to an obstacle

    void addNode(int x, int y, int SoG);                            // Special node/POI node addition
    
    //////////////////////////////////////////////////////// DIJKSTRA FUNCTIONS  ///////////////////////////////////////////////////////////////////////////////
    
    vector<int> algorithmManager();                                 // Manage the graph research based on the presence of multiPOIs and/or priorities
    
    vector<int> Dijkstra(int start, int dest, int* c);              // Dijkstra algorithm
    
    //////////////////////////////////////////////////// TRAJECTORY RESEARCH /////////////////////////////////////////////////////////////////////

    
    bool saveTrajectorySubImage(vector<int> p);

    int totalLen(vector<int> path);                                 // Path length
    

    ////////////////////////////////////////////////////// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////////////
    
    void loadPOIs();                                                // POIs load 
    
    void assign_dimBox();                                           // Based on the image information it sets a dimbox value

    float dist(int x1, int y1, int x2, int y2);                     // Calculates the distance between two point

    int factorial(int n);                                           // Calculates the factorial for number n
        
    bool AlreadyAdd(int ni, int *n);                                // Checks if the node is already in the list pointed from "n"
    
    bool notAlreadyAdd(int x, int y, vector<node> n);               // Checks if the node is already added
    
    bool isValid(int x, int y);                                     // Controls points inside the image
    
    bool isValidControll(int ind, int adjMax, int indControll);     // Check node  "indControll" is not already in the adjacencies of "ind"
    
    int inglobed(int ind, vector<int> indControll, vector<int> ing);  // Check if the cell is already inglobed
    
    bool isComparableCell(int ind1, int ind2, int c);               // Check if the cell can inglobe another cell
    
    bool isBlocked(int cell);                                       // Check if the cell is connected or not
      
    void adjMatGenerate();                                          // Adjacency matrix
    
    float CostCurve(Vector2d a, Vector2d b);                        // Calculates the amplitude cost of the rotation 

    int minCost(vector<param> p, vector<int> a);                    // It search the node id with the smallest cost in the active nodes set
    
    int deleteActived(int ind, vector<int> a);                      // The node is deleted from the active nodes set 
    
    bool notInClose(int index, vector<int> c_l);                    // Checks that the index node is not in the closed list 
   
    bool areConnectable(int x1, int y1, int x2, int y2, int d);     // Checks that two point are connectable with a segment dimBox-width
    
    int calculateOverlap(int p1, int p2, int m1, int m2, int* l);   // Calculates the overlapping length of two sides
    
    bool isNotInOpen(vector<int> o, int ind);                       // Checks if the ind node is in the open set
    
    void findVertex(int x, int y, int lx, int ly, int ind,  int* v); // Looks for cell vertexes 
    
    void findDatesCells(int x, int y, int lx, int ly, int ind, int* v); // Looks for data cell
        
    void checkVertex(int n1, int n2, int c);                        // Checks vertexes new cell after merge
    
    int checkPixel(int x, int y, bool vertical, int BW);            // Looks for delimitation pixel of cell
    
    void addPOIs(string c, int p);                                  // Points Of Interest loading
    
    vector<int> selectPOIs(string c);                               // POI node selection
  
    void sortGoals(int n);                                          // Sorts goals based on priority
        
    void findPermutations(int a[], int n);                          // Calculates permutations
        
    vector<vector<int>> findMultiPOIs(int ng);                      // Looks for multiPOIs to find the best one to reach 


    ////////////////////////////////////////////////////// PLOTTING FUNCTIONS /////////////////////////////////////////////////////////////////////////////
    
            
    void drawNodes();                                               // Nodes plotting

    void drawPath(vector<int> p);                                   // Path plotting 

    
}; 


