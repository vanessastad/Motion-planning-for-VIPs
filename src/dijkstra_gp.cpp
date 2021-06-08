#include "dijkstra_gp.h" // "MyNavigation/dijkstra_gp.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(class_planning, nav_core::BaseGlobalPlanner)

/////////////////////////////////////////  INITIALIZATION FUNCTION  /////////////////////////////////////////////////////////////////////////////////

void class_planning::init(ros::NodeHandle node)
{
    // Parameters initialization from launch file

    node.getParam("/plotting", plotting); // Enable/disable plotting
    node.getParam("/minAngle", minAngle); // Smallest desired angle
    node.getParam("/maxAngle", maxAngle); // Biggest desired angle
    node.getParam("/moreAdj", moreAdj);   // Adjacency type
    node.getParam("/w_length", w_length); // Arc cost weight
    node.getParam("/w_degRot", w_degRot); // Amplitude rotation weight
    node.getParam("/w_numRot", w_numRot); // Number of rotation weight
}

/////////////////////////////////////////  OVERRIDDEN CLASSES from interface nav_core::BaseGlobalPlanner  /////////////////////////////////////////////////////////////////////////////////

void class_planning::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{

    if (!initialized_)
    {

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);

        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();
        resolution = costmap_->getResolution();
        mapSize = width * height;
        float tBreak = 1 + 1 / (mapSize);

        ROS_INFO("Dijkstra planner initialized successfully");
        initialized_ = true;
    }

    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}

bool class_planning::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{

    int startPoint[2] = {start.pose.position.x, start.pose.position.y}; // Start node initialization 2200, 200
    int goalPoint[2] = {goal.pose.position.x, goal.pose.position.y};    // Goal node initialization 200, 1000
    vector<vector<int>> corners;

    //   ros::init(int arcs, char **argv, "makePlan");
    ros::NodeHandle planning;

    name_map = "/home/federica/blind_ws/src/my_navigation/src/map/my_map"; // vanecomplex5 //PianoSeminterrato5

    if (!initialized_)
    {
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    plan.clear();
    init(planning);

    assign_dimBox();
    corners = cornerHarris_fun(); // Corner detection
    correctCorners(corners);      // Corner correction

    loadPOIs();

    // Line tracking
    trackLines();

    n_poly = connectedComponents(src, clast) - 1;

    // Barycenter of each poligon is added to node vector
    poly2node();

    adjacencies();

    // Barycenter side of each poligon is added to node vector
    edge2node();

    // Arcs cost calculation
    archsCost();

    // Node cost calculation based on the distance from obstacles
    nodesCost();

    if (isValid(startPoint[0], startPoint[1]) && isValid(goalPoint[0], goalPoint[1]))
    {

        addNode(startPoint[0], startPoint[1], 0);
        addNode(goalPoint[0], goalPoint[1], 1);

        if (!block)
        {

            vector<int> path;
            path = algorithmManager();

            if (totalLen(path) == 0)
            {
                ROS_WARN("Error: it is not possible to find a path");
                return false;
            }
            else
            {
                for (int i = 0; i < path.size(); i++)
                {
                    for (int j = 0; j < nodes_final.size(); j++)
                    {
                        if (path[i] == nodes_final[j].id)
                        {
                            geometry_msgs::PoseStamped pose = goal;
                            pose.pose.position.x = nodes_final[j].cx;
                            pose.pose.position.y = nodes_final[j].cy;
                            pose.pose.position.z = 0.0;
                            pose.pose.orientation.x = 0.0;
                            pose.pose.orientation.y = 0.0;
                            pose.pose.orientation.z = 0.0;
                            pose.pose.orientation.w = 1.0;
                            plan.push_back(pose);
                        }
                    }
                }
            }
        }

        waitKey(0);

        return true;
    }
    else
        return false;
}

/////////////////////////////////////////  CORNERS DETECTION FUNCTIONS  ///////////////////////////////////////////////////////////////////

vector<vector<int>> class_planning::cornerHarris_fun()
{

    string file;
    file.append(name_map);
    file.append(".png");
    src = imread(file);
    imshow(source_window, src);

    // Image dimension set
    rows = src.rows;
    cols = src.cols;
    cvtColor(src, src_gray, CV_RGB2GRAY);                  // Grey scale transformation
    threshold(src_gray, src_gray, 10, 255, THRESH_BINARY); // Binary transformation
    src_gray.convertTo(src, CV_8UC1);                      // 8 bit uchar conversion

    Mat dst, dst_norm;
    dst = Mat::zeros(src.size(), CV_32FC1);
    vector<vector<int>> corner_tmp;

    // Corner Harris detection
    cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);

    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());

    for (int j = 0; j < dst_norm.rows; j++)
    {
        for (int i = 0; i < dst_norm.cols; i++)
        {
            if ((int)dst_norm.at<float>(j, i) > thresh)
            { // Detected Corner if over threshold
                corner_tmp.push_back({i, j});
            }
        }
    }

    return corner_tmp; // The result will be improved in another function
}

vector<vector<int>> class_planning::deleteCorners(vector<vector<int>> c)
{
    // Too close corners or marked corners are deleted

    float tol = 3.0;
    int n = c.size();
    vector<vector<int>> aux;
    bool flag = true;
    float d;
    for (int i = 0; i < n; i++)
    {
        if (c[i][0] != -1 && c[i][1] != -1)
        {
            for (int j = i + 1; j < n; j++)
            {
                d = dist(c[i][0], c[i][1], c[j][0], c[j][1]);
                if (d < tol)
                { // Two corners are too close
                    flag = false;
                    j = n;
                }
            }
            if (flag)
            {
                aux.push_back({c[i][0], c[i][1]});
            }
            flag = true;
        }
    }
    return aux;
}

void class_planning::correctCorners(vector<vector<int>> aux)
{

    n_cor = corners.size();
    int dim = 6;
    int x = -1;
    int y = -1;

    for (int i = 0; i < n_cor; i++)
    {

        // Maximum close pixel number to check without go out from the image
        dim = min(abs(corners[i][1]), dim);
        dim = min(abs(corners[i][0]), dim);
        dim = min(abs(corners[i][1] - rows), dim);
        dim = min(abs(corners[i][0] - cols), dim);

        // A submatrix centered in the detected corner is monitored to individuate the real corner improving the detected corner
        for (int j = 0; j < dim; j++)
        {
            for (int h = 0; h < dim; h++)
            {

                // Sub-diagonal matrix research
                if ((int(src.at<uchar>(j + corners[i][1], h + corners[i][0]))) == 0)
                {
                    if ((int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0]))) == 255 &&
                        (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] + 1))) == 255 &&
                        (int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0] + 1))) == 255)
                    {
                        y = j + corners[i][1];
                        x = h + corners[i][0];
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0]))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] + 1))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0] + 1))) == 255)
                    {
                        y = j + corners[i][1];
                        x = h + corners[i][0];
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0]))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] - 1))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0] - 1))) == 255)
                    {
                        y = j + corners[i][1];
                        x = h + corners[i][0];
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0]))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] - 1))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0] - 1))) == 255)
                    {
                        y = j + corners[i][1];
                        x = h + corners[i][0];
                        goto newcorner;
                    }
                }
                else
                {
                    if ((int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0]))) == 0 &&
                        (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] + 1))) == 0 &&
                        (int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0] + 1))) == 0)
                    {
                        y = j + corners[i][1] + 1;
                        x = h + corners[i][0] + 1;
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0]))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] + 1))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0] + 1))) == 0)
                    {
                        y = j + corners[i][1] - 1;
                        x = h + corners[i][0] + 1;
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0]))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] - 1))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0] - 1))) == 0)
                    {
                        y = j + corners[i][1] - 1;
                        x = h + corners[i][0] - 1;
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0]))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] - 1))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0] - 1))) == 0)
                    {
                        y = j + corners[i][1] + 1;
                        x = h + corners[i][0] - 1;
                        goto newcorner;
                    }
                }
            }
        }

        for (int j = 0; j > -dim; j--)
        {
            for (int h = 0; h > -dim; h--)
            {

                // Over-diagonal matrix research
                if ((int(src.at<uchar>(j + corners[i][1], h + corners[i][0]))) == 0)
                {
                    if ((int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0]))) == 255 &&
                        (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] + 1))) == 255 &&
                        (int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0] + 1))) == 255)
                    {
                        y = j + corners[i][1];
                        x = h + corners[i][0];
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0]))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] + 1))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0] + 1))) == 255)
                    {
                        y = j + corners[i][1];
                        x = h + corners[i][0];
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0]))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] - 1))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0] - 1))) == 255)
                    {
                        y = j + corners[i][1];
                        x = h + corners[i][0];
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0]))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] - 1))) == 255 &&
                             (int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0] - 1))) == 255)
                    {
                        y = j + corners[i][1];
                        x = h + corners[i][0];
                        goto newcorner;
                    }
                }
                else
                {
                    if ((int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0]))) == 0 &&
                        (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] + 1))) == 0 &&
                        (int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0] + 1))) == 0)
                    {
                        y = j + corners[i][1] + 1;
                        x = h + corners[i][0] + 1;
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0]))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] + 1))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0] + 1))) == 0)
                    {
                        y = j + corners[i][1] - 1;
                        x = h + corners[i][0] + 1;
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0]))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] - 1))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1] - 1, h + corners[i][0] - 1))) == 0)
                    {
                        y = j + corners[i][1] - 1;
                        x = h + corners[i][0] - 1;
                        goto newcorner;
                    }
                    else if ((int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0]))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1], h + corners[i][0] - 1))) == 0 &&
                             (int(src.at<uchar>(j + corners[i][1] + 1, h + corners[i][0] - 1))) == 0)
                    {
                        y = j + corners[i][1] + 1;
                        x = h + corners[i][0] - 1;
                        goto newcorner;
                    }
                }
            }
        }

    newcorner:
        // Corner modification
        if (x != -1)
        {
            corners[i][0] = x;
            corners[i][1] = y;
        }

        // Default conditions to delete the undetected corner
        x = -1;
        y = -1;
        dim = 6;
    }

    vector<vector<int>> corners_tmp;
    corners_tmp = corners;
    corners.clear();

    corners = deleteCorners(corners_tmp);
}

void class_planning::trackLines()
{

    // Tracking lines manager

    n_cor = corners.size();
    switch (flag)
    {
    case 0:
        trackLinesV();
        break;
    case 1:
        trackLinesO();
        break;
    case 2:
        trackLinesV();
        trackLinesO();
        break;
    }

    if (plotting)
    {
        imshow(source_window, src);
    }
}

void class_planning::trackLinesV()
{

    // Vertical tracking lines

    int start_x, start_y;
    int index = 0;
    vector<vector<int>> aux;

    for (int i = 0; i < n_cor; i++)
    {

        start_x = corners[i][1];
        start_y = corners[i][0];

        if (start_x - 1 >= 0)
        {
            // If there is not an obstacle, the area on the top of the corner is inspected looking for the end of the side delimiting the cell
            // More pixels are controlled together to avoid small errors of corner positioning
            for (int j = start_x - 1; j > -1; j--)
            {
                if (src_gray.at<uchar>(j, start_y) == 0)
                {
                    // The loop stops if the vertex is found
                    j = -1;
                }
                else
                {
                    src.at<uchar>(j, start_y) = 0;
                }
            }
        }

        if (start_x + 1 < rows)
        {
            // If there is not an obstacle, the area on the bottom of the corner is inspected looking for the end of the side delimiting the cell
            // More pixels are controlled together to avoid small errors of corner positioning
            for (int j = start_x + 1; j < rows; j++)
            {
                if (src_gray.at<uchar>(j, start_y) == 0)
                {
                    j = rows;
                }
                else
                {
                    src.at<uchar>(j, start_y) = 0;
                }
            }
        }
    }

    // Delimitation lines are tracked on the image bounderies
    for (int i = 0; i < cols; i++)
    {
        src.at<uchar>(0, i) = 0;
        src.at<uchar>(rows - 1, i) = 0;
    }
}

void class_planning::trackLinesO()
{

    // Horizontal tracking lines

    int start_x, start_y;
    int index = 0;
    vector<vector<int>> aux;

    for (int i = 0; i < n_cor; i++)
    {

        start_x = corners[i][1];
        start_y = corners[i][0];

        if (start_y - 1 >= 0)
        {
            // If there is not an obstacle, the area on the left of the corner is inspected looking for the end of the side delimiting the cell
            // More pixels are controlled together to avoid small errors of corner positioning
            for (int j = start_y - 1; j > -1; j--)
            {
                if (src_gray.at<uchar>(start_x, j) == 0)
                {
                    j = -1;
                }
                else
                {
                    src.at<uchar>(start_x, j) = 0;
                }
            }
        }

        if (start_y + 1 < cols)
        {
            // If there is not an obstacle, the area on the right of the corner is inspected looking for the end of the side delimiting the cell
            // More pixels are controlled together to avoid small errors of corner positioning
            for (int j = start_y + 1; j < cols; j++)
            {
                if (src_gray.at<uchar>(start_x, j) == 0)
                {
                    j = cols;
                }
                else
                {
                    src.at<uchar>(start_x, j) = 0;
                }
            }
        }
    }

    // Delimitation lines are tracked on the image bounderies
    for (int i = 0; i < rows; i++)
    {
        src.at<uchar>(i, 0) = 0;
        src.at<uchar>(i, cols - 1) = 0;
    }
}

/////////////////////////////// CELL AND NODES FUNCTIONS ////////////////////////////////////////////////

void class_planning::poly2node()
{

    // Barycenter of each poligon is added to node vector

    int tmp_node[n_poly][3] = {0};
    int tmp_firstPixel[n_poly][2] = {0};
    int x, y, lx, ly;
    int tmp_vertex[8];
    int label;

    // Scrolls the clasted image
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            label = clast.at<int>(i, j);
            if (label != 0)
            { // If label = 0 there is an obstacle, otherwise update
                tmp_node[label - 1][1] = tmp_node[label - 1][1] + i;
                tmp_node[label - 1][2] = tmp_node[label - 1][2] + j;
                tmp_node[label - 1][0]++; // Number of pixel in a polygon
                if (tmp_firstPixel[label - 1][0] == 0)
                {
                    tmp_firstPixel[label - 1][0] = i;
                    tmp_firstPixel[label - 1][1] = j;
                }
            }
        }
    }

    int minDim = (int)(3) * dimBox;

    int count = 0;
    vector<int> indChangeCell;
    vector<int> flagChangeCell;
    int tmpFlag;
    int tmpLMax;

    // A node is set in the center of the cell (xg, yg)
    // xg = sum(AreaXi)/numPixelLabel; yg = sum(AreaYi)/numPixelLabel

    for (int i = 0; i < n_poly; i++)
    {
        x = (int)(tmp_node[i][2] / tmp_node[i][0] + 0.5);
        y = (int)(tmp_node[i][1] / tmp_node[i][0] + 0.5);
        lx = abs(tmp_firstPixel[i][1] - x) * 2 + 1;
        ly = abs(tmp_firstPixel[i][0] - y) * 2 + 1;
        findVertex(x, y, lx, ly, i, &tmp_vertex[0]);
        nodes.push_back({x, y, lx, ly, 0, tmp_vertex[0], tmp_vertex[1], tmp_vertex[2], tmp_vertex[3],
                         tmp_vertex[4], tmp_vertex[5], tmp_vertex[6], tmp_vertex[7], 0});
        if (lx <= minDim || ly <= minDim)
        {
            indChangeCell.push_back({i}); // Small cells are stored
        }
    }

    n_nodes = nodes.size();

    indChangeCell = reMerge(indChangeCell, minDim); // Merged cell index

    for (int i = 0; i < indChangeCell.size(); i++)
    {

        // Delete index of nonexistent cells
        nodes.erase(nodes.begin() + indChangeCell[i]);
        count = count + 1;
    }
    n_nodes = nodes.size();
    indChangeCell.clear();

    int maxDim = (2) * minDim; // 6*dimBox

    // Cell split
    for (int i = 0; i < n_nodes; i++)
    {
        tmpFlag = 0;
        if (nodes[i].lx > maxDim)
        {
            // If x length is too long a vertical line is tracked
            tmpFlag = 1;
        }
        if (nodes[i].ly > maxDim)
        {
            // If y length is too long a horizontal line is tracked
            tmpFlag = tmpFlag + 2;
        }
        if (tmpFlag != 0)
        {
            flagChangeCell.push_back({tmpFlag});
            indChangeCell.push_back({i});
        }
    }

    n_poly = connectedComponents(src, clast) - 1;

    count = 0;
    divideCell(indChangeCell, maxDim, flagChangeCell, &count);
    n_nodes = nodes.size();

    n_poly = connectedComponents(src, clast) - 1;

    vector<node> nodesTMP;
    nodesTMP = nodes;

    nodes.clear();

    int mySort[n_nodes - count] = {-1};
    int mySortVertex[count];
    int indSort;
    int indSortVertex = 0;
    int maxInd = 0;
    for (int i = 0; i < n_nodes; i++)
    {
        indSort = clast.at<int>(nodesTMP[i].cy, nodesTMP[i].cx) - 1;
        if (indSort != -1)
        {
            mySort[indSort] = i;
            if (indSort > maxInd)
                maxInd = indSort;
        }
        else
        {
            mySortVertex[indSortVertex] = i;
            indSortVertex = indSortVertex + 1;
        }
    }

    for (int i = 0; i <= maxInd; i++)
    {
        if (mySort[i] != -1)
            nodes.push_back(nodesTMP[mySort[i]]);
    }

    n_nodes = nodes.size();

    nodes.clear();
    int tmp_node2[n_poly][3] = {0};
    int tmp_firstPixel2[n_poly][2] = {0};
    int x2, y2, lx2, ly2;
    int tmp_vertex2[8];
    int label2;

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            label2 = clast.at<int>(i, j);
            if (label2 != 0)
            {
                tmp_node2[label2 - 1][1] = tmp_node2[label2 - 1][1] + i;
                tmp_node2[label2 - 1][2] = tmp_node2[label2 - 1][2] + j;
                tmp_node2[label2 - 1][0]++;
                if (tmp_firstPixel2[label2 - 1][0] == 0)
                {
                    tmp_firstPixel2[label2 - 1][0] = i;
                    tmp_firstPixel2[label2 - 1][1] = j;
                }
            }
        }
    }

    for (int i = 0; i < n_poly; i++)
    {
        x2 = (int)(tmp_node2[i][2] / tmp_node2[i][0] + 0.5);
        y2 = (int)(tmp_node2[i][1] / tmp_node2[i][0] + 0.5);
        lx2 = abs(tmp_firstPixel2[i][1] - x2);
        ly2 = abs(tmp_firstPixel2[i][0] - y2);
        findDatesCells(x2, y2, lx2, ly2, i + 1, &tmp_vertex2[0]);
        lx2 = tmp_vertex2[2] - tmp_vertex2[0];
        ly2 = tmp_vertex2[5] - tmp_vertex2[1];
        nodes.push_back({x2, y2, lx2, ly2, 0, tmp_vertex2[0], tmp_vertex2[1], tmp_vertex2[2], tmp_vertex2[3],
                         tmp_vertex2[4], tmp_vertex2[5], tmp_vertex2[6], tmp_vertex2[7], 0});
    }
}

void class_planning::edge2node()
{

    // Side cell nodes research

    n_nodes = nodes.size();

    int i, j, h;
    bool save_new = false;
    int nx, ny, l;
    int tmp1, tmp2;
    int vertical; // 1 means the side is verical, 0 horizontal
    int xp1, yp1, xm1, ym1;
    int xp2, yp2, xm2, ym2;
    int tol = 5;
    bool f1 = false;
    bool f2 = false;
    int adj1, adj2;

    int index1, index2, index3, dim, n_old;
    n_old = n_nodes;

    vector<vector<int>> adj_tmp, adj_aux;
    vector<node> nodes_tmp, nodes_aux;

    for (i = 0; i < n_nodes; i++)
    {

        xm1 = nodes[i].vertex[0];
        ym1 = nodes[i].vertex[1];
        xp1 = nodes[i].vertex[6];
        yp1 = nodes[i].vertex[7];

        for (j = 0; j < neighbor[i].size(); j++)
        {

            xm2 = nodes[neighbor[i][j]].vertex[0]; // j node is close to i node
            ym2 = nodes[neighbor[i][j]].vertex[1];
            xp2 = nodes[neighbor[i][j]].vertex[6];
            yp2 = nodes[neighbor[i][j]].vertex[7];

            // Node position is calculated
            for (h = 0; h <= tol; h++)
            {

                // There are not information about cell adiacency. We looks for a common coordinate and then the shared side

                // Left proximity
                if (xp1 == xm2 - h)
                { // h is a tolerance interval
                    nx = xp1;
                    vertical = 1;
                    ny = calculateOverlap(yp1, yp2, ym1, ym2, &l);

                    if ((nx - tol) > 0 && (nx + tol) < cols)
                    { // It's inside the image
                        f1 = false;
                        f2 = false;
                        for (int g = -tol; g <= tol; g++)
                        {
                            if (!f1)
                                if (clast.at<int>(ny, nx - g) - 1 == i)
                                {
                                    f1 = true;
                                    tmp1 = nx - g;
                                }
                            if (!f2)
                                if (clast.at<int>(ny, nx + g) - 1 == neighbor[i][j])
                                {
                                    f2 = true;
                                    tmp2 = nx + g;
                                }
                            if (f1 & f2)
                            {
                                nx = (tmp1 + tmp2) / 2;
                                save_new = true;
                                goto save;
                            }
                        }
                    }
                }

                // Right proximity
                if (xm1 == xp2 + h)
                {
                    nx = xm1;
                    ny = calculateOverlap(yp1, yp2, ym1, ym2, &l);
                    vertical = 1;

                    if ((nx - tol) > 0 && (nx + tol) < cols)
                    {
                        f1 = false;
                        f2 = false;
                        for (int g = -tol; g <= tol; g++)
                        {
                            if (!f1)
                                if (clast.at<int>(ny, nx + g) - 1 == i)
                                {
                                    f1 = true;
                                    tmp1 = nx + g;
                                }
                            if (!f2)
                                if (clast.at<int>(ny, nx - g) - 1 == neighbor[i][j])
                                {
                                    f2 = true;
                                    tmp2 = nx - g;
                                }
                            if (f1 & f2)
                            {
                                nx = (tmp1 + tmp2) / 2;
                                save_new = true;
                                goto save;
                            }
                        }
                    }
                }

                // Upper proximity
                if (yp1 == ym2 - h)
                {
                    nx = calculateOverlap(xp1, xp2, xm1, xm2, &l);
                    ny = yp1;
                    vertical = 0;

                    if ((ny - tol) > 0 && (ny + tol) < rows)
                    {
                        f1 = false;
                        f2 = false;
                        for (int g = -tol; g <= tol; g++)
                        {
                            if (!f1)
                                if (clast.at<int>(ny - g, nx) - 1 == i)
                                {
                                    f1 = true;
                                    tmp1 = ny - g;
                                }
                            if (!f2)
                                if (clast.at<int>(ny + g, nx) - 1 == neighbor[i][j])
                                {
                                    f2 = true;
                                    tmp2 = ny + g;
                                }
                            if (f1 && f2)
                            {
                                ny = (tmp1 + tmp2) / 2;
                                save_new = true;
                                goto save;
                            }
                        }
                    }
                }

                // Bottom proximity
                if (ym1 == yp2 + h)
                {
                    nx = calculateOverlap(xp1, xp2, xm1, xm2, &l);
                    ny = ym1;
                    vertical = 0;

                    if ((ny - tol) > 0 && (ny + tol) < rows)
                    {
                        f1 = false;
                        f2 = false;
                        for (int g = -tol; g <= tol; g++)
                        {
                            if (!f1)
                                if (clast.at<int>(ny + g, nx) - 1 == i)
                                {
                                    f1 = true;
                                    tmp1 = ny + g;
                                }
                            if (!f2)
                                if (clast.at<int>(ny - g, nx) - 1 == neighbor[i][j])
                                {
                                    f2 = true;
                                    tmp2 = ny - g;
                                }
                            if (f1 && f2)
                            {
                                ny = (tmp1 + tmp2) / 2;
                                save_new = true;
                                goto save;
                            }
                        }
                    }
                }
            }
        save:

            if (save_new)
            {
                for (int it = 0; it < 10; it++)
                {
                    if (int(src_gray.at<uchar>(ny, nx)) != 0)
                    {
                        goto saveOk;
                    }
                    else
                    {
                        if (it == 0)
                        {
                            nx = nx - (1 - vertical) * l;
                            ny = ny - vertical * l;
                        }
                        else
                        {
                            nx = nx + (1 - vertical) * l / 4;
                            ny = ny + vertical * l / 4;
                        }
                    }
                }
                goto notSave;

            saveOk:

                // The new node is added if connectable to both nodes sharing the same side. If it doesn't happen, a fake
                // node is added in the insersection of two perpendicular straight lines passing through the connectabl nodes of the segment

                f1 = false;
                f2 = false;

                if (!areConnectable(nodes[neighbor[i][j]].cx, nodes[neighbor[i][j]].cy, nx, ny, 1))
                {
                    f1 = true;
                }
                if (!areConnectable(nx, ny, nodes[i].cx, nodes[i].cy, 1))
                {
                    f2 = true;
                }

                if (!f1 && !f2)
                {
                    // If the node is connectable to both the other nodes, it is added and the adjacency is updated

                    nodes_tmp.push_back({nx, ny, 2 * vertical * l, 2 * (1 - vertical) * l, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}); // Coordinates, dimentions, Vertex, weight, flag (1 = side node)
                    adj_tmp.push_back({neighbor[i][j], i});
                }
                else if (notAlreadyAdd(nx, ny, nodes_aux))
                {

                    // Otherwise the fake node is added
                    if (f1)
                    {
                        nodes_aux.push_back({nx, ny, 2 * vertical * l, 2 * (1 - vertical) * l, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1});
                        nodes_aux.push_back({nx * (1 - vertical) + nodes[neighbor[i][j]].cx * vertical,
                                             ny * vertical + nodes[neighbor[i][j]].cy * (1 - vertical),
                                             1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1});
                        adj1 = n_old + nodes_aux.size() - 1;

                        adj_aux.push_back({adj1, i});

                        adj[i].push_back({adj1 - 1});

                        int sizeTMP = adj[neighbor[i][j]].size();
                        vector<int> adjTMP;
                        for (int g = 0; g < sizeTMP; g++)
                        {
                            int ind = adj[neighbor[i][j]][g];
                            if (ind < n_old)
                            {
                                adjTMP.push_back({ind});
                                adj[ind].push_back({adj1});
                            }
                        }

                        adjTMP.push_back({adj1 - 1});
                        adjTMP.push_back({neighbor[i][j]});
                        adj_aux.push_back(adjTMP);

                        adj[neighbor[i][j]].resize(sizeTMP + 1);
                        adj[neighbor[i][j]][sizeTMP] = adj1;
                    }
                    else if (f2)
                    {
                        nodes_aux.push_back({nx, ny, 2 * vertical * l, 2 * (1 - vertical) * l, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1});
                        nodes_aux.push_back({nx * (1 - vertical) + nodes[i].cx * vertical,
                                             ny * vertical + nodes[i].cy * (1 - vertical),
                                             1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1});
                        adj2 = n_old + nodes_aux.size() - 1;

                        int sizeTMP = adj[neighbor[i][j]].size();

                        adj_aux.push_back({adj2, neighbor[i][j]});

                        adj[neighbor[i][j]].push_back({adj2 - 1});

                        sizeTMP = adj[i].size();
                        vector<int> adjTMP;
                        for (int g = 0; g < sizeTMP; g++)
                        {
                            int ind = adj[i][g];
                            if (ind < n_old)
                            {
                                adjTMP.push_back({ind});
                                adj[ind].push_back({adj2});
                            }
                        }
                        adjTMP.push_back({adj2 - 1});
                        adjTMP.push_back({i});
                        adj_aux.push_back(adjTMP);

                        adj[i].resize(sizeTMP + 1);
                        adj[i][sizeTMP] = adj2;
                    }
                }

            notSave:
                save_new = false;
            }
        }
    }

    if (nodes_aux.size() != 0)
    {
        for (i = 0; i < nodes_aux.size(); i++)
        {
            nodes.push_back(nodes_aux[i]);
            adj.push_back(adj_aux[i]);
        }
    }

    // Double node check
    vector<int> nodesOK;
    for (i = 0; i < nodes_tmp.size(); i++)
    {
        for (j = i + 1; j < nodes_tmp.size(); j++)
        {
            if (((adj_tmp[i][0] == adj_tmp[j][0] && adj_tmp[i][1] == adj_tmp[j][1]) ||
                 (adj_tmp[i][0] == adj_tmp[j][1] && adj_tmp[i][1] == adj_tmp[j][0])))
            {
                nodesOK.push_back(i);
            }
        }
    }

    n_old = nodes.size();
    index3 = nodes.size();

    // Global variables update
    for (i = 0; i < nodesOK.size(); i++)
    {

        index1 = nodesOK[i];
        nodes.push_back(nodes_tmp[index1]);
        adj.push_back(adj_tmp[index1]);

        index2 = adj_tmp[index1][0];
        dim = adj[index2].size();
        adj[index2].resize(dim + 1);
        adj[index2][dim] = index3;

        index2 = adj_tmp[index1][1];
        dim = adj[index2].size();
        adj[index2].resize(dim + 1);
        adj[index2][dim] = index3;

        index3 = index3 + 1;
    }

    n_nodes = nodes.size();

    for (i = n_old; i < n_nodes; i++)
    {
        for (j = 0; j < 2; j++)
        {
            index1 = adj[i][j];
            for (h = 0; h < adj[index1].size(); h++)
            {
                index2 = adj[index1][h];
                if (nodes[index2].flag == 1)
                {
                    index3 = adj[i].size();
                    if (isValidControll(i, index3, index2))
                    {
                        if (areConnectable(nodes[i].cx, nodes[i].cy, nodes[index2].cx, nodes[index2].cy, dimBox))
                        {
                            adj[i].resize(index3 + 1);
                            adj[i][index3] = index2;
                            usleep(10);
                            index3 = adj[index2].size();
                            adj[index2].resize(index3 + 1);
                            adj[index2][index3] = i;
                            usleep(10);
                        }
                    }
                }
            }
        }
    }
}

void class_planning::adjacencies()
{

    // This function looks for adjacencies

    int tmp_neighborMatrix[n_poly][MAX_ADJ];

    for (int i = 0; i < n_poly; i++)
    {
        tmp_neighborMatrix[i][0] = 0;
        for (int j = 1; j < MAX_ADJ; j++)
        {
            tmp_neighborMatrix[i][j] = -1;
        }
    }

    int local_flag = 0;
    int last_label;

    int startDist = 4; // Frame of research width
    int cx;
    int cy;
    int lx;
    int ly;
    int distMaxX1;
    int distMaxX2;
    int distMaxY1;
    int distMaxY2;
    int j, h, g;
    float m;

    int moreAdj_local = 4 * int(moreAdj == 0); // Additional diagonal adjacency

    int label;

    for (int i = 0; i < n_poly; i++)
    {
        cx = nodes[i].cx;
        cy = nodes[i].cy;
        lx = nodes[i].lx / 2;
        ly = nodes[i].ly / 2;

        distMaxX1 = startDist;
        distMaxX2 = startDist;
        distMaxY1 = startDist;
        distMaxY2 = startDist;

        // Check to not go out form the memory space of the image
        for (g = 1; g <= startDist; g++)
        {
            if ((cx - lx - distMaxX1) <= 1)
            { // Outside left side
                distMaxX1 = distMaxX1 - 1;
            }
            if ((cy - ly - distMaxY1) <= 1)
            { // Outside upper side
                distMaxY1 = distMaxY1 - 1;
            }
            if ((cx + lx + distMaxX2) >= cols - 1)
            { // Outside right side
                distMaxX2 = distMaxX2 - 1;
            }
            if ((cy + ly + distMaxY2) >= rows - 1)
            { // Outside bottom side
                distMaxY2 = distMaxY2 - 1;
            }
        }

        // Search in the upper rectangle
        for (h = cy - ly - distMaxY1; h < cy - ly - 1; h++)
        {
            for (j = cx - lx - distMaxX1 + moreAdj_local; j <= cx + lx + distMaxX2 - moreAdj_local; j++)
            {
                label = clast.at<int>(h, j);
                if (label != 0)
                {
                    if (label - 1 != i)
                    {
                        if (!AlreadyAdd(label - 1, &tmp_neighborMatrix[i][0]))
                        {
                            if (tmp_neighborMatrix[label - 1][0] < MAX_ADJ - 1)
                            {
                                tmp_neighborMatrix[label - 1][0] = tmp_neighborMatrix[label - 1][0] + 1;
                                tmp_neighborMatrix[label - 1][tmp_neighborMatrix[label - 1][0]] = i;
                            }
                            if (tmp_neighborMatrix[i][0] < MAX_ADJ - 1)
                            {
                                tmp_neighborMatrix[i][0] = tmp_neighborMatrix[i][0] + 1;
                                tmp_neighborMatrix[i][tmp_neighborMatrix[i][0]] = label - 1;
                            }
                        }
                    }
                }
            }
        }

        // Search in the left rectangle
        for (h = cy - ly - 1 + moreAdj_local; h < cy + ly + 1 - moreAdj_local; h++)
        {
            for (j = cx - lx - distMaxX1; j < cx - lx - 1; j++)
            {
                label = clast.at<int>(h, j);
                if (label != 0)
                {
                    if (label - 1 != i)
                    {
                        if (!AlreadyAdd(label - 1, &tmp_neighborMatrix[i][0]))
                        {
                            if (tmp_neighborMatrix[label - 1][0] < MAX_ADJ - 1)
                            {
                                tmp_neighborMatrix[label - 1][0] = tmp_neighborMatrix[label - 1][0] + 1;
                                tmp_neighborMatrix[label - 1][tmp_neighborMatrix[label - 1][0]] = i;
                            }
                            if (tmp_neighborMatrix[i][0] < MAX_ADJ - 1)
                            {
                                tmp_neighborMatrix[i][0] = tmp_neighborMatrix[i][0] + 1;
                                tmp_neighborMatrix[i][tmp_neighborMatrix[i][0]] = label - 1;
                            }
                        }
                    }
                }
            }
        }

        // Search in the right rectangle
        for (h = cy - ly - 1 + moreAdj_local; h < cy + ly + 1 - moreAdj_local; h++)
        {
            for (j = cx + lx + 1; j < cx + lx + distMaxX2; j++)
            {
                label = clast.at<int>(h, j);
                if (label != 0)
                {
                    if (label - 1 != i)
                    {
                        if (!AlreadyAdd(label - 1, &tmp_neighborMatrix[i][0]))
                        {
                            if (tmp_neighborMatrix[label - 1][0] < MAX_ADJ - 1)
                            {
                                tmp_neighborMatrix[label - 1][0] = tmp_neighborMatrix[label - 1][0] + 1;
                                tmp_neighborMatrix[label - 1][tmp_neighborMatrix[label - 1][0]] = i;
                            }
                            if (tmp_neighborMatrix[i][0] < MAX_ADJ - 1)
                            {
                                tmp_neighborMatrix[i][0] = tmp_neighborMatrix[i][0] + 1;
                                tmp_neighborMatrix[i][tmp_neighborMatrix[i][0]] = label - 1;
                            }
                        }
                    }
                }
            }
        }

        // Search in the bottom rectangle
        for (h = cy + ly + 1; h < cy + ly + distMaxY2; h++)
        {
            for (j = cx - lx - distMaxX1 + moreAdj_local; j < cx + lx + distMaxX2 - moreAdj_local; j++)
            {
                label = clast.at<int>(h, j);
                if (label != 0)
                {
                    if (label - 1 != i)
                    {
                        if (!AlreadyAdd(label - 1, &tmp_neighborMatrix[i][0]))
                        {
                            if (tmp_neighborMatrix[label - 1][0] < MAX_ADJ - 1)
                            {
                                tmp_neighborMatrix[label - 1][0] = tmp_neighborMatrix[label - 1][0] + 1;
                                tmp_neighborMatrix[label - 1][tmp_neighborMatrix[label - 1][0]] = i;
                            }
                            if (tmp_neighborMatrix[i][0] < MAX_ADJ - 1)
                            {
                                tmp_neighborMatrix[i][0] = tmp_neighborMatrix[i][0] + 1;
                                tmp_neighborMatrix[i][tmp_neighborMatrix[i][0]] = label - 1;
                            }
                        }
                    }
                }
            }
        }
    }

    int nA = 0;
    int nN = 0;

    vector<int> aux;
    vector<int> auxNeighbor;

    int indAdj[MAX_ADJ];

    for (int i = 0; i < n_poly; i++)
    {
        for (int j = 1; j <= tmp_neighborMatrix[i][0]; j++)
        {
            if (areConnectable(nodes[i].cx, nodes[i].cy, nodes[tmp_neighborMatrix[i][j]].cx,
                               nodes[tmp_neighborMatrix[i][j]].cy, dimBox))
            {

                indAdj[nA] = tmp_neighborMatrix[i][j];
                nA = nA + 1;
            }
        }

        aux.resize(nA);
        auxNeighbor.resize(tmp_neighborMatrix[i][0]);

        for (int j = 1; j <= tmp_neighborMatrix[i][0]; j++)
        {
            auxNeighbor[j - 1] = tmp_neighborMatrix[i][j];
        }

        for (int j = 0; j < nA; j++)
        {
            aux[j] = indAdj[j];
        }

        adj.push_back(aux);
        neighbor.push_back(auxNeighbor);
        nA = 0;
        nN = 0;
    }

    if (moreAdj == 2)
    {
        moreAdjacencies();
    }
}

void class_planning::moreAdjacencies()
{

    // Every visible node can be connected to each other if there is a free segment dimBox width

    n_nodes = nodes.size();
    int myDim_tmp, myDim, otherDim1, otherDim2, indNode1, indNode2;

    for (int i = 0; i < n_nodes; i++)
    {
        myDim = adj[i].size();
        myDim_tmp = myDim;
        for (int j = 0; j < myDim_tmp; j++)
        {
            indNode1 = adj[i][j];
            otherDim1 = adj[indNode1].size();
            for (int h = 0; h < otherDim1; h++)
            {
                indNode2 = adj[indNode1][h];
                if (isValidControll(i, myDim, indNode2))
                {
                    if (areConnectable(nodes[i].cx, nodes[i].cy, nodes[indNode2].cx, nodes[indNode2].cy, dimBox))
                    {
                        adj[i].resize(myDim + 1);
                        adj[i][myDim] = indNode2;
                        myDim++;
                        otherDim2 = adj[indNode2].size();
                        adj[indNode2].resize(otherDim2 + 1);
                        adj[indNode2][otherDim2] = i;
                    }
                }
            }
        }
    }
}

vector<int> class_planning::reMerge(vector<int> ind, int minDim)
{

    int i, j, index, indNeighbor;
    int n = ind.size();
    int obs = 0;
    int stop = 5;
    int fixed1 = 0;
    int fixed2 = 0;
    int fixed3 = 0;
    int fixed4 = 0;
    int caseRemerge = 0;
    int start, finish;
    bool borderLine = false;
    int A1, A2;
    vector<int> toDelete;
    vector<int> englobingCell;
    int indEnglobing;

    // Backward scroll of every cell that has to be modified
    for (i = n - 1; i > -1; i--)
    {
        index = ind[i]; // id analyzed cell
        // Check about which dimension has to be modified
        // If x side is too small, a y side will be deleted or vice versa
        if (nodes[index].ly <= minDim)
        {
            goto xLine;
        }
    testY:
        if (nodes[index].lx > minDim)
        {
            // In case of any errors, cell surrounded by obstacle
            goto noDeleteLine;
        }

    yLine:
        obs = 0;
        fixed3 = 0;
        fixed4 = 0;
        borderLine = false;
        caseRemerge = 0; // Error

        fixed1 = nodes[index].cy;
        fixed2 = nodes[index].cx - nodes[index].lx / 2 + 1;
        for (j = 0; j <= stop; j++)
        {
            // Check that there are not two lines in a row and neither an obstacle
            if (fixed2 - j > 1 && obs < 3)
            {
                if (src.at<uchar>(fixed1, fixed2 - j) == 0 && src_gray.at<uchar>(fixed1, fixed2 - j) != 0)
                {
                    obs = obs + 1;
                    if (obs == 1)
                    {
                        // The first line is found
                        fixed3 = fixed2 - j;
                        borderLine = true;
                    }
                    else if (obs == 2)
                    {
                        if (borderLine)
                        {
                            // The second line is found
                            fixed4 = fixed3 - 1;
                        }
                        else
                            j = stop;
                    }
                }
                else
                    borderLine = false;
            }
        }

        // Lines and obstacles are not found
        if (obs > 0 && obs < 3)
        {
            if (fixed4 == 0)
                indNeighbor = clast.at<int>(fixed1, fixed3 - 1) - 1;
            else
                indNeighbor = clast.at<int>(fixed1, fixed4 - 1) - 1;
            indEnglobing = inglobed(indNeighbor, toDelete, englobingCell);
            if (indEnglobing != -1)
                indNeighbor = englobingCell[indEnglobing];
            if (indNeighbor != -1 && indNeighbor != index)
            {
                caseRemerge = 1; // Right check
                goto deleteLineY;
            }
        }

    secondTestY:
        obs = 0;
        fixed2 = nodes[index].cx + nodes[index].lx / 2 - 1;
        fixed3 = 0;
        fixed4 = 0;
        stop = 5;
        for (j = 0; j <= stop; j++)
        {
            if (fixed2 + j < cols - 1 && obs < 3)
            {
                if (src.at<uchar>(fixed1, fixed2 + j) == 0 && src_gray.at<uchar>(fixed1, fixed2 + j) != 0)
                {
                    obs = obs + 1;
                    if (obs == 1)
                    {
                        // // The first line is found
                        fixed3 = fixed2 + j;
                        borderLine = true;
                    }
                    else if (obs == 2)
                    {
                        if (borderLine)
                        {
                            // // The second line is found
                            fixed4 = fixed3 + 1;
                        }
                        else
                            j = stop;
                    }
                }
                else
                    borderLine = false;
            }
        }

        // Lines and obstacles are not found
        if (obs > 0 && obs < 3)
        {
            if (fixed4 <= 0)
                indNeighbor = clast.at<int>(fixed1, fixed3 + 1) - 1;
            else
                indNeighbor = clast.at<int>(fixed1, fixed4 + 1) - 1;
            indEnglobing = inglobed(indNeighbor, toDelete, englobingCell);
            if (indEnglobing != -1)
                indNeighbor = englobingCell[indEnglobing];
            if (indNeighbor != -1 && indNeighbor != index)
            {
                caseRemerge = 2; // Left check
                goto deleteLineY;
            }
            else
            {
                goto noDeleteLine;
            }
        }
        else
        {
            goto noDeleteLine;
        }

    deleteLineY:
        if (isComparableCell(indNeighbor, index, caseRemerge))
        {
            start = nodes[index].vertex[1];
            finish = nodes[index].vertex[5];
            for (j = start; j <= finish; j++)
            {
                src.at<uchar>(j, fixed3) = 255;
                if (fixed4 != 0)
                    src.at<uchar>(j, fixed4) = 255;
            }
            A1 = 4 * nodes[indNeighbor].lx * nodes[indNeighbor].ly;
            A2 = 4 * nodes[index].lx * nodes[index].ly;
            nodes[indNeighbor].cx = (int)(nodes[indNeighbor].cx * A1 + nodes[index].cx * A2) / (A1 + A2);
            nodes[indNeighbor].lx = (nodes[indNeighbor].lx + nodes[index].lx) + (1 + (fixed4 = !0));
            checkVertex(indNeighbor, index, caseRemerge);

            toDelete.push_back({index});
            englobingCell.push_back({indNeighbor});
        }
        else
        {
            if (caseRemerge == 1)
                goto secondTestY;
        }
        goto noDeleteLine;

    xLine:
        fixed1 = nodes[index].cx;
        fixed2 = nodes[index].cy - nodes[index].ly / 2 + 1; // Cell vertex
        for (j = 0; j <= stop; j++)
        {

            if (fixed2 - j > 0 && obs < 3)
            {
                if (src.at<uchar>(fixed2 - j, fixed1) == 0 && src_gray.at<uchar>(fixed2 - j, fixed1) != 0)
                {
                    obs = obs + 1;
                    if (obs == 1)
                    {

                        fixed3 = fixed2 - j;
                        borderLine = true;
                    }
                    else if (obs == 2)
                    {
                        if (borderLine)
                        {

                            fixed4 = fixed3 - 1;
                        }
                        else
                            j = stop;
                    }
                }
                else
                    borderLine = false;
            }
        }

        if (obs > 0 && obs < 3)
        {
            if (fixed4 == 0)
                indNeighbor = clast.at<int>(fixed3 - 1, fixed1) - 1;
            else
                indNeighbor = clast.at<int>(fixed4 - 1, fixed1) - 1;

            indEnglobing = inglobed(indNeighbor, toDelete, englobingCell);

            if (indEnglobing != -1) // If it has been englobed
                indNeighbor = englobingCell[indEnglobing];
            if (indNeighbor != -1 && indNeighbor != index)
            {
                caseRemerge = 3; // Check on the upper side
                goto deleteLineX;
            }
        }

    secondTestX:
        obs = 0;
        fixed2 = nodes[index].cy + nodes[index].ly / 2 - 1;
        fixed3 = 0;
        fixed4 = 0;
        stop = 5;
        for (j = 0; j <= stop; j++)
        {
            if (fixed2 + j < rows - 1 && obs < 3)
            {
                if (src.at<uchar>(fixed2 + j, fixed1) == 0 && src_gray.at<uchar>(fixed2 + j, fixed1) != 0)
                {
                    obs = obs + 1;
                    if (obs == 1)
                    {

                        fixed3 = fixed2 + j;
                        borderLine = true;
                    }
                    else if (obs == 2)
                    {
                        if (borderLine)
                        {

                            fixed4 = fixed3 + 1;
                        }
                        else
                            j = stop;
                    }
                }
                else
                    borderLine = false;
            }
        }

        if (obs > 0 && obs < 3)
        {
            if (fixed4 == 0)
                indNeighbor = clast.at<int>(fixed3 + 1, fixed1) - 1;
            else
                indNeighbor = clast.at<int>(fixed4 + 1, fixed1) - 1;
            indEnglobing = inglobed(indNeighbor, toDelete, englobingCell);
            if (indEnglobing != -1)
                indNeighbor = englobingCell[indEnglobing];
            if (indNeighbor != -1 && indNeighbor != index)
            {
                caseRemerge = 4;
                goto deleteLineX;
            }
            else
            {
                goto testY;
            }
        }
        else
        {
            goto testY;
        }

    deleteLineX:
        if (isComparableCell(indNeighbor, index, caseRemerge))
        {
            start = nodes[index].vertex[0];
            finish = nodes[index].vertex[2];
            for (j = start; j <= finish; j++)
            {
                src.at<uchar>(fixed3, j) = 255;
                if (fixed4 != 0)
                    src.at<uchar>(fixed4, j) = 255;
            }
            A1 = 4 * nodes[indNeighbor].lx * nodes[indNeighbor].ly;
            A2 = 4 * nodes[index].lx * nodes[index].ly;
            nodes[indNeighbor].cy = (int)(nodes[indNeighbor].cy * A1 + nodes[index].cy * A2) / (A1 + A2); // Barycenter
            nodes[indNeighbor].ly = (nodes[indNeighbor].ly + nodes[index].ly) + (1 + (fixed4 = !0));
            checkVertex(indNeighbor, index, caseRemerge);
            toDelete.push_back({index});
            englobingCell.push_back({indNeighbor});
        }
        else
        {
            if (caseRemerge == 3)
                goto secondTestX;
            goto testY;
        }

    noDeleteLine:
        obs = 0;
        fixed3 = 0;
        fixed4 = 0;
        borderLine = false;
        caseRemerge = 0;
    }

    return toDelete;
}

void class_planning::divideCell(vector<int> ind, int maxDim, vector<int> f, int *nV)
{
    // f includes ind = 1 if lx too long, ind = 2 if ly too long, ind = 3 if both too long
    // *nV memorizes the number of deleted cells
    int index, lx, ly, cx, cy, j;
    int start, finish;
    int n = ind.size();
    for (int i = 0; i < n; i++)
    {
        index = ind[i]; // Index of the cell that has to be divided
        if (f[i] == 1)
        {
            // If lx is too big, vertical line tracking
            cx = nodes[index].cx;
            cy = nodes[index].cy;
            lx = nodes[index].lx / 4;
            ly = nodes[index].ly;
            start = max(2, checkPixel(cx, cy - ly / 2, true, 0));
            finish = min(rows - 3, checkPixel(cx, cy + ly / 2, true, 0));

            for (j = start; j <= finish; j++)
            {
                src.at<uchar>(j, cx) = 0; // Line tracking
            }
            // A new node is added in the right cell
            nodes.push_back({cx + lx, cy, 2 * lx, ly, 0, cx + 1, cy - ly / 2, nodes[index].vertex[2], nodes[index].vertex[3],
                             cx + 1, cy + ly / 2, nodes[index].vertex[6], nodes[index].vertex[7], 0});
            // The old node of the left cell is centered
            nodes[index].cx = cx - lx;
            nodes[index].lx = 2 * lx;
            nodes[index].vertex[2] = cx - 1;
            nodes[index].vertex[3] = cy - ly / 2;
            nodes[index].vertex[6] = cx - 1;
            nodes[index].vertex[7] = cy + ly / 2;
        }
        else if (f[i] == 2)
        {
            // If lx is too big, horizontal line tracking
            cx = nodes[index].cx;
            cy = nodes[index].cy;
            lx = nodes[index].lx;
            ly = nodes[index].ly / 4;
            start = max(2, checkPixel(cx - lx / 2, cy, false, 0));
            finish = min(cols - 3, checkPixel(cx + lx / 2, cy, false, 0));

            for (j = start; j <= finish; j++)
            {
                src.at<uchar>(cy, j) = 0;
            }

            // A new node is added in the lower cell
            nodes.push_back({cx, cy + ly, lx, 2 * ly, 0, cx - lx / 2, cy + 1, cx + lx / 2, cy + 1, nodes[index].vertex[4],
                             nodes[index].vertex[5], nodes[index].vertex[6], nodes[index].vertex[7], 0});
            //The old node is recentered in the upper cell
            nodes[index].cy = nodes[index].cy - ly;
            nodes[index].ly = 2 * ly;
            nodes[index].vertex[4] = cx - lx / 2;
            nodes[index].vertex[5] = cy - 1;
            nodes[index].vertex[6] = cx + lx / 2;
            nodes[index].vertex[7] = cy + 1;
        }
        else
        {
            // lx and ly are too big
            cx = nodes[index].cx;
            cy = nodes[index].cy;
            lx = nodes[index].lx / 4;
            ly = nodes[index].ly / 4;

            // Vertical line tracking
            start = max(2, checkPixel(cx, cy - ly * 2, true, 0));
            finish = min(rows - 3, checkPixel(cx, cy + ly * 2, true, 0));
            for (j = start; j <= finish; j++)
            {
                src.at<uchar>(j, cx) = 0;
            }

            // horizontal line tracking
            start = max(2, checkPixel(cx - lx * 2, cy, false, 0));
            finish = min(cols - 3, checkPixel(cx + lx * 2, cy, false, 0));
            for (j = start; j <= finish; j++)
            {
                src.at<uchar>(cy, j) = 0;
            }

            // A new node is added in the upper-left cell
            nodes.push_back({cx - lx, cy - ly, 2 * lx, 2 * ly, 0, nodes[index].vertex[0], nodes[index].vertex[1], cx - 1,
                             cy - 2 * ly, cx - 2 * lx, cy - 1, cx - 1, cy - 1, 0});
            // A new node is added in the upper-right cell
            nodes.push_back({cx + lx, cy - ly, 2 * lx, 2 * ly, 0, cx + 1, cy - 2 * ly, nodes[index].vertex[2],
                             nodes[index].vertex[3], cx + 1, cy - 1, cx + 2 * lx, cy - 1, 0});
            // A new node is added in the lower-right cell
            nodes.push_back({cx + lx, cy + ly, 2 * lx, 2 * ly, 0, cx + 1, cy + 1, cx + 2 * lx, cy + 1, cx + 1, cy + 2 * ly,
                             nodes[index].vertex[6], nodes[index].vertex[7], 0});
            // A new node is added in the lower-left cell
            nodes[index].cx = cx - lx;
            nodes[index].cy = cy + ly;
            nodes[index].lx = 2 * lx;
            nodes[index].ly = 2 * ly;
            nodes[index].vertex[0] = cx - 2 * lx;
            nodes[index].vertex[1] = cy + 1;
            nodes[index].vertex[2] = cx - 1;
            nodes[index].vertex[3] = cy + 1;
            nodes[index].vertex[6] = cx - 1;
            nodes[index].vertex[7] = cy + 2 * ly;
        }
    }
}

void class_planning::archsCost()
{

    // Based on the distance between two node, a cost is associated to each arch
    n_nodes = nodes.size();
    int n = 0;
    int ind;
    vector<float> max_v;
    vector<float> aux;
    vector<float> div, div1;
    vector<vector<float>> cost1;
    div.resize(n_nodes);
    div1.resize(n_nodes);
    max_v.resize(n_nodes);
    for (int i = 0; i < n_nodes; i++)
    {
        n = adj[i].size();

        aux.resize(n);
        for (int j = 0; j < n; j++)
        {
            ind = adj[i][j];
            aux[j] = dist(nodes[i].cx, nodes[i].cy, nodes[ind].cx, nodes[ind].cy);
        }
        max_v[i] = *max_element(aux.begin(), aux.end());

        cost.push_back(aux);
    }
    Max = *max_element(max_v.begin(), max_v.end());
}

void class_planning::nodesCost()
{

    // To each node is associated a cost inversely proportional to the distance from the node to the closest obstacle
    n_nodes = nodes.size();
    int xc, yc;
    int x, y;

    float lMin;
    float lMax = 10 * dimBox;
    int cost0 = 100;
    float cost = 0;
    int j, sampling;
    sampling = 32;
    float uno = 1.0;

    for (int i = 0; i < n_nodes; i++)
    {
        lMin = min((float(nodes[i].lx)) / 2, (float(nodes[i].ly)) / 2);
        lMin = max(uno, lMin);
        xc = nodes[i].cx;
        yc = nodes[i].cy;

        if (lMin >= lMax)
            goto newnode;

        for (j = lMin; j <= lMax - lMin; j++)
        {
            for (int theta = 0; theta <= sampling; theta++)
            {
                x = xc + j * cos(2 * theta * M_PI / sampling);
                y = yc + j * sin(2 * theta * M_PI / sampling);

                if (x > cols || x < 1 || y > rows || y < 1)
                { // The point is outside the image
                    cost = abs(uno / j);
                    goto newnode;
                }
                else
                {
                    if (int(src_gray.at<uchar>(y, x)) == 0)
                    { // There is a boundery
                        cost = abs(uno / j);
                        goto newnode;
                    }
                }
            }
        }

    newnode:
        nodes[i].CostNode = cost;
        cost = 0;
    }
}

void class_planning::addNode(int x, int y, int SoG)
{

    nodes.push_back({x, y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2});

    int label = clast.at<int>(y, x);

    if (label == 0)
    {
        if (src_gray.at<int>(y, x) == 0)
        {
            cout << "The position (" << x << "," << y << ") correspond to a position with an obstacle. " << endl;
            block = true;
            return;
        }
        else
        {
            label = clast.at<int>(y + 1, x + 1);
            if (label == 0)
                label = clast.at<int>(y - 1, x - 1);
            if (label == 0)
            {
                cout << "The position (" << x << "," << y << ") correspond to a position with an obstacle." << endl;
                block = true;
                return;
            }
        }
    }

    label = label - 1;

    if (isBlocked(label))
    {
        // Check if the goal is in a cell connected with other cells otherwise an error message is printed
        cout << "The insert position correspond to an unconnected graph area." << endl;
        block = true;
        return;
    }

    n_nodes = nodes.size();

    int n, ind1, ind2;
    float cost_tmp;

    int offset;

    adj.push_back({label});
    cost_tmp = dist(x, y, nodes[label].cx, nodes[label].cy);
    cost.push_back({cost_tmp});
    offset = 1;

    ind2 = adj[label].size();
    adj[label].resize(ind2 + 1);
    adj[label][ind2] = n_nodes - 1;
    cost[label].resize(ind2 + 1);
    cost[label][ind2] = dist(x, y, nodes[label].cx, nodes[label].cy);

    // Goal node data insert
    n = adj[label].size() - 1;
    adj[n_nodes - 1].resize(n + offset);
    cost[n_nodes - 1].resize(n + offset);

    for (int i = 0; i < n; i++)
    {
        ind1 = adj[label][i];
        if (areConnectable(nodes[n_nodes - 1].cx, nodes[n_nodes - 1].cy, nodes[ind1].cx, nodes[ind1].cy, (1 - nodes[ind1].flag) * (dimBox - 1) + 1))
        {
            adj[n_nodes - 1][i + offset] = ind1;
            ind2 = adj[ind1].size();
            adj[ind1].resize(ind2 + 1);
            adj[ind1][ind2] = n_nodes - 1;
            cost_tmp = dist(x, y, nodes[ind1].cx, nodes[ind1].cy);
            cost[n_nodes - 1][i + offset] = cost_tmp;
            cost[ind1].resize(ind2 + 1);
            cost[ind1][ind2] = cost_tmp;
        }
        else
        {
            offset--;
        }
    }

    adj[n_nodes - 1].resize(n + offset);
    cost[n_nodes - 1].resize(n + offset);

    if (SoG == 0)
        start = n_nodes - 1;
    else
        goals.push_back({n_nodes - 1});
}

///////////////////////////////////// DIJKSTRA FUNCTIONS ///////////////////////////////////////////////

vector<int> class_planning::algorithmManager()
{

    // This function manage the presence of priorities or multiPOIs

    vector<int> Path, pathTmp, aux;
    int np = goalsPriority.size();
    int nm = multiPOIs.size();
    int ng = goals.size();
    int s, g;
    int costOld, costNew, costTmp;
    costOld = INT_MAX;
    int nPermutations;

    int c1 = (np > 0);
    int c2 = (nm > 0);
    int aG[ng];

    cost_final = INT_MAX;

    switch (c1)
    {
    case 0: // There is not priority
        switch (c2)
        {
        case 0: // There are not multiPOIs

            nPermutations = factorial(ng);
            for (int i = 0; i < ng; i++)
            {
                aG[i] = goals[i];
            }

            findPermutations(aG, ng);

            for (int i = 0; i < nPermutations; i++)
            {
                costNew = 0;
                goals = multiGoals[i]; // Single goals sequence
                for (int j = 0; j < ng; j++)
                {

                    // At the endo of every loop, the old goal becomes the new start
                    if (j == 0)
                        s = start;
                    else
                        s = g;

                    g = goals[j];

                    pathTmp = Dijkstra(s, g, &costTmp); // Vector of node index in the found path

                    if (costTmp == INT_MAX)
                    {
                        goto stopInternalLoop1; // If start and goal nodes are not connectable, the research is interrupted
                    }

                    // Sub-paths found saving
                    if (aux.size() == 0)
                        aux = pathTmp;
                    else
                        aux.insert(aux.end(), pathTmp.begin() + 1, pathTmp.end());

                    costNew = costNew + costTmp;
                }

                if (costNew < costOld)
                {
                    // A better path was found
                    Path = aux;
                    costOld = costNew;
                    cost_final = costOld;
                }
            stopInternalLoop1:
                aux.clear();
            }

            break;

        case 1: // There are multiPOIs
            vector<vector<int>> tmpGoals;
            tmpGoals = findMultiPOIs(ng); // Calculates combinations of goals and multiPOIs

            ng = tmpGoals[0].size();
            nPermutations = factorial(ng);
            vector<int> aux;

            for (int h = 0; h < tmpGoals.size(); h++)
            {
                //cout << endl<< endl<< "Iteration #" << h+1 << endl<< endl;
                for (int i = 0; i < ng; i++)
                {
                    aG[i] = tmpGoals[h][i];
                    cout << aG[i] << " ";
                }
                cout << endl;

                findPermutations(aG, ng);

                for (int i = 0; i < nPermutations; i++)
                {
                    costNew = 0;
                    goals = multiGoals[i];
                    for (int j = 0; j < ng; j++)
                    {

                        if (j == 0)
                            s = start;
                        else
                            s = g;

                        g = goals[j];

                        pathTmp = Dijkstra(s, g, &costTmp);

                        if (costTmp == INT_MAX)
                        {
                            goto stopInternalLoop2;
                        }

                        if (aux.size() == 0)
                            aux = pathTmp;
                        else
                            aux.insert(aux.end(), pathTmp.begin() + 1, pathTmp.end());

                        costNew = costNew + costTmp;
                    }

                    if (costNew < costOld)
                    {

                        Path = aux;
                        costOld = costNew;
                        cost_final = costOld;
                    }

                stopInternalLoop2:

                    aux.clear();
                }
            }
            break;
        }
        break;

    case 1: // There are priorities
        switch (c2)
        {
        case 0:            // There aren't multiPOIs
            sortGoals(np); // Sorts goals based on the priority
            for (int i = 0; i < ng; i++)
            {

                if (i == 0)
                    s = start;
                else
                    s = g;

                g = goals[i];

                pathTmp = Dijkstra(s, g, &costTmp);

                if (costTmp == INT_MAX)
                {
                    cost_final = costTmp;
                    Path.clear();
                    return Path;
                }

                if (Path.size() == 0)
                    Path = pathTmp;
                else
                    Path.insert(Path.end(), pathTmp.begin() + 1, pathTmp.end());

                costNew = costNew + costTmp;
            }

            cost_final = costNew;
            break;

        case 1: // There are multiPOIs
            vector<vector<int>> tmpGoals;
            tmpGoals = findMultiPOIs(ng);

            ng = tmpGoals[0].size();
            nPermutations = factorial(ng);
            vector<int> aux;

            for (int h = 0; h < tmpGoals.size(); h++)
            {
                //  cout << endl<< endl<< "Iteration #" << h+1 << endl<< endl;
                for (int i = 0; i < ng; i++)
                {
                    aG[i] = tmpGoals[h][i];
                    cout << aG[i] << " ";
                }
                cout << endl;

                findPermutations(aG, ng);

                for (int i = 0; i < nPermutations; i++)
                {
                    costNew = 0;
                    goals = multiGoals[i];
                    sortGoals(np);
                    for (int j = 0; j < ng; j++)
                    {

                        if (j == 0)
                            s = start;
                        else
                            s = g;

                        g = goals[j];

                        pathTmp = Dijkstra(s, g, &costTmp);

                        if (costTmp == INT_MAX)
                        {
                            goto stopInternalLoop4;
                        }

                        if (aux.size() == 0)
                            aux = pathTmp;
                        else
                            aux.insert(aux.end(), pathTmp.begin() + 1, pathTmp.end());

                        costNew = costNew + costTmp;
                    }
                    if (costNew < costOld)
                    {

                        Path = aux;
                        costOld = costNew;
                        cost_final = costOld;
                    }

                stopInternalLoop4:
                    aux.clear();
                }
            }
            break;
        }
        break;
    }

    // Printing the final path
    cout << "The final path move through the following nodes: " << endl;
    for (int i = 0; i < Path.size(); i++)
    {
        cout << Path[i] << " ";
    }
    cout << endl;
    cout << "Number of nodes in the final path: " << Path.size() << endl;

    return Path;
}

vector<int> class_planning::Dijkstra(int start, int dest, int *c)
{

    vector<param> queue;    // Queue with characteristic parameters
    vector<param> p_q;      // Queue with nodes alredy analyzed
    vector<int> path, open; // Path and alive nodes vectors
    int i, j, ind, id, id_o, n_p, ind_aux, it;
    int x, y;
    float CostTot;
    int past1, past2;
    Vector2d v1, v2;
    int numRotTot = 0;
    n_nodes = nodes.size();
    int m1 = max(w_length, w_degRot); // Safety weight
    int w_safety = max(m1, w_numRot);

    x = nodes[dest].cx;
    y = nodes[dest].cy;

    if (moreAdj == 2)
    {
        if (areConnectable(nodes[start].cx, nodes[start].cy, nodes[dest].cx, nodes[dest].cy, dimBox))
        {
            path.push_back(start);
            path.push_back(dest);
            return path;
        }
    }

    // Parameters initialization
    for (i = 0; i < n_nodes; i++)
    {

        queue.push_back({i, FLT_MAX, FLT_MAX, -1});
    }

    // Start node cost equals to zero
    queue[start].CostTot = 0;
    queue[start].heuristicCost = 0;

    bool goalNotAchieved = true; // true = destination not reached
                                 // false = destination reached
    it = n_nodes - 1;            // Maximum number of iterations
    param aux_q = queue[start];

    while (it > 0 && goalNotAchieved)
    {

        if (it == n_nodes - 1)
        {
            past1 = start;
            past2 = start;
        }
        else if (it == n_nodes - 2)
        {
            past1 = aux_q.id;
            past2 = start;
        }
        else
        {
            past1 = aux_q.id;
            past2 = aux_q.parent;
        }

        v1 << (nodes[past1].cx - nodes[past2].cx), (nodes[past1].cy - nodes[past2].cy);
        v1 = v1;

        for (j = 0; j < adj[aux_q.id].size(); j++)
        {

            ind = adj[aux_q.id][j];
            v2 << (nodes[ind].cx - nodes[past1].cx), (nodes[ind].cy - nodes[past1].cy);
            v2 = v2;

            // Calculate CosTot(x)= [CosTot(x-1)+l(x-1 --> x)] + a*CostNode + b*CostAng + c*CostRot= CostToCome(x) + a*CostNode + b*CostAng + c*CostRot
            CostTot = (aux_q.CostTot + w_length * cost[aux_q.id][j] / Max) + w_safety * nodes[ind].CostNode + w_degRot * (CostCurve(v1, v2)) + w_numRot * numRot;

            // Check if the calculated cost is lower compared to the saved one
            if (queue[ind].CostTot > CostTot)
            {
                if (isNotInOpen(open, ind))
                {
                    open.push_back(ind);
                }

                queue[ind].CostTot = CostTot;
                queue[ind].heuristicCost = CostTot;
                queue[ind].parent = aux_q.id;
            }
        }

        ind_aux = minCost(queue, open); // id for the next node to analyze
        if (ind_aux == -1)
        {
            cout << "Error in the algorithm execution" << endl;
            *c = INT_MAX;
            return {-1};
        }

        aux_q = queue[ind_aux];

        // If activated it is deleted from the active nodes list
        ind_aux = deleteActived(ind_aux, open);

        if (ind_aux == -1)
        {
            cout << "Error in the algorithm execution" << endl;
            *c = INT_MAX;
            return {-1};
        }

        if (ind_aux <= open.size() - 1)
        {
            open.erase(open.begin() + ind_aux);
        }
        else
        {
            // There are not active node. We are in unconnected area
            it = 0;
            cout << "Algorithm has found an unconnected area" << endl;
            *c = INT_MAX;
        }

        // The analyzed node is insert in the analyzed node list and it will not be modified anymore
        p_q.push_back(aux_q);

        // Update loop conditions
        goalNotAchieved = !(aux_q.id == dest);
        it--;
    }

    // If the destination is not reached, an error message is printed
    if (goalNotAchieved)
    {
        cout << "Algorithm not completed" << endl;
        path.push_back({-1});
        *c = INT_MAX;
        goto DijkstraEnd;
    }

    // cout << "Number of iterations equalt to: " << it << endl;

    n_p = p_q.size();
    ind = n_p - 1;

    // Goal node is insert in the path
    path.insert(path.begin(), dest);
    *c = p_q[ind].CostTot;

    while (id != start)
    {
        // Backward research of the analyzed nodes from goal to start
        id = p_q[ind].parent;
        for (i = 0; i < n_p; i++)
        {
            if (p_q[i].id == id)
            {
                ind = i;
            }
        }
        // Nodes are insert at the beginning of the vector path
        path.insert(path.begin(), id);
    }

DijkstraEnd:
    return path;
}

int class_planning::totalLen(vector<int> path)
{ // path

    int TotLen = 0;

    for (int i = 1; i < path.size(); i++)
    {

        int id_new = path[i];
        int id = path[i - 1];
        TotLen += dist(nodes[id_new].cx, nodes[id_new].cy, nodes[id].cx, nodes[id].cy);
    }

    return TotLen;
}

//////////////////////////////////////////////////// TRAJECTORY RESEARCH /////////////////////////////////////////////////////////////////////

bool class_planning::saveTrajectorySubImage(vector<int> p)
{

    subImage = Mat::ones(src_gray.size(), CV_32FC1) * 255;

    int xc, yc, x, y, r;
    bool obstacle = false;
    int sampling1 = 10;
    int sampling2 = 360;
    int lim = int(1000 / scala);

    for (int i = 0; i < p.size() - 1; i++)
    {
        for (int h = 0; h < sampling1; h++)
        {
            xc = nodes[p[i]].cx;
            yc = nodes[p[i]].cy;

            float mx = float(xc - nodes[p[i + 1]].cx);
            float my = float(yc - nodes[p[i + 1]].cy);

            mx = (mx / sampling1);
            my = (my / sampling1);

            xc = xc - int(h * mx);
            yc = yc - int(h * my);
            for (int theta = 0; theta < sampling2; theta++)
            {
                r = 0;
                obstacle = false;
                while (!obstacle)
                {
                    r = r + 1;
                    x = xc + r * cos(2 * theta * M_PI / sampling2);
                    y = yc + r * sin(2 * theta * M_PI / sampling2);

                    if (x<cols & x> 0 & y<rows & y> 1 &
                        abs(xc - x) < lim & abs(yc - y) < lim)
                    {
                        if (src_gray.at<uchar>(y, x) == 0)
                        {
                            subImage.at<int>(y, x) = 0;
                            obstacle = true;
                        }
                    }
                    else
                    {
                        obstacle = true;
                    }
                }
            }
        }
    }

    imshow(corners_window, subImage);
    imwrite("/home/vanessa/catkin_ws/provaMap.png", subImage);
}

////////////////////////////////////////////////////// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////////////

void class_planning::loadPOIs()
{
    // If there is a file with data about POIs it is read and the POIs are loaded

    string file;
    file.append(name_map);
    file.append(".txt");

    int codificaASCII = 48;
    int c = 0;

    poi aux;

    string line;
    ifstream POISfile(file);
    if (POISfile.is_open())
    {
        while (getline(POISfile, line, '-'))
        {
            switch (c)
            {
            case 0:
                aux.nameId = line;
                break;
            case 1:
                aux.x = stoi(line);
                break;
            case 2:
                aux.y = stoi(line);
                break;
            }
            c = c + 1;
            if (c == 3)
            {
                POIS.push_back(aux);
                c = 0;
            }
        }
        POISfile.close();

        cout << "The POIs of the map are: " << endl;
        for (int i = 0; i < POIS.size(); i++)
        {
            cout << POIS[i].nameId << " " << POIS[i].x << " " << POIS[i].y << endl;
        }
    }

    else
        cout << "There are not POIs for this map." << endl;
}

void class_planning::assign_dimBox()
{
    // DimBox value is set based on the name of the image. The last number indicate the relation: # cm = 1 pixel

    string tmp = name_map;
    int n = tmp.length();
    int aux = 0;
    int dec = 1;
    int f = 0;
    int codificaASCII = 48;
    for (int i = n; i > 0; i--)
    {
        if (isdigit(tmp[i]))
        {
            aux = aux + dec * (int(tmp[i]) - codificaASCII);
            dec = dec * 10;
            f = 1;
        }
        else if (f == 1)
        {
            goto assign;
        }
    }

assign:
    if (aux == 0)
        aux = 5;

    dimBox = int(70 / aux);
    scala = aux;
    scale = aux / 100;
}

float class_planning::dist(int x1, int y1, int x2, int y2)
{

    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

int class_planning::factorial(int n)
{

    int count = 1;
    for (int i = n; i > 1; i--)
    {
        count = count * i;
    }
    return count;
}

bool class_planning::AlreadyAdd(int ni, int *n)
{
    // Checks if the node is already in the list pointed from "n"

    int l = *n;

    for (int g = 1; g <= l; g++)
    {

        if (*(n + g) == ni)
        {
            return true; // It was already added
        }
    }
    return false;
}

bool class_planning::notAlreadyAdd(int x, int y, vector<node> n)
{
    // Checks if the node was already added

    int l = n.size();
    if (l == 0)
        return true; // A fake node is added
    for (int g = 0; g < l; g++)
    {

        if (n[g].cx == x && n[g].cy == y)
            return false; // It was already added a fake node
    }
    return true;
}

bool class_planning::isValid(int x, int y)
{

    // Checks if the inserted point is valid

    return (x > 0) && (x < cols) &&
           (y > 0) && (y < rows);
}

bool class_planning::isValidControll(int ind, int adjMax, int indControll)
{

    // Check node "indControll" is not already in the adjacencies of "ind"

    if (ind == indControll)
        return false;

    for (int g = 0; g < adjMax; g++)
    {
        if (adj[ind][g] == indControll)
            return false;
    }
    return true;
}

int class_planning::inglobed(int ind, vector<int> indControll, vector<int> ing)
{

    // Check if the cell is already inglobed. It returns the index of the inclusive cell

    int index = -1;
    int controll = ind;
    for (int g = 0; g < indControll.size(); g++)
    {
        if (controll == indControll[g])
        {
            index = g;
            controll = ing[g];
        }
    }
    return index; // If -1 not inglobed
}

bool class_planning::isComparableCell(int ind1, int ind2, int c)
{
    // Checks if two cells are able to be merged. It's necessary they have the same
    // vertexes of the shared side (with same length)

    int tol = 4;

    int xm1, xp1, ym1, yp1;
    int xm2, xp2, ym2, yp2;
    switch (c)
    {
    case 0:
        // Error
        return false;
        break;

    case 1:
        // Right check
        xp1 = nodes[ind1].vertex[2];
        ym1 = nodes[ind1].vertex[3];
        yp1 = nodes[ind1].vertex[7];
        xm2 = nodes[ind2].vertex[0];
        ym2 = nodes[ind2].vertex[1];
        yp2 = nodes[ind2].vertex[5];

        if ((xp1 >= (xm2 - tol)) && (ym1 == ym2) && (yp1 == yp2))
            return true;
        break;

    case 2:
        // Left check
        xm1 = nodes[ind1].vertex[0];
        ym1 = nodes[ind1].vertex[1];
        yp1 = nodes[ind1].vertex[5];
        xp2 = nodes[ind2].vertex[2];
        ym2 = nodes[ind2].vertex[3];
        yp2 = nodes[ind2].vertex[7];

        if ((xm1 <= (xp2 + tol)) && (ym1 == ym2) && (yp1 == yp2))
            return true;
        break;

    case 3:
        // Upper check
        xm1 = nodes[ind1].vertex[4];
        ym1 = nodes[ind1].vertex[5];
        xp1 = nodes[ind1].vertex[6];
        xm2 = nodes[ind2].vertex[0];
        yp2 = nodes[ind2].vertex[1];
        xp2 = nodes[ind2].vertex[2];

        if (((xm1 == xm2) && (xp1 == xp2)) && (ym1 <= (yp2 + tol)))
            return true;
        break;

    case 4:
        // Lower check
        xm1 = nodes[ind1].vertex[0];
        yp1 = nodes[ind1].vertex[1];
        xp1 = nodes[ind1].vertex[2];
        xm2 = nodes[ind2].vertex[4];
        ym2 = nodes[ind2].vertex[5];
        xp2 = nodes[ind2].vertex[6];

        if (((xm1 == xm2) && (xp1 == xp2)) && (yp1 >= (ym2 - tol)))
            return true;
        break;
    }

    return false;
}

bool class_planning::isBlocked(int cell)
{

    // Ajacency check
    if (adj[cell].size() >= 1)
        return false;
    else
        return true; // Unconnected area
}

void class_planning::adjMatGenerate()
{

    // Adjacency matrix creation
    n_nodes = nodes.size();
    A.resize(n_nodes, n_nodes);

    for (int i = 0; i < n_nodes; i++)
    {
        for (int j = 0; j < adj[i].size(); j++)
        {
            A(i, adj[i][j]) = cost[i][j];
        }
    }
}

int class_planning::minCost(vector<param> p, vector<int> a)
{

    // Looks for the minimum cost of the active nodes

    int ind;
    float c = FLT_MAX;
    if (a.size() == 0)
    {
        cout << "Secondo i parametri inseriti la pozione di partenza e di arrivo non sono connettibili" << endl;
        return -1;
    }
    for (int k = 0; k < a.size(); k++)
    {
        if (p[a[k]].heuristicCost < c)
        {
            c = p[a[k]].heuristicCost;
            ind = a[k];
        }
    }

    return ind;
}

float class_planning::CostCurve(Vector2d a, Vector2d b)
{

    // Required cost to change orientation from "a" direction to "b" direction
    numRot = 0;
    float angle;

    // To avoid singolarities of "atan2"
    float dotProduct = a(0) * b(0) + a(1) * b(1);
    float crossProduct = a(0) * b(1) - a(1) * b(0);
    angle = abs(atan2(crossProduct, dotProduct) * 180 / M_PI); // Degrees

    // Cost set
    if (angle == 0) // There is not rotation
        return 0;

    else
    {
        numRot = 1; // There is a rotation
        if (angle >= minAngle && angle <= maxAngle)
            return 0.2;
        else
        {
            return 0.4 + (0.6 / 180) * angle;
        }
    }
}

int class_planning::deleteActived(int ind, vector<int> a)
{

    // Look for the index of the node that has to be deleted
    for (int k = 0; k < a.size(); k++)
    {

        if (a[k] == ind)
            return k;
    }

    // The element wasn't found
    return -1;
}

bool class_planning::areConnectable(int x1, int y1, int x2, int y2, int d)
{

    // Check if q1 and q2 are safetly connectable with a path dimBox-width

    float mx = float(x2 - x1);
    float my = float(y2 - y1);

    //d := Bounding Box dimension

    int num_loop = max(abs(mx), abs(my));

    mx = (mx / num_loop);
    my = (my / num_loop);

    int x = x1;
    int y = y1;

    for (int i = 1; i <= num_loop; i++)
    {

        for (int j = -d; j <= d; j++)
        {
            x = int(x1 + j * my + mx * i) + 1;
            y = int(y1 + j * mx + my * i) + 1;

            if (int(src_gray.at<uchar>(y, x)) == 0)
            {
                return false;
            }
        }
    }

    return true;
}

int class_planning::calculateOverlap(int p1, int p2, int m1, int m2, int *l)
{

    // Overlap side evaluation and calculation of new center (it will be the coordinate of the new node)

    int e1 = max(m1, m2);
    int e2 = min(p1, p2);
    int d = (e2 - e1) / 2;
    *l = d;

    return (e1 + d);
}

bool class_planning::isNotInOpen(vector<int> o, int ind)
{

    // Checks if the index is in open vector

    for (int k = 0; k < o.size(); k++)
    {
        if (ind == o[k])
            return false;
    }

    return true;
}

void class_planning::findVertex(int x, int y, int lx, int ly, int ind, int *v)
{

    // Looks for vertexes cell

    int err = 3;
    int xm, xp, ym, yp;
    int p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y;

    xm = (int)(x - lx / 2) + err; // Side with tolerance
    xp = (int)(x + lx / 2) - err;
    ym = (int)(y - ly / 2) + err;
    yp = (int)(y + ly / 2) - err;

    bool fx, fy;

    p1x = xm - err;
    p1y = ym - err; // The error is deleted
    p2x = xp + err;
    p2y = ym - err;
    p3x = xm - err;
    p3y = yp + err;
    p4x = xp + err;
    p4y = yp + err;

    for (int i = 0; i <= 2 * err; i++)
    {

        fx = ((xm - i) > 0);
        fy = ((ym - i) > 0);
        if (fx && fy)
        {
            if (ind == (clast.at<int>(ym - i, xm - i) - 1))
            {
                p1x = xm - i;
                p1y = ym - i;
            }
            else if (ind == (clast.at<int>(p1y, xm - i) - 1))
            {
                p1x = xm - i;
            }
            else if (ind == (clast.at<int>(ym - i, p1x) - 1))
            {
                p1y = ym - i;
            }
        }
        else if (fx)
        {
            if (ind == (clast.at<int>(p1y, xm - i) - 1))
                p1x = xm - i;
        }
        else if (fy)
        {
            if (ind == (clast.at<int>(ym - i, p1x) - 1))
                p1y = ym - i;
        }

        fx = ((xm + i) < cols);
        if (fx && fy)
        {
            if (ind == (clast.at<int>(ym - i, xp + i) - 1))
            {
                p2x = xp + i;
                p2y = ym - i;
            }
            else if (ind == (clast.at<int>(p2y, xp + i) - 1))
            {
                p2x = xp + i;
            }
            else if (ind == (clast.at<int>(ym - i, p2x) - 1))
            {
                p2y = ym - i;
            }
        }
        else if (fx)
        {
            if (ind == (clast.at<int>(p2y, xp + i) - 1))
                p2x = xp + i;
        }
        else if (fy)
        {
            if (ind == (clast.at<int>(ym - i, p2x) - 1))
                p2y = ym - i;
        }

        fx = ((xm - i) > 0);
        fy = ((ym + i) < rows);
        if (fx && fy)
        {
            if (ind == (clast.at<int>(yp + i, xm - i) - 1))
            {
                p3x = xm - i;
                p3y = yp + i;
            }
            else if (ind == (clast.at<int>(p3y, xm - i) - 1))
            {
                p3x = xm - i;
            }
            else if (ind == (clast.at<int>(yp + i, p3x) - 1))
            {
                p3y = yp + i;
            }
        }
        else if (fx)
        {
            if (ind == (clast.at<int>(p3y, xm - i) - 1))
                p3x = xm - i;
        }
        else if (fy)
        {
            if (ind == (clast.at<int>(yp + i, p3x) - 1))
                p3y = yp + i;
        }

        fx = ((xm + i) < cols);
        if (fx && fy)
        {
            if (ind == (clast.at<int>(yp + i, xp + i) - 1))
            {
                p4x = xp + i;
                p4y = yp + i;
            }
            else if (ind == (clast.at<int>(p4y, xp + i) - 1))
            {
                p4x = xp + i;
            }
            else if (ind == (clast.at<int>(yp + i, p4x) - 1))
            {
                p4y = yp + i;
            }
        }
        else if (fx)
        {
            if (ind == (clast.at<int>(p4y, xp + i) - 1))
                p4x = xp + i;
        }
        else if (fy)
        {
            if (ind == (clast.at<int>(yp + i, p4x) - 1))
                p4y = yp + i;
        }
    }

    p1y = (int)(p1y + p2y) / 2;
    p2y = p1y;
    p3y = (int)(p3y + p4y) / 2;
    p4y = p3y;
    p1x = (int)(p1x + p3x) / 2;
    p3x = p1x;
    p2x = (int)(p2x + p4x) / 2;
    p4x = p2x;

    *v = p1x;
    *(v + 1) = p1y;
    *(v + 2) = p2x;
    *(v + 3) = p2y;
    *(v + 4) = p3x;
    *(v + 5) = p3y;
    *(v + 6) = p4x;
    *(v + 7) = p4y;
}

void class_planning::findDatesCells(int x, int y, int lx, int ly, int ind, int *v)
{

    // Looks for data cell

    int err = 1;
    bool find = false;
    int tmpXm, tmpXp, tmpYm, tmpYp;

    tmpXm = x - lx + err;
    while (!find)
    {
        if (clast.at<int>(y, tmpXm) == ind)
        {
            tmpXm = tmpXm - 1;
        }
        else
        {
            tmpXm = tmpXm + 1;
            find = true;
        }
    }

    find = false;
    tmpXp = x + lx - err;
    while (!find)
    {
        if (clast.at<int>(y, tmpXp) == ind)
        {
            tmpXp = tmpXp + 1;
        }
        else
        {
            tmpXp = tmpXp - 1;
            find = true;
        }
    }

    find = false;
    tmpYm = y - ly + err;
    while (!find)
    {
        if (clast.at<int>(tmpYm, x) == ind)
        {
            tmpYm = tmpYm - 1;
        }
        else
        {
            tmpYm = tmpYm + 1;
            find = true;
        }
    }

    find = false;
    tmpYp = y + ly - err;
    while (!find)
    {
        if (clast.at<int>(tmpYp, x) == ind)
        {
            tmpYp = tmpYp + 1;
        }
        else
        {
            tmpYp = tmpYp - 1;
            find = true;
        }
    }

    *v = tmpXm;
    *(v + 1) = tmpYm;
    *(v + 2) = tmpXp;
    *(v + 3) = tmpYm;
    *(v + 4) = tmpXm;
    *(v + 5) = tmpYp;
    *(v + 6) = tmpXp;
    *(v + 7) = tmpYp;
}

void class_planning::checkVertex(int n1, int n2, int c)
{

    // After remerge function, cell vertexes are updated

    switch (c)
    {
    case 1:
        nodes[n1].vertex[2] = nodes[n2].vertex[2];
        nodes[n1].vertex[3] = nodes[n2].vertex[3];
        nodes[n1].vertex[6] = nodes[n2].vertex[6];
        nodes[n1].vertex[7] = nodes[n2].vertex[7];
        break;

    case 2:

        nodes[n1].vertex[0] = nodes[n2].vertex[0];
        nodes[n1].vertex[1] = nodes[n2].vertex[1];
        nodes[n1].vertex[3] = nodes[n2].vertex[4];
        nodes[n1].vertex[5] = nodes[n2].vertex[5];
        break;

    case 3:
        nodes[n1].vertex[4] = nodes[n2].vertex[4];
        nodes[n1].vertex[5] = nodes[n2].vertex[5];
        nodes[n1].vertex[6] = nodes[n2].vertex[6];
        nodes[n1].vertex[7] = nodes[n2].vertex[7];
        break;

    case 4:
        nodes[n1].vertex[0] = nodes[n2].vertex[0];
        nodes[n1].vertex[1] = nodes[n2].vertex[1];
        nodes[n1].vertex[2] = nodes[n2].vertex[2];
        nodes[n1].vertex[3] = nodes[n2].vertex[3];
        break;
    }
}

int class_planning::checkPixel(int x, int y, bool vertical, int BW)
{

    // Looks for pixel delimiting the cell

    int tol = 10;

    if (vertical)
    {
        int localY;
        for (int i = 0; i < tol; i++)
        {

            localY = y + i;
            if ((localY) < rows)
            {
                if ((int)(src.at<uchar>(localY, x)) == BW)
                {
                    return localY;
                }
            }
            else
                return (rows - 1);

            localY = y - i;
            if ((localY) > 0)
            {
                if ((int)(src.at<uchar>(localY, x)) == BW)
                {
                    return localY;
                }
            }
            else
                return 0;
        }
    }
    else
    {
        int localX;
        for (int i = 0; i < tol; i++)
        {

            localX = x + i;
            if ((localX) < cols)
            {
                if ((int)(src.at<uchar>(y, localX)) == BW)
                {
                    return localX;
                }
            }
            else
                return (cols - 1);

            localX = x - i;
            if ((localX) > 0)
            {
                if ((int)(src.at<uchar>(y, localX)) == BW)
                {
                    return localX;
                }
            }
            else
                return 0;
        }
    }
}

void class_planning::addPOIs(string c, int p)
{

    // Adds POIs to goals list

    vector<int> newGoals;
    newGoals = selectPOIs(c);
    n_nodes = nodes.size();
    int n = newGoals.size();
    int ind;
    if (n > 1)
    {                             // There is at least a double POIs
        ind = multiPOIs.size();   // ind is zero if there is not already a double POIs otherwise it's bigger
        multiPOIs.push_back({0}); // Insert an empty row
    }

    for (int i = 0; i < n; i++)
    { // Check multiPOis
        addNode(POIS[newGoals[i]].x, POIS[newGoals[i]].y, 1);
        if (n > 1)
        {
            multiPOIs.push_back(goals.size() - 1);
            if (p != -1)
                goalsPriority.push_back({p});
            multiPOIs[ind] = i + 1;
        }
    }
}

vector<int> class_planning::selectPOIs(string c)
{

    vector<int> out;
    for (int i = 0; i < POIS.size(); i++)
    {
        if (c.compare(POIS[i].nameId) == 0) // Equals
            out.push_back({i});
    }
    return out;
}

void class_planning::sortGoals(int n)
{

    // It sorts goals based on the priority assigned in main.cpp

    vector<int> tmpG;
    tmpG.resize(n);

    for (int j = 0; j < n; j++)
    {
        tmpG[goalsPriority[j]] = goals[j];
    }

    goals = tmpG;
}

void class_planning::findPermutations(int a[], int n)
{

    int loop = 0;
    vector<int> aux;
    aux.resize(n);

    sort(a, a + n); // Increasing order

    do
    {
        multiGoals.push_back(aux);
        for (int i = 0; i < n; i++)
        {
            multiGoals[loop][i] = a[i];
        }
        loop = loop + 1;
    } while (next_permutation(a, a + n));
}

vector<vector<int>> class_planning::findMultiPOIs(int ng)
{

    // If there are multiPOIs, this function calculate every possibile combination includind different POIs

    int loop = 0;
    bool countOk = false;
    int count = 0;
    int ind = 0;
    int j, h, g, z;
    int comb = 1;
    int nM = multiPOIs.size();

    while (!countOk)
    {
        // Number of combinations
        comb = comb * multiPOIs[ind];
        ind = ind + multiPOIs[ind] + 1;
        count = count + 1;
        if (ind >= nM)
            countOk = true;
    }

    int n = ng - ind + 2 * count; // Vector length
    vector<int> aux;              // This vector is used to store possible goals
    aux.resize(n);

    int n1 = n - count; // Numer of goals (it doesn't include POIs)
    for (j = 0; j < n1; j++)
        aux[j] = goals[j]; // Goals loading (no POIs)

    vector<vector<int>> tmp; // This matrix contains every combination that has to be explored
    for (j = 0; j < comb; j++)
        tmp.push_back(aux);

    ind = 0;
    int indNext = 0;
    int myInd = 0;
    int n2, n3;
    n2 = comb;

    for (j = 0; j < count; j++)
    {
        n = multiPOIs[ind++];
        n2 = n2 / n;
        indNext = ind + n;
        while (myInd != comb)
        {
            for (g = ind; g < indNext; g++)
            {
                for (z = 0; z < n2; z++)
                    tmp[myInd++][n1 + j] = goals[multiPOIs[g]];
            }
        }
        ind = indNext;
        myInd = 0;
    }

    return tmp;
}
