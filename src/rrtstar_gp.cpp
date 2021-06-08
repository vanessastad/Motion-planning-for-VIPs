#include "rrtstar_gp.h" // "MyNavigation/rrtstar_gp.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RRTstar, nav_core::BaseGlobalPlanner)

RRTstar::RRTstar()
{
}

RRTstar::RRTstar(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros);
}

/////////////////////////////////////////  OVERRIDDEN CLASSES from interface nav_core::BaseGlobalPlanner  /////////////////////////////////////////////////////////////////////////////////

void RRTstar::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{

    if (!initialized_)
    {

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        frame_id_ = "/map";
        ros::NodeHandle private_nh("~/" + name);

        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();
        resolution = costmap_->getResolution();

        dimBox = 15;

        mapSize = width * height;
        float tBreak = 1 + 1 / (mapSize);
        value = 0;

        cols = width;
        rows = height;

        occupancyGridMap.resize(height);
        for (vector<bool> &elem : occupancyGridMap)
        {
            elem.resize(width);
        }
        for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
        {
            for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                if (cost == 0)
                    occupancyGridMap[iy][ix] = 1; // free
                else
                    occupancyGridMap[iy][ix] = 0;
            }
        }

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        ROS_INFO("RRTstar planner initialized successfully");
        initialized_ = true;
        printTime = true;
    }

    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}

bool RRTstar::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    
    if(printTime == true){
        start_global = ros::Time::now().toSec();
        printTime = false;
    }

    float startX = start.pose.position.x; // [m]
    float startY = start.pose.position.y; // [m]

    float goalX = goal.pose.position.x; // [m]
    float goalY = goal.pose.position.y; // [m]

    int startPoint[2], goalPoint[2];

    int startGridSquare;
    int goalGridSquare;

    vector<int> path;
    path.clear();

    Qgoals.clear();
    multiPOIs.clear();

    block = false;

    if (isCoordinateInBounds(startX, startY) && isCoordinateInBounds(goalX, goalY))
    {
        startPoint[0] = int(startX / resolution);
        startPoint[1] = int(startY / resolution);
        goalPoint[0] = int(goalX / resolution);
        goalPoint[1] = int(goalY / resolution);

        addNode(startPoint[0], startPoint[1], 0);
        addNode(goalPoint[0], goalPoint[1], 1);
    }
    else
    {
        cout << "start: (" << startX << ", " << startY << ") " << endl;
        cout << "goal: (" << goalX << ", " << goalY << ") " << endl;
        ROS_WARN("the start or goal is out of the map");
        return false;
    }

    if (isValid(startPoint[0], startPoint[1]) && isValid(goalPoint[0], goalPoint[1]) && block == false)
    {

        path = algorithmManager();

        for (int i = 0; i < path.size(); i++)
        {
            for (int j = 0; j < nodes_final.size(); j++)
            {
                if (path[i] == nodes_final[j].id)
                {

                    geometry_msgs::PoseStamped pose = goal;
                    pose.pose.position.x = (nodes_final[j].cx) * resolution;
                    pose.pose.position.y = (nodes_final[j].cy) * resolution;
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.x = 0.0;
                    pose.pose.orientation.y = 0.0;
                    pose.pose.orientation.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    plan.push_back(pose);
                }
            }
        }
        publishPlan(plan);
        end_global = ros::Time::now().toSec();

        if(plan.size() > 0){
            cout << "Required time to find the path: " << end_global - start_global << " s" << endl;
            cout << "Path length: " << totalLen(path) << " [px]" << endl;
            cout << "Number of rotations: " << numRotations(path) << endl;
        }

        return true;
    }
    else
        return false;
}

///////////////////////////////////////// RRTstar FUNCTIONS ///////////////////////////////////////////////////////////////////

node RRTstar::sampleFree()
{

    node qrand;
    int x;
    int y;
    int id;
    int it = 100000;
    int dim;

    while (it > 0)
    {
        dim = 0;
        it--;
        x = rand() % (cols - dimBox);
        y = rand() % (rows - dimBox);

        if (isValid(x, y))
        {
            for (int i = 0; i < nodes.size(); i++)
            {
                if (x == nodes[i].cx && y == nodes[i].cy)
                { // It's checked if the sample is already in the tree
                    dim = nodes.size();
                    break;
                }
            }
            if (dim == 0)
            { // The found sample is not in the tree and can be proved as output
                samples = samples + 1;
                id = encode(x, y);
                qrand = {x, y, id, 0, 0};
                return qrand;
            }
        }
    }

    cout << " There are not samples in the image" << endl;
    return {-1};
}

vector<node> RRTstar::nearest(node qrand)
{
    // Finds the nearest node (belonging to the three) of qrand

    bool found = false;
    float Dold = 10000;
    float D;
    node qnearest;
    vector<node> qnearest_qrand;
    int itN = 0;
    while (found == false && itN < 1000000)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            D = dist(qrand.cx, qrand.cy, nodes[i].cx, nodes[i].cy);

            if (D < Dold && obstacleFree(qrand, nodes[i], dimBox))
            {
                Dold = D;
                qnearest = nodes[i];
                found = true;
            }
        }
        if (found == true)
        {
            qnearest_qrand.push_back(qnearest);
            qnearest_qrand.push_back(qrand);
            return qnearest_qrand;
        }
        else
        {
            samples--;
            qrand = sampleFree(); // If there is any qnearest it's found a new sample
            itN++;
        }
    }
    cout << "It was not possible to find qnearest" << endl;
    return {{-1}};
}

node RRTstar::steer(node qnearest, node qrand)
{

    node qnew;
    float vx, vy;
    float D;
    int x, y;
    int id, i, c;
    bool valid = false;
    bool present = false;
    int e = eta;
    int e_old;

    D = dist(qrand.cx, qrand.cy, qnearest.cx, qnearest.cy);

    if (D <= eta)
    { //qrand is enough close to qnearest. We check if it is not already in the tree
        for (int k = 0; k < nodes.size(); k++)
        {
            if (qrand.id == nodes[k].id)
                present = true;
        }
        if (present == false)
        {
            qnew = qrand;
            goto End;
        }
    }

    // qrand is further than eta to qnearest. It's necessary to update qrand.
    // The new qrand is looked for on the line joining the two point
    vx = (qrand.cx - qnearest.cx) / D;
    vy = (qrand.cy - qnearest.cy) / D;

    while (!valid)
    {
        x = qnearest.cx + e * vx;
        y = qnearest.cy + e * vy;
        id = encode(x, y);
        qnew = {x, y, id, 0, 0};
        i = 0;
        present = false;
        while (present == false && i < nodes.size())
        {
            if (qnew.id == nodes[i].id)
            { // qnew is already in the tree. It's necessary to decrease e and update qnew
                present = true;
                e--;
                if (e == 0)
                {
                    cout << "Impossible to complete Steer" << endl;
                    return {-1};
                }
            }
            i++;
        }

        if (present == false && i == nodes.size()) // There is a valid qnew
            goto End;
    }

End:

    nodes_parent.push_back(qnearest); // The parents vector is updated
    nodes.push_back(qnew);            // The tree is updated
    return qnew;
}

vector<node> RRTstar::near(node qnew, float r)
{

    // This function looks for other node (already in the tree) close to qnew

    vector<node> Qnear;
    int pro = 0;

    for (int i = 0; i < nodes.size(); i++)
    {
        if (nodes[i].id != qnew.id && dist(qnew.cx, qnew.cy, nodes[i].cx, nodes[i].cy) <= r && obstacleFree(qnew, nodes[i], dimBox))
        {
            Qnear.push_back(nodes[i]);
            pro++;
        }
    }
    if (pro > 0)
    { // It means there is at least one node in Qnear
        return Qnear;
    }
}

vector<int> RRTstar::findPath(node start, node dest, float *c)
{

    vector<int> path;
    int it;
    int x, y, index;
    node qrand;
    node qnearest;
    node qnew;
    vector<node> Qnear, qnearest_qrand;
    float r;
    int m = 0;
    int parent_id;

    nodes.clear();
    nodes_parent.clear();
    CostToCome.clear();
    // Initial conditions
    nodes.push_back(start);
    nodes_parent.push_back(start);
    CostToCome.push_back(0); // This cost is set small because we don't want to change it. The partent of start will be always start
    nodes.push_back(dest);
    nodes_parent.push_back(dest);
    CostToCome.push_back(8000); // This cost is set big because we want to update tha parent of dest node

    samples = 0;

    // Check if start and dest are connectable
    if (obstacleFree(start, dest, dimBox) && dist(start.cx, start.cy, dest.cy, dest.cy < eta))
    {

        path.push_back(start.id);
        path.push_back(dest.id);

        return path;
    }

    while (samples < maxSamples)
    {
        // The loop finishes when there are enough samples
        qrand = sampleFree();
        qnearest_qrand = nearest(qrand);
        qnearest = qnearest_qrand[0];
        qrand = qnearest_qrand[1];
        qnew = steer(qnearest, qrand);

        r = 2 * eta; // Distance limite used in Near function to look for near nodes
        Qnear = near(qnew, r);

        compareCost(Qnear, qnew, qnearest);

        if (nodes.size() == 0)
        {
            cout << "Algorithm execution error" << endl;
            *c = INT_MAX;
            return {-1};
        }
    }

    // There is not a path between start and dest
    if (nodes_parent[1].id == dest.id)
    {
        cout << "Failed algorithm" << endl;
        path.push_back({-1});
        *c = INT_MAX;
        goto FindPathEnd;
    }

    // dest id is insert in path
    path.push_back(dest.id);
    *c = CostToCome[1];
    index = dest.id;

    while (index != start.id && m < nodes.size())
    {
        // Backwards research from dest node to start goal
        for (int i = 0; i < nodes.size(); i++)
        {
            if (index == nodes[i].id)
                parent_id = i;
        }
        index = nodes_parent[parent_id].id;
        path.insert(path.begin(), index);
        m++;
    }

    if (path[0] != start.id || path[path.size() - 1] != dest.id)
    {
        cout << "Unconnected start and goal nodes" << endl;
        path.push_back({-1});
        *c = INT_MAX;
        goto FindPathEnd;
    }

FindPathEnd:
    return path;
}

///////////////////////////////////////// COST FUNCTIONS  ///////////////////////////////////////////////////////////////////

node RRTstar::nodesCost(node q1)
{

    // To each node is associated a cost inversely proportional to the distance from the node to the closest obstacle
    int xc, yc;
    int x, y;

    float lMin;
    float lMax = 10 * dimBox;
    float cost = 0;
    int j, sampling;
    sampling = 32;
    float uno = 1.0;

    xc = q1.cx;
    yc = q1.cy;

    for (float j = 0; j < eta; j++)
    {
        for (int theta = 0; theta <= sampling; theta++)
        {
            x = xc + j * cos(2 * theta * M_PI / sampling);
            y = yc + j * sin(2 * theta * M_PI / sampling);

            if (x > cols || x < 0 || y > rows || y < 0)
            { // The point is outside the image
                cost = abs((eta - j) / eta);
                goto newnode;
            }
            else
            {
                if (isFree(x, y) == 0)
                { // There is a boundery
                    cost = abs((eta - j) / eta);
                    goto newnode;
                }
            }
        }
    }

newnode:
    q1.CostNode = cost;
    cost = 0;
    return q1;
}

float RRTstar::costCurve(Vector2d a, Vector2d b)
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

float RRTstar::costMin(node q1, node q2)
{
    // It calculates the cost to move from q1 to q2

    float CostTot = 0;
    float costdist;
    int m1 = max(w_length, w_degRot);
    int w_safety = max(m1, w_numRot); // Safety weight
    Vector2d v1, v2;
    int index;

    costdist = (dist(q1.cx, q1.cy, q2.cx, q2.cy)) / eta; // eta is used to normalize
    q2 = nodesCost(q2);
    for (int j = 0; j < nodes.size(); j++)
    {
        if (q1.id == nodes[j].id)
            index = j;
    }

    v1 << (q1.cx - nodes_parent[index].cx), (q1.cy - nodes_parent[index].cy);
    v1 = v1;
    v2 << (q2.cx - q1.cx), (q2.cy - q1.cy);
    v2 = v2;

    CostTot = CostToCome[index] + w_length * costdist + w_safety * q2.CostNode + w_degRot * (costCurve(v1, v2)) + w_numRot * numRot;
    return CostTot;
}

void RRTstar::compareCost(vector<node> Qnear, node qnew, node qnearest)
{

    // Based on the cost of each node the parents are updated

    float Costnew_near, Costnear_new, CostMin;
    vector<float> CostNew;
    CostNew.resize(Qnear.size());
    CostMin = costMin(qnearest, qnew);
    int change;

    for (int i = 0; i < Qnear.size(); i++)
    { // Check to understand if it's more advantageous to connect qnew to qnear instead of to qnearest based on the cost function
        CostNew[i] = costMin(Qnear[i], qnew);
        if (CostNew[i] <= CostMin)
        {
            CostMin = CostNew[i];
            int l = nodes_parent.size() - 1;
            nodes_parent.at(l) = Qnear[i];
        }
    }

    CostToCome.push_back(CostMin);

    for (int i = 0; i < Qnear.size(); i++)
    { // Parents update
        Costnew_near = costMin(qnew, Qnear[i]);
        for (int k = 0; k < nodes.size(); k++)
        {
            if (Qnear[i].id == nodes[k].id)
                change = k;
        }
        if (Costnew_near < CostToCome[change])
        {
            CostMin = Costnew_near;

            CostToCome.at(change) = CostMin;
            nodes_parent.at(change) = qnew;
        }
    }
}

////////////////////////////////////////////////////// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////////////

bool RRTstar::isValid(int x, int y)
{

    // Check if the point is inside the image and it is not in an obstacle

    return (isFree(x, y) != 0) && (x >= 0) && (x < cols) && (y >= 0) && (y < rows);
}

float RRTstar::dist(float x1, float y1, float x2, float y2)
{

    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void RRTstar::addNode(int x, int y, int SoG)
{

    int id;

    if (x >= 0 && x < cols && y > 0 && y < rows)
    {

        if (isFree(x, y) == 0)
        {
            cout << "The position (" << x << "," << y << ") correspond to a position with an obstacle. " << endl;
            block = true;
            return;
        }
        else
        {
            if (isFree(x + 1, y + 1) == 0)
            {
                if (isFree(x - 1, y - 1) == 0)
                {
                    cout << "The position (" << x << "," << y << ") correspond to a position with an obstacle." << endl;
                    block = true;
                    return;
                }
            }
        }

        if (SoG == 0)
        { // start node

            id = encode(x, y);
            qstart = {x, y, id, 0, 2};
        }

        else
        { // goal node

            id = encode(x, y);
            Qgoals.push_back({x, y, id, 1, 2});
        }

        n_nodes++;
    }
    else
        cout << "The position of start and/or goal is wrong" << endl;
}

void RRTstar::addPOIs(int x, int y, string nameId)
{

    if (isValid(x, y))
        POIs.push_back({x, y, nameId});
    else
        cout << " The POI insert in: (" << x << "," << y << ") is not valid" << endl;
}

void RRTstar::checkPOIs()
{

    vector<int> save; // It contains the index of multiPOIs
    vector<vector<int>> multi;
    int dim = 0;
    int index;
    int id;
    int l = 1;
    int dim_s = 0;

    for (int i = 0; i < POIs.size(); i++)
    {
        if (count(save.begin(), save.end(), i) == 0)
        { // Check if i-multiPOIs is already in save
            for (int j = 0; j < POIs.size(); j++)
            {
                if (i != j && (POIs[i].nameId).compare(POIs[j].nameId) == 0)
                { // Check if there are double POIs
                    save.push_back(j);
                }
            }
            if (save.size() > dim) // A double of i-POI was found, i is added to save
                save.push_back(i);
        }

        if (dim_s == 0 && save.size() > 0)
            multi.resize(l);

        dim_s = save.size();

        if (dim_s > dim)
        { // If there are new multiPOIs, multi is updated
            multi[l - 1].insert(multi[l - 1].end(), save.begin() + dim, save.end());
            l++;
        }

        dim = dim_s;
    }

    for (int i = 0; i < POIs.size(); i++)
    { // Update Qgoals with POIs nodes
        if (count(save.begin(), save.end(), i) == 0)
        {
            id = encode(POIs[i].x, POIs[i].y);
            Qgoals.push_back({POIs[i].x, POIs[i].y, id, 0, -1});
        }
    }

    if (multi.size() > 0)
    { // Update multiPOIs with multiPOIs nodes
        multiPOIs.resize(multi.size());
        for (int i = 0; i < multi.size(); i++)
        {
            for (int j = 0; j < multi[i].size(); j++)
            {
                index = multi[i][j];
                id = encode(POIs[index].x, POIs[index].y);
                multiPOIs[i].push_back({POIs[index].x, POIs[index].y, id, 0, -2});
            }
        }
    }
}

vector<node> RRTstar::convert(vector<int> goals)
{

    // Converts id into node

    vector<node> QgoalsTmp;

    for (int i = 0; i < goals.size(); i++)
    {
        for (int j = 0; j < Qgoals.size(); j++)
        {
            if (goals[i] == Qgoals[j].id)
                QgoalsTmp.push_back(Qgoals[j]);
        }

        if (QgoalsTmp.size() < goals.size())
        {
            for (int k = 0; k < multiPOIs.size(); k++)
            {
                for (int l = 0; l < multiPOIs[k].size(); l++)
                {
                    if (goals[i] == multiPOIs[k][l].id)
                        QgoalsTmp.push_back(multiPOIs[k][l]);
                }
            }
        }
    }

    return QgoalsTmp;
}

vector<vector<int>> RRTstar::findMultiPOIs(vector<int> aG)
{

    // This function returns a matrix of int where on each row there are every goal and each different POIs
    vector<vector<int>> Combinations, multiGoalsTmp;
    vector<int> aGTmp;

    Combinations = comb(); // Combination of possibile POIs chosen inside multiPOIs
    for (int i = 0; i < Combinations.size(); i++)
    {
        aGTmp = aG; // goals
        aGTmp.insert(aGTmp.end(), Combinations[i].begin(), Combinations[i].end());
        multiGoalsTmp.push_back(aGTmp); // multiPOIs are added to goals and are stored
        aGTmp.clear();
    }
    return multiGoalsTmp;
}

vector<vector<int>> RRTstar::comb()
{

    // It returns a matrix of int where on each row there are every different POIs

    int dim = 1;
    vector<int> c;
    vector<vector<int>> C;
    bool valid = false;
    int k;

    for (int i = 0; i < multiPOIs.size(); i++)
        dim = dim * multiPOIs[i].size();

    while (C.size() < dim)
    {

        for (int i = 0; i < multiPOIs.size(); i++)
        {
            k = random() % multiPOIs[i].size();
            c.push_back(multiPOIs[i][k].id);
        }

        while (!valid)
        {
            valid = checkValid(c, C); // Check if the combination c was alredy found
            if (valid)
                break;
            c = update(c); // If c is already in C (matrix of all combinations found), it is updated
        }

        C.push_back(c);
        valid = false;
    }

    return C;
}

bool RRTstar::checkValid(vector<int> c, vector<vector<int>> C)
{

    // Check if the combination c was alredy in C

    int l;
    bool stop;

    for (int j = 0; j < C.size(); j++)
    {
        stop = false;
        l = 0;
        while (!stop)
        {
            if (count(C[j].begin(), C[j].end(), c[l]) > 0)
                l++;
            else
                stop = true;

            if (l == c.size())
                return false;
        }
    }

    return true;
}

vector<int> RRTstar::update(vector<int> c)
{

    // Modifies c

    int k, k_val;

    k = rand() % multiPOIs.size();
    k_val = rand() % multiPOIs[k].size();
    c.at(k) = k_val;
    return c;
}

vector<int> RRTstar::sortGoals(vector<int> goals, int n)
{

    // It sorts goals based on the priority assigned in main.cpp

    int index;
    int it = 0;
    int i = n;
    vector<int> tmpG;
    int ng = goals.size();
    tmpG.resize(ng);

    for (int j = 0; j < n; j++)
    {
        index = goalsPriority[j];
        tmpG[j] = goals[index];
    }
    while (i < ng)
    {
        if (count(goalsPriority.begin(), goalsPriority.end(), it) == 0)
        {
            tmpG[i] = goals[it];
            i++;
        }
        it++;
    }
    goals = tmpG;
    return goals;
}

void RRTstar::findPermutations(vector<int> a, int n)
{

    int loop = 0;
    vector<int> aux;
    aux.resize(n);

    sort(a.begin(), a.end()); // a, a+n

    do
    {
        multiGoals.push_back(aux);
        for (int i = 0; i < n; i++)
        {
            multiGoals[loop][i] = a[i];
        }
        loop = loop + 1;
    } while (next_permutation(a.begin(), a.end()));
}

int RRTstar::factorial(int n)
{

    int count = 1;
    for (int i = n; i > 1; i--)
    {
        count = count * i;
    }
    return count;
}

vector<int> RRTstar::algorithmManager()
{

    // This function manage the presence of priorities or multiPOIs

    vector<int> Path, pathTmp, aux;
    int np = goalsPriority.size();
    int nm, ng;
    vector<int> goals;
    vector<node> nodesTmp, nodesTmp_old, QgoalsTmp;
    vector<node> nodes_parentsTmp, nodes_parentsTmp_old;

    checkPOIs(); // Update Qgoals and multiPOIs based on the presence of POIs and multiPOIs

    nm = multiPOIs.size();
    ng = Qgoals.size();

    node s, g;
    float costOld, costNew, costTmp;
    costOld = INT_MAX;
    cost_final = INT_MAX;
    int nPermutations;

    vector<int> aG;

    nPermutations = factorial(ng);
    for (int i = 0; i < ng; i++)
    {
        aG.push_back(Qgoals[i].id);
    }

    findPermutations(aG, ng);

    costNew = 0;
    goals = multiGoals[0]; // Single goals sequence

    QgoalsTmp = convert(goals); // ids are converted into nodes

    s = qstart;
    g = QgoalsTmp[0];

    pathTmp = findPath(s, g, &costTmp); // It calculates the path from s to g nodes

    if (costTmp == INT_MAX)
    {
        goto stopInternalLoop1; // s and g are not connectable. Break
    }

    // Sub-paths found saving
    if (aux.size() == 0)
    {
        aux = pathTmp;
        nodesTmp = nodes;
        nodes_parentsTmp = nodes_parent;
    }

    costNew = costNew + costTmp;

    if (costNew < costOld)
    {
        // A better path was found
        Path = aux;
        costOld = costNew;
        cost_final = costOld;
        nodes_final = nodesTmp;
        nodes_parent_final = nodes_parentsTmp;
    }

stopInternalLoop1:
    aux.clear();

    return Path;
}

bool RRTstar::obstacleFree(node q1, node q2, int d)
{

    // Check if q1 and q2 are safetly connectable with a path dimBox-width

    float x1, x2, y1, y2;
    x1 = q1.cx;
    x2 = q2.cx;
    y1 = q1.cy;
    y2 = q2.cy;

    float mx = float(x2 - x1);
    float my = float(y2 - y1);

    // d := Bounding Box dimension

    int num_loop = max(abs(mx), abs(my));

    mx = (mx / num_loop);
    my = (my / num_loop);

    int x = x1;
    int y = y1;

    int cell_id;

    for (int i = 1; i <= num_loop; i++)
    {

        for (int j = -d; j <= d; j++)
        {
            x = int(x1 + j * my + mx * i) + 1;
            y = int(y1 + j * mx + my * i) + 1;

            if (x >= 0 && y >= 0 && x < cols && y < rows)
            {

                if (isFree(x, y) == 0)
                { // It is not obstacle free

                    return false;
                }
            }
        }
    }

    return true;
}

float RRTstar::totalLen(vector<int> path)
{

    // Calculates the length of the path (#pixel)

    float TotLen = 0;
    int id_new, id;
    for (int i = 1; i < path.size(); i++)
    {
        for (int j = 0; j < nodes_final.size(); j++)
        {
            if (path[i] == nodes_final[j].id)
                id = j;
        }
        TotLen += dist(nodes_parent_final[id].cx, nodes_parent_final[id].cy, nodes_final[id].cx, nodes_final[id].cy);
    }

    return TotLen;
}

int RRTstar::encode(int x, int y)
{

    // Univocal id based on coordinates

    int id;

    id = x * rows + y;
    return id;
}

int RRTstar::numRotations(vector<int> p)
{

    int num_rot = 0;
    int id_old, id, id_next;
    node q0, q1, q2;
    Vector2d v1, v2;

    for (int i = 1; i < p.size() - 1; i++)
    {

        for (int j = 0; j <= nodes_final.size(); j++)
        {

            if (p[i - 1] == nodes_final[j].id)
                id_old = j;
            if ((p[i] == nodes_final[j].id))
                id = j;
            if ((p[i - 1] == nodes_final[j].id))
                id_next = j;
        }

        q0 = nodes_final[id_old];
        q1 = nodes_final[id];
        q2 = nodes_final[id_next];

        v1 << (q1.cx - q0.cx), (q1.cy - q0.cy);
        v1 = v1;
        v2 << (q2.cx - q1.cx), (q2.cy - q1.cy);
        v2 = v2;

        (costCurve(v1, v2));

        num_rot += numRot;
    }

    return num_rot;
}

void RRTstar::coordinate(vector<int> p)
{

    for (int i = 0; i < p.size(); i++)
    {
        for (int j = 0; j < nodes_final.size(); j++)
        {
            if (p[i] == nodes_final[j].id)
                cout << "node with id: " << p[i] << " and coordinates ( x = " << nodes_final[j].cx << ", y = " << nodes_final[j].cy << ")" << endl;
        }
    }
}

bool RRTstar::isCoordinateInBounds(float x, float y)
{
    //  Function to check if gridSquare coordinates are in map bounds
    bool valid = true;

    if (x > (width * resolution) || y > (height * resolution))
        valid = false;

    return valid;
}

bool RRTstar::isFree(int i, int j)
{
    // Checks if gridSquare at (i,j) is free
    return (occupancyGridMap[j][i]);
}

void RRTstar::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}