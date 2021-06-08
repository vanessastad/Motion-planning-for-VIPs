#include "ga_gp.h" // "MyNavigation/ga_gp.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(GA, nav_core::BaseGlobalPlanner)

GA::GA()
{
}

GA::GA(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros);
}

/////////////////////////////////////////  OVERRIDDEN CLASS from interface nav_core::BaseGlobalPlanner  /////////////////////////////////////////////////////////////////////////////////

void GA::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
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
        int value = 0;

        cols = width;
        rows = height;

        Max = sqrt(pow(rows, 2) + pow(cols, 2));


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
        ROS_INFO("GA planner initialized successfully");
        initialized_ = true;
        printTime = true;
    }

    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}

bool GA::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
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

    nodes.clear();

    numGenerations = 0;
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

    if (isValid(startPoint[0], startPoint[1]) && isValid(goalPoint[0], goalPoint[1]) && (block == false))
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
    {
        return false;
    }
}

//////////////////////////////// GA FUNCTIONS////////////////////////////////////////////////

vector<path> GA::initialPopulation(node start, node goal, int numIndividuals)
{
    // This function create the first generation

    vector<path> firstGeneration; // First generation(vector of individuals)
    graph Graph;                  // Nodes and arcs based on paths are found
    Graph = createGraph(start, goal);
    int dim = Graph.seq.size() - 1; // Numer of steps necessary to complete the graph
    vector<node> p;
    node qnew;
    int j, ind_rand, count;
    bool insert = true;
    int i = 0;
    int inc = 0;

    if (error == true)
    { // global variable set in createGraph()
        cout << "There is not initial population" << endl;
        firstGeneration.push_back({{{-1}}, -1});
        return firstGeneration;
    }

    while (i < numIndividuals && inc < 100)
    {                                   // We remain in the loop untile there are enough individuals or it is not possible to find new individuals
        p.push_back(Graph.id_son[dim]); // goal node is insert
        j = dim;

        while (j != 0)
        { // It means we are back at start node

            ind_rand = rand() % Graph.id_p[j].size(); // Random selection of a parent index

            qnew = Graph.id_p[j][ind_rand]; // Selection of the parent
            p.push_back(qnew);              // The parent is insert in the path
            for (int k = dim; k > 0; k--)
            {
                if (Graph.id_son[k - 1].id == qnew.id)
                { // Based on the chosen path, we look for a parent of the parent
                    j = k - 1;
                    break;
                }
            }
        }

        insert = true;
        for (int m = 0; m < firstGeneration.size(); m++)
        { // Check if this path was already found
            count = 0;
            for (int n = 0; n < firstGeneration[m].path.size(); n++)
            {
                if (p.size() == firstGeneration[m].path.size())
                {
                    if (p[n].id == firstGeneration[m].path[n].id)
                    {
                        count++;
                    }
                }
            }

            if (count == firstGeneration[m].path.size())
            { // The path was already found
                insert = false;
                p.clear();
                break;
            }
        }
        if (insert)
        { // The path wasn't already found and it is insert in the firstGeneration
            firstGeneration.push_back({p, 0});
            i++;
            inc = 0;
            p.clear();
        }
        else
            inc++;
    }

    return firstGeneration;
}

graph GA::createGraph(node q1, node q2)
{
    // This function builds a graph based on which we extract different individuals of our first generation

    int seq = 0;
    node q, qold;
    vector<node> save;
    vector<node> last_added_n; // It saves the nodes insert in the graph
    int lim;
    int qold_it;
    bool inside;
    int it_max = 500; //500;

    qold = q1; // Start
    q = q1;

    // The global variable Graph is clean in order to use it again in case of multi goals
    Graph.seq.clear();
    Graph.id_son.clear();
    Graph.id_p.clear();
    Graph.seq.push_back(seq);
    Graph.id_son.push_back(q1);
    Graph.id_p.push_back({q1});
    last_added_n.push_back(q1);
    seq++;
    error = false;

    while (!obstacleFree(q, q2, dimBox) && seq < it_max)
    { // We remain in the loop until there is a conncection between the current q and goal or until the iteration reach the maximum value

    Start:

        q = randomNode();

        if (!obstacleFree(qold, q, dimBox))
        {

            q = findFurther(qold, q); // It returns the further valid node from qold on the line q-qold

            if (q.cx = -1)
                goto Start;
        }

        lim = last_added_n.size() - 1;
        lim = min(m, lim);

        save = sortDist(last_added_n, lim, qold, q);

        inside = false;
        for (int i = 0; i < last_added_n.size(); i++)
        { // Check if q node is already in last_added_n (already found)
            if (q.id == last_added_n[i].id)
            {
                inside = true;
                break;
            }
        }

        if (inside == false)
        {
            last_added_n.push_back(q);
            Graph.seq.push_back(seq);
            Graph.id_son.push_back(q);
            Graph.id_p.push_back(save);

            if (seq % 100 == 0)
            {
                cout << "seq: " << seq << endl;
            }

            seq++;
        }

        qold_it = rand() % last_added_n.size();
        qold = last_added_n[qold_it];
    }

    if (seq == it_max)
    {
        cout << "Path not found" << endl;
        error = true;
        return Graph;
    }

    lim = last_added_n.size() - 1;
    lim = min(m, lim);

    save = sortDist(last_added_n, lim, q, q2);
    Graph.seq.push_back(seq);
    Graph.id_son.push_back(q2);
    Graph.id_p.push_back(save);

    return Graph;
}

vector<node> GA::sortDist(vector<node> last_added_n, int lim, node qold, node q)
{
    // It sorts nodes from the closest to the further respect to q and check if q and those nodes are obstacleFee.
    // lim node are connected to q

    vector<float> sorted;       // Sorted nodes from the closest to the further respect to q
    vector<vector<float>> Dist; // Contiene la distanza tra il nodo considerato e q e l'indice relativo del nodo considerato
    float d, flo;
    int min_ind;
    vector<node> sort, save;
    int stop = 0;
    int cont = 0;
    int k = 0;
    int x, y;
    float s = 5000;
    int memory;

    for (int i = 0; i < last_added_n.size(); i++)
    {
        x = last_added_n[i].cx;
        y = last_added_n[i].cy;
        d = dist(x, y, q.cx, q.cy);
        flo = float(last_added_n[i].id);
        Dist.push_back({d, flo}); // In each row we save the distance bewteen q and another node, and the index of this last node.
    }

    while (k < last_added_n.size())
    { // sort
        for (int i = 0; i < Dist.size(); i++)
        {
            if (Dist[i][0] < s)
            {
                memory = i;
                s = Dist[i][0];
            }
        }

        sorted.push_back(Dist[memory][1]);
        Dist.erase(Dist.begin() + memory);
        k++;
    }
    save = convertReturn(sorted); // Index is converted in node

    while (cont < lim && stop < save.size())
    {
        if (obstacleFree(save[stop], q, dimBox) && save[stop].id != qold.id)
        {

            sort.push_back(save[stop]);
            cont++;
        }
        stop++;
    }
    sort.push_back(qold);

    return sort;
}

node GA::randomNode()
{

    node qrand;
    int x;
    int y;
    int id;
    int it = 10000;
    int dim;

    while (it > 0)
    {
        dim = 0;
        it--;

        x = rand() % (cols - dimBox);
        y = rand() % (rows - dimBox);

        if (isValid(x, y))
        {
            for (int i = 0; i < Graph.id_son.size(); i++)
            {
                if (x == Graph.id_son[i].cx && y == Graph.id_son[i].cy)
                { // Check if the node is not already in the graph
                    dim = Graph.id_son.size();
                    break;
                }
            }
            if (dim == 0)
            { // If the node is not already in the graph
                id = encode(x, y);

                return qrand = {x, y, id, 0, 0};
            }
        }
    }

    cout << "There are not samples in the image" << endl;
    return {-1};
}

node GA::findFurther(node q1, node q2)
{
    // It provides the further valid node respect to qold on the line q-qold

    float D;
    float vx, vy;
    int x, y, id, xc, yc;
    node q = q2;
    int j = 1;
    int sampling = 32;
    bool ok = true;
    bool valid = false;
    int dim;

    while (!valid && j < D)
    {

        D = dist(q1.cx, q1.cy, q2.cx, q2.cy);
        vx = (q1.cx - q2.cx) / D;
        vy = (q1.cy - q2.cy) / D;

        x = q2.cx + j * vx;
        y = q2.cy + j * vy;

        id = encode(x, y);

        q = {x, y, id, 0, 0};

        if (obstacleFree(q, q1, dimBox) && isValid(x, y))
        {

            // if q is valid, we move dimBox far from the obstacle
            x = x + vx * dimBox;
            y = y + vy * dimBox;

            // We check if the new node has a neighbourhood empty of obstacles
            for (int theta = 0; theta <= sampling; theta++)
            {
                xc = x + dimBox * cos(2 * theta * M_PI / sampling);
                yc = y + dimBox * sin(2 * theta * M_PI / sampling);

                if (!(isValid(xc, yc)))
                {
                    ok = false;
                    break;
                }
            }
            if (ok == true)
            {
                id = encode(x, y);
                q = {x, y, id, 0, 0};

                if (obstacleFree(q, q1, dimBox))
                {
                    for (int i = 0; i < Graph.id_son.size(); i++)
                    {
                        if (x == Graph.id_son[i].cx && y == Graph.id_son[i].cy)
                        { // Viene controllato che il campione trovato non sia già nell'albero
                            dim = Graph.id_son.size();
                            break;
                        }
                    }

                    if (dim == 0)

                        return q;
                }
            }
        }
        j++;
    }

    return {-1};
}

vector<path> GA::fitness(vector<path> population)
{
    // A score is attributed to each path based on the cost of the path.
    // A higher score means a worse path

    vector<node> pop;
    float c;

    for (int i = 0; i < population.size(); i++)
    {
        pop = population[i].path;
        c = costMin(pop);

        population.at(i).score = c;
    }

    return population;
}

vector<path> GA::Sort(vector<path> population)
{
    // The paths are sorted based on the fitness score

    vector<path> sorted;
    vector<path> populationTmp = population;
    int max_ind, end;
    float s;
    int memory;

    while (populationTmp.size() > 0)
    {

        s = INT_MAX;

        for (int i = 0; i < populationTmp.size(); i++)
        {
            if (populationTmp[i].score < s)
            {
                memory = i;
                s = populationTmp[memory].score;
            }
        }

        sorted.push_back(populationTmp[memory]);
        populationTmp.erase(populationTmp.begin() + memory);
    }

    return sorted;
}

vector<path> GA::memory(vector<path> sortedPopulation, vector<path> pathParents, int sizeOffspring)
{
    // It saves paths from the old generation to bring them in the new one

    int addPath = sortedPopulation.size() - sizeOffspring;

    vector<path> save;
    int rand_int;
    int count_zero, count_one, update, i, select, son, save_it;

    update = 0;
    i = 0;
    son = 0;
    save_it = 0;

    while (update < pathParents.size() && i < sortedPopulation.size() && son < sizeOffspring)
    {
        // The path already used in crossovermutation() are deleted from sortedPopulation because
        // we want to bring in the new generation different paths

        count_zero = 0;
        count_one = 0;
        if (pathParents[0].path.size() == sortedPopulation[i].path.size())
        {

            for (int j = 0; j < sortedPopulation[i].path.size(); j++)
            {
                if (pathParents[0].path[j].id == sortedPopulation[i].path[j].id)
                {
                    count_zero++; // We increase it each timesortedPopulation[i].id is equal to pathParents[0]
                }
            }
        }
        if (pathParents[1].path.size() == sortedPopulation[i].path.size())
        {
            for (int j = 0; j < sortedPopulation[i].path.size(); j++)
            {
                if (pathParents[1].path[j].id == sortedPopulation[i].path[j].id)
                {
                    count_one++;
                }
            }
        }
        if (count_zero == sortedPopulation[i].path.size() || count_one == sortedPopulation[i].path.size())
        { // If count_zero or count_one are equal to the size of sortedPopulation[i] , it means we have found a double and it has to be deleted
            sortedPopulation.erase(sortedPopulation.begin() + i);
            update++;
            son++;
        }
        else
            i++;
    }

    // Based on this valude there will be selected the path with the best score, with the worse score or in a random way
    select = rand() % 10;

    save.resize(addPath);

    if (select < 7)
    { // Selection of paths with the best fitness score (lowest)

        for (int j = 0; j < addPath; j++)
        {

            save[j].path = sortedPopulation[j].path;
            save[j].score = sortedPopulation[i].score;
        }
    }
    else if (select == 7)
    { // Selection of paths with the worst fitness score (highest)

        int limit = sortedPopulation.size() - addPath - 1;
        int j_it = sortedPopulation.size() - 1;

        while (j_it != limit && save_it < addPath)
        {

            save[save_it].path = sortedPopulation[j_it].path;
            save[save_it].score = sortedPopulation[j_it].score;

            j_it--;
            save_it++;
        }
    }
    else
    { // Selection random of paths

        for (int j = 0; j < addPath; j++)
        {

            rand_int = rand() % sortedPopulation.size();
            save[j].path = sortedPopulation[rand_int].path;
            save[j].score = sortedPopulation[rand_int].score;
        }
    }

    return save;
}

vector<path> GA::select(vector<path> sortedPopulation)
{
    // Selection of paths to use as parents in crossovermutation()

    vector<path> pathParents;
    int rand_int = -1;
    int select = rand() % 10;
    int num_parents = 2;
    int j_it = sortedPopulation.size() - 1;
    int stop = 0;
    int rand_int_old = -1;

    if (select < 7)
    {
        for (int i = 0; i < num_parents; i++)
            pathParents.push_back(sortedPopulation[i]);
    }

    else if (select == 7)
    {
        while (stop != num_parents)
        {
            pathParents.push_back(sortedPopulation[j_it]);
            j_it--;
            stop++;
        }
    }
    else
    {
        for (int i = 0; i < num_parents; i++)
        {
            while (rand_int == rand_int_old)
                rand_int = rand() % sortedPopulation.size();
            rand_int_old = rand_int;
            pathParents.push_back(sortedPopulation[rand_int]);
        }
    }

    return pathParents;
}

vector<path> GA::crossoverMutation(vector<path> pathParents, int numPopulation)
{
    // Based on a selector it executed crossover and/or mutation

    float select;
    node qnew;
    int k, ind, element;
    vector<int> index;
    vector<node> change;
    path offspring_one;      // First son of crossover
    path offspring_two;      // Second son of crossover
    path offspring_mut;      // son from mutation
    vector<path> offsprings; // sons
    int cont;
    int it = 10000;
    int dim = pathParents[0].path.size() - 2;
    int stop = min(5, dim);
    bool mut = false;
    bool cros = false;
    dim = pathParents[1].path.size() - 2;
    stop = min(stop, dim);
    select = float(rand() % 100) / 100;
    int select_cross = rand() % 3;
    int limOne, limTwo;
    int q;
    limOne = pathParents[0].path.size() / 2;
    limTwo = pathParents[1].path.size() / 2;
    float prob;

    offspring_one.path = pathParents[0].path;
    offspring_two.path = pathParents[1].path;

    prob = float(numGenerations) / TotGenerations;

    if (select <= prob)
    { // crossover
        if (select_cross == 0)
        { //first stop elements are changed

            while (stop < pathParents[0].path.size() - 1 && stop < pathParents[1].path.size() - 1)
            {
                change.clear();
                if (obstacleFree(pathParents[0].path[stop - 1], pathParents[1].path[stop], dimBox) && obstacleFree(pathParents[1].path[stop - 1], pathParents[0].path[stop], dimBox))
                {
                    for (int p = 0; p < 2; p++)
                    {

                        for (int i = 0; i < stop; i++)
                        {
                            change.push_back(pathParents[p].path[i]);
                        }
                    }

                    for (int i = 0; i < stop; i++)
                    {
                        offspring_one.path.at(i) = change[stop + i];
                        offspring_two.path.at(i) = change[i];
                    }

                    for (int n = 0; n < offspring_one.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_one.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + j, offspring_one.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + n, offspring_one.path.begin() + j);
                                }
                            }
                        }
                    }

                    for (int n = 0; n < offspring_two.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_two.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + j, offspring_two.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + n, offspring_two.path.begin() + j);
                                }
                            }
                        }
                    }

                    offsprings.resize(pathParents.size());
                    offsprings[0] = offspring_one;
                    offsprings[1] = offspring_two;

                    return offsprings;
                }
                else
                {
                    stop++;
                }
            }
        }
        else if (select_cross == 1 && limOne > stop && limTwo > stop)
        { // Stop elements in the middle of the path are changed

            while (stop < pathParents[0].path.size() - 1 && stop < pathParents[1].path.size() - 1)
            {
                q = 0;
                change.clear();
                if (obstacleFree(pathParents[0].path[limOne - 1], pathParents[1].path[limTwo], dimBox) && obstacleFree(pathParents[1].path[limTwo - 1], pathParents[0].path[limOne], dimBox))
                {
                    for (int p = 0; p < 2; p++)
                    {

                        for (int i = pathParents[p].path.size() / 2; i < pathParents[p].path.size() / 2 + stop; i++)
                        {
                            change.push_back(pathParents[p].path[i]);
                        }
                    }

                    for (int i = limTwo; i < limTwo + stop; i++)
                    {
                        offspring_two.path.at(i) = change[q];
                        q++;
                    }
                    for (int i = limOne; i < limOne + stop; i++)
                    {
                        offspring_one.path.at(i) = change[q];
                        q++;
                    }

                    for (int n = 0; n < offspring_one.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_one.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + j, offspring_one.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + n, offspring_one.path.begin() + j);
                                }
                            }
                        }
                    }

                    for (int n = 0; n < offspring_two.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_two.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + j, offspring_two.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + n, offspring_two.path.begin() + j);
                                }
                            }
                        }
                    }

                    offsprings.resize(pathParents.size());
                    offsprings[0] = offspring_one;
                    offsprings[1] = offspring_two;

                    return offsprings;
                }
                else
                {
                    stop++;
                }
            }
        }
        else
        { //last stop elements are changed

            while (stop < pathParents[0].path.size() - 1 && stop < pathParents[1].path.size() - 1)
            {
                q = 0;
                change.clear();
                if (obstacleFree(pathParents[0].path[pathParents[0].path.size() - stop - 1], pathParents[1].path[pathParents[1].path.size() - stop], dimBox) && obstacleFree(pathParents[1].path[pathParents[1].path.size() - stop - 1], pathParents[0].path[pathParents[0].path.size() - stop], dimBox))
                {
                    for (int p = 0; p < 2; p++)
                    {

                        for (int i = pathParents[p].path.size() - 1; i > pathParents[p].path.size() - 1 - stop; i--)
                        {
                            change.push_back(pathParents[p].path[i]);
                        }
                    }

                    for (int i = pathParents[1].path.size() - 1; i > pathParents[1].path.size() - 1 - stop; i--)
                    {
                        offspring_two.path.at(i) = change[q];
                        q++;
                    }
                    for (int i = pathParents[0].path.size() - 1; i > pathParents[0].path.size() - 1 - stop; i--)
                    {
                        offspring_one.path.at(i) = change[q];
                        q++;
                    }
                    for (int n = 0; n < offspring_one.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_one.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + j, offspring_one.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + n, offspring_one.path.begin() + j);
                                }
                            }
                        }
                    }

                    for (int n = 0; n < offspring_two.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_two.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + j, offspring_two.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + n, offspring_two.path.begin() + j);
                                }
                            }
                        }
                    }

                    offsprings.resize(pathParents.size());
                    offsprings[0] = offspring_one;
                    offsprings[1] = offspring_two;

                    return offsprings;
                }
                else
                {
                    stop++;
                }
            }
        }

        //        cout << "It is not possible to accomplish crossover" << endl;

        offspring_one.score = -1;

        offsprings.resize(1);
        offsprings[0] = offspring_one;

        return offsprings;
    }
    else if (select <= (3 + prob) / 4 && numPopulation > 2)
    { // crossover and mutazione

        //crossover

        if (select_cross == 0)
        { //First stop elements are changed

            while (stop < pathParents[0].path.size() - 1 && stop < pathParents[1].path.size() - 1)
            {
                change.clear();
                if (obstacleFree(pathParents[0].path[stop - 1], pathParents[1].path[stop], dimBox) && obstacleFree(pathParents[1].path[stop - 1], pathParents[0].path[stop], dimBox))
                {
                    for (int p = 0; p < 2; p++)
                    {

                        for (int i = 0; i < stop; i++)
                        {
                            change.push_back(pathParents[p].path[i]);
                        }
                    }

                    for (int i = 0; i < stop; i++)
                    {
                        offspring_one.path.at(i) = change[stop + i];
                        offspring_two.path.at(i) = change[i];
                    }

                    for (int n = 0; n < offspring_one.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_one.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + j, offspring_one.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + n, offspring_one.path.begin() + j);
                                }
                            }
                        }
                    }

                    for (int n = 0; n < offspring_two.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_two.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + j, offspring_two.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + n, offspring_two.path.begin() + j);
                                }
                            }
                        }
                    }

                    offsprings.resize(pathParents.size());
                    offsprings[0] = offspring_one;
                    offsprings[1] = offspring_two;
                    cros = true;
                    stop = pathParents[0].path.size() - 1;
                }
                else
                {
                    stop++;
                }
            }
        }
        else if (select_cross == 1 && limOne > stop && limTwo > stop)
        { // stop element in the middle fo path are changed

            while (stop < pathParents[0].path.size() - 1 && stop < pathParents[1].path.size() - 1)
            {
                q = 0;
                change.clear();
                if (obstacleFree(pathParents[0].path[limOne - 1], pathParents[1].path[limTwo], dimBox) && obstacleFree(pathParents[1].path[limTwo - 1], pathParents[0].path[limOne], dimBox))
                {
                    for (int p = 0; p < 2; p++)
                    {

                        for (int i = pathParents[p].path.size() / 2; i < pathParents[p].path.size() / 2 + stop; i++)
                        {
                            change.push_back(pathParents[p].path[i]);
                        }
                    }

                    for (int i = limTwo; i < limTwo + stop; i++)
                    {
                        offspring_two.path.at(i) = change[q];
                        q++;
                    }
                    for (int i = limOne; i < limOne + stop; i++)
                    {
                        offspring_one.path.at(i) = change[q];
                        q++;
                    }

                    for (int n = 0; n < offspring_one.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_one.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + j, offspring_one.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + n, offspring_one.path.begin() + j);
                                }
                            }
                        }
                    }

                    for (int n = 0; n < offspring_two.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_two.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + j, offspring_two.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + n, offspring_two.path.begin() + j);
                                }
                            }
                        }
                    }

                    offsprings.resize(pathParents.size());
                    offsprings[0] = offspring_one;
                    offsprings[1] = offspring_two;
                    cros = true;
                    stop = pathParents[0].path.size() - 1;
                }
                else
                {
                    stop++;
                }
            }
        }
        else
        { // last stop elements are changed

            while (stop < pathParents[0].path.size() - 1 && stop < pathParents[1].path.size() - 1)
            {
                q = 0;
                change.clear();
                if (obstacleFree(pathParents[0].path[pathParents[0].path.size() - stop - 1], pathParents[1].path[pathParents[1].path.size() - stop], dimBox) && obstacleFree(pathParents[1].path[pathParents[1].path.size() - stop - 1], pathParents[0].path[pathParents[0].path.size() - stop], dimBox))
                {
                    for (int p = 0; p < 2; p++)
                    {

                        for (int i = pathParents[p].path.size() - 1; i > pathParents[p].path.size() - 1 - stop; i--)
                        {
                            change.push_back(pathParents[p].path[i]);
                        }
                    }

                    for (int i = pathParents[1].path.size() - 1; i > pathParents[1].path.size() - 1 - stop; i--)
                    {
                        offspring_two.path.at(i) = change[q];
                        q++;
                    }

                    for (int i = pathParents[0].path.size() - 1; i > pathParents[0].path.size() - 1 - stop; i--)
                    {
                        offspring_one.path.at(i) = change[q];
                        q++;
                    }

                    for (int n = 0; n < offspring_one.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_one.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + j, offspring_one.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_one.path[n].id == offspring_one.path[j].id)
                                {
                                    offspring_one.path.erase(offspring_one.path.begin() + n, offspring_one.path.begin() + j);
                                }
                            }
                        }
                    }

                    for (int n = 0; n < offspring_two.path.size(); n++)
                    {
                        for (int j = 0; j < offspring_two.path.size(); j++)
                        {
                            if (n > j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + j, offspring_two.path.begin() + n);
                                }
                            }
                            if (n < j)
                            {
                                if (offspring_two.path[n].id == offspring_two.path[j].id)
                                {
                                    offspring_two.path.erase(offspring_two.path.begin() + n, offspring_two.path.begin() + j);
                                }
                            }
                        }
                    }

                    offsprings.resize(pathParents.size());
                    offsprings[0] = offspring_one;
                    offsprings[1] = offspring_two;
                    cros = true;
                    stop = pathParents[0].path.size() - 1;
                }
                else
                {
                    stop++;
                }
            }
        }

        //    if (cros == false)
        //       cout << "It is not possible to accomplish crossover" << endl;

        // Mutazione
        k = rand() % 2;

        offspring_mut = pathParents[k];
        while (it > 0)
        { // This loop ends at the end of the mutation
            qnew = randomNode();

            cont = 0;
            for (int i = 0; i < pathParents[k].path.size(); i++)
            {
                if (qnew.id == pathParents[k].path[i].id)
                {
                    cont++;
                }
            }
            if (cont == 0)
            {
                for (int i = 1; i < pathParents[k].path.size() - 1; i++)
                {
                    if (obstacleFree(qnew, pathParents[k].path[1 + i], dimBox))
                    {

                        if (obstacleFree(pathParents[k].path[i - 1], qnew, dimBox))
                        {
                            index.push_back(i);
                        }
                    }
                }

                if (index.size() != 0)
                {
                    ind = rand() % index.size();
                    element = index[ind];
                    offspring_mut.path.at(element) = qnew;
                    mut = true;
                    if (cros == true)
                    {
                        offsprings.resize(pathParents.size() + 1);
                        offsprings[2] = offspring_mut;
                    }
                    else
                    {
                        offsprings.resize(1);
                        offsprings[0] = offspring_mut;
                    }

                    return offsprings;
                }
            }
            it--;
        }

        //       if (mut == false)
        //         cout << "It is not possible to accomplish mutation" << endl;

        if (mut == false && cros == false)
        {

            offspring_one.score = -1;

            offsprings.resize(1);
            offsprings[0] = offspring_one;
        }

        return offsprings;
    }
    else
    { // mutation

        k = rand() % 2;
        offspring_mut = pathParents[k];

        while (it > 0)
        { // This loop ends at the end of the mutation
            qnew = randomNode();

            cont = 0;
            for (int i = 0; i < pathParents[k].path.size(); i++)
            {
                if (qnew.id == pathParents[k].path[i].id)
                {
                    cont++;
                }
            }
            if (cont == 0)
            {
                for (int i = 1; i < pathParents[k].path.size() - 1; i++)
                {

                    if (obstacleFree(qnew, pathParents[k].path[1 + i], dimBox))
                    {

                        if (obstacleFree(pathParents[k].path[i - 1], qnew, dimBox))
                        {
                            index.push_back(i);
                        }
                    }
                }

                if (index.size() != 0)
                {
                    ind = rand() % index.size();
                    element = index[ind];
                    offspring_mut.path.at(element) = qnew;
                    offsprings.resize(1);
                    offsprings[0] = offspring_mut;

                    return offsprings;
                }
            }
        }
        it--;

        //    cout << "It is not possible to accomplish mutation" << endl;

        offspring_mut.score = -1;

        offsprings.resize(1);
        offsprings[0] = offspring_mut;

        return offsprings;
    }
}

vector<int> GA::findPath(node qs, node qg, float *c)
{
    // A path is found executing the GA steps

    vector<int> Path; // Path index
    vector<path> population, sortedPopulation, save, pathParents, offspring;
    float score;
    vector<node> pathTmp;
    int dim;
    vector<node> firstPath_nodesTmp;
    float costo;

    if (obstacleFree(qs, qg, dimBox))
    { // Check if start and goal are direcly connectable

        pathTmp.push_back(qs);
        pathTmp.push_back(qg);
        finalScore = 0;
        for (int i = 0; i < pathTmp.size(); i++)
        {
            Path.push_back(pathTmp[i].id);
        }
        *c = 0;
        nodes.push_back(qs);
        nodes.push_back(qg);
        firstPath_nodes = nodes;
        firstPath = Path;

        return Path;
    }

    while (numGenerations < TotGenerations)
    {
        // This loop ends when we reach the last generation

        if (numGenerations == 0)
        { // The first generation has to be created
            population = initialPopulation(qs, qg, numIndividuals);

            if (error == true)
            {
                Path = {-1};
                *c = INT_MAX;

                return Path;
            }
            else if (population.size() == 1)
            { // If we got just ona path from the graph we return it

                population = fitness(population);
                reverse(population[0].path.begin(), population[0].path.end());
                for (int i = 0; i < population[0].path.size(); i++)
                    Path.push_back(population[0].path[i].id);
                *c = population[0].score;
                nodes = population[0].path;
                for (int i = 0; i < population[0].path.size(); i++)
                {
                    firstPath_nodesTmp = population[0].path;
                    ;
                    reverse(firstPath_nodesTmp.begin(), firstPath_nodesTmp.end());
                    firstPath_nodes = firstPath_nodesTmp;
                    firstPath.push_back(firstPath_nodesTmp[i].id);
                }

                return Path;
            }

            numGenerations++;
        }

        population = fitness(population); // Fitness score calculation

        sortedPopulation = Sort(population); // The paths are sorted based on the fitness score

        if (numGenerations == 1)
        { // We save the best path of the first generation to compare it with the final path in a plot

            for (int i = 0; i < sortedPopulation[0].path.size(); i++)
            {
                firstPath_nodesTmp = sortedPopulation[0].path;
                ;
                reverse(firstPath_nodesTmp.begin(), firstPath_nodesTmp.end());
                firstPath_nodes = firstPath_nodesTmp;
                firstPath.push_back(firstPath_nodesTmp[i].id);
            }
        }

        pathParents = select(sortedPopulation); // Paths choosen as parents

        offspring = crossoverMutation(pathParents, sortedPopulation.size()); // execution of crossover and/or mutation

        if (offspring[0].score == -1)
            dim = 0;
        else
            dim = offspring.size();

        save = memory(sortedPopulation, pathParents, dim); // It saves paths from the old generation to bring those in the new one

        population.clear();

        dim = 0;
        if (!(offspring[0].score == -1))
        {                           // If there are offsprings
            population = offspring; // New generation created inserting offsprings
            dim = population.size();
        }

        population.resize(sortedPopulation.size());

        for (int i = 0; i < save.size(); i++)
        {
            population.at(dim + i) = save[i];
        }

        numGenerations++;

        if (population.size() == 0)
        {
            cout << "Error in the execution" << endl;
            *c = INT_MAX;
            return {-1};
        }
    }

    population = fitness(population);

    sortedPopulation = Sort(population);

    reverse(sortedPopulation[0].path.begin(), sortedPopulation[0].path.end());
    for (int i = 0; i < sortedPopulation[0].path.size(); i++)
    {
        Path.push_back(sortedPopulation[0].path[i].id);
    }

    nodes = sortedPopulation[0].path;

    if (qg.id != Path[Path.size() - 1])
    { // In the final path there is not the goal
        cout << "Algorithm not completed" << endl;
        cout << "Start and goals are not connected..." << endl;
        *c = INT_MAX;
        Path = {-1};
    }

    *c = sortedPopulation[0].score;

    return Path;
}

///////////////////////////////////////// COST FUNCTIONS  ///////////////////////////////////////////////////////////////////

node GA::nodesCost(node q1)
{

    // To each node is associated a cost inversely proportional to the distance from the node to the closest obstacle
    int xc, yc;
    int x, y;

    float lMax = min(rows, cols) / 2;
    float cost = 0;
    int j, sampling;
    sampling = 32;
    float uno = 1.0;

    xc = q1.cx;
    yc = q1.cy;

    for (float j = 0; j < lMax; j++)
    {
        for (int theta = 0; theta <= sampling; theta++)
        {
            x = xc + j * cos(2 * theta * M_PI / sampling);
            y = yc + j * sin(2 * theta * M_PI / sampling);

            if (x > cols || x < 0 || y > rows || y < 0)
            { // The point is outside the image

                cost = abs((lMax - j) / lMax);
                goto newnode;
            }
            else
            {
                if (isFree(x, y) == 0)
                { // There is a boundery
                    cost = abs((lMax - j) / lMax);
                    goto newnode;
                }
            }
        }
    }

newnode:
    q1.CostNode = cost;
    return q1;
}

float GA::costCurve(Vector2d a, Vector2d b)
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

float GA::costMin(vector<node> p)
{
    // Calculates the total cost of a path

    float CostTot = 0;
    float costdist;
    int m1 = max(w_length, w_degRot);
    int w_safety = 20 * max(m1, w_numRot); // Safety weight
    Vector2d v1, v2;
    int index;
    node q1, q2, qparent;
    float L, D, N;

    q1 = p[0];
    q2 = p[1];
    qparent = q1;

    for (int i = 1; i < p.size(); i++)
    {

        q2 = nodesCost(q2);

        costdist = (dist(q1.cx, q1.cy, q2.cx, q2.cy)) / Max;

        v1 << (q1.cx - qparent.cx), (q1.cy - qparent.cy);
        v1 = v1;
        v2 << (q2.cx - q1.cx), (q2.cy - q1.cy);
        v2 = v2;

         
        if (w_degRot > w_numRot){
            CostTot += w_length * costdist + w_safety * q2.CostNode + (float(numGenerations)/TotGenerations) * w_degRot * (costCurve(v1, v2)) + w_numRot * numRot;
        }

        else if(w_numRot > w_length){
            CostTot += w_length * costdist + w_safety * q2.CostNode + w_degRot * (costCurve(v1, v2)) + (float(numGenerations)/TotGenerations) *w_numRot * numRot;
        }

        else 
            CostTot += (float(numGenerations)/TotGenerations)*(w_length * costdist + w_safety * q2.CostNode) + w_degRot * (costCurve(v1, v2)) + w_numRot * numRot;

        
        
        L += costdist;
        D += costCurve(v1, v2);
        N += numRot;

        q1 = p[i];
        q2 = p[i + 1];
        qparent = p[i - 1];
    }

    return CostTot;
}

////////////////////////////////////////////////////// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////////////

bool GA::isValid(int x, int y)
{

    // Check if the point is inside the image and it is not in an obstacle

    return ((isFree(x, y) == 1) && (x >= 0) && (x < cols) && (y >= 0) && (y < rows));
}

float GA::dist(int x1, int y1, int x2, int y2)
{

    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void GA::max_dist(vector<path> population)
{

    float max_old = 0;
    float max;

    for (int i = 0; i < population.size(); i++)
    {
        for (int j = 0; j < population[i].path.size() - 1; j++)
        {
            max = dist(population[i].path[j].cx, population[i].path[j].cy, population[i].path[j + 1].cx, population[i].path[j + 1].cy);

            if (max > max_old)
                max_old = max;
        }
    }

    Max = max_old;
}

void GA::addNode(int x, int y, int SoG)
{

    int id;

    if (x >= 0 && x < cols && y >= 0 && y < rows)
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
            qstart = {x, y, id, 0, 1};
        }

        else
        { // goal node

            id = encode(x, y);
            Qgoals.push_back({x, y, id, 0, 1});
        }

        n_nodes++;
    }
    else
        cout << "The position of start and/or goal is wrong" << endl;
}

void GA::addPOIs(int x, int y, string nameId)
{

    if (isValid(x, y))
        POIs.push_back({x, y, nameId});
    else
        cout << " The POI insert in: (" << x << "," << y << ") is not valid" << endl;
}

void GA::checkPOIs()
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

vector<node> GA::convert(vector<int> goals)
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

vector<node> GA::convertReturn(vector<float> vect)
{

    vector<node> vectNode;

    for (int i = 0; i < vect.size(); i++)
    {
        for (int j = 0; j < Graph.id_son.size(); j++)
        {
            if (vect[i] == Graph.id_son[j].id)
                vectNode.push_back(Graph.id_son[j]);
        }
    }

    return vectNode;
}

vector<vector<int>> GA::findMultiPOIs(vector<int> aG)
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

vector<vector<int>> GA::comb()
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

bool GA::checkValid(vector<int> c, vector<vector<int>> C)
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

vector<int> GA::update(vector<int> c)
{

    // Modifies c

    int k, k_val;

    k = rand() % multiPOIs.size();
    k_val = rand() % multiPOIs[k].size();
    c.at(k) = k_val;
    return c;
}

vector<int> GA::sortGoals(vector<int> goals, int n)
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

void GA::findPermutations(vector<int> a, int n)
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

int GA::factorial(int n)
{

    int count = 1;
    for (int i = n; i > 1; i--)
    {
        count = count * i;
    }
    return count;
}

vector<int> GA::algorithmManager()
{
    // This function manage the presence of priorities or multiPOIs

    vector<int> Path, pathTmp, aux;
    int np = goalsPriority.size();
    int nm, ng;
    vector<int> goals;
    vector<node> nodesTmp, nodesTmp_old, QgoalsTmp;

    vector<int> firstPathTmp;
    vector<node> firstnodesTmp;

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

    QgoalsTmp = convert(goals); // ids are conver into nodes

    s = qstart;
    g = QgoalsTmp[0];

    pathTmp = findPath(s, g, &costTmp); // it calculates the path from s to g nodes

    if (costTmp == INT_MAX)
    {
        goto stopInternalLoop1; // s and g are not connectable. Break
    }

    // sub-paths found saving
    if (aux.size() == 0)
    {
        aux = pathTmp;
        nodesTmp = nodes;
        firstnodesTmp = firstPath_nodes;
        firstPathTmp = firstPath;
    }

    costNew = costNew + costTmp;

    if (costNew < costOld)
    {
        // A better path was found
        Path = aux;
        costOld = costNew;
        cost_final = costOld;
        nodes_final = nodesTmp;
        firstPath_final = firstPathTmp;
        firstPath_nodes_final = firstnodesTmp;
    }

stopInternalLoop1:
    aux.clear();

    return Path;
}

bool GA::obstacleFree(node q1, node q2, int d)
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

float GA::totalLen(vector<int> path)
{
    // Calculates the length of the path (#pixel)

    float TotLen = 0;
    int id_p, id;
    for (int i = 1; i < path.size(); i++)
    {
        for (int j = 0; j < nodes_final.size(); j++)
        {
            if (path[i] == nodes_final[j].id)
                id = j;
            if (path[i - 1] == nodes_final[j].id)
                id_p = j;
        }
        TotLen += dist(nodes_final[id_p].cx, nodes_final[id_p].cy, nodes_final[id].cx, nodes_final[id].cy);
    }

    return TotLen;
}

int GA::encode(int x, int y)
{
    // Univocal id based on coordinates

    int id;

    id = x * rows + y;
    return id;
}

void GA::coordinate(vector<int> p)
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

int GA::numRotations(vector<int> p)
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

//  Function to check if gridSquare coordinates are in map bounds

bool GA::isCoordinateInBounds(float x, float y)
{
    bool valid = true;

    if (x > (width * resolution) || y > (height * resolution))
        valid = false;

    return valid;
}

bool GA::isFree(int i, int j)
{
    // Checks if gridSquare at (i,j) is free
    // Returns 1 if free, 0 otherwise
    return occupancyGridMap[j][i];
}


void GA::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
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