#include "planning_pkg/class_planning.h"

size_t callbackfunction(void *ptr, size_t size, size_t nmemb, void* userdata){

    FILE* stream = (FILE*)userdata; 
    if(!stream){
        printf("!!! No stream\n");
        return 0;
    }

    size_t written = fwrite((FILE*)ptr, size, nmemb, stream);
    return written;
}

bool download(char* url){
    FILE* fp = fopen("mappa.png", "wb"); 
    if(!fp){
        printf("!!! Failed to create file on the disk\n");
        return false;
    }

    CURL* curlCtx = curl_easy_init(); 
    curl_easy_setopt(curlCtx, CURLOPT_URL, url);
    curl_easy_setopt(curlCtx, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt(curlCtx, CURLOPT_WRITEFUNCTION, callbackfunction);
    curl_easy_setopt(curlCtx, CURLOPT_FOLLOWLOCATION, 1);

    CURLcode rc = curl_easy_perform(curlCtx);
    if(rc){
        printf("!!! Download failed: %s\n", url);
        return false;
    }

    long res_code = 0;
    curl_easy_getinfo(curlCtx, CURLINFO_RESPONSE_CODE, &res_code);
    if(!((res_code == 200 || res_code == 201) && rc != CURLE_ABORTED_BY_CALLBACK)){
        printf("!!! Answer code: %d\n", res_code);
        return false;
    }

    curl_easy_cleanup(curlCtx);

    fclose(fp); 
    return true;
}


// Global variables 
int start[2] = {2200, 200};    // Start node initialization 2200, 200
int goal[2] = {200, 1000};     // Goal  node initialization 200, 1000


int main(int argc, char **argv){ 
    
    ros::init(argc, argv, "main");
    ros::NodeHandle planning;
    ros::Time begin = ros::Time::now();

                                        
     class_planning plan;    // plan is an object belonging to the class "class_planning"
     plan.init(planning);
     
  
    plan.name = "/home/vanessa/catkin_ws/src/planning_pkg/src/immagini/vanecomplex5"; // vanecomplex5 //PianoSeminterrato5
    
    plan.assign_dimBox();

    plan.corners = plan.cornerHarris_fun(); // Corner detection
    
    ros::Time DC = ros::Time::now();
    cout << " Corner detection time: " <<  DC - begin << ", with corners number equal to " << plan.corners.size() << " and image size equal to " << plan.cols << "x" << plan.rows<< endl;

    plan.correctCorners(plan.corners);      // Corner correction
        
    ros::Time CC = ros::Time::now();
    cout << "Corner correction time: " <<  CC - DC << ", with final corners number equal to " << plan.n_cor << endl;
    
    plan.loadPOIs();

    // Line tracking
    plan.trackLines();
    
    ros::Time TL = ros::Time::now();
    cout << "Tracking lines time: " <<  TL - CC << endl;
    
    // Connectable areas research. An identificative label is associated to each pixel of the image based on the different connected area
    plan.n_poly = connectedComponents(plan.src, plan.clast) - 1; 
   
    cout << "Connection areas number equal to: " << plan.n_poly << endl;

    // Barycenter of each poligon is added to node vector 
    plan.poly2node();

    ros::Time Poly = ros::Time::now();
    cout << "Cell development time: " <<  Poly - TL << endl;
    
    // Adjacenties calculation
    plan.adjacencies();

    ros::Time AD = ros::Time::now();
    cout << "Adjacencies detection time: " <<  AD - TL << endl;
    
    // Barycenter side of each poligon is added to node vector 
    plan.edge2node();

    ros::Time ED = ros::Time::now();
    cout << "Side node determination time: " <<  ED - TL << endl;
    
    cout << "Total number of nodes: " << plan.n_nodes << endl;
    
    // Arcs cost calculation
    plan.archsCost();
    
    ros::Time CostArch = ros::Time::now();
    cout << "Time required to calculate arcs cost: " <<  CostArch - ED << endl;
    
    // Node cost calculation based on the distance from obstacles
    plan.nodesCost();
    
    ros::Time Cost = ros::Time::now();
    cout << "Time required to calculate node cost: " <<  Cost - CostArch << endl;
    
    // If start and goal nodes are valid it is possible to continue otherwise it prints an error
    if(plan.isValid(start[0],start[1]) && plan.isValid(goal[0],goal[1])){
        

        plan.addNode(start[0],start[1], 0);
        plan.addNode(goal[0], goal[1], 1);
        plan.addNode(1200, 350, 1);
       // plan.addNode(1800, 1400, 1);
       // plan.addNode(200, 1200, 1);

//         plan.addNode(goal[0]+30, goal[1]-300, 1);
//         plan.goalsPriority.push_back({3});
//         plan.goalsPriority.push_back({1});
//         plan.goalsPriority.push_back({0});
//         plan.goalsPriority.push_back({2});
//         plan.addPOIs("Supermarket",-1);
//         plan.addPOIs("Supermarket",-1);
        
    
        ros::Time SG = ros::Time::now();
        cout << "Time required to add start and goals nodes: " <<  SG - Cost << endl;


       if(!plan.block){
            vector<int> path;

            path = plan.algorithmManager();

            cout << "Path lenght: " << plan.totalLen(path) << endl;

            if(plan.totalLen(path) == 0)
                cout << "Error: it is not possible to find a path" << endl;
            else{
                cout << "Final path cost: " << plan.cost_final << endl;

            }
            
            ros::Time Path = ros::Time::now();
            cout << "Research time: " <<  Path - SG << endl;


            if(path.size() != 1 || ((start[0] == goal[0]) && (start[1] == goal[1]))){
                // Plots


                 if(plan.plotting){
                    plan.drawNodes();
                }

                if(plan.plotting){
                    plan.drawPath(path);
                }
                plan.saveTrajectorySubImage(path);
                
            }
            
        }else{
            cout << "Start and goal positions are not connectable..." << endl;
        }

    }else{
        cout << "Wrong position of start and or goal..." << endl;
    } 

    ros::Time end = ros::Time::now();
    cout << "Execution time: " <<  end - begin << endl;

    
    waitKey(0); 
    return 0;
}
