
#include "simple_local_planner.h"
// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(SimplePlannerROS, nav_core::BaseLocalPlanner)

SimplePlannerROS::SimplePlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

SimplePlannerROS::SimplePlannerROS(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
      // initialize planner
      initialize(name, tf, costmap_ros);
}

SimplePlannerROS::~SimplePlannerROS() {}

/////////////////////////////////////////  OVERRIDDEN CLASSES from interface nav_core::BaseLocalPlanner /////////////////////////////////////////////////////////////////////////////////

void SimplePlannerROS::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
{

      if (!initialized_)
      {

            beginning2 = ros::Time::now().toSec();

            costmap_ros_ = costmap_ros;
            tf_ = tf;

            ros::NodeHandle gn;
            amcl_sub = gn.subscribe("amcl_pose", 100, &SimplePlannerROS::amclCallback, this);
            path_pub = gn.advertise<visualization_msgs::Marker>("visualization_marker", 10);
            
        
            //initializing the visualization markers
            points.header.frame_id = "/map";
            points.header.stamp = ros::Time::now();
            points.ns = "path_drawing";
            points.action = visualization_msgs::Marker::ADD;
            points.id = 0;
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.1;
            points.scale.y = 0.1;
            points.color.g = 1.0f;
            points.color.a = 1.0;

            

            average = 0;
            num = 0;
            firstTime = 1;
            number1 = 1;
            hasStarted = 0;
            pathLength = 0;
            p = 0;
            minus = 0;

            hasStarted = 0; // booleana used in auxiliar functions

            initialized_ = true; // set initialized flag

            ROS_DEBUG("Simple Local Planner plugin initialized.");
      }

      else
            ROS_WARN("This planner has already been initialized, doing nothing.");
}

bool SimplePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &origin_global_plan)
{

      if (!initialized_)
      { // check if local planner has been initialized

            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
      }

      count = 1; // reset the counter


      plan = origin_global_plan;


      length = plan.size(); // number of element in plan

      for(int i = 0; i < length; i++){
            cout << "x(" << i << "): " << plan[i].pose.position.x << " y(" << i << "): " << plan[i].pose.position.y << endl;
      }

      setNext();

      ending2 = ros::Time::now().toSec();

      if (number1)
      {
            four = ros::Time::now().toSec() - beginning2;
            number1 = 0;
      }

      average += 0.05;

      p++;
      minus = p;

      goal_reached_ = false; // goal not reached

      return true;
}

bool SimplePlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{

      if (!initialized_)
      { // check if local planner has been initialized

            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
      }

      if (length != 0)
      { // if length = 0 there are not poses in plan

            if (firstTime)
            {
                  startTime = ros::Time::now().toSec();
                  firstTime = 0;
            }

            double beginning = ros::Time::now().toSec();

            setNowError();

            if (distance < 0.1) // default 0.2
            { // if distance between current and next pose is under a certain threshold the node is considered reached

                  if (count < (length - 1))
                  { // it is not the final goal

                        count++;
                        setVelZ();
                        setNext();
                        
                  }
                  else
                  { // reached goal

                        stopTime = ros::Time::now().toSec();

                        setVelZ();
                        goal_reached_ = true;
                        firstTime = 1;
                        cout << "Required time to follow the path: " << (stopTime - startTime) << " s" << endl;

                  }
            }
            else
            {
                if (fabs(nError.az) > 0.1) // default 0.1
                {
                  setRot();
                }
                else
                {
                  setVel();
                }
            }
            double ending = ros::Time::now().toSec();
            average += (ending - beginning);
            num++;
      }

      cmd_vel = cmd;

      return true;
}

bool SimplePlannerROS::isGoalReached()
{

      if (!initialized_)
      { // check if local planner has been initialized

            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
      }
   
      return goal_reached_;
}
///////////////////////////////////////// SETTING FUNCTIONS ///////////////////////////////////////////////////////////////////

void SimplePlannerROS::setVel()
{

  
      cmd.linear.x = 0.2;  // We prefer a costant velocity to a proportional to the distance between current and next frames
      cmd.angular.z = nError.az; // This angular velocity is very small but helps to correct the trajectory during the navigation
}

void SimplePlannerROS::setRot()
{

      cmd.angular.z = (nError.az) * 0.8;
      cmd.linear.x = 0.0;
}

void SimplePlannerROS::setVelZ()
{

      cmd.linear.x = 0; // Linear and angular velocity set to zero; cmd is a twist global variable
      cmd.linear.y = 0;
      cmd.angular.z = 0;
}

void SimplePlannerROS::setNowError()
{

      double d;

      nError.x = (next.x - now.x); // Error x
      nError.y = (next.y - now.y); // Error y

      if (nError.y == 0 & nError.x == 0) // if these two variables are null, the tangent doesn't exist and the angle error is irrelevant because we have arrived at our destination
            d = now.az;
      else
            d = std::atan2(nError.y, nError.x);

      nError.az = d - now.az; // Error theta

      // To select the smallest angle
      if (nError.az > M_PI)
      {
            nError.az -= 2 * M_PI;
      }
      if (nError.az < -M_PI)
      {
            nError.az += 2 * M_PI;
      }

      distance = std::sqrt(nError.x * nError.x + nError.y * nError.y); // distance between current and next frames
}

void SimplePlannerROS::setNext()
{

      next.x = plan[count].pose.position.x; // set the next pose to follow
      next.y = plan[count].pose.position.y;

      cout << "now x: " << now.x << " now y: " << now.y << endl;
      cout << "next x: " << next.x << " next y: " << next.y << endl;

}

///////////////////////////////////////// AUXILIAR FUNCTIONS ///////////////////////////////////////////////////////////////////

void SimplePlannerROS::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{


      pos before;
      if (hasStarted)
      {
            before.x = now.x;
            before.y = now.y;
      }

      now.x = msg->pose.pose.position.x;
      now.y = msg->pose.pose.position.y;
      now.az = getYaw(*msg);
      setNowError();

      /*  if(hasStarted){
                  pathLength += std::sqrt((now.x-before.x)*(now.x-before.x) + (now.y-before.y)*(now.y-before.y));
                  ROS_INFO("%f, %f, %f", stopTime, startTime, pathLength);
      } */

      pathVisualization();

      hasStarted = 1;
}

double SimplePlannerROS::getYaw(geometry_msgs::PoseWithCovarianceStamped msg)
{

      double q[4];
      q[0] = msg.pose.pose.orientation.x;
      q[1] = msg.pose.pose.orientation.y;
      q[2] = msg.pose.pose.orientation.z;
      q[3] = msg.pose.pose.orientation.w;

      double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
      double t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);

      return std::atan2(t3, t4);
}

void SimplePlannerROS::pathVisualization()
{

      geometry_msgs::Point p;

      p.x = now.x;
      p.y = now.y;
      p.z = 0.5;

      points.points.push_back(p);

      path_pub.publish(points);
}
