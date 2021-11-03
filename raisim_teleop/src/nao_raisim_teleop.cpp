#include <iostream>
#include <iomanip>

#include <vector>
#include <string>
#include <unistd.h>
#include <termios.h>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "lipm_msgs/MotionPlanActionGoal.h"
#include "nav_msgs/Odometry.h"
#include "lipm_msgs/StepTarget.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>


#include "raisim_teleop/NaoStepPlanner2D.h"
#include "raisim_teleop/nao_raisim_teleop.h"


/*
Callback to get data from the /nao_raisim_ros/CoM topic
*/
void COM_odom_callback(const nav_msgs::Odometry & msg )
{
  COM_odom.pose.pose.position.x = msg.pose.pose.position.x;
  COM_odom.pose.pose.position.y = msg.pose.pose.position.y;
  COM_odom.pose.pose.position.z = msg.pose.pose.position.z;

  COM_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x;
  COM_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y;
  COM_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;
  COM_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
  CoM = true;
}


/*
Callback to get data from the /nao_raisim_ros/RLeg/odom topic
*/
void RLeg_odom_callback(const nav_msgs::Odometry & msg )
{
  RLeg_odom.pose.pose.position.x = msg.pose.pose.position.x;
  RLeg_odom.pose.pose.position.y = msg.pose.pose.position.y;
  RLeg_odom.pose.pose.position.z = msg.pose.pose.position.z;

  RLeg_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x;
  RLeg_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y;
  RLeg_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;
  RLeg_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
  RLeg = true;
}



/*
Callback to get data from the /nao_raisim_ros/LLeg/odom topic
*/
void LLeg_odom_callback(const nav_msgs::Odometry & msg)
{
  LLeg_odom.pose.pose.position.x = msg.pose.pose.position.x;
  LLeg_odom.pose.pose.position.y = msg.pose.pose.position.y;
  LLeg_odom.pose.pose.position.z = msg.pose.pose.position.z;

  LLeg_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x;
  LLeg_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y;
  LLeg_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;
  LLeg_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
  LLeg = true;
}


/*
Given a quaternion (x,y,z,w), this function returns the yaw
*/
float Quat2Euler(float x, float y ,float z, float w)
{
  tf::Quaternion q;
  q[0] = x;
  q[1] = y;
  q[2] = z;
  q[3] = w;
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}


/*
Given the yaw, it returns the quaternion by assuming roll = pitch = 0
*/
tf2::Quaternion Euler2Quat(float yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(0,0,yaw);
  return quat;
}



void getData(lipm_msgs::MotionPlanActionGoal & goal_pos, const nav_msgs::Odometry & LLeg_odom, const nav_msgs::Odometry & RLeg_odom, const nav_msgs::Odometry & COM_odom)
{
  goal_pos.goal.lfoot.position.x = LLeg_odom.pose.pose.position.x;
  goal_pos.goal.lfoot.position.y = LLeg_odom.pose.pose.position.y;
  goal_pos.goal.lfoot.position.z = LLeg_odom.pose.pose.position.z;
  goal_pos.goal.lfoot.orientation.x = LLeg_odom.pose.pose.orientation.x;
  goal_pos.goal.lfoot.orientation.y = LLeg_odom.pose.pose.orientation.y;
  goal_pos.goal.lfoot.orientation.z = LLeg_odom.pose.pose.orientation.z;
  goal_pos.goal.lfoot.orientation.w = LLeg_odom.pose.pose.orientation.w;

  goal_pos.goal.rfoot.position.x = RLeg_odom.pose.pose.position.x;
  goal_pos.goal.rfoot.position.y = RLeg_odom.pose.pose.position.y;
  goal_pos.goal.rfoot.position.z = RLeg_odom.pose.pose.position.z;
  goal_pos.goal.rfoot.orientation.x = RLeg_odom.pose.pose.orientation.x;
  goal_pos.goal.rfoot.orientation.y = RLeg_odom.pose.pose.orientation.y;
  goal_pos.goal.rfoot.orientation.z = RLeg_odom.pose.pose.orientation.z;
  goal_pos.goal.rfoot.orientation.w = RLeg_odom.pose.pose.orientation.w;

  goal_pos.goal.CoM.pose.pose.position.x = COM_odom.pose.pose.position.x;
  goal_pos.goal.CoM.pose.pose.position.y = COM_odom.pose.pose.position.y;
  goal_pos.goal.CoM.pose.pose.position.z = COM_odom.pose.pose.position.z;
  goal_pos.goal.CoM.twist.twist.linear.x = COM_odom.twist.twist.linear.x;
  goal_pos.goal.CoM.twist.twist.linear.y = COM_odom.twist.twist.linear.y;
  goal_pos.goal.CoM.twist.twist.linear.z = COM_odom.twist.twist.linear.z;

  goal_pos.goal.COP.x = COM_odom.pose.pose.position.x;
  goal_pos.goal.COP.y = COM_odom.pose.pose.position.y;
  goal_pos.goal.COP.z = 0.;
}


/*  Reads input on keystroke */
char getch() {
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0)
          perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0)
          perror("tcsetattr ICANON");
  if (read(0, &buf, 1) < 0)
          perror ("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
          perror ("tcsetattr ~ICANON");
  return (buf);
}


int main(int argc, char **argv)
{

  std::cout.precision(6);
  std::cout << std::fixed;

  // Guide screen
  std::cout << "\n Move Forward   -> W \n Move Backwards -> S \n Move Left      -> A \n Move Right     -> D \n Rotate         -> Q/E \n EXIT program   ->  CTRL + C  \n";

  ros::init(argc, argv, "nao_raism_teleop"); // Initialize node

  ros::NodeHandle nh;


  /* Parameters */
  int num_steps,ros_rate_value;
  float speed_x,speed_y,speed_w;
  std::string robot_name;

  nh.getParam("/nao_teleop/num_steps",num_steps);
  nh.getParam("/nao_teleop/velocity_x",speed_x);
  nh.getParam("/nao_teleop/velocity_y",speed_y);
  nh.getParam("/nao_teleop/angular_velocity",speed_w);
  nh.getParam("/nao_teleop/ros_rate_value",ros_rate_value);
  nh.getParam("/nao_teleop/robot_name",robot_name);



  ros::Rate r(ros_rate_value);



  std::string LLeg_odom_topic = "/" + robot_name + "_raisim_ros/LLeg/odom";
  std::string RLeg_odom_topic = "/" + robot_name + "_raisim_ros/RLeg/odom";
  std::string COM_odom_topic  = "/" + robot_name + "_raisim_ros/CoM";


  /* Subcribers/Publishers */
  ros::Subscriber LLeg_odom_sub = nh.subscribe(LLeg_odom_topic, 1000, LLeg_odom_callback);
  ros::Subscriber RLeg_odom_sub = nh.subscribe(RLeg_odom_topic, 1000, RLeg_odom_callback);
  ros::Subscriber COM_odom_sub  = nh.subscribe(COM_odom_topic , 1000, COM_odom_callback);

  ros::Publisher goal_pub = nh.advertise<lipm_msgs::MotionPlanActionGoal>("/lipm_motion/plan/goal", 1000);

  /* Messages */
  lipm_msgs::MotionPlanActionGoal goal_pos;  // Published msg
  lipm_msgs::StepTarget tmp;

  goal_pos.goal.footsteps = std::vector<lipm_msgs::StepTarget>();





  // Make sure we have new data
  while (ros::ok()){
    LLeg = false;
    RLeg = false;
    CoM = false;
    ros::spinOnce();
    r.sleep();

    // Check if new data arrived
    if (LLeg && RLeg && CoM){
      break;
    }
  }

  // Don't know exactly why, but check action client walking.py
  getData(goal_pos,LLeg_odom,RLeg_odom,COM_odom);
  RobotParameters robot;
  StepPlanner2D StepPlanner(robot);

  WalkInstruction target_step;


  /* Variables for Control */
  float ux,uy,w;        // Linear and angular velocity variables
  tf2::Quaternion q;

  /* Main loop to get input and send command */
  while (ros::ok()){


    switch (getch()) {

      case 'w':
        // Go Forward
        ux = speed_x;
        uy = 0.0;
         w = 0.0;

        break;

      case 'a':
        // Go Left
        ux = 0.0;
        uy = speed_y;
        w  = 0.0;

        break;

      case 'd':
        // Go Right
        ux = 0.0;
        uy = -speed_y ;
        w = 0.0;

        break;

      case 's':
        // Go Back
        ux = -speed_x;
        uy = 0.0;
        w  = 0.0;

        break;

      case 'q':
        // Go Back
        ux = 0.0;
        uy = 0.0;
        w  = speed_w;

        break;

      case 'e':
        // Go Back
        ux = 0.0;
        uy = 0.0;
        w  = -speed_w;

        break;

      case 'x':
        std::cout << "\nEXIT \n";

        return -1;
      default:
         // Fail input
         ux = 0.0;
         uy = 0.0;
         w  = 0.0;
         std::cout << "Invalid input \n";
         continue;
    }

    std::cout << "\n NEW STEP \n";
    // std::cout << "PREVIOUS STEP " << target_step.target[0] << "  " << target_step.target[1] <<  "  " << target_step.target[2] << '\n' ;
    Eigen::Vector3d velocity_command{ux,uy,w};

    // Get Data from odometry
    while (ros::ok()){
      LLeg = false;
      RLeg = false;
      CoM  = false;

      ros::spinOnce();
      r.sleep();

      // Check if new data arrived
      if (LLeg && RLeg && CoM){
        break;
      }
    }

    // MPAKALIA, Pernei apo to odom gia na diorthosei to fail tou megalou target step
    target_step.targetSupport = SUPPORT_LEG_RIGHT;

    target_step.target[0] = LLeg_odom.pose.pose.position.x;
    target_step.target[1] = LLeg_odom.pose.pose.position.y;
    target_step.target[2] = Quat2Euler(LLeg_odom.pose.pose.orientation.x,LLeg_odom.pose.pose.orientation.y,LLeg_odom.pose.pose.orientation.z,LLeg_odom.pose.pose.orientation.w);

    // std::cout << "Step after tune "<< target_step.target[0] << "  " << target_step.target[1] <<  "  " << target_step.target[2] << '\n' ;
    getData(goal_pos,LLeg_odom,RLeg_odom,COM_odom);


    for (int step_i = 1 ; step_i <= num_steps; ++step_i){
      target_step = StepPlanner.planStep2D(velocity_command,target_step);

      q = Euler2Quat(target_step.target[2]);
      tmp.leg = step_i%2;
      tmp.pose.position.x = target_step.target[0];
      tmp.pose.position.y = target_step.target[1];
      tmp.pose.position.z = LLeg_odom.pose.pose.position.z;
      tmp.pose.orientation.x = q[0];
      tmp.pose.orientation.y = q[1];
      tmp.pose.orientation.z = q[2];
      tmp.pose.orientation.w = q[3];

      goal_pos.goal.footsteps.push_back(tmp);
    }




    // Publish exactly one msg
    while (ros::ok()){
       auto connections = goal_pub.getNumSubscribers();
       if (connections > 0){
           goal_pub.publish(goal_pos);
           r.sleep();
           break;
       }else{
         r.sleep();
       }
    }
    goal_pos.goal.footsteps.clear();

 }

  return 0;
}
