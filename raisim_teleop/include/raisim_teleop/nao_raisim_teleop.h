using namespace Eigen;

/* Global Variables */
bool CoM = false;       // Check if I got data
bool LLeg = false;
bool RLeg = false;
nav_msgs::Odometry LLeg_odom,RLeg_odom,COM_odom; // Store data  from callbacks

/* Declarations */


// Callback functions
void COM_odom_callback( const nav_msgs::Odometry & msg);
void RLeg_odom_callback(const nav_msgs::Odometry & msg);
void LLeg_odom_callback(const nav_msgs::Odometry & msg);

// Geometry transforms
float Quat2Euler(float x, float y ,float z, float w);
//Quaternionf Euler2Quat(float yaw);

// Utility functions
void getData(lipm_msgs::MotionPlanActionGoal & goal_pos, const nav_msgs::Odometry & LLeg_odom, const nav_msgs::Odometry & RLeg_odom, const nav_msgs::Odometry & COM_odom);
char getch();
