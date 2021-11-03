/*! \file StepPlanner.h
 *	\brief A Monas Activity that first plans the trajectories needed by the Walk Engine
 and executes the desired walking gait!
 *
 */

#ifndef _STEPPLANNER2D_H
#define _STEPPLANNER2D_H

#include <raisim_teleop/NaoRobotParameters.h>
#include <raisim_teleop/MotionDefines.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace std;

class StepPlanner2D
{
    public:
        Eigen::Vector3d v_;
        Eigen::Vector2d targetXY, pivotXY, centerXY, c, h, tempV, MaxStep, MinStep;
        Eigen::Rotation2Dd rot;
        int cmd, step_id;
        double eps, support_foot_x, support_foot_y, support_foot_orientation, targetTheta;
    RobotParameters robot;
    public:
    StepPlanner2D(RobotParameters robot_);
    WalkInstruction planStep2D(Eigen::Vector3d v, WalkInstruction si);
    bool planAvailable;
    void emptyPlan();
    void plan(WalkInstruction si);
    boost::circular_buffer<WalkInstruction> stepAnkleQ;
    boost::circular_buffer<Eigen::Vector3d> velocityQ;
    float cropStep(double f_, double max_, double min_)
    {
        return max(min_, min(f_, max_));
    }
};
#endif
