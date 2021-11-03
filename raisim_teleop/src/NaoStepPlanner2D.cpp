#include <raisim_teleop/NaoStepPlanner2D.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


StepPlanner2D::StepPlanner2D(RobotParameters robot_):robot(robot_),stepAnkleQ(robot_.getWalkParameter(StepPlanSize)), velocityQ(robot_.getWalkParameter(StepPlanSize))
{
        tempV.setZero();
        targetXY.setZero();
        pivotXY.setZero();
        centerXY.setZero();
        MaxStep.setZero();
        MinStep.setZero();
        c.setZero();
        h.setZero();
        step_id = 0;
        cmd = 0;
        support_foot_x = 0.0;
        support_foot_y = 0.0;
        support_foot_orientation = 0.0;
        v_.setZero();
        eps=1.0e-2;
        planAvailable = false;
}

void StepPlanner2D::plan(WalkInstruction si)
{
    unsigned int jjj = 0;
    planAvailable = false;

    while(velocityQ.size()>0)
    {
        WalkInstruction pi;
        if(jjj==0)
        {
            pi = planStep2D(velocityQ.front(), si);
        }
        else
        {
            pi = planStep2D(velocityQ.front(), pi);
        }
        stepAnkleQ.push_back(pi);
        velocityQ.pop_front();
        jjj++;
        planAvailable = true;
    }
}

void StepPlanner2D::emptyPlan()
{
    while(velocityQ.size()>0)
        velocityQ.pop_front();

    while(stepAnkleQ.size()>0)
        stepAnkleQ.pop_front();

    planAvailable = false;

}

// here
WalkInstruction StepPlanner2D::planStep2D(Eigen::Vector3d v, WalkInstruction si) // v -> velocity
{

	WalkInstruction ci;

        if (abs(v(0))<eps && abs(v(1))<eps && abs(v(2))<eps)
        {
            v.setZero();
        }

        //Crop the Velocity in feasible limits
        // if(v(0)>0.75)
        //     v(0) = 0.75;
        // if(v(0)<-1.00)
        //     v(0) = -1.00;
        //
        // if(v(1)>1.00)
        //     v(1) = 1.00;
        // if(v(1)<-1.00)
        //     v(1) = -1.00;
        //
        // if(v(2)>0.80)
        //     v(2) = 0.80;
        // if(v(2)<-0.80)
        //     v(2) = -0.80;


        //Previous Swing Foot Becomes Support
        support_foot_x = si.target(0);
        support_foot_y = si.target(1);
        support_foot_orientation = si.target(2);
        //Support Leg exchange
        if(si.targetSupport ==  SUPPORT_LEG_LEFT)
        {
            si.targetSupport = SUPPORT_LEG_RIGHT;
        }
        else
        {
            si.targetSupport =  SUPPORT_LEG_LEFT;
        }

        if(si.targetSupport == SUPPORT_LEG_LEFT)
        {
            h = Eigen::Vector2d(0.0, -2.0*robot.getWalkParameter(H0));
        }
        else
        {
            h = Eigen::Vector2d(0.0, 2.0*robot.getWalkParameter(H0));
        }



        rot =  Eigen::Rotation2Dd(support_foot_orientation);
        h = rot*h;

        targetXY = h;
        targetTheta = support_foot_orientation;

        if(v(0)==v_(0) && v(1)==v_(1) and v(2)==v_(2) && v(0)==0 && v(1)==0 && v(2)==0)
        {
            cmd = STAND;


        }
        else
        {
            cmd = WALK;
        }




        if(v(2)>0)
        {
            if(si.targetSupport == SUPPORT_LEG_RIGHT)
            {
                rot = Eigen::Rotation2Dd(v(2)*robot.getWalkParameter(MaxStepTheta));
            }
            else
            {
                rot = Eigen::Rotation2Dd(-v(2)*robot.getWalkParameter(MinStepTheta));
            }
        }
        else
        {
            if(si.targetSupport == SUPPORT_LEG_RIGHT)
            {
                rot = Eigen::Rotation2Dd(-v(2)*robot.getWalkParameter(MinStepTheta));
            }
            else
            {
                 rot = Eigen::Rotation2Dd(v(2)*robot.getWalkParameter(MaxStepTheta));
            }
        }
        targetXY = rot * targetXY;

        //Generate Constraints
        tempV.setZero();
        if(v(0) > 0.00)
        {
            tempV(0) = v(0) * robot.getWalkParameter(MaxStepX);
        }
        else
        {
            tempV(0) = -v(0) * robot.getWalkParameter(MinStepX);
        }


        if(v(1) > 0.00)
        {
            if(si.targetSupport == SUPPORT_LEG_RIGHT)
            {
                tempV(1) = v(1) * robot.getWalkParameter(MaxStepY);
            }
            else
            {
                tempV(1) = -v(1) * robot.getWalkParameter(MinStepY)* 0.0;
            }
        }
        else
        {
            if(si.targetSupport == SUPPORT_LEG_RIGHT)
            {
                tempV(1) = -v(1) * robot.getWalkParameter(MinStepY) * 0.0;
            }
            else
            {
                tempV(1) = v(1) * robot.getWalkParameter(MaxStepY);
            }
        }

        rot = Eigen::Rotation2Dd(support_foot_orientation); //TODO
        tempV = rot * tempV;
        targetXY += tempV;

        if(v(2) > 0.00)
        {
            if(si.targetSupport == SUPPORT_LEG_RIGHT)
            {
                targetTheta += v(2) * robot.getWalkParameter(MaxStepTheta);
            }
            else
            {
                targetTheta -= v(2) * robot.getWalkParameter(MinStepTheta);
            }
        }
        else
        {
            if(si.targetSupport == SUPPORT_LEG_RIGHT)
            {
                targetTheta -= v(2) * robot.getWalkParameter(MinStepTheta);
            }
            else
            {
                targetTheta += v(2) * robot.getWalkParameter(MaxStepTheta);
            }
        }

        //Fix Constraints
        MaxStep(0) = robot.getWalkParameter(MaxStepX);
        MinStep(0) = robot.getWalkParameter(MinStepX);

        if(si.targetSupport == SUPPORT_LEG_RIGHT)
        {
            MaxStep(1) = robot.getWalkParameter(MaxStepY);
            MinStep(1) = robot.getWalkParameter(MinStepY);

        }
        else
        {
            MaxStep(1) = -robot.getWalkParameter(MinStepY);
            MinStep(1) = -robot.getWalkParameter(MaxStepY);
        }



        tempV = rot.inverse() * targetXY;


        tempV(0) = cropStep(tempV(0),MaxStep(0),MinStep(0));
        tempV(1) = cropStep(tempV(1),MaxStep(1),MinStep(1));


        targetXY = rot * tempV;


        //Output must be a Walk Instruction
        ci.target(0) = targetXY(0) + support_foot_x;

        ci.target(1) = targetXY(1) + support_foot_y;
        ci.target(2) = targetTheta;

        ci.targetSupport = si.targetSupport;

        ci.targetZMP = si.targetSupport;


        if(cmd == STAND)
        {
            ci.targetZMP = SUPPORT_LEG_BOTH;
        }

        ci.steps = robot.getWalkParameter(SS_instructions);
        step_id = si.step_id + 1;
	      ci.step_id = step_id;
        v_ = v;
        return ci;
}
