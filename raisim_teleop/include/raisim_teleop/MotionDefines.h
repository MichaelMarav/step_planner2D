#ifndef __MOTIONDEFINES_H__
#define __MOTIONDEFINES_H__
#include <Eigen/Dense>
#include <Eigen/Geometry>

enum SupportLeg
{
    SUPPORT_LEG_NONE = 0, SUPPORT_LEG_LEFT, SUPPORT_LEG_RIGHT, SUPPORT_LEG_BOTH
};


enum {
        LeftFoot=0, RightFoot
     };

enum {
        double_support=0, single_support
     };

enum {
        WALK=0, STAND
     };

     /** A class with the necessary data
 * for the walking procedure
 **/
class WalkInstruction
{
public:
    Eigen::Vector3d target;   //x y theta
    SupportLeg targetSupport, targetZMP; //which IS the support foot in this instruction
    unsigned steps; //walk instruction steps  Posa time samples exei to trajectorie
    bool phase; //DS or SS;
    int step_id;
};
#endif
