#ifndef VISION_GAIT_H
#define VISION_GAIT_H

#include <Robot_Gait.h>

enum robotMove
{
    nomove = 0,
    turn = 1,
    flatmove = 2,
    bodymove = 3,
    stepup = 4,
    stepdown = 5,
};

struct VISION_WALK_PARAM :public Robots::GAIT_PARAM_BASE
{
    robotMove movetype = nomove;
    int counter = 5000;
    double turndata = 0;
    double movedata[3] = {0, 0, 0};
    double bodymovedata[3] = {0, 0, 0};
    double stepupdata[6] = {0, 0, 0, 0, 0, 0};
    double stepdowndata[6] = {0, 0, 0, 0, 0, 0};
};

int RobotTurn(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam);

int RobotMove(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam);

int RobotBody(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam);

int RobotStepUp(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam);

int RobotStepDown(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam);

#endif // VISION_GAIT_H
