#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

#include <time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

#include <stdlib.h>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_Control.h>
#include <Robot_Server.h>

#include "Kinect_Test.h"
#include "VisionControl.h"
#include "Vision_Gait.h"


using namespace Aris::Core;

Aris::Core::MSG parseWalk(const std::string &cmd, const map<std::string, std::string> &params)
{
    Robots::WALK_PARAM  param;

    for(auto &i:params)
    {
        if(i.first=="totalCount")
        {
            param.totalCount=std::stoi(i.second);
        }
        else if(i.first=="n")
        {
            param.n=stoi(i.second);
        }
        else if(i.first=="walkDirection")
        {
            param.walkDirection=stoi(i.second);
        }
        else if(i.first=="upDirection")
        {
            param.upDirection=stoi(i.second);
        }
        else if(i.first=="distance")
        {
            param.d=stod(i.second);
        }
        else if(i.first=="height")
        {
            param.h=stod(i.second);
        }
        else if(i.first=="alpha")
        {
            param.alpha=stod(i.second);
        }
        else if(i.first=="beta")
        {
            param.beta=stod(i.second);
        }
    }

    Aris::Core::MSG msg;

    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

Aris::Core::MSG parseAdjust(const std::string &cmd, const map<std::string, std::string> &params)
{
    double firstEE[18] =
    {
        -0.3,-0.75,-0.65,
        -0.45,-0.75,0,
        -0.3,-0.75,0.65,
        0.3,-0.75,-0.65,
        0.45,-0.75,0,
        0.3,-0.75,0.65,
    };

    double beginEE[18]
    {
        -0.3,-0.85,-0.65,
        -0.45,-0.85,0,
        -0.3,-0.85,0.65,
        0.3,-0.85,-0.65,
        0.45,-0.85,0,
        0.3,-0.85,0.65,
    };

    Robots::ADJUST_PARAM  param;

    std::copy_n(firstEE, 18, param.targetPee[0]);
    std::fill_n(param.targetBodyPE[0], 6, 0);
    std::copy_n(beginEE, 18, param.targetPee[1]);
    std::fill_n(param.targetBodyPE[1], 6, 0);

    param.periodNum = 2;
    param.periodCount[0]=1000;
    param.periodCount[1]=1500;

    std::strcpy(param.relativeCoordinate,"B");
    std::strcpy(param.relativeBodyCoordinate,"B");

    for(auto &i:params)
    {
        if(i.first=="all")
        {

        }
        else if(i.first=="first")
        {
            param.legNum=3;
            param.motorNum=9;

            param.legID[0]=0;
            param.legID[1]=2;
            param.legID[2]=4;

            int motors[9] = { 0,1,2,6,7,8,12,13,14 };
            std::copy_n(motors, 9, param.motorID);
        }
        else if(i.first=="second")
        {
            param.legNum=3;
            param.motorNum=9;

            param.legID[0]=1;
            param.legID[1]=3;
            param.legID[2]=5;

            int motors[9] = { 3,4,5,9,10,11,15,16,17 };
            std::copy_n(motors, 9, param.motorID);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
            return MSG{};
        }
    }

    Aris::Core::MSG msg;

    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

Kinect kinect1;
int i = 0;

enum Terrain
{
    terrainnotknown = 0,
    terrainstepup = 1,
    terrainstepdown = 2,
    terrainstepover = 3,
};

Terrain terrain1 = terrainnotknown;
bool IsWalkVisionEnd = false;

Aris::Core::MSG parseVisionWalk(const std::string &cmd, const map<std::string, std::string> &params)
{
    VISION_WALK_PARAM vswalkparam;

    if(terrain1 == terrainnotknown)
    {
        kinect1.capture(&i);

        while(1)
        {
            if(Kinect::IsCaptureEnd == true)
            {
                Kinect::IsCaptureEnd = false;

                cout<<Kinect::Terrain<<endl;

                if(Kinect::Terrain != FlatTerrain)
                {
                    /*Adjust x y z theta*/
                    double paramAdjust[4] = {0, 0, 0, 0};
                    bool adjustFinished = false;
                    visionAdjust(paramAdjust, &adjustFinished);

                    if(adjustFinished == false)
                    {
                        /*let robot move*/
                        if(paramAdjust[3] != 0)
                        {
                            vswalkparam.movetype = turn;
                            vswalkparam.turndata = paramAdjust[3];
                            vswalkparam.counter = 6001;
                        }
                        else
                        {
                            vswalkparam.movetype = flatmove;
                            memcpy(vswalkparam.movedata,paramAdjust,3*sizeof(double));
                            vswalkparam.counter = 5000;
                        }
                    }
                    else
                    {
                        switch (Kinect::Terrain)
                        {
                        case StepUpTerrain:
                        {
                            /*the robot move body*/
                            double movebody[3] = {0, 0.2, 0};
                            vswalkparam.movetype = bodymove;
                            memcpy(vswalkparam.bodymovedata, movebody,3*sizeof(double));
                            vswalkparam.counter = 2500;
                            terrain1 = terrainstepup;
                        }
                            break;
                        case StepDownTerrain:
                        {
                            terrain1 = terrainstepdown;
                            double nextfootpos[6] = {0, 0, 0, 0, 0, 0};
                            visionStepDown(kinect1, nextfootpos);
                            vswalkparam.movetype = stepdown;
                            vswalkparam.counter = 18000;
                            memcpy(vswalkparam.stepdowndata,nextfootpos,sizeof(nextfootpos));
                        }
                            break;
                        case DitchTerrain:
                        {
                            terrain1 = terrainstepover;
                            double stepoverdata[4] = {0, 0, 0, 0};
                            visionStepOver(kinect1, stepoverdata);
                            vswalkparam.movetype = flatmove;
                            vswalkparam.counter = 5000;
                            memcpy(vswalkparam.movedata,stepoverdata + 1, 3*sizeof(double));
                        }
                            break;
                        default:
                            break;
                        }
                    }
                }
                else
                {
                    cout<<"FLAT TERRAIN MOVE"<<endl;
                    cout<<"MOVE FORWARD: "<<0.325<<endl;
                    double move_data[3] = {0, 0, 0.325};

                    vswalkparam.movetype = flatmove;
                    vswalkparam.counter = 5000;
                    memcpy(vswalkparam.movedata,move_data,sizeof(move_data));
                }

                break;
            }
        }
    }
    else
    {
        switch (terrain1)
        {
        case terrainstepup:
        {
            bool stepUpFinished = true;
            double nextfootpos[6] = {0, 0, 0, 0, 0, 0};
            visionStepUp(kinect1, nextfootpos);
            vswalkparam.movetype = stepup;
            vswalkparam.counter = 18000;
            memcpy(vswalkparam.stepupdata, nextfootpos,sizeof(nextfootpos));
            for(int i = 0; i < 6; i++)
            {
                if(nextfootpos[i] != -0.85)
                {
                    stepUpFinished = false;
                }
            }
            if(stepUpFinished == true)
            {
                terrain1 = terrainnotknown;
            }
        }
            break;
        case terrainstepdown:
        {
            bool stepDownFinished = true;
            double nextfootpos[6] = {0, 0, 0, 0, 0, 0};
            static double lastfootpos[6] = {0, 0, 0, 0, 0, 0};

            visionStepDown(kinect1, nextfootpos);

            for(int i = 0; i < 6; i++)
            {
                if(nextfootpos[i] != -1.05||lastfootpos[i] != -1.05)
                {
                    stepDownFinished = false;
                }

            }
            if(stepDownFinished == true)
            {
                double movebody[3] = {0, -0.2, 0};
                vswalkparam.movetype = bodymove;
                vswalkparam.counter = 2500;
                memcpy(vswalkparam.bodymovedata, movebody, sizeof(movebody));
                terrain1 = terrainnotknown;
            }
            else
            {
                vswalkparam.movetype = stepdown;
                vswalkparam.counter = 18000;
                memcpy(vswalkparam.stepdowndata,nextfootpos,sizeof(nextfootpos));
            }
            memcpy(lastfootpos, nextfootpos, 6*sizeof(double));
        }
            break;
        case terrainstepover:
        {
            double stepoverdata[4] = {0, 0, 0, 0};
            visionStepOver(kinect1, stepoverdata);
            vswalkparam.movetype = flatmove;
            vswalkparam.counter = 5000;
            memcpy(vswalkparam.movedata,stepoverdata + 1, 3*sizeof(double));

            if(int(stepoverdata[0]) == 4)
            {
                terrain1 = terrainnotknown;
            }
        }
            break;
        default:
            break;
        }
    }
    Aris::Core::MSG msg;

    msg.CopyStruct(vswalkparam);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int VisionWalk(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    auto planParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    switch(planParam->movetype)
    {
    case turn:
    {
        return RobotTurn(pRobot, planParam);
    }
        break;
    case flatmove:
    {
        return RobotMove(pRobot, planParam);
    }
        break;
    case bodymove:
    {
        return RobotBody(pRobot, planParam);
    }
        break;
    case stepup:
    {
        return RobotStepUp(pRobot, planParam);
    }
        break;
    case stepdown:
    {
        return RobotStepDown(pRobot, planParam);
    }
        break;
    }
}

Aris::Core::MSG parseStepWalk(const std::string &cmd, const map<std::string, std::string> &params)
{
    VISION_WALK_PARAM vswalkparam;

    double stepoverdata[4] = {0, 0, 0, 0};
    visionStepOver(kinect1, stepoverdata);

    vswalkparam.counter = 5000;
    memcpy(vswalkparam.movedata,stepoverdata + 1, 3*sizeof(double));

    if(int(stepoverdata[0]) == 4)
    {
        cout<<"Step Over will finished!!!"<<endl;
    }

    Aris::Core::MSG msg;

    msg.CopyStruct(vswalkparam);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int StepWalk(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    auto planParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    return RobotMove(pRobot, planParam);
}

Aris::Core::MSG parseMoveBody(const std::string &cmd, const map<std::string, std::string> &params)
{
    VISION_WALK_PARAM vswalkparam;

    vswalkparam.counter = 2500;

    for(auto &i:params)
    {
        if(i.first == "XDirection")
        {
            vswalkparam.bodymovedata[0] = stod(i.second);
        }
        else if(i.first == "YDirection")
        {
            vswalkparam.bodymovedata[1] = stod(i.second);
        }
        else if(i.first == "ZDirection")
        {
            vswalkparam.bodymovedata[2] = stod(i.second);
        }
    }

    Aris::Core::MSG msg;

    msg.CopyStruct(vswalkparam);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int MoveBody(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    auto planParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    //cout<<"body move"<<endl;
    return RobotBody(pRobot, planParam);
}

Aris::Core::MSG parseMoveRobot(const std::string &cmd, const map<std::string, std::string> &params)
{
    VISION_WALK_PARAM vswalkparam;

    vswalkparam.counter = 5000;

    for(auto &i:params)
    {
        if(i.first == "XDirection")
        {
            vswalkparam.movedata[0] = stod(i.second);
        }
        else if(i.first == "YDirection")
        {
            vswalkparam.movedata[1] = stod(i.second);
        }
        else if(i.first == "ZDirection")
        {
            vswalkparam.movedata[2] = stod(i.second);
        }
    }

    Aris::Core::MSG msg;

    msg.CopyStruct(vswalkparam);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int MoveRobot(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    auto planParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    return RobotMove(pRobot, planParam);
}

Aris::Core::MSG parseTurnRobot(const std::string &cmd, const map<std::string, std::string> &params)
{
    VISION_WALK_PARAM vswalkparam;

    vswalkparam.counter = 6001;

    for(auto &i:params)
    {
        if(i.first == "TurnAngle")
        {
            vswalkparam.turndata = stod(i.second);
        }
    }

    Aris::Core::MSG msg;

    msg.CopyStruct(vswalkparam);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int TurnRobot(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    auto planParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    return RobotTurn(pRobot, planParam);
}

Aris::Core::MSG parseWalkVision(const std::string &cmd, const map<std::string, std::string> &params)
{
    VISION_WALK_PARAM vswalkparam;

    for(auto &i:params)
    {
        if(i.first == "start")
        {
            vswalkparam.walkvisionprocess1 = walkvisionstart;
            double movebody[3] = {0, 0.2, 0};
            memcpy(vswalkparam.bodymovedata, movebody,3*sizeof(double));
            vswalkparam.counter = 2500;
        }
        else if(i.first == "continuous")
        {
            vswalkparam.walkvisionprocess1 = walkvisioncontinuous;
            double nextfootpos[6] = {0, 0, 0, 0, 0, 0};
            walkVision(kinect1, nextfootpos);
            vswalkparam.counter = 18000;
            memcpy(vswalkparam.stepupdata, nextfootpos,sizeof(nextfootpos));
        }
        else if(i.first == "end")
        {
            vswalkparam.walkvisionprocess1 = walkvisionend;
            double nextfootpos[6] = {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85};
            vswalkparam.counter = 18000;
            vswalkparam.walkvisiondistance = 0;
            memcpy(vswalkparam.stepupdata, nextfootpos,sizeof(nextfootpos));
            IsWalkVisionEnd = true;
        }
    }

    Aris::Core::MSG msg;

    msg.CopyStruct(vswalkparam);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int WalkVision(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    auto planParam = static_cast<const VISION_WALK_PARAM *>(pParam);

    switch (planParam->walkvisionprocess1)
    {
    case walkvisionstart:
    {
        return RobotBody(pRobot, planParam);
    }
        break;
    case walkvisioncontinuous:
    {
        return RobotWalkVision(pRobot, planParam);
    }
        break;
    case walkvisionend:
    {
        return RobotWalkVision(pRobot, planParam);
    }
        break;
    }
}



int main()
{
    time_t t = time(NULL);
    tm* local = new tm;
    char buf[26] = {0};
    localtime_r(&t, local);
    strftime(buf, 64, "%Y-%m-%d %H-%M-%S", local);
    mkdir(buf, S_IRWXU | S_IRWXG);
    chdir(buf);

    kinect1.viewcloud();
    kinect1.start();

    auto rs = Robots::ROBOT_SERVER::GetInstance();
    rs->CreateRobot<Robots::ROBOT_III>();
    rs->LoadXml("/home/hex/git_cx/Test_Vision_805/resource/HexapodIII.xml");
    rs->AddGait("wk",Robots::walk,parseWalk);
    rs->AddGait("ad",Robots::adjust,parseAdjust);
    rs->AddGait("vwk", VisionWalk, parseVisionWalk);
    rs->AddGait("body", MoveBody, parseMoveBody);
    rs->AddGait("turn", TurnRobot, parseTurnRobot);
    rs->AddGait("move", MoveRobot, parseMoveRobot);
    rs->AddGait("wkv", WalkVision, parseWalkVision);
    rs->AddGait("steptest", StepWalk, parseStepWalk);
    rs->Start();
    /**/
    std::cout<<"finished"<<std::endl;

    Aris::Core::RunMsgLoop();

    return 0;
}


