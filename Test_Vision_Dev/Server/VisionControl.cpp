#include "VisionControl.h"

int visionAdjust(double *param_Adjust, bool *adjust_Finished)
{
    if(abs(Kinect::leftedge_z[0] - Kinect::rightedge_z[0]) > 2)
    {
        cout<<"TURN!"<<endl;
        double turn_ang = atan2((Kinect::rightedge_z[0] - Kinect::leftedge_z[0]), 24);
        /*max turn*/
        if(abs(turn_ang) > 35*M_PI/180)
        {
            turn_ang = turn_ang > 0? 35*M_PI/180 : -35*M_PI/180;
        }
        cout<<"LEFT EDGE Z: "<<Kinect::leftedge_z[0]<<endl;
        cout<<"RIGHT EDGE Z: "<<Kinect::rightedge_z[0]<<endl;
        turn_ang = turn_ang*180/M_PI;
        cout<<"NEEDS TURN: "<<turn_ang<<endl;

        /*let robot turn turn_ang*/
        param_Adjust[3] = turn_ang;
    }
    else
    {
        /*Move Closer*/
        if((Kinect::leftedge_z[0] < 30) || (Kinect::leftedge_z[0] > 32))
        {
            cout<<"MOVE FORWARD AND BACKWARD!"<<endl;
            double movez_data[3] = {0, 0, 0};
            movez_data[2] = (Kinect::leftedge_z[0] - 31)*0.025;
            /*max walk*/
            if(Kinect::leftedge_z[0] < 30)
            {
                movez_data[2] = movez_data[2] < -0.325? -0.325 : movez_data[2];
            }
            else
            {
                movez_data[2] = movez_data[2] > 0.325? 0.325 : movez_data[2];
            }
            cout<<"LEFT EDGE Z: "<<Kinect::leftedge_z[0]<<endl;
            cout<<"RIGHT EDGE Z: "<<Kinect::rightedge_z[0]<<endl;
            cout<<"MOVE ALONG Z "<<movez_data[2]<<endl;

            /*let robot move movez_data*/
            param_Adjust[2] = movez_data[2];
        }
        else
        {
            if((Kinect::rightedge_x[0] !=0&&Kinect::rightedge_x[0] > 35)
                    ||(Kinect::leftedge_x[0] !=0&&Kinect::leftedge_x[0] < 85))
            {
                /*Move Middle*/
                cout<<"MOVE LFET AND RIGHT!"<<endl;
                if(Kinect::rightedge_x[0] < 35)
                {
                    double movexr_data[3] = {0, 0, 0};
                    movexr_data[0] = (Kinect::leftedge_x[0] - 85)*0.025;
                    /*max walk*/
                    movexr_data[0] = movexr_data[0] < -0.1 ? -0.1 : movexr_data[0];
                    cout<<"LEFT_X: "<<Kinect::leftedge_x[0]<<endl;
                    cout<<"RIGHT_X: "<<Kinect::rightedge_x[0]<<endl;
                    cout<<"MOVE RIGHT: "<<movexr_data[0]<<endl;

                    /*let the robot move (right) movexr_data*/
                    param_Adjust[0] = movexr_data[0];
                }
                else
                {
                    double movexl_data[3] = {0, 0, 0};
                    movexl_data[0] = (Kinect::rightedge_x[0] - 35)*0.025;
                    /*max walk*/
                    movexl_data[0] = movexl_data[0] > 0.1 ? 0.1 : movexl_data[0];
                    cout<<"LEFT_X: "<<Kinect::leftedge_x[0]<<endl;
                    cout<<"RIGHT_X: "<<Kinect::rightedge_x[0]<<endl;
                    cout<<"MOVE LEFT: "<<movexl_data[0]<<endl;

                    /*let the robot move (left) movexl_data*/
                    param_Adjust[0] = movexl_data[0];
                }
            }
            else
            {
                *adjust_Finished = true;
            }
        }
    }

    return 0;
}

int visionStepUp(Kinect &kinect_1, double *nextfoot_pos)
{
    cout<<"Step Up!!!"<<endl;

    static int StepUp_Num = -1;

    static double StepUp_Foot_Height[5][6] =
    {
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
    };

    kinect_1.capture(&StepUp_Num);

    while(1)
    {
        if(Kinect::IsCaptureEnd == true)
        {
            for(int m = 0; m < 4; m++)
            {
                if(abs(Kinect::CurrentHeight[m] + 0.85) < 0.035 )
                {
                    Kinect::CurrentHeight[m] = -0.85;
                }

                if(Kinect::CurrentHeight[m] > -0.85 )
                {
                    Kinect::CurrentHeight[m] = -0.85;
                }

                if(Kinect::CurrentHeight[m] < -1.05 )
                {
                    Kinect::CurrentHeight[m] = -1.05;
                }
            }
            StepUp_Foot_Height[(4+StepUp_Num)%5][0] = Kinect::CurrentHeight[1];
            StepUp_Foot_Height[(2+StepUp_Num)%5][1] = Kinect::CurrentHeight[0];
            StepUp_Foot_Height[(0+StepUp_Num)%5][2] = Kinect::CurrentHeight[1];
            StepUp_Foot_Height[(4+StepUp_Num)%5][3] = Kinect::CurrentHeight[3];
            StepUp_Foot_Height[(2+StepUp_Num)%5][4] = Kinect::CurrentHeight[2];
            StepUp_Foot_Height[(0+StepUp_Num)%5][5] = Kinect::CurrentHeight[3];

            cout<<"FOOT HEIGHT"<<endl<<StepUp_Foot_Height[StepUp_Num%5][0]<<endl<<StepUp_Foot_Height[StepUp_Num%5][1]<<endl<<StepUp_Foot_Height[StepUp_Num%5][2]
                    <<endl<<StepUp_Foot_Height[StepUp_Num%5][3]<<endl<<StepUp_Foot_Height[StepUp_Num%5][4]<<endl<<StepUp_Foot_Height[StepUp_Num%5][5]<<endl;

            Kinect::IsCaptureEnd = false;
            memcpy(nextfoot_pos, StepUp_Foot_Height[StepUp_Num%5], sizeof(StepUp_Foot_Height[StepUp_Num%5]));

            break;
        }
    }
    return 0;
}

int visionStepDown(Kinect &kinect_1, double *next_foot_pos)
{
    cout<<"Step Down!!!"<<endl;

    static int StepDown_Num = -1;
    static double StepDown_Foot_Height[5][6] =
    {
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
    };

    kinect_1.capture(&StepDown_Num);

    while(1)
    {
        if(Kinect::IsCaptureEnd == true)
        {
            for(int m = 0; m < 4; m++)
            {
                if(abs(Kinect::CurrentHeight[m] + 1.05) < 0.035 )
                {
                    Kinect::CurrentHeight[m] = -1.05;
                }

                if(Kinect::CurrentHeight[m] < -1.05 )
                {
                    Kinect::CurrentHeight[m] = -1.05;
                }

                if(Kinect::CurrentHeight[m] > -0.85 )
                {
                    Kinect::CurrentHeight[m] = -0.85;
                }
            }

            StepDown_Foot_Height[(4+StepDown_Num)%5][0] = Kinect::CurrentHeight[1];
            StepDown_Foot_Height[(2+StepDown_Num)%5][1] = Kinect::CurrentHeight[0];
            StepDown_Foot_Height[(0+StepDown_Num)%5][2] = Kinect::CurrentHeight[1];
            StepDown_Foot_Height[(4+StepDown_Num)%5][3] = Kinect::CurrentHeight[3];
            StepDown_Foot_Height[(2+StepDown_Num)%5][4] = Kinect::CurrentHeight[2];
            StepDown_Foot_Height[(0+StepDown_Num)%5][5] = Kinect::CurrentHeight[3];

            cout<<"FOOT HEIGHT"<<endl<<StepDown_Foot_Height[StepDown_Num%5][0]<<endl<<StepDown_Foot_Height[StepDown_Num%5][1]<<endl<<StepDown_Foot_Height[StepDown_Num%5][2]
                    <<endl<<StepDown_Foot_Height[StepDown_Num%5][3]<<endl<<StepDown_Foot_Height[StepDown_Num%5][4]<<endl<<StepDown_Foot_Height[StepDown_Num%5][5]<<endl;

            Kinect::IsCaptureEnd = false;

            memcpy(next_foot_pos, StepDown_Foot_Height[StepDown_Num%5], sizeof(StepDown_Foot_Height[StepDown_Num%5]));

            break;
        }
    }
    return 0;
}

int visionStepOver(Kinect &kinect_1, double *step_over_data)
{
    cout<<"Step Over!!!"<<endl;

    static int StepOver_Num = -1;

    static double counter = 0.0;

    kinect_1.capture(&StepOver_Num);

    while(1)
    {
        if(Kinect::IsCaptureEnd == true)
        {
            double stepover_data[4] = {0, 0, 0, 0};
            stepover_data[0] = counter;
            if(int(stepover_data[0]) % 2 == 0)
            {
                stepover_data[3] = 0.55;
            }
            else
            {
                stepover_data[3] = 0.10;
            }
            cout<<"Count "<<stepover_data[0]<<endl;
            cout<<"X "<<stepover_data[1]<<endl;
            cout<<"Y "<<stepover_data[2]<<endl;
            cout<<"Z "<<stepover_data[3]<<endl;

            counter++;
            Kinect::IsCaptureEnd = false;
            memcpy(step_over_data,stepover_data,4*sizeof(double));
            break;
        }
    }
    return 0;
}
