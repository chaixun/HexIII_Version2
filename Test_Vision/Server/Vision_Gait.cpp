#include "Vision_Gait.h"
#include <math.h>

#define pi 3.14159265358979323846

#ifndef PI
#define PI 3.141592653589793
#endif

void CalPee(int n, double theta, double ymax, double foot_pos[18], double eur_pos[6])
{
    /***** cal of geometric parameters *****/
    /*cal of initial pos*/
    double t = 6;
    double alpha[6];
    alpha[5] = atan(0.3 / 0.65);
    alpha[3] = pi - alpha[5];
    alpha[0] = pi + alpha[5];
    alpha[2] = 2 * pi - alpha[5];
    alpha[1] = 3 * pi / 2;
    alpha[4] = pi / 2;

    double delta0 = theta * pi / 180;

    /*coordinate of foot in CG*/
    double pos_initial[6][3] =
    { -0.3, -0.85, -0.65,
      -0.45, -0.85, 0,
      -0.3, -0.85, 0.65,
      0.3, -0.85, -0.65,
      0.45, -0.85, 0,
      0.3, -0.85, 0.65 };
    double a1 = 0.3, a2 = 0.65;
    int i, j, k = 2;
    double b1 = pow(a1, k), b2 = pow(a2, k);
    double R1346 = sqrt(b1 + b2);
    double R25 = 0.45;
    double R[6] = { R1346, R25, R1346, R1346, R25, R1346 };

    double pos_end[6][3];
    double pos_ymax[6];


    for (i = 0; i < 6; i++)
    {
        pos_end[i][0] = R[i] * sin(alpha[i] + delta0);
        pos_end[i][1] = pos_initial[i][1];
        pos_end[i][2] = R[i] * cos(alpha[i] + delta0);

        pos_ymax[i] = pos_initial[i][1] + ymax;
    }

    /***** cal of pos of foot *****/
    /*pos design：x = Acos(wt) + K*/
    double Ax[6], Kx[6], Ay[6], Ky[6], Az[6], Kz[6];
    for (i = 0; i < 6; i++)
    {
        Ax[i] = (pos_initial[i][0] - pos_end[i][0]) / 2;
        Kx[i] = (pos_initial[i][0] + pos_end[i][0]) / 2;
        Ay[i] = (pos_initial[i][1] - pos_ymax[i]) / 2;
        Ky[i] = (pos_initial[i][1] + pos_ymax[i]) / 2;
        Az[i] = (pos_initial[i][2] - pos_end[i][2]) / 2;
        Kz[i] = (pos_initial[i][2] + pos_end[i][2]) / 2;
    }

    int length_t_half = t * 500;
    int length_t = 2 * length_t_half;

    /*cal trajectory in half time of each leg in x,y,z*/
    double x[6], y[6], z[6];
    for (j = 0; j < 6; j++)
    {
        if (n < length_t_half)
        {
            x[j] = Ax[j] * cos(2 * pi / t * n*0.001) + Kx[j];
            y[j] = Ay[j] * cos(4 * pi / t * n*0.001) + Ky[j];
            z[j] = Az[j] * cos(2 * pi / t * n*0.001) + Kz[j];
        }
        else
        {
            x[j] = Ax[j] * cos(2 * pi / t * (n-length_t_half)*0.001) + Kx[j];
            y[j] = Ay[j] * cos(4 * pi / t * (n - length_t_half)*0.001) + Ky[j];
            z[j] = Az[j] * cos(2 * pi / t * (n - length_t_half)*0.001) + Kz[j];
        }

    }

    /*Get the output 6*3 foot_pos matrix for trajectory at given time*/
    if (n<length_t_half)
    {
        /*leg 1,3,5 move, leg 2,4,6 stay pos_initial*/
        foot_pos[0] = x[0];
        foot_pos[1] = y[0];
        foot_pos[2] = z[0];

        foot_pos[3] = pos_initial[1][0];
        foot_pos[4] = pos_initial[1][1];
        foot_pos[5] = pos_initial[1][2];

        foot_pos[6] = x[2];
        foot_pos[7] = y[2];
        foot_pos[8] = z[2];

        foot_pos[9] = pos_initial[3][0];
        foot_pos[10] = pos_initial[3][1];
        foot_pos[11] = pos_initial[3][2];

        foot_pos[12] = x[4];
        foot_pos[13] = y[4];
        foot_pos[14] = z[4];

        foot_pos[15] = pos_initial[5][0];
        foot_pos[16] = pos_initial[5][1];
        foot_pos[17] = pos_initial[5][2];
    }
    else
    {
        /*leg 1,3,5 stay pos_end, leg 2,4,6 move*/
        foot_pos[0] = pos_end[0][0];
        foot_pos[1] = pos_end[0][1];
        foot_pos[2] = pos_end[0][2];

        foot_pos[3] = x[1];
        foot_pos[4] = y[1];
        foot_pos[5] = z[1];

        foot_pos[6] = pos_end[2][0];
        foot_pos[7] = pos_end[2][1];
        foot_pos[8] = pos_end[2][2];

        foot_pos[9] = x[3];
        foot_pos[10] = y[3];
        foot_pos[11] = z[3];

        foot_pos[12] = pos_end[4][0];
        foot_pos[13] = pos_end[4][1];
        foot_pos[14] = pos_end[4][2];

        foot_pos[15] = x[5];
        foot_pos[16] = y[5];
        foot_pos[17] = z[5];
    }

    /*Get the output 6*1 eur_pos matrix for trajectory at given time*/
    double Ab = -delta0 / 2;
    double Kb = delta0 / 2;
    double delta = Ab*cos(pi/t*n*0.001) + Kb;
    eur_pos[0] = pi / 2;
    eur_pos[1] = delta;
    eur_pos[2] = -pi / 2;
    eur_pos[3] = 0;
    eur_pos[4] = 0;
    eur_pos[5] = 0;
}

int RobotTurn(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam)
{
    auto pRealParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    double pEE[18], pBodyPE[6];

    CalPee(pRealParam->count, pRealParam->turndata, 0.05, pEE, pBodyPE);
    pRobot->SetPee(pEE, pBodyPE);

    return pRealParam->counter - pRealParam->count - 1;
}

int RobotMove(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam)
{
    auto pRealParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    double pEE[18], pBodyPE[6];

    for(int i = 0; i < 18; i++)
    {
        pEE[i] = pRealParam->beginPee[i];
    }

    for(int i = 0; i < 6; i++)
    {
        pBodyPE[i] = pRealParam->beginBodyPE[i];
    }

    double StepH = 0.05;
    double StepDZ = pRealParam->movedata[2];
    double StepDX = pRealParam->movedata[0];

    double Ellipse_ax = StepDX/2;
    double Ellipse_az = StepDZ/2;
    double Ellipse_b = StepH;

    int halfcounter = pRealParam->counter/2;

    if(pRealParam->count < halfcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1) / halfcounter) + PI / 2;

        for(int i = 0; i < 18; i += 6)
        {
            pEE[i] += Ellipse_ax - Ellipse_ax * cos(s);
            pEE[i + 1] += Ellipse_b * sin(s);
            pEE[i + 2] += Ellipse_az - Ellipse_az * cos(s);
        }

        pBodyPE[3] += Ellipse_ax*(1-cos(s))/2;
        pBodyPE[5] += Ellipse_az*(1-cos(s))/2;
    }
    else
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - halfcounter) / halfcounter) + PI / 2;

        for(int i = 0; i < 18; i += 6)
        {
            pEE[i] += 2 * Ellipse_ax;
            pEE[i + 2] += 2 * Ellipse_az;
        }

        for(int i = 3; i < 18; i += 6)
        {
            pEE[i] += Ellipse_ax - Ellipse_ax * cos(s);
            pEE[i + 1] += Ellipse_b * sin(s);
            pEE[i + 2] += Ellipse_az - Ellipse_az * cos(s);
        }


        pBodyPE[3] += Ellipse_ax + Ellipse_ax*(1-cos(s))/2;
        pBodyPE[5] += Ellipse_az + Ellipse_az*(1-cos(s))/2;
    }

    pRobot->SetPee(pEE, pBodyPE);

    return pRealParam->counter - pRealParam->count - 1;
}

int RobotBody(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam)
{
    auto pRealParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    double pEE[18], pBodyPE[6];

    for(int i = 0; i < 18; i++)
    {
        pEE[i] = pRealParam->beginPee[i];
    }

    for(int i = 0; i < 6; i++)
    {
        pBodyPE[i] = pRealParam->beginBodyPE[i];
    }

    double s = -(PI / 2)*cos(PI * (pRealParam->count + 1) / pRealParam->counter) + PI / 2;

    pBodyPE[3] += pRealParam->bodymovedata[0] * (1 - cos(s))/2;
    pBodyPE[4] += pRealParam->bodymovedata[1] * (1 - cos(s))/2;
    pBodyPE[5] += pRealParam->bodymovedata[2] * (1 - cos(s))/2;

    pRobot->SetPee(pEE, pBodyPE);

    return pRealParam->counter - pRealParam->count - 1;
}

int RobotStepUp(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam)
{
    auto pRealParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    double pEE[18], pBodyPE[6];

    for(int i = 0; i < 18; i++)
    {
        pEE[i] = pRealParam->beginPee[i];
    }

    for(int i = 0; i < 6; i++)
    {
        pBodyPE[i] = pRealParam->beginBodyPE[i];
    }

    double stepUpH = 0.25;
    double stepUpD = 0.325;

    double StepUpNextPos[6] = {0, 0, 0, 0, 0, 0};

    memcpy(StepUpNextPos,pRealParam->stepupdata,6*sizeof(double));

    static double StepUpCurrentPos[6] = {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05};

    int periodcounter = pRealParam->counter / 6;

    if(pRealParam->count < periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05)) * (1 - cos(s))/2;
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05)) * (1 - cos(s))/2;
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05)) * (1 - cos(s))/2;
    }
    else if(pRealParam->count >= periodcounter && pRealParam->count < 2 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05));

        pEE[2] += stepUpD * (1 - cos(s))/2;
        pEE[8] += stepUpD * (1 - cos(s))/2;
        pEE[14] += stepUpD * (1 - cos(s))/2;

        pBodyPE[5] += stepUpD/2 * (1 - cos(s))/2;
    }
    else if(pRealParam->count >= 2 * periodcounter && pRealParam->count < 3 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - 2*periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05)) * (1 - cos(s)) / 2;
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05)) * (1 - cos(s)) / 2;
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05)) * (1 - cos(s)) / 2;

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[5] += stepUpD/2;
    }
    else if(pRealParam->count >= 3 * periodcounter && pRealParam->count < 4 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - 3*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05)) * (1 - cos(s))/2;
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05)) * (1 - cos(s))/2;
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05)) * (1 - cos(s))/2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[5] += stepUpD/2;
    }
    else if(pRealParam->count >= 4 * periodcounter && pRealParam->count < 5 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - 4*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05));
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05));
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05));

        pEE[5] += stepUpD * (1 -cos(s))/2;
        pEE[11] += stepUpD * (1 -cos(s))/2;
        pEE[17] += stepUpD * (1 -cos(s))/2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[5] += stepUpD/2 + stepUpD/2 * (1 - cos(s))/2;
    }
    else
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - 5*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05))
                - (stepUpH - (StepUpNextPos[1] + 1.05)) * (1 - cos(s))/2;
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05))
                - (stepUpH - (StepUpNextPos[3] + 1.05)) * (1 - cos(s))/ 2;
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05))
                - (stepUpH - (StepUpNextPos[5] + 1.05)) * (1 - cos(s))/2;

        pEE[5] += stepUpD;
        pEE[11] += stepUpD;
        pEE[17] += stepUpD;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[5] += stepUpD/2 + stepUpD/2;
    }

    if (pRealParam->counter - pRealParam->count - 1 == 0)
    {
        memcpy(StepUpCurrentPos, StepUpNextPos, 6*sizeof(double));
    }

    pRobot->SetPee(pEE, pBodyPE);

    return pRealParam->counter - pRealParam->count - 1;

}

int RobotStepDown(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam)
{
    auto pRealParam = static_cast<const VISION_WALK_PARAM *>(pParam);
    double pEE[18], pBodyPE[6];

    for(int i = 0; i < 18; i++)
    {
        pEE[i] = pRealParam->beginPee[i];
    }

    for(int i = 0; i < 6; i++)
    {
        pBodyPE[i] = pRealParam->beginBodyPE[i];
    }

    static double StepDownCurrentPos[6] = {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85};

    double stepDownH = 0.05;
    double stepDownD = 0.325;

    double StepDownNextPos[6] = {0, 0, 0, 0, 0, 0};

    memcpy(StepDownNextPos,pRealParam->stepdowndata,6*sizeof(double));

    int periodcounter = pRealParam->counter / 6;

    if(pRealParam->count < periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85)) * (1 - cos(s))/2;
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85)) * (1 - cos(s))/2;
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85)) * (1 - cos(s))/2;
    }
    else if(pRealParam->count >= periodcounter && pRealParam->count < 2 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85));

        pEE[2] += stepDownD * (1 -cos(s))/2;
        pEE[8] += stepDownD * (1 -cos(s))/2;
        pEE[14] += stepDownD * (1 -cos(s))/2;

        pBodyPE[5] += stepDownD/2 * (1 - cos(s))/2;
    }
    else if(pRealParam->count >= 2 * periodcounter && pRealParam->count < 3 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - 2*periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85)) * (1 - cos(s)) / 2;
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85)) * (1 - cos(s)) / 2;
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85)) * (1 - cos(s)) / 2;

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[5] += stepDownD/2;
    }
    else if(pRealParam->count >= 3 * periodcounter && pRealParam->count < 4 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - 3*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85)) * (1 - cos(s))/2;
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85)) * (1 - cos(s))/2;
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85)) * (1 - cos(s))/2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[5] += stepDownD/2;
    }
    else if(pRealParam->count >= 4 * periodcounter && pRealParam->count < 5 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - 4*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85));
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85));
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85));

        pEE[5] += stepDownD * (1 -cos(s))/2;
        pEE[11] += stepDownD * (1 -cos(s))/2;
        pEE[17] += stepDownD * (1 -cos(s))/2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[5] += stepDownD/2 + stepDownD/2 * (1 - cos(s))/2;
    }
    else
    {
        double s = -(PI / 2)*cos(PI * (pRealParam->count + 1 - 5*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85))
                - (stepDownH + (-StepDownNextPos[1] - 0.85)) * (1 - cos(s)) / 2;
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85))
                - (stepDownH + (-StepDownNextPos[3] - 0.85)) * (1 - cos(s)) / 2;
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85))
                - (stepDownH + (-StepDownNextPos[5] - 0.85)) * (1 - cos(s)) / 2;

        pEE[5] += stepDownD;
        pEE[11] += stepDownD;
        pEE[17] += stepDownD;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[5] += stepDownD/2 + stepDownD/2;
    }

    if (pRealParam->counter - pRealParam->count - 1 == 0)
    {
        memcpy(StepDownCurrentPos, StepDownNextPos, 6*sizeof(double));
    }

    pRobot->SetPee(pEE, pBodyPE);

    return pRealParam->counter - pRealParam->count - 1;
}
