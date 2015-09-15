#include "Kinect_Test.h"

double Kinect::CurrentHeight[4];
bool Kinect::IsCaptureEnd;
int Kinect::ControlCommand;
int Kinect::leftedge_z[6] = {0, 0, 0, 0, 0, 0};
int Kinect::rightedge_z[6] = {0, 0, 0, 0, 0, 0};
int Kinect::leftedge_x[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int Kinect::rightedge_x[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int Kinect::Terrain;

Kinect::Kinect():interface(new pcl::OpenNIGrabber())
{
    IsCapture = false;
    IsCaptureEnd = false;
    frames_num = 0;
    Terrain = FlatTerrain;
}
Kinect::~Kinect()
{
    interface->stop();
    cout<<"Depth Close"<<endl;
}

void Kinect::start()
{
    interface->start();
    cout<<"Depth Open"<<endl;
}

void Kinect::capture(int *num)
{
    (*num)++;
    frames_num++;
    IsCapture = true;
    cout<<"Capture Step "<<*num<<" Beging !!!"<<endl;
}

void Kinect::pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    if (IsCapture == true)
    {
        bool positive[6] = {false, false, false, false, false, false};
        bool negative[6] = {false, false, false, false, false, false};

        for (int p = 0; p < 6; p++)
        {
            leftedge_z[p] = 0;
            rightedge_z[p] = 0;
        }

        for (int q = 0; q < 10; q++)
        {
            leftedge_x[q] = 0;
            rightedge_x[q] = 0;
        }

        Terrain = FlatTerrain;

        //bool Obstacle = false;
        IsCapture = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr SensorPoint(new pcl::PointCloud<pcl::PointXYZ>);

        Eigen::Matrix4f transformation1;
        transformation1 << -1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        pcl::transformPointCloud(*cloud,*SensorPoint,transformation1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr RobotPoint(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f transformation2;
        transformation2 << 0.9995, 0.0134, -0.0273, 0.0224,
                -0.0304, 0.5120, -0.8584, 0.2026 + 0.038,
                0.0025, 0.8589, 0.5122, 0.5733,
                0, 0, 0, 1;
        // 0.2026 + 0.85 + 0.038 + 0.2
        //        transformation2 << 0.9996, 0.0233, -0.0137, 0.0224,
        //                -0.0270, 0.8622, -0.5059, 0.2026 - 0.12,
        //                0, 0.5061, 0.8625, 0.5733,
        //                0, 0, 0, 1;
        pcl::transformPointCloud(*SensorPoint,*RobotPoint,transformation2);

        //Mapping GridMap

        Eigen::MatrixXf GridMap = Eigen::MatrixXf::Zero(120,120);
        Eigen::MatrixXf GridNum = Eigen::MatrixXf::Zero(120,120);
        for (size_t i = 0;i < RobotPoint->points.size(); ++i)
        {
            if(RobotPoint->points[i].x>-1.5&&RobotPoint->points[i].x<1.5&&
                    RobotPoint->points[i].z>0&&RobotPoint->points[i].z<3)
            {
                int m, n;
                n = floor(RobotPoint->points[i].x/0.025) + 60;
                m = floor(RobotPoint->points[i].z/0.025);

                //Mean

                GridMap(m,n) = (GridMap(m,n)*GridNum(m,n) + RobotPoint->points[i].y)/(GridNum(m,n) + 1);
                GridNum(m,n) = GridNum(m,n) + 1;

                //Max

                //if (GridMap(m,n) < RobotPoint->points[i].y)
                //{
                //GridMap(m,n) = RobotPoint->points[i].y;
                //}

            }
        }

        //Judge Terrain

        bool* positive_pointer = positive;
        bool* negative_pointer = negative;

        for(int k = 29; k <= 49; k++)
        {
            // fill in nan data along z middle 60
            int r = 2;
            while(GridMap(k + 1, 60) == 0 && r < 10)
            {
                GridMap(k + 1, 60) = GridMap(k + r, 60);
                r++;
            }

            if(GridMap(k + 1, 60) - GridMap(k, 60) > 0.05)
            {
                *positive_pointer = true;
                positive_pointer++;
            }

            if(GridMap(k + 1, 60) - GridMap(k, 60) < -0.05)
            {
                *negative_pointer = true;
                negative_pointer++;
            }
        }

        if(positive[0] == true&& negative[0] == false)
        {
            Terrain = StepUpTerrain;
        }
        if(positive[0] == false&& negative[0] == true)
        {
            Terrain = StepDownTerrain;
        }
        if(positive[0] == true&& negative[0] == true)
        {
            Terrain = DitchTerrain;
        }
        if(positive[0] == false&& negative[0] == false)
        {
            Terrain = FlatTerrain;
        }

        if(Terrain != FlatTerrain)
        {
            //Find Edge

            int* rightz_pointer = rightedge_z;
            int* leftz_pointer = leftedge_z;
            int* rightx_pointer = rightedge_x;
            int* leftx_pointer = leftedge_x;

            /*Find Edge Along Z*/
            for(int m = 29; m <= 55; m++)
            {
                //fill in nan data along z right 49
                int p = 2;
                while(GridMap(m + 1, 49) == 0 && p < 10)
                {
                    GridMap(m + 1, 49) = GridMap(m + p, 49);
                    p++;
                }

                if(abs(GridMap(m+1, 49)-GridMap(m, 49)) > 0.05)
                {
                    *rightz_pointer = m + 1;
                    rightz_pointer++;
                }

                //fill in nan data along z left 49
                int q = 2;
                while(GridMap(m + 1, 72) == 0 && q < 10)
                {
                    GridMap(m + 1, 72) = GridMap(m + q, 72);
                    q++;
                }

                if(abs(GridMap(m+1, 72)-GridMap(m, 72)) > 0.05)
                {
                    *leftz_pointer = m + 1;
                    leftz_pointer++;
                }
            }

            /*Find Edge Along X*/
            for(int k = 0; k < 30; k++)
            {
                //fill in nan data along x right 50
                int s = 1;
                while(GridMap(60, 60-k-1) == 0 && (60-k-1-s) > 23)
                {
                    GridMap(60, 60-k-1) = GridMap(60, 60-k-1-s);
                    s++;
                }

                if(abs(GridMap(60, 60-k-1) - GridMap(60, 60-k)) > 0.05 )
                {
                    *rightx_pointer = 60 - k;
                    rightx_pointer++;
                }

                //fill in nan data along x left 50
                int t = 1;
                while(GridMap(60, 60+k+1) == 0 && (60+k+1+t) < 97)
                {
                    GridMap(60, 60+k+1) = GridMap(60, 60+k+1+t);
                    t++;
                }

                if(abs(GridMap(60, 60+k+1) - GridMap(60, 60+k)) > 0.05 )
                {
                    *leftx_pointer = 60 + k;
                    leftx_pointer++;
                }
            }
        }
        //Judge Command

        CurrentHeight[0] = (GridMap(39,43) + GridMap(39,44) + GridMap(40,43) + GridMap(40,44))/4;
        CurrentHeight[1] = (GridMap(39,48) + GridMap(39,49) + GridMap(40,48) + GridMap(40,49))/4;
        CurrentHeight[2] = (GridMap(39,78) + GridMap(39,79) + GridMap(40,78) + GridMap(40,79))/4;
        CurrentHeight[3] = (GridMap(39,72) + GridMap(39,73) + GridMap(40,72) + GridMap(40,73))/4;

        std::stringstream out;
        out<<frames_num;
        std::string filename = "GridMap" + out.str() + ".txt";
        std::ofstream Gridmapfile(filename);
        if (Gridmapfile.is_open())
        {
            Gridmapfile<<GridMap<<endl;
        }
        IsCaptureEnd = true;
    }
}

void Kinect::viewcloud()
{
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&Kinect::pointcloud, this, _1);

    interface->registerCallback (f);
}

void Kinect::stop()
{
    interface->stop();
}
