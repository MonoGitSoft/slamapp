#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>

using namespace std;

inline void FiNorm(VectorXd& pose)
{
   if(pose(2) > M_PI) {
       pose(2) = -(2*M_PI - pose(2));
   }
   if(pose(2) < -M_PI) {
       pose(2) = 2*M_PI + pose(2);
   }
}

int main(int argc, char *argv[])
{
/*    Feature b;
    Feature c(3);
    VectorXd d2(2);
    VectorXd d3(3);
    d2<< 1 , 2;
    d3<< 2 , 2 , 2;
    MatrixXd cov2(2,2);
    MatrixXd cov3(3,3);
    cov2<< 1, 0, 0, 1;
    cov3<< 1 , 0 , 0 , 0 , 1 , 0, 0 , 0 , 1;
    b.SetPose(d2);
    c.SetPose(d3);
    b.SetCovMatrix(cov2);
    c.SetCovMatrix(cov3);*/
    DifferencialRobotSim robot(100);
    VectorXd pose;
    MatrixXd cov3(3,3);
    MatrixXd temp(2,2);
    vector<MatrixXd> buffer(50);
    for(int i = 0; i < 100; i++) {
        robot.GetMotionCov(robot.DeadReckoningPose());

    }
    robot.SavePoses();
   /* QCoreApplication a(argc, argv);
    return a.exec();*/
    return 0;
}
