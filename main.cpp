#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>
#include <simfeaturebase.h>
#include <ekfslam.h>
#include <linefeaturebase.h>

using namespace std;



int main(int argc, char *argv[])
{
    DifferencialRobotSim simRobot;
    SimFeatureBase simMap(50000,simRobot);
    LineBase simLines(5000,simRobot);
    EKFSlam slam(simRobot,simLines);
    for(int i = 0; i < 300; i++) {
        slam.Step();
        cout<<i<<endl;
        cout<<"diff"<<endl;
        cout<<simRobot.realPose - slam.SlamPose()<<endl;
        cout<<"real"<<endl;
        cout<<simRobot.realPose<<endl;
        cout<<"slam"<<endl;
        cout<<slam.SlamPose()<<endl;

    }

    simRobot.SavePoses();
    slam.Save();
    simMap.SaveMap();
    return 0;
}
