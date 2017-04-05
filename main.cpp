#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>
#include <simfeaturebase.h>

using namespace std;



int main(int argc, char *argv[])
{
    DifferencialRobotSim simRobot;
    SimFeatureBase simMap(simRobot.R, 100);
    for(int i = 0; i < 300; i++) {
        simRobot.GetMotionCov(simRobot.DeadReckoningPose());
    }
    simRobot.SavePoses();
    simMap.SaveMap();
    //cout<<simFbase.simRobot->GetRealPose()<<endl;
   /* QCoreApplication a(argc, argv);
    return a.exec();*/
    return 0;
}
