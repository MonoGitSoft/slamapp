#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>
#include <simfeaturebase.h>

using namespace std;



int main(int argc, char *argv[])
{
    DifferencialRobotSim simRobot;
    SimFeatureBase simMap(simRobot.R, 250,simRobot);
    simMap.FeatureExtraction();
    for(int i = 0; i < 100; i++) {
        for(int j = 0; j < 301; j++) {
            simRobot.GetMotionCov(simRobot.DeadReckoningPose());
        }
        simRobot.ResetDeadReckoning();
    }
    simRobot.SavePoses();
    simMap.SaveMap();
    //cout<<simFbase.simRobot->GetRealPose()<<endl;
   /* QCoreApplication a(argc, argv);
    return a.exec();*/
    return 0;
}
