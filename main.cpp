#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>
#include <simfeaturebase.h>

using namespace std;



int main(int argc, char *argv[])
{
    DifferencialRobotSim simRobot;
    SimFeatureBase simMap(simRobot.R, 600,simRobot);;
    simMap.FeatureExtraction();
    for(int j = 0; j < 400; j++) {
        auto pose = simRobot.DeadReckoningPose();;
        simRobot.GetMotionCov(pose);
        simMap.FeatureExtraction();
        simMap.MatchedFeatures(pose);
    }
    simRobot.SavePoses();
    simMap.SaveMap();
    //cout<<simFbase.simRobot->GetRealPose()<<endl;
   /* QCoreApplication a(argc, argv);
    return a.exec();*/
    return 0;
}
