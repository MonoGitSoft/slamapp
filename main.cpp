#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>
#include <simfeaturebase.h>

using namespace std;



int main(int argc, char *argv[])
{
    DifferencialRobotSim simRobot;
    for(int i = 0; i < 400; i++) {
        simRobot.GetMotionCov(simRobot.DeadReckoningPose());
    }
    simRobot.SavePoses();
    //cout<<simFbase.simRobot->GetRealPose()<<endl;
   /* QCoreApplication a(argc, argv);
    return a.exec();*/
    return 0;
}
