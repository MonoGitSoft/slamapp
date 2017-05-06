#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>
#include <simfeaturebase.h>
#include <ekfslam.h>
#include <linefeaturebase.h>
#include <particlefilter.h>

using namespace std;



int main(int argc, char *argv[])
{
   /* DifferencialRobotSim simRobot;
    SimFeatureBase simMap(300,simRobot);
    LineBase simLines(500,simRobot);
    EKFSlam slam(simRobot,simLines);
    for(int i = 0; i < 500; i++) {
        slam.Step();
        cout<<i<<endl;
    }

    simRobot.SavePoses();
    slam.Save();
    simMap.SaveMap();*/
    DifferencialRobotSim simRobot;
    LineBase simLines(1000,simRobot);
    ParticleFilter ptf(simRobot,simLines,1000);
    for(int i = 0; i < 2; i++) {
        ptf.Sampling();
        ptf.Weighting();
        ptf.ReSampling();
        cout<<"step"<<i<<endl;
    }
    ptf.Save();
    simRobot.SavePoses();
    return 0;
}
