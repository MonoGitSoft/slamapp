#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>
#include <simfeaturebase.h>
#include <ekfslam.h>
#include <linefeaturebase.h>
#include <particlefilter.h>
#include <simfeaturebase.h>

using namespace std;



int main(int argc, char *argv[])
{
   /* DifferencialRobotSim simRobot;
    DifferencialRobotSim debugRobot;
    SimFeatureBase simFeature(1000,simRobot);
    LineBase simLines(1000,simRobot);
    EKFSlam slam(simRobot,simLines);
    debugRobot.DeadReckoningPose();
    for(int i = 0; i < 20; i++) {
        slam.Step();
        debugRobot.DeadReckoningPose();
        cout<<i<<endl;
    }
    slam.Save();
    debugRobot.SavePoses();
    simLines.Save();*/
 /*   DifferencialRobotSim simRobot;
    LineBase simLines(1000,simRobot);
    SimFeatureBase simFeature(1000,simRobot);
    ParticleFilter ptf(simRobot,simLines,100);
    for(int i = 0; i < 10; i++) {
        cout<<"step"<<i<<endl;
        ptf.Sampling();
        ptf.Weighting();
        ptf.ReSampling();
    }
    ptf.Save();
    simRobot.SavePoses();
    simLines.Save();*/
    DifferencialRobotSim prob;
    for(int j = 0; j < 1000; j++) {
        for(int i = 0; i < 5; i++) {
            prob.GetMotionCov( prob.DeadReckoningPose());
        }
        prob.ResetDeadReckoning();
        prob.ResetMotionCov();
    }
    prob.SavePoses();
    return 0;
}
