#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>
#include <simdiffrobot.h>
#include <simfeaturebase.h>
#include <ekfslam.h>

using namespace std;



int main(int argc, char *argv[])
{
    DifferencialRobotSim simRobot;
    SimFeatureBase simMap(simRobot.R, 400,simRobot);;
    EKFSlam simEKF(simRobot,simMap);
    for(int i = 0; i < 600; i++) {
        simEKF.Step();
    }
   /* set<LookUp> lookUpTable;
    for(int i =0; i < 10; i++) {
        lookUpTable.insert(LookUp(i,2*i));
    }
    std::set<LookUp>::iterator it;
    it = lookUpTable.find(LookUp(5,0));
    cout<<it->GetSlamID();*/
    //cout<<simFbase.simRobot->GetRealPose()<<endl;
   /* QCoreApplication a(argc, argv);
    return a.exec();*/
    simRobot.SavePoses();
    simMap.SaveMap();
    return 0;
}
