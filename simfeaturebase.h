#ifndef SIMFEATUREBASE_H
#define SIMFEATUREBASE_H

#include <featurebase.h>
#include <simfeaturebase.h>
#include <stdlib.h>
#include <simdiffrobot.h>
#include <math.h>

int NumOfFeatures = 100;

int x = 0;
int y = 1;
int fi = 2;
int r = 0;
int theta = 1;

class SimFeatureBase: public FeatureBase {
    vector<Feature> simFeatures;
    vector<Feature> featuresInWorld; // features in world frame in x,y(z)
    const int mapR;
    double sensorR;
    DifferencialRobotSim& robot;
public:
    SimFeatureBase(double mapR, double sensorR,DifferencialRobotSim& robot): mapR(mapR), sensorR(sensorR), robot(robot) {
        simFeatures.reserve(NumOfFeatures),featuresInWorld.reserve(NumOfFeatures);
        double x,y;
        VectorXd fpose(2);
        MatrixXd fcov(2,2);
        fcov<< 100, 0 , 0 , 100;
        for(int i = 0; i < NumOfFeatures; i++) {
            x = 3*mapR*(double)(rand() % (1000))*0.001 - 1.5*mapR;
            y = 3*mapR*(double)(rand() % (1000))*0.001 - 2.5*mapR;
            fpose << x , y;
            simFeatures.push_back(Feature(2,fpose, fcov));
        }
    }

    void FeatureExtraction() {
        VectorXd robotpose = robot.GetRealPose();
        VectorXd dest = robotpose;
        VectorXd r(2);
        for(auto i : simFeatures) {
            dest = robotpose - i.GetPose();
            r[0] = sqrt(dest.transpose()*dest);
            if( r[0] < sensorR) {
                r[1] = atan2(dest(y),dest(x));
                tempFeatureBuffer.push_back(Feature(2,r,i.GetCovMatrix()));
                tempFeatureBuffer.back().SetID(i.GetID());
            }
        }
    }

    MatrixXd JacobianOfOdservation(VectorXd robotPose, VectorXd featureWolrdFrame) {
        VectorXd rho(2);
        MatrixXd jacobi(2,5);
        rho<< featureWolrdFrame(x) - robotPose(x), featureWolrdFrame(y) - robotPose(y);
        double q = rho.transpose()*rho;
        jacobi << -sqrt(q)*rho(x), -sqrt(q)*rho(y), 0, sqrt(q)*rho(x), sqrt(q)*rho(y),
                    rho(y),         -rho(x),       -q,    -rho(y),       rho(x);
        jacobi *= q;
        return jacobi;
    }

    VectorXd ExpectedObservation(VectorXd robotPose, VectorXd featureWorldFrame) {
        VectorXd rho(2);
        rho<< featureWorldFrame(x) - robotPose(x),
              featureWorldFrame(y) - robotPose(y);
        double q = rho.transpose()*rho;
        VectorXd ret(2);
        ret<<sqrt(q),atan2(rho(y),rho(x) - robotPose(fi));
        return ret;
    }

    VectorXd FeatureInWorldFrame(VectorXd robotPose, VectorXd relative) {
        VectorXd wpose(2);
        wpose<< robotPose(x) + relative(r)*cos(robotPose(fi) + relative(theta)),
                robotPose(y) + relative(r)*sin(robotPose(fi) + relative(theta));
        return wpose;
    }

    void SaveMap() {
        ofstream saveMap;
        saveMap.open ("map.m");
        saveMap<<"# name: mappose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<NumOfFeatures*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : simFeatures) {
            saveMap<<i.GetPose()<<endl;
        }
        saveMap<<"# name: mapcov"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<NumOfFeatures*2<<endl
                 <<"# columns: 2"<<endl;
        for(auto i : simFeatures) {
            saveMap<<i.GetCovMatrix()<<endl;
        }
    }

};



#endif // SIMFEATUREBASE_H
