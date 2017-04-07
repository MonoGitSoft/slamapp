#ifndef SIMFEATUREBASE_H
#define SIMFEATUREBASE_H

#include <featurebase.h>
#include <simfeaturebase.h>
#include <stdlib.h>
#include <simdiffrobot.h>
#include <math.h>


int NumOfFeatures = 60;
int covdiag = 20;

int x = 0;
int y = 1;
int fi = 2;
int r = 0;
int theta = 1;
int simEnv = 1;

class SimFeatureBase: public FeatureBase {

    vector<Feature> featuresInWorld; // features in world frame in x,y(z)
    vector<Feature> newFeaturesInWorld;
    const int mapR;
    double sensorR;
    DifferencialRobotSim& robot;
public:
        vector<Feature> simFeatures;
    SimFeatureBase(double mapR, double sensorR,DifferencialRobotSim& robot): mapR(mapR), sensorR(sensorR), robot(robot),
        simFeatures(),featuresInWorld(), newFeaturesInWorld(){
        envType = simEnv;
        double x,y;
        VectorXd fpose(2);
        MatrixXd fcov(2,2);
        fcov<< covdiag, 0 , 0 , covdiag;
        for(int i = 0; i < NumOfFeatures; i++) {
            x = 3*mapR*(double)(rand() % (1000))*0.001 - 1.5*mapR;
            y = 3*mapR*(double)(rand() % (1000))*0.001 - 2.5*mapR;
            fpose << x , y;
            simFeatures.push_back(Feature(2,fpose, fcov));
        }
    }

    void FeatureExtraction() {
        tempFeatureBuffer.clear();
        newFeaturesInWorld.clear();
        VectorXd robotRealPose = robot.GetRealPose();
        VectorXd robotPose(2);
        double fi = robotRealPose(2);
        robotPose << robotRealPose(0), robotRealPose(1);
        VectorXd dist = robotPose;
        VectorXd z(2);
        MatrixXd newcov(2,2);
        VectorXd noise(2);
        noise << (double)(rand() % 3*covdiag - 3*covdiag/2), (double)(rand() % 3*covdiag - 3*covdiag/2);
        double sigmAlf;
        for(auto i : simFeatures) {
            dist = (i.GetPose() + noise) - robotPose;
            z[0] = sqrt(dist.transpose()*dist);
            if( z[0] < sensorR) {
                z[1] = atan2(dist(y),dist(x)) - fi;
                newcov = i.GetCovMatrix();
                sigmAlf = atan2(newcov(0),z[0]);
                newcov << newcov(0), 0,
                                 0,     sigmAlf;
                tempFeatureBuffer.push_back(Feature(2,z,newcov));
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

    VectorXd FeatureInRobotFrame(VectorXd robotPose, VectorXd featureWorldPose) {
        VectorXd rho(2);
        rho<< featureWorldPose(x) - robotPose(x),
              featureWorldPose(y) - robotPose(y);
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

     vector<FEATURE_ID> MatchedFeatures(VectorXd robotPose) {
         vector<FEATURE_ID> matchdfeatures;
         bool isNew = true;
         for(auto i : tempFeatureBuffer) { //UUSEE SET or unordered setl;
             isNew = true;
             for(auto j : featuresInWorld) {
                 if(i.GetID() == j.GetID()){
                     matchdfeatures.push_back(i.GetID());
                     isNew = false;
                     break;
                 }
             }
             if(isNew) {
                 Feature temp(i);
                 temp.SetPose(FeatureInWorldFrame(robotPose,i.GetPose()));
                 newFeaturesInWorld.push_back(temp);
             }
         }
         featuresInWorld.insert(featuresInWorld.end(),newFeaturesInWorld.begin(),newFeaturesInWorld.end());
         return matchdfeatures;
    }


     vector<Feature> NewFeaturesInWorld(VectorXd robotPose) {
         vector<Feature> temp(newFeaturesInWorld);
         newFeaturesInWorld.clear();
         return temp;
     }

     void FeaturePoseCorrection(VectorXd newPose, FEATURE_ID featureID) {
         for(auto i : featuresInWorld) {
             if(i.GetID() == featureID) {
                 i.SetPose(newPose);
                 return;
             }
         }
         cerr<<"Invalid feature ID in FeaturePoseCorrection"<<endl;
     }

     int EnvType() {
         return envType;
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
        saveMap<<"# name: dead"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<2*featuresInWorld.size()<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : featuresInWorld) {
            saveMap<<i.GetPose()<<endl;
        }
     }

};



#endif // SIMFEATUREBASE_H
