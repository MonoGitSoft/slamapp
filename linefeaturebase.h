#ifndef LINEFEATUREBASE_H
#define LINEFEATUREBASE_H

#include <featurebase.h>
#include <simdiffrobot.h>
#include <set>
#include <random>


class LineBase: public FeatureBase {
    set<int> featuresID; // features in world frame in x,y(z)
    vector<Feature> newFeaturesInWorld;
    vector<Feature> featuresInWorld;
    vector<Feature> matchedFeatures;
    double sensorR;
    DifferencialRobotSim& robot;
    double linecov;
public:
    LineBase(double sensorR,DifferencialRobotSim& robot): sensorR(sensorR), robot(robot), newFeaturesInWorld(), featuresInWorld() ,linecov(1)
    ,matchedFeatures() {}

    void AngleNorm(VectorXd& feature) {
        if(feature(0) > M_PI) {
            feature(0) = -(2*M_PI - feature(0));
        }
        if(feature(0) < -M_PI) {
            feature(0) = 2*M_PI + feature(0);
        }
    }

    void SyncFeatures(VectorXd& features ) {
        for(int i = 0; i < features.rows(); i+=2) {
            if(features(i) > M_PI) {
                features(i) = -(2*M_PI - features(i));
            }
            if(features(i) < -M_PI) {
                features(i) = 2*M_PI + features(i);
            }
        }
    }

    void FeatureExtraction() {
        tempFeatureBuffer.clear();
        newFeaturesInWorld.clear();
        matchedFeatures.clear();
        default_random_engine generator;
        uniform_real_distribution<double> rDistro(0,sensorR);
        uniform_real_distribution<double> piDistro(-M_PI/2,M_PI/2);
        normal_distribution<double> alfaNoise(0,sqrt(0.008));
        normal_distribution<double> rNoise(0,sqrt(2));
        VectorXd robotRealPose(3);
        robotRealPose = robot.GetRealPose();
        VectorXd robotPose(2);
        robotPose << robotRealPose(0), robotRealPose(1);
        double fi = robotRealPose(2);
        double r;
        VectorXd fpose(2);
        MatrixXd fcov(2,2);
        int count(0);
        for(auto i : featuresInWorld) {
            //r = i.GetPose()(1) - (robotPose(0)*cos(i.GetPose()(0)) + robotPose(1)*sin(i.GetPose()(0)));
            fpose = FeatureInRobotFrame(robotRealPose,i.GetPose());
            if( abs(fpose(1)) < sensorR) {
                tempFeatureBuffer.push_back(i);
                //fpose << i.GetPose()(0) - fi, r;
                fpose = FeatureInRobotFrame(robotRealPose,i.GetPose());
                fpose<< fpose(0) + alfaNoise(generator), fpose(1) + rNoise(generator);
                AngleNorm(fpose);
                fcov<< 0.008, 0,
                         0,        2;
                tempFeatureBuffer.back().SetPose(fpose);
                tempFeatureBuffer.back().SetCovMatrix(fcov);
                matchedFeatures.push_back(tempFeatureBuffer.back());
                count++;
            }
        }
        if(count < 8) {
            for(int i = 0; i < 8; i++) {
                r = rDistro(generator);
                fpose << piDistro(generator),r;
                fcov<< 0.008, 0,
                         0,         2;
                Feature temp(2,fpose,fcov);
                tempFeatureBuffer.push_back(temp);
                newFeaturesInWorld.push_back(temp);
                newFeaturesInWorld.back().SetPose(FeatureInWorldFrame(robotRealPose,temp.GetPose()));
                featuresInWorld.push_back(newFeaturesInWorld.back());
            }
        }
    }

    VectorXd FeatureInWorldFrame(VectorXd robotPose, VectorXd relative) {
        VectorXd wpose(2);
        double alfa = relative(0) + robotPose(2);
        wpose<< alfa,
                relative(1) + robotPose(0)*cos(alfa) + robotPose(1)*sin(alfa);
        AngleNorm(wpose);
        return wpose;
    }

    VectorXd FeatureInRobotFrame(VectorXd robotPose, VectorXd featureWorldPose) {
        VectorXd rpose(2);
        double alfa = featureWorldPose(0);
        rpose<<alfa - robotPose(2), featureWorldPose(1) - (robotPose(0)*cos(alfa) + robotPose(1)*sin(alfa));
        AngleNorm(rpose);
        return  rpose;
    }

    vector<Feature> NewFeaturesInWorld(VectorXd robotPose) {
        return newFeaturesInWorld;
    }

    vector<Feature> MatchedFeatures(VectorXd robotPose) {
        return matchedFeatures;
    }

    MatrixXd JacobianOfOdservation(VectorXd robotPose, VectorXd featureWolrdFrame) {
        MatrixXd jacobi(2,5);
        double alfa = featureWolrdFrame(0);
        double x = robotPose(0);
        double y = robotPose(1);
        double r = featureWolrdFrame(1);
        jacobi<<    0,         0,     -1,       1,             0,
                -cos(alfa),-sin(alfa), 0 , x*sin(alfa) - y*cos(alfa), 1;
        return jacobi;
    }

    int EnvType() {
        return 10;
    }

    void Save() {
        ofstream savePoses;
        savePoses.open ("mappose.m");
        savePoses<<"# name: mappose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<featuresInWorld.size()*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : featuresInWorld) {
            savePoses<<i.GetPose()<<endl;
        }
    }

};


#endif // LINEFEATUREBASE_H
