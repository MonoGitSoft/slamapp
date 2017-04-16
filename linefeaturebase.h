#ifndef LINEFEATUREBASE_H
#define LINEFEATUREBASE_H

#include <featurebase.h>
#include <simdiffrobot.h>


class LineBase: public FeatureBase {
    vector<Feature> featuresInWorld; // features in world frame in x,y(z)
    vector<Feature> newFeaturesInWorld;
    const int mapR;
    double sensorR;
    DifferencialRobotSim& robot;
    vector<Feature> simFeatures;
    double linecov;
    int NumOfFeatures;
public:
    LineBase(double sensorR,DifferencialRobotSim& robot): mapR(2000), sensorR(sensorR), robot(robot),
        simFeatures(),featuresInWorld(), newFeaturesInWorld(), NumOfFeatures(20), linecov(100) {
        double x,y;
        VectorXd fpose(2);
        MatrixXd fcov(2,2);
        fcov<< covdiag, 0 , 0 , covdiag;
        for(int i = 0; i < NumOfFeatures; i++) {
            x = 3*mapR*(double)(rand() % (1000))*0.001 - 1.5*mapR;
            y = 3*mapR*(double)(rand() % (1000))*0.001 - 2.5*mapR;
            fpose << atan2(y,x) , sqrt(y*y + x*x);
            simFeatures.push_back(Feature(2,fpose, fcov));
        }
    }

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
        VectorXd robotRealPose(3);
        robotRealPose = robot.GetRealPose();
        VectorXd robotPose(2);
        robotPose << robotRealPose(0), robotRealPose(1);
        double fi = robotRealPose(2);
        double r;
        VectorXd fpose(2);
        MatrixXd fcov(2,2);
        for(auto i : simFeatures) {
            r = i.GetPose()(1) - (robotPose(0)*cos(i.GetPose()(0)) + robotPose(1)*sin(i.GetPose()(0)));
            if(r < sensorR) {
                tempFeatureBuffer.push_back(i);
                fpose << i.GetPose()(0) - fi, r;
                AngleNorm(fpose);
                fcov<< 4*linecov*linecov, 0,
                         0,         4*atan2(linecov,r)*atan2(linecov,r);
                tempFeatureBuffer.back().SetPose(fpose);
                tempFeatureBuffer.back().SetCovMatrix(fcov);
            }
        }
    }

    VectorXd FeatureInWorldFrame(VectorXd robotPose, VectorXd relative) {
        VectorXd wpose(2);
        double alfa = relative(0) + robotPose(2);
        wpose<< alfa,
                relative(1) + robotPose(0)*cos(alfa) + robotPose(1)*sin(alfa);
        return wpose;
    }

    VectorXd FeatureInRobotFrame(VectorXd robotPose, VectorXd featureWorldPose) {
        VectorXd rpose(2);
        double alfa = featureWorldPose(0);
        rpose<<alfa - robotPose(2), featureWorldPose(1) - (robotPose(0)*cos(alfa) + robotPose(1)*sin(alfa));
        return  rpose;
    }

    vector<Feature> NewFeaturesInWorld(VectorXd robotPose) {
        vector<Feature> temp;
        for(auto i : newFeaturesInWorld) {
            temp.push_back(i);
            temp.back().SetPose(FeatureInWorldFrame(robotPose,i.GetPose()));
        }
        newFeaturesInWorld.clear();
        return temp;
    }

    vector<Feature> MatchedFeatures(VectorXd robotPose) {
        //vector<FEATURE_ID> matchdFeatures;
        matchedFeatures.clear();
        bool isNew = true;
        for(auto i : tempFeatureBuffer) { //UUSEE SET or unordered setl;
            isNew = true;
            for(auto j : featuresInWorld) {
                if(i.GetID() == j.GetID()){
                    matchedFeatures.push_back(i);
                    isNew = false;
                    break;
                }
            }
            if(isNew) {
                Feature temp(i);
                newFeaturesInWorld.push_back(temp);
            }
        }
        featuresInWorld.insert(featuresInWorld.end(),newFeaturesInWorld.begin(),newFeaturesInWorld.end());
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

};


#endif // LINEFEATUREBASE_H
