#ifndef FEATUREBASE_H
#define FEATUREBASE_H

#include<feature.h>
#include<vector>

int tempBufferSize = 200;

using namespace std;

class Enviroment {
public:
    Enviroment() {}
    virtual int EnvType() = 0;
};

class FeatureBase: public Enviroment {

protected:
    int envType;
    vector<Feature> matchedFeatures;
    static vector<Feature> tempFeatureBuffer; //for particle filter do not have extract feature for each particle
public:
    FeatureBase () {}
    virtual void FeatureExtraction() {}                                                   // get sensor data and extract features form it (put those feature in the tempFeatureBuffer) pl particle filter                                     //inRoboFrame it needs to particle filter
    virtual VectorXd FeatureInWorldFrame(VectorXd robotPose, VectorXd relative) {}
    virtual vector<Feature> NewFeaturesInWorld(VectorXd robotPose) {}                                 // new observed features in wordframe
    virtual vector<Feature> MatchedFeatures(VectorXd robotPose) {}                                      //vector of features ID that matched
    virtual VectorXd FeatureInRobotFrame(VectorXd robotPose, VectorXd featureWorldPose) {}
    virtual MatrixXd JacobianOfOdservation(VectorXd robotPose, VectorXd featureWolrdFrame) {}// it is the Jacobian of the FeatureInRobotFrame function
    virtual void SyncFeatures(VectorXd& features ) {}
    virtual void AngleNorm(VectorXd& feature) {}
    static vector<Feature> GetTempFeatureBuffer() {
        return tempFeatureBuffer;
    }
    static void ClearTempFeatureBuffer() {
        tempFeatureBuffer.clear();
    }
    // by dx, dy, dfi, dfx, dfy (fx fy the feature's pose)
};

vector<Feature> FeatureBase:: tempFeatureBuffer;

#endif // FEATUREBASE_H
