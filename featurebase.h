#ifndef FEATUREBASE_H
#define FEATUREBASE_H

#include<feature.h>
#include<vector>


using namespace Eigen;
using namespace std;

class FeatureBase {

protected:
    static vector<Feature> tempFeatureBuffer; //for particle filter do not have extract feature for each particle
public:
    FeatureBase () {}
    virtual void FeatureExtraction() = 0;                                                    // get sensor data and extract features form it (put those feature in the tempFeatureBuffer) pl particle filter                                     //inRoboFrame it needs to particle filter
    virtual VectorXd FeatureInWorldFrame(VectorXd robotPose, VectorXd relative) = 0;
    virtual vector<Feature> NewFeatures(VectorXd robotPose) {}                                 // new observed features in wordframe
    virtual vector<unsigned int> MatchedFeatures(void) {}                                      //vector of features ID that matched
    virtual VectorXd ExpectedObservation(VectorXd robotPose, VectorXd featureWorldFrame) = 0;
    virtual MatrixXd JacobianOfOdservation(VectorXd robotPose, VectorXd featureWolrdFrame) = 0;  // it is the Jacobian of the FeatureInRobotFrame function
    virtual void CorrectionFeaturePose(VectorXd newPose, unsigned int featureID) {}

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
