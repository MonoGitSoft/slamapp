#ifndef SIMFEATUREBASE_H
#define SIMFEATUREBASE_H

#include <featurebase.h>
#include <simdiffrobot.h>
#include <set>
#include <random>

bool first = true;

int x = 0;
int y = 1;
int fi = 2;
int r = 0;
int theta = 1;

class SimFeatureBase: public FeatureBase {
    set<int> featuresID; // features in world frame in x,y(z)
    vector<Feature> newFeaturesInWorld;
    vector<Feature> featuresInWorld;
    vector<Feature> matchedFeatures;
    double sensorR;
    DifferencialRobotSim& robot;
    double linecov;
public:
    SimFeatureBase(double sensorR,DifferencialRobotSim& robot): sensorR(sensorR), robot(robot), newFeaturesInWorld(), featuresInWorld() ,linecov(1)
    ,matchedFeatures() {}

    void AngleNorm(VectorXd& feature) {
        if(feature(1) > M_PI) {
            feature(1) = -(2*M_PI - feature(1));
        }
        if(feature(1) < -M_PI) {
            feature(1) = 2*M_PI + feature(1);
        }
    }

    void SyncFeatures(VectorXd& features ) {

    }

    void FeatureExtraction() {
        tempFeatureBuffer.clear();
        newFeaturesInWorld.clear();
        matchedFeatures.clear();
        default_random_engine generator;
        uniform_real_distribution<double> rDistro(0,sensorR);
        uniform_real_distribution<double> piDistro(-M_PI/2,M_PI/2);
        normal_distribution<double> alfaNoise(0,sqrt(0.004));
        normal_distribution<double> rNoise(0,sqrt(1));
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
            if( abs(fpose(0)) < sensorR) {
                tempFeatureBuffer.push_back(i);
                //fpose << i.GetPose()(0) - fi, r;
                fpose = FeatureInRobotFrame(robotRealPose,i.GetPose());
                fpose<< fpose(0), fpose(1) ;
                AngleNorm(fpose);
                fcov<< 1, 0,
                         0,        0.004;
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
                fcov<< 1, 0,
                         0,        0.004;
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
        wpose<< robotPose(x) + relative(r)*cos(robotPose(fi) + relative(theta)),
                robotPose(y) + relative(r)*sin(robotPose(fi) + relative(theta));
        return wpose;
    }

    VectorXd FeatureInRobotFrame(VectorXd robotPose, VectorXd featureWorldPose) {
        VectorXd rho(2);
        rho<< featureWorldPose(x) - robotPose(x),
              featureWorldPose(y) - robotPose(y);
        double q = rho.transpose()*rho;
        VectorXd ret(2);
        ret<<sqrt(q),atan2(rho(y),rho(x)) - robotPose(fi);
        return ret;
    }

    vector<Feature> NewFeaturesInWorld(VectorXd robotPose) {
        return newFeaturesInWorld;
    }

    vector<Feature> MatchedFeatures(VectorXd robotPose) {
        return matchedFeatures;
    }

    MatrixXd JacobianOfOdservation(VectorXd robotPose, VectorXd featureWolrdFrame) {
        VectorXd rho(2);
        MatrixXd jacobi(2,5);
        rho<< featureWolrdFrame(x) - robotPose(x), featureWolrdFrame(y) - robotPose(y);
        double q = rho.transpose()*rho;
        jacobi << -sqrt(q)*rho(x), -sqrt(q)*rho(y), 0, sqrt(q)*rho(x), sqrt(q)*rho(y),
                    rho(y),         -rho(x),       -q,    -rho(y),       rho(x);
        jacobi *= 1/q;
        return jacobi;
    }

    int EnvType() {
        return 10;
    }

};


#endif // SIMFEATUREBASE_H
