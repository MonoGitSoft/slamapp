#ifndef EKFSLAM_H
#define EKFSLAM_H

#include<slam.h>
#include<set>



class LookUp {
    int ID;
    int slamID;
public:
    LookUp(int ID, int slamID): ID(ID), slamID(slamID) {}
    int GetSlamID() const{
        return slamID;
    }
    bool operator<(const LookUp& other) const{
        if(ID < other.ID) {
            return true;
        }
        else {
            return false;
        }
    }
};

class EKFSlam : public Slam {
    VectorXd pose;
    VectorXd previousPose;
    MatrixXd stateCov;
    MatrixXd state; // inluce pose + featue pose;
    FeatureBase &feature;
    set<LookUp> lookUpTable;
    int stateSize;
    int poseSize;
    double featureRow;

public:
    EKFSlam(MotionModell &robot, Enviroment &enviroment): Slam(robot,enviroment), feature(dynamic_cast<FeatureBase&>(enviroment)), lookUpTable(),featureRow(0) {
        previousPose = robot.DeadReckoningPose();
        if(previousPose.rows() == 0) {
            cerr<<"Robot return inlvalid pose"<<endl;
            return;
        }
        auto motionCov = robot.GetMotionCov(previousPose);
        if((motionCov.rows() == 0) || (motionCov.rows() != motionCov.cols())) {
            cerr<<"Robot return invalid cov"<<endl;
            return;
        }
        stateSize = previousPose.rows();
        poseSize = stateSize;
        stateCov.resize(poseSize,poseSize);
        stateCov = motionCov;
        state = previousPose;
        feature.FeatureExtraction();
        feature.MatchedFeatures(previousPose);
        auto newfeatures = feature.NewFeaturesInWorld(previousPose);
        PutNewFeatures(newfeatures);
    }

    void PutNewFeatures(vector<Feature>& NewFeatures) {
        if(NewFeatures.size() == 0) {
            return;
        }
        double plusstateSize;
        if(featureRow == 0) {
            featureRow = NewFeatures.back().GetPose().rows();
        }
        if(featureRow == 0) {
            cerr<<"FeatureBase return invalid feature pose"<<endl;
            return;
        }
        plusstateSize = NewFeatures.size()*featureRow;
        state.conservativeResize(stateSize + plusstateSize,1);
        stateCov.conservativeResize(stateSize + plusstateSize,stateSize + plusstateSize);
        stateCov.block(stateSize,0,plusstateSize,plusstateSize + stateSize ) = MatrixXd::Zero(plusstateSize, plusstateSize + stateSize);
        stateCov.block(0,stateSize,stateSize,plusstateSize) = MatrixXd::Zero(stateSize, plusstateSize);
        for(auto i : NewFeatures) {
            state.block(stateSize,0,featureRow,1) = i.GetPose();
            stateCov.block(stateSize,stateSize,featureRow,featureRow) = i.GetCovMatrix();
            lookUpTable.insert(LookUp(i.GetID(),stateSize));
            stateSize += featureRow;
        }
    }

    void FiNorm() {
        if(state(2,0) > M_PI) {
            state(2,0) = -(2*M_PI - state(2,0));
        }
        if(state(2,0) < -M_PI) {
            state(2,0) = 2*M_PI + state(2,0);
        }
    }

    VectorXd SlamPose() {
        return state.block(0,0,poseSize,1);
    }

    void Step() {
        pose = robot.DeadReckoningPose();
        state.block(0,0,poseSize,1) += (pose - previousPose);
        previousPose = pose;
        auto motionCov = robot.GetMotionCov(SlamPose());
        robot.ResetMotionCov(); // RST motion cov
        auto jacobiOfmotion = robot.JacobianOfMotion(SlamPose());
        stateCov.block(0,0,poseSize,poseSize) =  jacobiOfmotion*stateCov.block(0,0,poseSize,poseSize)*jacobiOfmotion.transpose();
        MatrixXd GtxM(poseSize,stateSize - poseSize);
        GtxM = jacobiOfmotion*stateCov.block(0,poseSize,poseSize,stateSize - poseSize);
        stateCov.block(0,poseSize,poseSize,stateSize - poseSize) = GtxM;
        stateCov.block(poseSize,0,stateSize - poseSize,poseSize) = GtxM.transpose();
        stateCov.block(0,0,poseSize,poseSize) += motionCov;
        feature.FeatureExtraction();
        auto matchedFeatures = feature.MatchedFeatures(SlamPose());
        auto newFeatures = feature.NewFeaturesInWorld(SlamPose());
        PutNewFeatures(newFeatures);
        MatrixXd Ht;
        std::set<LookUp>::iterator it;
        int slamID;
        VectorXd matchedFeature;
        MatrixXd Hhigh;
        Hhigh.resize(featureRow,stateSize);
        MatrixXd Kalman; // bácsi megmondta
        for(auto i : matchedFeatures) {
            Hhigh=MatrixXd::Zero(featureRow, stateSize);
            it = lookUpTable.find(LookUp(i.GetID(),0));
            slamID = it->GetSlamID();
            matchedFeature = state.block(slamID,0,featureRow,1);
            Ht = feature.JacobianOfOdservation(SlamPose(),matchedFeature);
            Hhigh.block(0,0,featureRow,poseSize) = Ht.block(0,0,featureRow,poseSize);
            Hhigh.block(0,slamID,featureRow,featureRow) = Ht.block(0,poseSize,featureRow,featureRow);
            Kalman = stateCov*Hhigh.transpose()*(Hhigh*stateCov*Hhigh.transpose() + i.GetCovMatrix()).inverse();
            state = state + Kalman*(i.GetPose() - feature.FeatureInRobotFrame(SlamPose(),matchedFeature));
            stateCov = (MatrixXd::Identity(Kalman.rows(),Kalman.rows()) - Kalman*Hhigh)*stateCov;
            FiNorm();

        }
    }

};


#endif // EKFSLAM_H
