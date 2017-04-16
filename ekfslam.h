#ifndef EKFSLAM_H
#define EKFSLAM_H

#include <limits>
#include <slam.h>
#include <set>



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

    vector<VectorXd> path;
    vector<MatrixXd> pathcov;
    vector<MatrixXd> featuresPose;

public:
    EKFSlam(MotionModell &robot, Enviroment &enviroment): Slam(robot,enviroment), feature(dynamic_cast<FeatureBase&>(enviroment)), lookUpTable(),featureRow(0), path(), pathcov(), featuresPose() {
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
        stateCov << 200, 0 , 0,
                    0,  200, 0,
                    0,  0,  0.2;
        stateCov += motionCov;
        state = previousPose;
        feature.FeatureExtraction();
        feature.MatchedFeatures(previousPose);
        auto newfeatures = feature.NewFeaturesInWorld(previousPose);
        PutNewFeatures(newfeatures);;
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
        stateCov.block(stateSize,0,plusstateSize,plusstateSize + stateSize ) = MatrixXd::Ones(plusstateSize, plusstateSize + stateSize)*0;
        stateCov.block(0,stateSize,stateSize,plusstateSize) = MatrixXd::Ones(stateSize, plusstateSize)*0;
        for(auto i : NewFeatures) {
            state.block(stateSize,0,featureRow,1) = i.GetPose();
            stateCov.block(stateSize,stateSize,featureRow,featureRow) = MatrixXd::Ones(featureRow,featureRow)*1000000000;
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

    void FiNorm(VectorXd& vec) {
        if(vec(2) > M_PI) {
            vec(2) = -(2*M_PI - vec(2));
        }
        if(vec(2) < -M_PI) {
            vec(2) = 2*M_PI + vec(2);
        }
    }

    VectorXd SlamPose() {
        return state.block(0,0,poseSize,1);
    }

    MatrixXd SlamPoseCov() {
        return stateCov.block(0,0,poseSize,poseSize);
    }

    void SaveFeatures() {
        for(int i = 0; i < lookUpTable.size();i++) {
            featuresPose.push_back(state.block(poseSize + i*featureRow,0,featureRow,1));
        }
    }


    void StateNorm() {
        VectorXd temp(state.col(0));
        VectorXd sync(stateSize - poseSize);
        sync = temp.tail(stateSize - poseSize);
        feature.SyncFeatures(sync);
        state.block(poseSize,0,stateSize - poseSize,1) = sync;

    }

    void Step() {
        pose = robot.DeadReckoningPose();
        VectorXd temp(pose);
        temp = pose - previousPose;
        FiNorm(temp);
        state.block(0,0,poseSize,1) += (temp);
        FiNorm();
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
        auto newFeatures = feature.NewFeaturesInWorld(SlamPose());;
        MatrixXd Ht;
        std::set<LookUp>::iterator it;
        int slamID;
        VectorXd matchedFeature;
        MatrixXd Hhigh;
        Hhigh.resize(featureRow,stateSize);
        MatrixXd Kalman;
        MatrixXd tempstate(stateSize,1);
        tempstate = MatrixXd::Zero(stateSize,1);
        MatrixXd tempKH(stateSize,stateSize);
        tempKH = MatrixXd::Zero(stateSize,stateSize);
        auto real = dynamic_cast<DifferencialRobotSim&>(robot);
        for(auto i : matchedFeatures) {
            Hhigh=MatrixXd::Zero(featureRow, stateSize);
            it = lookUpTable.find(LookUp(i.GetID(),0));
            slamID = it->GetSlamID();
            matchedFeature = state.block(slamID,0,featureRow,1);
            Ht = feature.JacobianOfOdservation(SlamPose(),matchedFeature);
            Hhigh.block(0,0,featureRow,poseSize) = Ht.block(0,0,featureRow,poseSize);
            Hhigh.block(0,slamID,featureRow,featureRow) = Ht.block(0,poseSize,featureRow,featureRow);
            Kalman = stateCov*Hhigh.transpose()*((Hhigh*stateCov*Hhigh.transpose() + i.GetCovMatrix()).inverse());
            VectorXd dif;
            dif = i.GetPose() - feature.FeatureInRobotFrame(SlamPose(),matchedFeature);
            feature.AngleNorm(dif);
            state += Kalman*dif;
            StateNorm();
            stateCov = (MatrixXd::Identity(Kalman.rows(),Kalman.rows()) - Kalman*Hhigh)*stateCov;
            FiNorm();
        };
        path.push_back(SlamPose());
        pathcov.push_back(SlamPoseCov());
        SaveFeatures();
        PutNewFeatures(newFeatures);
    }

    void Save() {
        ofstream savePoses;
        savePoses.open ("slampose.m");
        savePoses<<"# name: slampose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<path.size()*3<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : path) {
            savePoses<<i<<endl;
        }
        savePoses<<"# name: slamcov"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<pathcov.size()*3<<endl
                 <<"# columns: 3"<<endl;
        for(auto i : pathcov) {
            savePoses<<i<<endl;
        }

        savePoses<<"# name: features"<<endl
               <<"# type: matrix"<<endl
               <<"# rows: "<<featuresPose.size()*2<<endl
               <<"# columns: 1"<<endl;
        for(auto i : featuresPose) {
            savePoses<<i<<endl;
        }
        savePoses.close();
    }

};


#endif // EKFSLAM_H
