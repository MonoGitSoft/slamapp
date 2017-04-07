#ifndef FEATURE_H
#define FEATURE_H

#include<iostream>
#include<Eigen/Dense>

using namespace Eigen;

#define FEATURE_ID int

class Feature {
    VectorXd featurePose;
    MatrixXd covMatrix;
    static int featureIDcounter;
    FEATURE_ID featureID;
public:

    Feature() : featurePose(2), covMatrix(2,2), featureID(featureIDcounter) {
        featureIDcounter++;
    } //deafult constructor creature 2D pose and its cov matrix
    Feature( Index dim ) : featurePose(dim), covMatrix(dim,dim), featureID(featureIDcounter) {
        featureIDcounter++;
    }

    Feature(Index dim,VectorXd pose, MatrixXd cov): featurePose(dim), covMatrix(dim,dim), featureID(featureIDcounter) {
        featurePose = pose;
        covMatrix = cov;
        featureIDcounter++;
    }

    Feature(const Feature& other):featureID(other.featureID) {
        this->featurePose = other.featurePose;
        this->covMatrix = other.covMatrix;
    }

    VectorXd GetPose() {
        return featurePose;
    }

    int GetID(){
        return featureID;
    }

    void SetID(int id) { // not responsible for id is unique
        featureID = id;
    }

    MatrixXd GetCovMatrix() {
        return covMatrix;
    }

    void SetCovMatrix(MatrixXd newMatrix ) {
        if( (newMatrix.rows() == covMatrix.rows()) && (newMatrix.cols() == covMatrix.cols()) ) {
            covMatrix = newMatrix;
        }
        else {
            std::cerr<<"Invalid new matrix in feature::SetMatrix()!!!"<<std::endl;
        }
    }

    void SetPose(VectorXd newpose ) {
        if( (newpose.rows() == featurePose.rows() ) && ( newpose.cols() == featurePose.cols() )) {
            featurePose = newpose;
        }
        else {
            std::cerr<<"Invalid new pose in feature::Setpose()!!!"<<std::endl; //TODO change itt qtDebug
        }
    }
};

std::ostream& operator<<(std::ostream& os, Feature& feature) {
    os<<"ID: "<<feature.GetID()<<std::endl;
    os<<"Pose: "<<std::endl<<feature.GetPose()<<std::endl;
    os<<"Cov Matrix: "<<std::endl<<feature.GetCovMatrix()<<std::endl;
    return os;
}

int Feature::featureIDcounter = 0;


#endif // FEATURE_H
