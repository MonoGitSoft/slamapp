#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <slam.h>
#include <sample.h>
#include <featurebase.h>
#include <list>
#include <simdiffrobot.h>

class FeatureParticle {
    VectorXd pose;
    list<Feature> features;
    double weight;
    FeatureBase& feature;
    double treshold;
    static Eigen::EigenMultivariateNormal<double> normX_solver;
    DifferencialRobotSim robot;
public:
    FeatureParticle(int dim,FeatureBase& feature):pose(dim),features(), weight(), treshold(0.01), feature(feature), robot() {
    }

    FeatureParticle(const FeatureParticle& other): pose(other.pose),features(other.features), weight(other.weight),treshold(0.01), feature(other.feature), robot(other.robot) {

    }

    void AddPose(VectorXd newpose) {
        pose += newpose;
        FiNorm(pose);
    }

    void Sampling() {
        VectorXd poseSample;
        poseSample = robot.DeadReckoningPose();
        MatrixXd cov;
        cov = robot.GetMotionCov(poseSample);
        robot.ResetMotionCov();
        normX_solver.setMean(poseSample);
        normX_solver.setCovar(cov);
        pose = normX_solver.samples(1);
        robot.SetPose(pose);
    }

    void FiNorm(VectorXd& vec) {
        if(vec(2) > M_PI) {
            vec(2) = -(2*M_PI - vec(2));
        }
        if(vec(2) < -M_PI) {
            vec(2) = 2*M_PI + vec(2);
        }
    }

    VectorXd GetPose() {
        VectorXd temp(2);
        temp<<pose(0),pose(1);
        return temp;
    }

    double GetWeight() {
        return weight;
    }

    void SetWeight(double newWeight) {
        weight = newWeight;
    }

    double Weight(list<Feature>::iterator it, Feature toThis) {
        VectorXd predict;
        VectorXd delta;
        MatrixXd H;
        MatrixXd tempH;
        MatrixXd Q;
        double featureSize;
        double w;
        predict = feature.FeatureInRobotFrame(pose,it->GetPose());
        tempH = feature.JacobianOfOdservation(pose,it->GetPose());
        featureSize = it->GetPose().rows();
        H = tempH.block(0,pose.rows(),featureSize,featureSize);
        Q = H*(it->GetCovMatrix())*H.transpose() + toThis.GetCovMatrix();
        delta = toThis.GetPose() - predict;
        feature.AngleNorm(delta);
        w = 1/sqrt((2*M_PI*Q.determinant()))*exp(-0.5*delta.transpose()*Q.inverse()*delta);
        return w;
    }

    list<Feature>::iterator Nearest(Feature toThis) {
        double tempW;
        double w;
        list<Feature>::iterator nearest;
        if(features.empty()) {
            cerr<<"No features in this particle"<<endl;
            return nearest;
        }
        for (list<Feature>::iterator it = features.begin(); it != features.end(); it++) {
            tempW = Weight(it,toThis);
            if(tempW > w) {
                w = tempW;
                nearest = it;
            }
        }
        return nearest;
    }

    void Weighting() {
        weight = 0.1; //kináozm elejen mert most igyű
        list<Feature>::iterator nearest;
        double w;
        VectorXd predict;
        VectorXd delta;
        MatrixXd H;
        MatrixXd tempH;
        MatrixXd Q;
        double featureSize;
        MatrixXd K;
        if(features.empty()) {
            for(auto i : feature.GetTempFeatureBuffer()) {
                featureSize = i.GetPose().rows();
                features.push_back(i);
                features.back().SetPose(feature.FeatureInWorldFrame(pose,i.GetPose()));
                tempH = feature.JacobianOfOdservation(pose,features.back().GetPose());
                H = tempH.block(0,pose.rows(),featureSize,featureSize);
                H = H.inverse();
                features.back().SetCovMatrix(H*i.GetCovMatrix()*H.transpose());
            }
        }
        for(auto i : feature.GetTempFeatureBuffer()) {
            nearest = Nearest(i);
            featureSize = i.GetPose().rows();
            w = Weight(nearest,i);;
            if(w < treshold) {
                features.push_back(i);
                features.back().SetPose(feature.FeatureInWorldFrame(pose,i.GetPose()));
                tempH = feature.JacobianOfOdservation(pose,features.back().GetPose());
                H = tempH.block(0,pose.rows(),featureSize,featureSize);
                H = H.inverse();
                features.back().SetCovMatrix(H*i.GetCovMatrix()*H.transpose());
            }
            else {
                predict = feature.FeatureInRobotFrame(pose,nearest->GetPose());
                tempH = feature.JacobianOfOdservation(pose,nearest->GetPose());
                H = tempH.block(0,pose.rows(),featureSize,featureSize);
                Q = H*(nearest->GetCovMatrix())*H.transpose() + i.GetCovMatrix();
                delta = i.GetPose() - predict;
                feature.AngleNorm(delta);
                K = nearest->GetCovMatrix()*H.transpose()*Q.inverse();
                nearest->SetPose(nearest->GetPose() + K*delta);
                nearest->SetCovMatrix( (MatrixXd::Identity(featureSize,featureSize) - K*H)*nearest->GetCovMatrix() );
                weight += w;
            }
        }
    }
};

VectorXd a(3); MatrixXd c(3,3);

Eigen::EigenMultivariateNormal<double> FeatureParticle::normX_solver(a, c);

class ParticleFilter: public Slam {
    FeatureBase& feature;
    list<FeatureParticle> particles;
    list<FeatureParticle>::iterator bestFit;
    VectorXd currentPose;
    VectorXd previousPose;
    VectorXd deltaPose;
    MatrixXd motionCov;
    int numOfparticles;
    Eigen::EigenMultivariateNormal<double> *normX_solver;
    vector<VectorXd> path;
    vector<VectorXd> afterPath;

public:
    ParticleFilter(MotionModell &robot, Enviroment &enviroment,int numOfparticles): Slam(robot,enviroment),feature(dynamic_cast<FeatureBase&>(enviroment)), particles(), numOfparticles(numOfparticles)
    ,currentPose(), previousPose(), motionCov(), deltaPose(), path(),afterPath() {
        currentPose = robot.DeadReckoningPose();
        previousPose = currentPose;
        deltaPose = currentPose;
        if(previousPose.rows() == 0) {
            cerr<<"Robot return inlvalid pose"<<endl;
            return;
        }
        motionCov = robot.GetMotionCov(currentPose);
        robot.ResetMotionCov();
        if((motionCov.rows() == 0) || (motionCov.rows() != motionCov.cols())) {
            cerr<<"Robot return invalid cov"<<endl;
            return;
        }

        for(int i = 0; i < numOfparticles; i++) {
            particles.push_back(FeatureParticle(3,feature));
        }

        feature.FeatureExtraction(); // importanteee

        for(auto& i : particles) {
            i.Weighting();
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

    void Save() {
        ofstream savePoses;
        savePoses.open ("particlepose.m");
        savePoses<<"# name: particlepose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<path.size()*3<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : path) {
            savePoses<<i<<endl;
        }
        savePoses<<"# name: resamplepose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<afterPath.size()*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : afterPath) {
            savePoses<<i<<endl;
        }
    }

    void Sampling() {
        for(auto& i : particles) {
            i.Sampling();
        }
    }

    void Weighting() {
        feature.FeatureExtraction(); // importanteee
        double max;
        VectorXd maxPose;
        double sum;
        VectorXd debg(3);
        for(auto& i : particles) {
            i.Weighting();
            sum += i.GetWeight();
            if(i.GetWeight() > max) {
                max = i.GetWeight();
                maxPose = i.GetPose();
            }
        }
        for(auto& i : particles) {
            i.SetWeight(i.GetWeight()/sum);
            debg<<i.GetPose()(0),i.GetPose()(1),i.GetWeight();
            path.push_back(debg);
        }
    }

    void ReSampling() {
        default_random_engine generator;
        double Minv = 1/(double)numOfparticles;
        uniform_real_distribution<double> randR(0,Minv);
        list<FeatureParticle> nextGen;
        double r = randR(generator);
        double u = 0;
        double c = particles.begin()->GetWeight();
        list<FeatureParticle>::iterator i = particles.begin();

        for(int m(0); m < numOfparticles; m++) {
            u = r + m*Minv;
            while( u > c)  {
                i++;
                c += i->GetWeight();
            }
            nextGen.push_back(*i);
        }
        particles.clear();
        for(auto& i : nextGen) {
            particles.push_back(i);
            afterPath.push_back(i.GetPose());

        }
    }

    void Step() {}
};

#endif // PARTICLEFILTER_H

