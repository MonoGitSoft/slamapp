#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <slam.h>
#include <sample.h>
#include <featurebase.h>
#include <list>
#include <simdiffrobot.h>

VectorXd best(2);
double bestW = -1;
VectorXd carlo;

VectorXd bestpose(3);
double bestp = 10000;

int presample = 200;

MatrixXd tempcov(3,3);
VectorXd tempmean(3);


bool firstpre = true;

class FeatureParticle {
    VectorXd pose;
    list<Feature> features;
    double weight;
    FeatureBase& feature;
    double treshold;
    static Eigen::EigenMultivariateNormal<double> normX_solver;
    DifferencialRobotSim robot;
public:
    FeatureParticle(int dim,FeatureBase& feature):pose(dim),features(), weight(0.01), treshold(0.000001), feature(feature), robot() {
    }

    FeatureParticle(const FeatureParticle& other): pose(other.pose),features(other.features), weight(other.weight),treshold(0.000001), feature(other.feature), robot(other.robot) {

    }

    void AddPose(VectorXd newpose) {
        pose += newpose;
        FiNorm(pose);
    }

    void SetPose(VectorXd newpose) {
        pose = newpose;
        FiNorm(pose);
        robot.SetPose(pose);
    }

    double ScanMatching(VectorXd samplePose) {
        double sumW; //kináozm elejen mert most igyű
        list<Feature>::iterator nearest;
        double w;
        VectorXd predict;
        VectorXd delta;
        MatrixXd H;
        MatrixXd tempH;
        MatrixXd Q;
        double featureSize;
        MatrixXd K;
        VectorXd temp;
        if(features.empty()) {
            for(auto i : feature.GetTempFeatureBuffer()) {
                featureSize = i.GetPose().rows();
                features.push_back(i);
                features.back().SetPose(feature.FeatureInWorldFrame(samplePose,i.GetPose()));
                tempH = feature.JacobianOfOdservation(samplePose,features.back().GetPose());
                H = tempH.block(0,samplePose.rows(),featureSize,featureSize);
                H = H.inverse();
                features.back().SetCovMatrix(H*i.GetCovMatrix()*H.transpose());
            }
            return 0.1;
        }
        for(auto i : feature.GetTempFeatureBuffer()) {
            nearest = Nearest(i);
            featureSize = i.GetPose().rows();

            predict = feature.FeatureInRobotFrame(samplePose,nearest->GetPose());
            delta = i.GetPose() - predict;
            feature.AngleNorm(delta);
            Q = i.GetCovMatrix();

            if( (abs(delta(0)) > sqrt(Q(0,0))*10) || (abs(delta(1)) > sqrt(Q(1,1))*10)  ) {
                features.push_back(i);
                features.back().SetPose(feature.FeatureInWorldFrame(samplePose,i.GetPose()));
                tempH = feature.JacobianOfOdservation(samplePose,features.back().GetPose());
                H = tempH.block(0,samplePose.rows(),featureSize,featureSize);
                H = H.inverse();
                features.back().SetCovMatrix(H*i.GetCovMatrix()*H.transpose());
            }
            else {
                w = PreWeight(nearest,i,samplePose);
                tempH = feature.JacobianOfOdservation(samplePose,nearest->GetPose());
                H = tempH.block(0,samplePose.rows(),featureSize,featureSize);
                Q = H*(nearest->GetCovMatrix())*H.transpose() + i.GetCovMatrix();

                K = nearest->GetCovMatrix()*H.transpose()*Q.inverse();
                nearest->SetPose(nearest->GetPose() + K*delta);
                nearest->SetCovMatrix( (MatrixXd::Identity(featureSize,featureSize) - K*H)*nearest->GetCovMatrix() );
                sumW += w;
            }
        }
        return sumW;

    }

    void PreSampling(VectorXd& mean, MatrixXd& cov) {
        vector<VectorXd> samples;
        vector<double> weights;
        samples.reserve(presample);
        weights.reserve(presample);
        normX_solver.setMean(mean);
        normX_solver.setCovar(cov);
        VectorXd temp;
        double w = 0;;
        double wSum = 0;
        for(int i = 0; i < presample; i++) {
            temp = normX_solver.samples(1);
            w = ScanMatching(temp);
            wSum += w*w*w;
            samples.push_back(temp);
            weights.push_back(w*w*w);
        }
        mean<<0,0,0;
        for(int i = 0; i < presample; i++) {
            mean += samples[i]*weights[i];
        }
        if(wSum != 0) {
            mean *= 1/wSum;
        }

        VectorXd delta;
        cov<< 0,0,0,
              0,0,0,
              0,0,0;
        for(int i = 0; i < presample; i++) {
            delta = samples[i] - mean;
            FiNorm(delta);
            cov += delta*delta.transpose()*weights[i];
        }
        if( wSum != 0) {
            cov *= 1/wSum;
        }

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
   //     FiNorm(pose);
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

    VectorXd GetFullPose() {
        return pose;
    }

    double GetWeight() {
        return weight;
    }

    void SetWeight(double newWeight) {
        weight = newWeight;
    }


    double PreWeight(list<Feature>::iterator it, Feature toThis, VectorXd prePose) {
        VectorXd predict;
        VectorXd delta;
        MatrixXd H;
        MatrixXd tempH;
        MatrixXd Q;
        double featureSize;
        double w;
        predict = feature.FeatureInRobotFrame(prePose,it->GetPose());
        tempH = feature.JacobianOfOdservation(prePose,it->GetPose());
        featureSize = it->GetPose().rows();
        H = tempH.block(0,prePose.rows(),featureSize,featureSize);
        Q = H*(it->GetCovMatrix())*H.transpose() + toThis.GetCovMatrix();
        delta = toThis.GetPose() - predict;
        feature.AngleNorm(delta);
        w = 1/sqrt((2*M_PI*Q.determinant()))*exp(-0.5*delta.transpose()*Q.inverse()*delta);
        return w;
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
        double w = 100000;
        list<Feature>::iterator nearest;
        VectorXd delta;
        if(features.empty()) {
            cerr<<"No features in this particle"<<endl;
            return nearest;
        }
        for (list<Feature>::iterator it = features.begin(); it != features.end(); it++) {
            delta = feature.FeatureInRobotFrame(pose,it->GetPose()) - toThis.GetPose();
            feature.AngleNorm(delta);
            tempW = delta.norm();
            if(tempW < w) {
                w = tempW;
                nearest = it;
            }
        }

        return nearest;
    }

    void Weighting() {
        weight = 0.000001; //kináozm elejen mert most igyű
        list<Feature>::iterator nearest;
        double w;
        VectorXd predict;
        VectorXd delta;
        MatrixXd H;
        MatrixXd tempH;
        MatrixXd Q;
        double featureSize;
        MatrixXd K;
        VectorXd temp;
        temp = pose - robot.GetRealPose();
        FiNorm(temp);
        if( temp.norm() < bestp) {
            bestp = temp.norm();
            bestpose = pose;
        }
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
            return;
        }
        for(auto i : feature.GetTempFeatureBuffer()) {
            nearest = Nearest(i);
            featureSize = i.GetPose().rows();

            predict = feature.FeatureInRobotFrame(pose,nearest->GetPose());
            delta = i.GetPose() - predict;
            feature.AngleNorm(delta);
            Q = i.GetCovMatrix();

            if( (abs(delta(0)) > sqrt(Q(0,0))*5) || (abs(delta(1)) > sqrt(Q(1,1))*5)  ) {
                features.push_back(i);
                features.back().SetPose(feature.FeatureInWorldFrame(pose,i.GetPose()));
                tempH = feature.JacobianOfOdservation(pose,features.back().GetPose());
                H = tempH.block(0,pose.rows(),featureSize,featureSize);
                H = H.inverse();
                features.back().SetCovMatrix(H*i.GetCovMatrix()*H.transpose());
            }
            else {
                w = Weight(nearest,i);
                tempH = feature.JacobianOfOdservation(pose,nearest->GetPose());
                H = tempH.block(0,pose.rows(),featureSize,featureSize);
                Q = H*(nearest->GetCovMatrix())*H.transpose() + i.GetCovMatrix();

                K = nearest->GetCovMatrix()*H.transpose()*Q.inverse();
                nearest->SetPose(nearest->GetPose() + K*delta);
                nearest->SetCovMatrix( (MatrixXd::Identity(featureSize,featureSize) - K*H)*nearest->GetCovMatrix() );
                weight += w;
            }
        }
    }

    void WeightingWithOutKalma() { //Weighting()
        weight = 0; //kináozm elejen mert most igyű
        list<Feature>::iterator nearest;
        double w;
        VectorXd predict;
        VectorXd delta;
        MatrixXd H;
        MatrixXd tempH;
        MatrixXd Q;
        double featureSize;
        MatrixXd K;
        VectorXd temp;
        temp = pose - robot.GetRealPose();
        FiNorm(temp);
        if( temp.norm() < bestp) {
            bestp = temp.norm();
            bestpose = pose;
        }
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
            return;
        }
        for(auto i : feature.GetTempFeatureBuffer()) {
            nearest = Nearest(i);
            w = Weight(nearest,i);
            weight += w;
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
    Eigen::EigenMultivariateNormal<double> normX_solver;
    vector<VectorXd> path;
    vector<VectorXd> afterPath;

public:
    ParticleFilter(MotionModell &robot, Enviroment &enviroment,int numOfparticles): Slam(robot,enviroment),feature(dynamic_cast<FeatureBase&>(enviroment)), particles(), numOfparticles(numOfparticles)
    ,currentPose(), previousPose(), motionCov(), deltaPose(), path(),afterPath(), normX_solver(a,c) {
       /* currentPose = robot.DeadReckoningPose();
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
        }*/
        for(int i = 0; i < numOfparticles; i++) {
            particles.push_back(FeatureParticle(3,feature));
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
        robot.DeadReckoningPose();
        feature.FeatureExtraction();
        vector<VectorXd> samples;
        vector<double> weights;
        VectorXd scanMean(3);
        MatrixXd scanCov(3,3);
        VectorXd motionMean(3);
        MatrixXd motionCov(3,3);
        double wSum = 0;
        samples.reserve(numOfparticles);
        weights.reserve(numOfparticles);
        for(auto& i : particles) {
            i.Sampling();
            i.WeightingWithOutKalma();
            samples.push_back(i.GetFullPose());
            weights.push_back(i.GetWeight());
            wSum +=i.GetWeight();
        }
        scanMean<<0,0,0;
        motionMean<<0,0,0;
        for(int i = 0; i < numOfparticles; i++) {
            scanMean += samples[i]*weights[i];
            motionMean += samples[i];
        }
        if(wSum != 0) {
            scanMean *= 1/wSum;
            motionMean *= 1/(double)numOfparticles;
        }

        FiNorm(scanMean);
        FiNorm(motionMean);

        VectorXd delta;
        scanCov<< 0,0,0,
              0,0,0,
              0,0,0;
        motionCov << 0,0,0,
                     0,0,0,
                     0,0,0;
        for(int i = 0; i < numOfparticles; i++) {
            delta = samples[i] - scanMean;
            FiNorm(delta);
            scanCov += delta*delta.transpose()*weights[i];
            motionCov += delta*delta.transpose();
        }
        if( wSum != 0) {
            scanCov *= 1/wSum;
            motionCov *= 1/(double)numOfparticles;
        }
        MatrixXd comonCov;
        VectorXd comonMean;
        MatrixXd tempInv;
        if(scanMean(0) != 0 ) {
            tempInv = (scanCov + motionCov).inverse();
            comonCov = scanCov*tempInv*motionCov;
            comonMean = motionCov*tempInv*scanMean + scanCov*tempInv*motionMean;
            //scanMean = comonMean;
            //scanCov = comonCov;
            cout<<"mean"<<endl;
            cout<<scanMean<<endl;
            cout<<"cov"<<endl;
            cout<<scanCov<<endl;
        }
        normX_solver.setMean(scanMean);
        normX_solver.setCovar(scanCov);

        for(auto& i : particles) {
            i.SetPose(normX_solver.samples(1));
        }

    }

    void Weighting() {
        double max;
        VectorXd maxPose;
        double sum;
        VectorXd debg(3);
        best<<0,0;
        bestW = -1;
        bestpose<<0,0,0;
        bestp = 1000000;
        double neff;
        double f;
        for(auto& i : particles) {
            i.Weighting();
            f = i.GetWeight();
            i.SetWeight(pow(f,5));
            sum += i.GetWeight();
            if(i.GetWeight() > max) {
                max = i.GetWeight();
                maxPose = i.GetPose();
            }
        }
        for(auto& i : particles) {
            neff += pow(i.GetWeight()/sum,2);
            i.SetWeight(i.GetWeight()/sum);
            debg<<i.GetPose()(0),i.GetPose()(1),i.GetWeight();
            path.push_back(debg);
        }
        cout<<"neff"<<1/(neff*numOfparticles)<<endl;
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

