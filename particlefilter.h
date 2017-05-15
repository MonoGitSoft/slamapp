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

int presample = 50;

MatrixXd tempcov(3,3);
VectorXd tempmean(3);


bool firstpre = true;

class FeatureParticle {
    VectorXd pose;
    double weight;
    double absWeight;
    FeatureBase& feature;
    double treshold;
    static Eigen::EigenMultivariateNormal<double> normX_solver;
    DifferencialRobotSim robot;
public:
    list<Feature> features;
    vector<VectorXd> particlePath;
    FeatureParticle(int dim,FeatureBase& feature):pose(dim),features(), weight(0.01),
        treshold(0.000001), feature(feature), robot(), particlePath(),absWeight(0) {
        particlePath.reserve(1000);
    }

    FeatureParticle(const FeatureParticle& other): pose(other.pose),features(other.features), weight(other.weight),treshold(0.000001),
    feature(other.feature), robot(other.robot), particlePath(other.particlePath), absWeight(other.absWeight)
    {

    }

    double GetAbsWeight() {
        return absWeight;
    }

    void AddPose(VectorXd newpose) {
        pose += newpose;
        FiNorm(pose);
    }

    void SetPose(VectorXd newpose) {
        pose = newpose;
        FiNorm(pose);
        robot.SetPose(pose);
        VectorXd temp(2);
        temp<<pose(0),pose(1);
        particlePath.push_back(temp);
    }

    double ScanMatching(VectorXd samplePose) {
        list<Feature>::iterator nearest;
        double w = 0.000000001;
        VectorXd temp;
        temp = pose - robot.GetRealPose();
        FiNorm(temp);
        if( temp.norm() < bestp) {
            bestp = temp.norm();
            bestpose = pose;
        }
        if(features.empty()) {
            return w;
        }
        for(auto i : feature.GetTempFeatureBuffer()) {
            nearest = Nearest(i);
            w += Weight(nearest,i);
        }
        return w;
    }

    int SizeOfMap(){
        return features.size();
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
        FiNorm(pose);
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

    void FiNormFeature(VectorXd& vec) {
        if(vec(0) > M_PI) {
            vec(0) = -(2*M_PI - vec(0));
        }
        if(vec(0) < -M_PI) {
            vec(0) = 2*M_PI + vec(0);
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
        VectorXd tempF;
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

            if( (abs(delta(0)) > sqrt(Q(0,0))*4) || (abs(delta(1)) > sqrt(Q(1,1))*4)  ) {
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
                tempF = nearest->GetPose() + K*delta;
                FiNormFeature(tempF);
                nearest->SetPose(tempF);
                nearest->SetCovMatrix( (MatrixXd::Identity(featureSize,featureSize) - K*H)*nearest->GetCovMatrix() );
                weight += w;
                absWeight += w;
            }
        }
    }

    void WeightingWithOutKalma() { //Weighting()
        weight = 0.00000001; //kináozm elejen mert most igyű
        list<Feature>::iterator nearest;
        double w;
        VectorXd temp;
        temp = pose - robot.GetRealPose();
        FiNorm(temp);
        if( temp.norm() < bestp) {
            bestp = temp.norm();
            bestpose = pose;
        }
        if(features.empty()) {
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
   // list<FeatureParticle>::iterator bestFit;
    VectorXd currentPose;
    VectorXd previousPose;
    VectorXd deltaPose;
    MatrixXd motionCov;
    int numOfparticles;
    Eigen::EigenMultivariateNormal<double> normX_solver;
    vector<VectorXd> path;
    vector<VectorXd> afterPath;
    vector<double> mapSize;
    double Neff;
    vector<double> error;
    vector<double> deaderror;

public:
    ParticleFilter(MotionModell &robot, Enviroment &enviroment,int numOfparticles): Slam(robot,enviroment),feature(dynamic_cast<FeatureBase&>(enviroment)), particles(), numOfparticles(numOfparticles)
    ,currentPose(), previousPose(), motionCov(), deltaPose(), path(),afterPath(), normX_solver(a,c), Neff(), mapSize(), deaderror(), error() {
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
        mapSize.reserve(5000);
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

    list<FeatureParticle>::iterator BestParticle(){
        list<FeatureParticle>::iterator bestFit;
        list<FeatureParticle>::iterator iter;
        double best = -1;
        for(iter = particles.begin(); iter != particles.end(); ++iter) {
            if(iter->GetAbsWeight() > best) {
                best = iter->GetAbsWeight();
                bestFit = iter;
            }
        }
        return bestFit;
    }

    void Save() {
        ofstream savePoses;
        list<FeatureParticle>::iterator bestFit;
        bestFit = BestParticle();
        savePoses.open ("particlepose.m");
        savePoses<<"# name: num"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<1<<endl
                 <<"# columns: 1"<<endl;
                 savePoses<<numOfparticles<<endl;

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

        savePoses<<"# name: best"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<bestFit->particlePath.size()*2<<endl
                 <<"# columns: 1"<<endl;
        //FeatureParticle a(*bestFit);
        for(auto& i : bestFit->particlePath) {
            savePoses<<i<<endl;
        }

        savePoses<<"# name: particlemap"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<bestFit->features.size()*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto& i : bestFit->features) {
            savePoses<<i.GetPose()<<endl;
        }


        savePoses<<"# name: perror"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<error.size()<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : error) {
            savePoses<<i<<endl;
        }

        savePoses<<"# name: derror"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<deaderror.size()<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : deaderror) {
            savePoses<<i<<endl;
        }
    }

    void Sampling() {
        robot.DeadReckoningPose();
        feature.FeatureExtraction();
        vector<VectorXd> samples;
        vector<double> weights;
        VectorXd scanMean(2);
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
        scanMean<<0,0;
        motionMean<<0,0,0;
        VectorXd temp(2);
        VectorXd alfaMean(2);
        alfaMean<<0,0;
        temp<<0,0;
        for(int i = 0; i < numOfparticles; i++) {
            temp << samples[i](0), samples[i](1);
            scanMean += temp*weights[i];
            temp <<  cos(samples[i](2)), sin(samples[i](2));
            alfaMean += temp*weights[i];
            motionMean += samples[i];
        }
        if(wSum != 0) {
            scanMean *= 1/wSum;
            alfaMean *= 1/wSum;
            motionMean *= 1/(double)numOfparticles;
        }
        double alfa;
        alfa = atan2(alfaMean(1),alfaMean(0));
        FiNorm(motionMean);

        VectorXd delta;
        scanCov<< 0,0,0,
              0,0,0,
              0,0,0;
        motionCov << 0,0,0,
                     0,0,0,
                     0,0,0;
        VectorXd AllMean(3);
        AllMean<<0,0,0;
        for(int i = 0; i < numOfparticles; i++) {
            AllMean << scanMean(0),scanMean(1),alfa;
            delta = samples[i] - AllMean;
            FiNorm(delta);
            scanCov += delta*delta.transpose()*weights[i];
            motionCov += delta*delta.transpose();
        }
        if( wSum != 0) {
            scanCov *= 1/wSum;
            motionCov *= 1/(double)numOfparticles;
        }

        normX_solver.setMean(AllMean);
        normX_solver.setCovar(scanCov);

        for(auto& i : particles) {
            i.SetPose(normX_solver.samples(1));
        }

    }

    void ZeroPi(VectorXd& alfa) {
        if(alfa(2) < 0) {
            alfa(2) = 2*M_PI + alfa(2);
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
         //   i.SetWeight(pow(f,2));
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
        Neff = 1/(neff*numOfparticles);
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
        double tempSize = 0;
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
            tempSize += i.SizeOfMap();
        }
        cout<<"featuires"<<endl;
        cout<<tempSize/numOfparticles<<endl;
        list<FeatureParticle>::iterator bestFit;
        bestFit = BestParticle();
        DifferencialRobotSim& realrobot(dynamic_cast<DifferencialRobotSim&>(robot));
        mapSize.push_back((bestFit->SizeOfMap()));
        error.push_back((bestFit->GetFullPose() - realrobot.GetRealPose()).norm());
        deaderror.push_back((realrobot.GetRealPose() - realrobot.GetAllDeadPose()).norm());

    }

    void Step() {}
};

#endif // PARTICLEFILTER_H

