#ifndef SIMFEATUREBASE_H
#define SIMFEATUREBASE_H

#include <featurebase.h>
#include <simfeaturebase.h>
#include <stdlib.h>

int NumOfFeatures = 100;


class SimFeatureBase: public FeatureBase {
    vector<Feature> features;
    const int mapR;
    double sensorR;
public:
    SimFeatureBase(double mapR, double sensorR): mapR(mapR), sensorR(sensorR) {
        features.reserve(NumOfFeatures);
        double x,y;
        VectorXd fpose(2);
        MatrixXd fcov(2,2);
        fcov<< 80, 0 , 0 , 80;
        for(int i = 0; i < NumOfFeatures; i++) {
            x = 3*mapR*(double)(rand() % (1000))*0.001 - 1.5*mapR;
            y = 3*mapR*(double)(rand() % (1000))*0.001 - 2.5*mapR;
            fpose << x , y;
            features.push_back(Feature(2,fpose, fcov));
        }
    }
    void SaveMap() {
        ofstream saveMap;
        saveMap.open ("map.m");
        saveMap<<"# name: mappose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<NumOfFeatures*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : features) {
            saveMap<<i.GetPose()<<endl;
        }
        saveMap<<"# name: mapcov"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<NumOfFeatures*2<<endl
                 <<"# columns: 2"<<endl;
        for(auto i : features) {
            saveMap<<i.GetCovMatrix()<<endl;
        }
    }

};


#endif // SIMFEATUREBASE_H
