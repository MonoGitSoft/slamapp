#include <QCoreApplication>
#include <iostream>
#include <featurebase.h>

using namespace std;

int main(int argc, char *argv[])
{
    Feature b;
    Feature c(3);
    VectorXd d2(2);
    VectorXd d3(3);
    d2<< 1 , 2;
    d3<< 2 , 2 , 2;
    MatrixXd cov2(2,2);
    MatrixXd cov3(3,3);
    cov2<< 1, 0, 0, 1;
    cov3<< 1 , 0 , 0 , 0 , 1 , 0, 0 , 0 , 1;
    b.SetPose(d2);
    c.SetPose(d3);
    b.SetCovMatrix(cov2);
    c.SetCovMatrix(cov3);
    QCoreApplication a(argc, argv);
    cout<<b<<endl<<c;
    return a.exec();
}
