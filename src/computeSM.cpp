//
// Created by lu on 19-4-17.
//

#include "computeSM.h"
#include <cmath>
//#include <Eigen/Core>
//#include <Eigen/Dense>

const double PI = 3.1415926;
const double Pa = 0.5;
const double Ks = PI/2;
const double Ns = 3;

void ComputeSimilarityMatrix(const Delaunay<float > & net1, const Delaunay<float> & net2, Eigen::MatrixXd &similarityMatrix)
{
    std::vector<Triangle<float> > triangleList1(net1.GetTriangles());
    std::vector<Triangle<float> > triangleList2(net2.GetTriangles());

    for (int i = 0; i < triangleList1.size(); i++)
    {
        for (int j = 0; j < triangleList2.size(); j++)
        {
            double a = SimilarityValue(triangleList1[i], triangleList2[j]);
//            std::cout << "a="<< a <<std::endl;
            similarityMatrix(i,j) = (a < 0.75) ? 0 : a; //相似度阈值为0.75
        }
    }
}

double SimilarityValue(Triangle<float> t1, Triangle<float> t2)
{
    double q[3];
    double k[3];
    double dx[3];
    double u[3];
    t1.ComputeAngle();
    t2.ComputeAngle();
    for (int i = 0; i < 3; i++)
    {
//        std::cout << "angle[i] :" <<  t1.angle[i]  << std::endl;
        q[i] = (t1.angle[i] * Pa) / 3;
//        std::cout << "q[i]:" << q[i]  << std::endl;

        k[i] = 1 / (2 * q[i] * q[i]);
//        std::cout << "k[i]:" << k[i]  << std::endl;

        dx[i]= exp(-1 * k[i] * (t2.angle[i] - t1.angle[i]) * (t2.angle[i] - t1.angle[i]));
//        std::cout << "dx[i]:" << dx[i]  << std::endl;

        u[i] = pow(cos(Ks * (1 - dx[i])), Ns);
//        std::cout << "u[i]:" << u[i]  << std::endl;
    }

    return (u[0]+u[1]+u[2]) / 3;
}
