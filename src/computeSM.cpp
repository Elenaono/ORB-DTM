//
// Created by lu on 19-4-17.
//

#include "computeSM.h"
#include <math.h>

const double PI = 3.1415926;
const double Pa = 0.5;
const double Ks = PI/2;
const double Ns = 3;

//void computesimilarityMatrix(const Delaunay<float > & net1,const Delaunay<float> & net2,double *similarityMatrix)
double computesimilarityMatrix(const Delaunay<float > & net1,const Delaunay<float> & net2)
//void computesimilarityMatrix(const Delaunay<float > & net1,const Delaunay<float> & net2)
{
    double similarityMatrix=0;
    std::vector<Triangle<float> > triangleList1(net1.getTriangles());
    std::vector<Triangle<float> > triangleList2(net2.getTriangles());
//    triangleList1 = net1.getTriangles();
//    triangleList2 = net2.getTriangles();

    std::cerr << "in function of computeSM" << std::endl;

//    std::cerr << "triangleList1.size(): "  << triangleList1.size()<<std::endl;
//    std::cerr << "triangleList2.size(): "  << triangleList2.size()<<std::endl;


    for (int i = 0; i < triangleList1.size(); i++)
    {
        for (int j = 0; j < triangleList2.size(); j++)
        {
            double a = similarityValue(triangleList1[i],triangleList2[j]);
//            std::cerr << "count ++" <<std::endl;
            std::cerr << "a="<< a <<std::endl;
            similarityMatrix = (a < 0.75) ? 0 : a;
        }
    }

    std::cout << "\n" << std::endl;

    return similarityMatrix;
}

double similarityValue(Triangle<float> t1,Triangle<float> t2)
{
    double q[3];
    double k[3];
    double dx[3];
    double u[3];
    t1.computeAngle();
    t2.computeAngle();
    for (int i = 0; i < 3; i++)
    {
//        std::cout << "angle[i] :" <<  t1.angle[i]  << std::endl;
        q[i] = (t1.angle[i] * Pa) / 3;
//        std::cout << "q[i]:" << q[i]  << std::endl;

        k[i] = 1 / (2 * q[i] * q[i]);
//        std::cout << "k[i]:" << k[i]  << std::endl;

//        std::cout << "t1.angle[i]:" << t1.angle[i]  << std::endl;
//        std::cout << "t2.angle[i]:" << t2.angle[i]  << std::endl;

        dx[i]= exp(-1 * k[i] * (t2.angle[i] - t1.angle[i]) * (t2.angle[i] - t1.angle[i]));
//        std::cout << "dx[i]:" << dx[i]  << std::endl;

        u[i] = pow(cos(Ks * (1 - dx[i])), Ns);
//        std::cout << "u[i]:" << u[i]  << std::endl;


//        std::cout << "q[i] :" <<  q[i]  << std::endl;
//        double temp=0;
//        temp = (Ks * (1 - dx[i]) ) ;
//        std::cout << "cos:" <<  temp << std::endl;
//        std::cout << "u[i]:" << u[i]  << std::endl;
    }

//    std::cout << "simiValue:" << (u[0]+u[1]+u[2]) / 3  << std::endl;

    return (u[0]+u[1]+u[2]) / 3;
}
