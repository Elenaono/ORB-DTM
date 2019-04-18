//
// Created by lu on 19-4-17.
//

#ifndef ORB_DTM_CALCULATOR_H
#define ORB_DTM_CALCULATOR_H

#include "vector2.h"
#include "edge.h"
#include "triangle.h"
#include "delaunay.h"

#include <vector>
#include <algorithm>

template <class T>
class Calculator
{
public:
    using TriangleType = Triangle<T>;
    using EdgeType = Edge<T>;
    using VertexType = Vector2<T>;

    void computesimilarityMatrix(const TriangleType & net1,const TriangleType & net2,double *similarityMatrix)
    {
//        n1.
    }


public:
    std::vector<TriangleType> n1;
    std::vector<TriangleType> n2;
    double similarityMatrix;

};



#endif //ORB_DTM_CALCULATOR_H
