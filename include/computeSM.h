//
// Created by lu on 19-4-17.
//

#ifndef ORB_DTM_COMPUTESM_H
#define ORB_DTM_COMPUTESM_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "vector2.h"
#include "edge.h"
#include "triangle.h"
#include "delaunay.h"


// 计算相似度
double similarityValue(Triangle<float> t1,Triangle<float> t2);

// 计算相似度矩阵
//void computesimilarityMatrix(const Delaunay<float > & net1,const Delaunay<float> & net2);
void computesimilarityMatrix(const Delaunay<float > & net1,const Delaunay<float> & net2,Eigen::MatrixXd &similarityMatrix);

#endif //ORB_DTM_COMPUTESM_H
