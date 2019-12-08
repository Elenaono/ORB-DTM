//
// Created by elena on 2019/12/8.
//
#include "iostream"
#include"opencv2/core.hpp"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

#ifndef ORB_DTM_DATAIMG_H
#define ORB_DTM_DATAIMG_H

#endif //ORB_DTM_DATAIMG_H
string file1 = "./data/desk1.png";
string file2 = "./data/desk2.png";
cv::Mat img1 = cv::imread(file1, 0);
cv::Mat img2 = cv::imread(file2, 0);