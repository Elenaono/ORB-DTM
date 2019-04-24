//
// Created by lu on 19-3-23.
//
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include <array>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include "include/ORBextractor.h"
#include "include/vector2.h"
#include "include/triangle.h"
#include "include/delaunay.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

#define d_max_vaule 40

int main()
{
    string file1 = "./data/desk1.png";
    string file2 = "./data/desk2.png";
    /**************** 配置信息 ******************/
    int nFeatures =1000;        // 1000
    float fScaleFactor =1.2;    // 1.2
    int nLevels =8;             // 8
    int fIniThFAST =20;         // 20
    int fMinThFAST =8;          // 8

    int level = 0;
    int temp=0;
    /**************** 图片一：初始化信息 *********************/
    cv::Mat first_image = cv::imread(file1, 0);    // load grayscale image 灰度图
    cv::Mat feature1;
    std::vector<cv::Mat> mvImageShow1;   //图像金字塔
    vector<cv::KeyPoint> mvKeys1;        //一维特征点
    cv::Mat mDescriptors1;               //描述子
    vector<int> mnFeaturesPerLevel1;     //金字塔每层的特征点数量
    vector<vector<cv::KeyPoint>> mvvKeypoints1;  //每层的特征点
    cv::Mat mDes1;
    /**************** 图片一：提取特征点信息 ******************/
    ORBextractor *orb1 = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    (*orb1)(first_image,cv::Mat(),mvKeys1,mDescriptors1);

    orb1->myprint("picture1");
    mvImageShow1 = orb1->GetImagePyramid();   //获取图像金字塔

    mnFeaturesPerLevel1 = orb1->GetmnFeaturesPerLevel();  //获取每层金字塔的特征点数量

    mvvKeypoints1 = orb1->GetmvvKeypoints();
    mvKeys1 = mvvKeypoints1[level];
    mDes1 = mDescriptors1.rowRange(0,mnFeaturesPerLevel1[level]).clone();

    cout <<"KeyPoints:"<<mnFeaturesPerLevel1[level]<<endl;
    cv::drawKeypoints(mvImageShow1[level], mvKeys1, feature1, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);//DEFAULT  DRAW_OVER_OUTIMG     DRAW_RICH_KEYPOINTS
    ///delaunay
//    std::vector<Vector2<float> > points;
//    temp = 0;
//    for(const auto &k :mvKeys1)
//    {
//        points.emplace_back(Vector2<float>(k.pt.x,k.pt.y,temp));
//        temp++;
//    }
//    temp = 0;
//
//    cout << "Size of points1: " << points.size() << endl;
//
//    Delaunay<float> triangulation;
//    const std::vector<Triangle<float> > triangles = triangulation.triangulate(points);  //逐点插入法
//    std::cout << triangles.size() << " triangles generated\n";
//    const std::vector<Edge<float> > edges = triangulation.getEdges();
//    std::cout << "Edges : " << edges.size() << std::endl;
//
//    for(const auto &e : edges)
//    {
//        line(feature1, Point(e.p1.x, e.p1.y), Point(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
//    }
//
//    imwrite("feature1.png",feature1);
//    imshow("feature1",feature1);
//    waitKey(0);
    /**************** 图片二：初始化信息 *********************/
    cv::Mat second_image = cv::imread(file2, 0);    // load grayscale image 灰度图
    cv::Mat feature2;
    std::vector<cv::Mat> mvImageShow2;   //图像金字塔
    vector<cv::KeyPoint> mvKeys2;        //一维特征点
    cv::Mat mDescriptors2;               //描述子
    vector<int> mnFeaturesPerLevel2;     //金字塔每层的特征点数量
    vector<vector<cv::KeyPoint>> mvvKeypoints2;  //每层的特征点
    cv::Mat mDes2;
    /**************** 图片二：提取特征点信息 ******************/
    ORBextractor *orb2 = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    (*orb2)(second_image,cv::Mat(),mvKeys2,mDescriptors2);

    orb2->myprint("picture2");
    mvImageShow2 = orb2->GetImagePyramid();   //获取图像金字塔

    mnFeaturesPerLevel2 = orb2->GetmnFeaturesPerLevel();  //获取每层金字塔的特征点数量

    mvvKeypoints2 = orb2->GetmvvKeypoints();
    mvKeys2 = mvvKeypoints2[level];
    mDes2 = mDescriptors2.rowRange(0,mnFeaturesPerLevel2[level]).clone();

    cout <<"KeyPoints:"<<mnFeaturesPerLevel2[level]<<endl;
    cv::drawKeypoints(mvImageShow2[level], mvKeys2, feature2, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);//DEFAULT  DRAW_OVER_OUTIMG     DRAW_RICH_KEYPOINTS

    ///delaunay
//    std::vector<Vector2<float> > points2;
//    temp = 0;
//    for(const auto &k2 :mvKeys2)
//    {
//        points2.emplace_back(Vector2<float>(k2.pt.x,k2.pt.y,temp));
//        temp++;
//    }
//    temp = 0;
//
//    cout << "Size of points2: " << points2.size() << endl;
//    Delaunay<float> triangulation2;
//    const std::vector<Triangle<float> > triangles2 = triangulation2.triangulate(points2);  //逐点插入法
//    std::cout << triangles2.size() << " triangles generated\n";
//    const std::vector<Edge<float> > edges2 = triangulation2.getEdges();
//
//    for(const auto &e2 : edges2)
//    {
//        line(feature2, Point(e2.p1.x, e2.p1.y), Point(e2.p2.x, e2.p2.y), Scalar(0, 0, 255), 1);
//    }


//    imwrite("feature2.png",feature2);
//    imshow("feature2",feature2);
//    waitKey(0);
    /**************** 特征匹配 ******************/
    vector<DMatch> matches,good_matches;
    BFMatcher matcher (NORM_HAMMING);
    matcher.match(mDes1,mDes2,matches);
    //计算最大与最小距离
    double min_dist = 10000,max_dist = 0;
    for (int k = 0; k < mDes1.rows; k++) {
        double dist = matches[k].distance;
        if(dist < min_dist)
            min_dist = dist;
        if(dist > max_dist)
            max_dist = dist;
    }

    cerr << "min_dist:" << min_dist << endl;
    cerr << "max_dist:" << max_dist << endl;

    int d_max =d_max_vaule;
    //筛选匹配
    temp=0;
    for (int l = 0; l < mDes1.rows; l++)
    {
//        if(matches[l].distance <= max(2*min_dist,30.0) )
        if(matches[l].distance <= d_max_vaule )
        {
            matches[l].imgIdx=temp;
            good_matches.emplace_back(matches[l]);
            temp++;
        }
    }
    temp=0;

//    cerr << "test"<< endl;
//    for(const auto &m:good_matches)
//    {
//        cout << m.queryIdx << " , " << m.trainIdx << " , " << m.imgIdx << endl;
//    }
    /**************** 显示 ******************/
    cout << "match:" << good_matches.size()<<endl;
    Mat show;
    cv::drawMatches(mvImageShow1[level],mvKeys1,mvImageShow2[level],mvKeys2,good_matches,show);
    imwrite("matches.png",show);
    imshow("matches",show);
    waitKey(0);

    cout << "finish!" << endl;
    return 0;
}