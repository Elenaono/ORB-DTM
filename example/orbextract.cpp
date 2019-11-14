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
#include "include/Vertex.h"
#include "include/triangle.h"
#include "include/delaunay.h"
#include "include/DTMunit.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

#define d_max_value 50      // 暴力匹配的阈值
#define m_max_value 5       // DTM边矩阵相似度阈值

#define d_ransac_value 80
#define threshold_value 15
/**
 * @brief DTM
 * 1.分别对两幅图像进行特征提取；
 * 2.进行特征匹配；
 * 3.对第一次匹配的good_matches进行构建DT网络；
 */

/// 主函数
int main()
{

//    struct timespec time1 = {0, 0};       // 用于计时
//    struct timespec time2 = {0, 0};
    string file1 = "./data/desk1.png";
    string file2 = "./data/desk2.png";

//    string file1 = "./data/flag1.png";
//    string file2 = "./data/flag2.png";
    /**************** 配置信息 ******************/
    int nFeatures =1000;        // 特征点数量
    float fScaleFactor =1.2;    // 图像金字塔的缩放尺度
    int nLevels =8;             // 金字塔层数
    int fIniThFAST =18;         // 提取FAST角点的阈值  两个阈值进行选择
    int fMinThFAST =8;

    int level = 0;      // 特定层数得到的源图像

    cout << "显示特征提取的基本信息：" << endl;
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
    auto *orb1 = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    (*orb1)(first_image,cv::Mat(),mvKeys1,mDescriptors1);

    mvImageShow1 = orb1->GetImagePyramid();   //获取图像金字塔

    mnFeaturesPerLevel1 = orb1->GetmnFeaturesPerLevel();  //获取每层金字塔的特征点数量

    mvvKeypoints1 = orb1->GetmvvKeypoints();
    mvKeys1 = mvvKeypoints1[level];
    mDes1 = mDescriptors1.rowRange(0,mnFeaturesPerLevel1[level]).clone();

//    cout <<"\t\tKeyPoints:"<<mnFeaturesPerLevel1[level]<<endl;
    cv::drawKeypoints(mvImageShow1[level], mvKeys1, feature1, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);//DEFAULT  DRAW_OVER_OUTIMG     DRAW_RICH_KEYPOINTS

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

    mvImageShow2 = orb2->GetImagePyramid();   //获取图像金字塔

    mnFeaturesPerLevel2 = orb2->GetmnFeaturesPerLevel();  //获取每层金字塔的特征点数量

    mvvKeypoints2 = orb2->GetmvvKeypoints();
    mvKeys2 = mvvKeypoints2[level];
    mDes2 = mDescriptors2.rowRange(0,mnFeaturesPerLevel2[level]).clone();

//    cout <<"\t\tKeyPoints:"<<mnFeaturesPerLevel2[level]<<endl;
    cv::drawKeypoints(mvImageShow2[level], mvKeys2, feature2, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);//DEFAULT  DRAW_OVER_OUTIMG     DRAW_RICH_KEYPOINTS

    /***************   克隆图片   ******************/
    Mat debugOne   = feature1.clone();
    Mat debugTwo   = feature2.clone();
    /***************   特征匹配   *************/
    vector<DMatch> good_matches( BFmatchFunc(mDes1,mDes2,d_max_value) );
    /***************  构建DT网络  ******************************/
    vector<DMatch> new_matches(ComputeDTMunit(m_max_value, good_matches, mvKeys1, mvKeys2, debugOne, debugTwo) );   //5
    cout <<"size one:\t" << new_matches.size() << endl;
    /***************  RANSAC 实验对照组  ******************************/
    cout << "\n采用RANSAC作为control group的实验结果：" << endl;
//    clock_gettime(CLOCK_REALTIME, &time1);
    vector<DMatch> control_matches( BFmatchFunc(mDes1,mDes2,d_ransac_value) );
    UsingRansac(threshold_value,feature1,feature2,mvKeys1,mvKeys2,control_matches);
//    clock_gettime(CLOCK_REALTIME, &time2);
//    cout << "time passed is: " << (time2.tv_sec - time1.tv_sec)*1000 + (time2.tv_nsec - time1.tv_nsec)/1000000 << "ms" << endl;
    /****************************************/
    cout << "\nmain end, see you...";
    return 0;
}