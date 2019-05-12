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
#include "include/DTMunit.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

#define d_max_vaule 50  //35    55
#define d_max_vaule_two 60

/**
 * @brief DTM
 * 1.分别对两幅图像进行特征提取；
 * 2.进行第一次特征匹配；
 * 3.对第一次匹配的good_matches进行构建DT网络；
 * 4.对剩余的KeyPoints进行第二次特征匹配（更高阈值）；
 *
 * 在3.-4.之间，应进行DTM的优化，可能包括：（待完成）
 *      1.计算边矩阵，进行外点筛选，不断剔除外点、计算新的边矩阵，迭代；
 *      2.根据三角形相似函数，计算相似度，来剔除三角形/保留最优三角形；
 *
 */

// 主函数
int main()
{
//    string file1 = "./data/desk1.png";
//    string file2 = "./data/desk2.png";
    string file1 = "./data/test5.png";
    string file2 = "./data/test6.png";
    /**************** 配置信息 ******************/
    int nFeatures =1000;        // 1000
    float fScaleFactor =1.2;    // 1.2
    int nLevels =8;             // 8
    int fIniThFAST =20;         // 20
    int fMinThFAST =8;          // 8

    int level = 0;
    int temp=0;

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
    ORBextractor *orb1 = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    (*orb1)(first_image,cv::Mat(),mvKeys1,mDescriptors1);

    orb1->myprint("\tpicture1");
    mvImageShow1 = orb1->GetImagePyramid();   //获取图像金字塔

    mnFeaturesPerLevel1 = orb1->GetmnFeaturesPerLevel();  //获取每层金字塔的特征点数量

    mvvKeypoints1 = orb1->GetmvvKeypoints();
    mvKeys1 = mvvKeypoints1[level];
    mDes1 = mDescriptors1.rowRange(0,mnFeaturesPerLevel1[level]).clone();

    cout <<"\t\tKeyPoints:"<<mnFeaturesPerLevel1[level]<<endl;
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

    orb2->myprint("\tpicture2");
    mvImageShow2 = orb2->GetImagePyramid();   //获取图像金字塔

    mnFeaturesPerLevel2 = orb2->GetmnFeaturesPerLevel();  //获取每层金字塔的特征点数量

    mvvKeypoints2 = orb2->GetmvvKeypoints();
    mvKeys2 = mvvKeypoints2[level];
    mDes2 = mDescriptors2.rowRange(0,mnFeaturesPerLevel2[level]).clone();

    cout <<"\t\tKeyPoints:"<<mnFeaturesPerLevel2[level]<<endl;
    cv::drawKeypoints(mvImageShow2[level], mvKeys2, feature2, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);//DEFAULT  DRAW_OVER_OUTIMG     DRAW_RICH_KEYPOINTS

    /***************  克隆图片  ******************/
    Mat debugOne   = feature1.clone();
    Mat debugTwo   = feature2.clone();
    Mat debugThree = feature1.clone();
    Mat debugFour  = feature2.clone();
    /***************  第一次特征匹配      *************/
    vector<DMatch> good_matches( BFmatchFunc(mDes1,mDes2,d_max_vaule) );
    /***************  构建第一组 DT 网络  ******************************/
    vector<DMatch> new_matches( computeDTMunit(good_matches,mvKeys1,mvKeys2,debugOne,debugTwo) );


    /***************  剔除 good_matchs 中的点  *********************/
    vector<cv::KeyPoint> mvKeys1_new,mvKeys2_new;
    temp = (int)(mnFeaturesPerLevel1[level]-new_matches.size());
    cv::Mat mDes1_new(temp,32,CV_8U);   // 严格注意type  因为ORB对应的描述子是 8U，使用 32F时，会导致BFMatch出错 (吃了大亏。。。)
    cv::Mat mDes2_new(temp,32,CV_8U);

    updateKey( mnFeaturesPerLevel1[level],new_matches,mvKeys1,mvKeys2,mDes1,mDes2,mvKeys1_new,mvKeys2_new,mDes1_new,mDes2_new );
    /***************  第二次特征匹配 ******************/
    vector<DMatch> good_matches2( BFmatchFunc(mDes1_new,mDes2_new,d_max_vaule_two) );
    /***************  构建第二组 DT 网络  ******************************/
    vector<DMatch> new_matche2( computeDTMunit(good_matches2,mvKeys1_new,mvKeys2_new,debugThree,debugFour) );


    /****************************************/
    cout << "finish!" << endl;
    return 0;
}