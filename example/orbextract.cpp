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
#define d_max_vaule_new 30

// todo
//   第二次匹配效果太差
//   怀疑原因为描述子出错，因为两次的匹配距离区间反而增大了

// sort()时，自定义的排序条件
// 用于对vector对象内的指定成员进行排序

bool cmp1(const DMatch first, const DMatch second)
{
    return first.trainIdx < second.trainIdx;
}

// unique()时，自定义的去重条件
// 用于对vector对象内的指定成员进行去重
bool cmp2(const DMatch first,const DMatch second)
{
    return first.trainIdx == second.trainIdx;
}

// 主函数
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
    vector<cv::KeyPoint> mvKeys1_new;
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

    /**************** 图片二：初始化信息 *********************/
    cv::Mat second_image = cv::imread(file2, 0);    // load grayscale image 灰度图
    cv::Mat feature2;
    std::vector<cv::Mat> mvImageShow2;   //图像金字塔
    vector<cv::KeyPoint> mvKeys2;        //一维特征点
    vector<cv::KeyPoint> mvKeys2_new;
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

    /**************** 特征匹配 ******************/
    vector<DMatch> matches,good_matches,new_matches;
    BFMatcher matcher (NORM_HAMMING);
    matcher.match(mDes1,mDes2,matches);

    //计算最大与最小距离
    double min_dist = 10000,max_dist = 0;

    for (int k = 0; k < mDes1.rows; k++)
    {
        double dist = matches[k].distance;
        if(dist < min_dist)
            min_dist = dist;
        if(dist > max_dist)
            max_dist = dist;
    }

    cout << "min_dist:" << min_dist << endl;
    cout << "max_dist:" << max_dist << endl;

    int d_max =d_max_vaule;
    //筛选匹配
    temp=0;
    for (int l = 0; l < mDes1.rows; l++)
    {
        if(matches[l].distance <= d_max_vaule )
        {
            matches[l].imgIdx=temp;
            good_matches.emplace_back(matches[l]);
            temp++;
        }
    }
    temp=0;

    sort(good_matches.begin(), good_matches.end(), cmp1);
    good_matches.erase(unique(good_matches.begin(),good_matches.end(),cmp2),good_matches.end());

    /**********************  构建第一组 DT 网络  ******************************/
    ///delaunay one
    std::vector<Vector2<float> > points;

    for(const auto &g:good_matches)
    {
        points.emplace_back(Vector2<float>(mvKeys1[g.queryIdx].pt.x , mvKeys1[g.queryIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation;
    const std::vector<Triangle<float> > triangles = triangulation.triangulate(points);  //逐点插入法
    triangulation.computeEdgeMatrix();
    std::cout << triangles.size() << " triangles generated\n";
    const std::vector<Edge<float> > edges = triangulation.getEdges();
    std::cout << "Edges : " << edges.size() << std::endl;

    for(const auto &e : edges)
    {
        line(feature1, Point(e.p1.x, e.p1.y), Point(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }

    ///delaunay two
    std::vector<Vector2<float> > points2;

    for(const auto &g:good_matches)
    {
        points2.emplace_back(Vector2<float>(mvKeys2[g.trainIdx].pt.x , mvKeys2[g.trainIdx].pt.y ,g.imgIdx ));
    }

    cout << "Size of points2: " << points2.size() << endl;
    Delaunay<float> triangulation2;
    const std::vector<Triangle<float> > triangles2 = triangulation2.triangulate(points2);  //逐点插入法
    triangulation2.computeEdgeMatrix();
    std::cout << triangles2.size() << " triangles generated\n";
    const std::vector<Edge<float> > edges2 = triangulation2.getEdges();

    for(const auto &e2 : edges2)
    {
        line(feature2, Point(e2.p1.x, e2.p1.y), Point(e2.p2.x, e2.p2.y), Scalar(0, 0, 255), 1);
    }

    /**************** 显示 ******************/
    cout << "\nmatch:" << good_matches.size()<<endl;
    Mat show;
    cv::drawMatches(mvImageShow1[level],mvKeys1,mvImageShow2[level],mvKeys2,good_matches,show);
    cv::drawMatches(feature1,mvKeys1,feature2,mvKeys2,good_matches,show);
    imwrite("matches.png",show);
    imshow("matches",show);
    waitKey(0);

    /*******************  构建边矩阵，并计算相似度(范数)  *********************/
    Eigen::MatrixXd edgeMatrix = Eigen::MatrixXd::Zero(500,500);
    edgeMatrix = triangulation.getEdgeMatrix() - triangulation2.getEdgeMatrix();
    double value =0;
    value = edgeMatrix.norm();
    cout << "\nvalue: " << value <<  endl;

    //   cv::Mat中没有删除某一列或者行的函数
    //   只能构造新的Mat，在删除某一列后，将后边的复制到新的Mat当中去
    //   新的解决方案是：将Mat转换为vector，使用back() pop()等操作处理后，再转换成Mat
    //   注意：由于删除的是列，而转换成vector后操作的是行，因此可以对Mat进行转置后，再进行转换操作，即Mat.t()
    //   在循环外边完成Mat到vector的转换工作，进行循环操作并退出后，再进行转换回来

    mvKeys1_new = mvKeys1;
    mvKeys2_new = mvKeys2;
    temp = (int)(mnFeaturesPerLevel1[level]-good_matches.size());
    cv::Mat mDes1_new(temp,32,CV_8U);   /// 严格注意type  因为ORB对应的描述子是 8U，使用 32F时，会导致BFMatch出错 (吃了大亏。。。)
    cv::Mat mDes2_new(temp,32,CV_8U);
    temp = 0;

    vector<int> order1,order2;
    cout << "Size of goodmatchs:  " << good_matches.size() << endl;
    for(const auto &g:good_matches)
    {
        mvKeys1_new.erase(mvKeys1_new.begin()+g.queryIdx);
        mvKeys2_new.erase(mvKeys2_new.begin()+g.trainIdx);
        order1.emplace_back(g.queryIdx);
        order2.emplace_back(g.trainIdx);
    }
    sort(order1.begin(),order1.end());
    sort(order2.begin(),order2.end());

    int dele_temp_1=0;
    int dele_temp_2=0;
    int dele_temp_count1=0;
    int dele_temp_count2=0;
    for (int i = 0; i < mnFeaturesPerLevel1[level]-1; ++i)
    {
        if(i == *(order1.begin()+dele_temp_count1))
            dele_temp_count1++;
        else
        {
            mDes1.row(i).copyTo(mDes1_new.row(dele_temp_1));
            dele_temp_1++;
        }

        if(i == *(order2.begin()+dele_temp_count2))
            dele_temp_count2++;
        else
        {
            mDes2.row(i).copyTo(mDes2_new.row(dele_temp_2));
            dele_temp_2++;
        }

    }

    mvKeys1_new.pop_back();

    cout << "Sizes of mvKeys1_new: \t" << mvKeys1_new.size() << endl;
    cout << "Sizes of mDes1_new:\t\t" << mDes1_new.size << endl;
    cout << "Sizes of mvKeys2_new: \t" << mvKeys2_new.size() << endl;
    cout << "Sizes of mDes2_new:\t\t" << mDes2_new.size << endl;

    //  计算DT网络的边矩阵，并计算差的范数，提取外点
    //  更新keypoints，进行第二次match

    //      实现了二次匹配
    //      但是第二次效果并不好
    //      并且存在一些问题：
    //          1.修改第一次的匹配阈值后，出错；
    //          2.第二次效果太差了。。。
    /**************** 特征匹配 ******************/
    vector<DMatch> matches2,good_matches2;
    BFMatcher matcher2 (NORM_HAMMING);
    matcher2.match(mDes1_new,mDes2_new,matches2);

    //计算最大与最小距离
    for (int k = 0; k < mDes1_new.rows; k++) {
        double dist = matches2[k].distance;
        if(dist < min_dist)
            min_dist = dist;
        if(dist > max_dist)
            max_dist = dist;
    }

    cout << "\nmin_dist:" << min_dist << endl;
    cout << "max_dist:" << max_dist << endl;

    //筛选匹配
    temp=0;
    for (int l = 0; l < mDes1_new.rows; l++)
    {
        if(matches2[l].distance <= d_max_vaule_new )
        {
            matches2[l].imgIdx=temp;
            good_matches2.emplace_back(matches2[l]);
            temp++;
        }
    }
    temp=0;

    cout << "match:" << good_matches2.size()<<endl;
    Mat show2;
    cv::drawMatches(feature1,mvKeys1_new,feature2,mvKeys2_new,good_matches2,show2);
    imwrite("matches2.png",show2);
    imshow("matches2",show2);
    waitKey(0);
    /****************************************/
    cout << "finish!" << endl;
    return 0;
}