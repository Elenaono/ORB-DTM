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

#define d_max_vaule 35  //35
#define d_max_vaule_new 50

#define m_max_value 5

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


//   第二次匹配效果太差
//   怀疑原因为描述子出错，因为两次的匹配距离区间反而增大了
//   原因：keypoints的order变了，导致了描述子不匹配!!!

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

    cout << "显示特征提取的基本信息：" << endl;
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
    vector<cv::KeyPoint> mvKeys2_new;
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

    /**************** 特征匹配 ******************/
    cout << "\n显示第一次特征匹配的基本信息：" << endl;
    vector<DMatch> matches,matches_temp,good_matches,new_matches;
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

    cout << "\tmin_dist:" << min_dist << endl;
    cout << "\tmax_dist:" << max_dist << endl;

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


//    cout << "排序筛选前" << endl;
//    for(const auto &p:good_matches)
//    {
//        cout << p.queryIdx << "\t,\t" << p.trainIdx << "\t,\t" << p.imgIdx << endl;
//    }

    sort(good_matches.begin(), good_matches.end(), cmp1);   //排序
    good_matches.erase(unique(good_matches.begin(),good_matches.end(),cmp2),good_matches.end());    //去重

//    cout << "排序筛选后" << endl;
//    for(const auto &p:good_matches)
//    {
//        cout << p.queryIdx << "\t,\t" << p.trainIdx << "\t,\t" << p.imgIdx << endl;
//    }

    for(int i =0 ;i < good_matches.size();i++)
    {
        good_matches[i].imgIdx = i;
    }

//    cout << "排序纠正后" << endl;
//    for(const auto &p:good_matches)
//    {
//        cout << p.queryIdx << "\t,\t" << p.trainIdx << "\t,\t" << p.imgIdx << endl;
//    }

    Mat feature3 = feature1.clone();
    Mat feature4 = feature2.clone();
    Mat feature5 = feature1.clone();
    Mat feature6 = feature2.clone();
    Mat feature7 = feature1.clone();
    Mat feature8 = feature2.clone();
    /**********************  构建第一组 DT 网络  ******************************/
    cout << "\n构建DT网络：" << endl;
    ///delaunay one
    cout << "\tDT one:" << endl;
    std::vector<Vector2<float> > points;
    for(const auto &g:good_matches)
    {
        points.emplace_back(Vector2<float>(mvKeys1[g.queryIdx].pt.x , mvKeys1[g.queryIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation;
    const std::vector<Triangle<float> > triangles = triangulation.triangulate(points);  //逐点插入法
    triangulation.computeEdgeMatrix();
    std::cout << "\t\t" <<triangles.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges = triangulation.getEdges();

    for(const auto &e : edges)
    {
//        cout << e.p2.index << "," << e.p1.index << endl;
        line(feature1, Point(e.p1.x, e.p1.y), Point(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
//        imshow("feature1",feature1);
//        waitKey(0);
    }

    ///delaunay two
    cout << "\tDT two:" << endl;
    std::vector<Vector2<float> > points2;
    for(const auto &g:good_matches)
    {
        points2.emplace_back(Vector2<float>(mvKeys2[g.trainIdx].pt.x , mvKeys2[g.trainIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation2;
    const std::vector<Triangle<float> > triangles2 = triangulation2.triangulate(points2);  //逐点插入法
    triangulation2.computeEdgeMatrix();
    std::cout << "\t\t" << triangles2.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges2 = triangulation2.getEdges();

    for(const auto &e2 : edges2)
    {
        line(feature2, Point(e2.p1.x, e2.p1.y), Point(e2.p2.x, e2.p2.y), Scalar(0, 0, 255), 1);
    }

    cout << "\nSize of points2:\t" << points2.size() << endl;
    for(const auto &p:points2)     //  显示点
    {
//        cout << p << endl;
    }

    cout << "\nSize of edges2:\t" << edges2.size() << endl;
    for(const auto &p:edges2)     //  显示边
    {
//        cout << p << endl;
    }
    /**************** 显示 ******************/
    cout << "\t匹配:" << endl;
    cout << "\t\tmatch:" << good_matches.size()<<endl;
    Mat show;
    cv::drawMatches(mvImageShow1[level],mvKeys1,mvImageShow2[level],mvKeys2,good_matches,show);
    cv::drawMatches(feature1,mvKeys1,feature2,mvKeys2,good_matches,show);
    imwrite("matches.png",show);
    imshow("matches",show);
    waitKey(0);

    /*******************  构建边矩阵，并计算相似度(范数)  *********************/
    cout << "\n计算DTM的相关信息：" << endl;
    Eigen::MatrixXd::Index maxRow,maxCol;
    Eigen::MatrixXd edgeMatrix = Eigen::MatrixXd::Zero(50,50);  //computeEdgeMatrix() 在此处也修改了 20,20 ，需要同步修改，后期改进此处
    edgeMatrix = triangulation.getEdgeMatrix() - triangulation2.getEdgeMatrix();
//    cout <<  "DT1的边矩阵：\n"  << triangulation.getEdgeMatrix().row(4)  << endl;
//    cout <<  "DT2的边矩阵：\n"  << triangulation2.getEdgeMatrix() << endl; //.row(4)
    double value =0;
    value = edgeMatrix.norm();
    cout << "\tvalue: " << value <<  endl;


    edgeMatrix.cwiseAbs().colwise().sum().maxCoeff(&maxRow,&maxCol);    // 边矩阵.绝对值.列.和.最大值(行序号,列序号)

    cout <<  "提取候选外点：\t"  << maxCol << endl;
    cout << "显示sum:\n" << edgeMatrix.cwiseAbs().colwise().sum() << endl;

    cout << "计算列和：\n" << edgeMatrix.cwiseAbs().colwise().sum()<< endl;

    cout <<"显示边矩阵之差：\n"<< edgeMatrix.cwiseAbs().col(maxCol).transpose() << endl;

    cout << "二者之差：\n" << edgeMatrix.cwiseAbs().colwise().sum() - edgeMatrix.cwiseAbs().col(maxCol).transpose()<< endl;

    cout << "候选外点：" << mvKeys2[good_matches[maxCol].trainIdx].pt << endl;


    cout << "\nold size:\t" << good_matches.size()<<endl;

    for(int i = 50;i != 0 ;i--)
    {
        if((edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) >= m_max_value )
        {
            cout << (edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) << "\t,\t" << mvKeys1[good_matches[i-1].queryIdx].pt <<"\t,\t" << mvKeys2[good_matches[i-1].trainIdx].pt << endl;
            good_matches.erase(good_matches.begin()+i-1);
        }
    }

    cout << "new size:\t" << good_matches.size()<<endl;


    ///delaunay three
    cout << "\tDT three:" << endl;
    std::vector<Vector2<float> > points3;
    for(const auto &g:good_matches)
    {
        points3.emplace_back(Vector2<float>(mvKeys1[g.queryIdx].pt.x , mvKeys1[g.queryIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation3;
    const std::vector<Triangle<float> > triangles3 = triangulation3.triangulate(points3);  //逐点插入法
    triangulation3.computeEdgeMatrix();
    std::cout << "\t\t" << triangles3.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges3 = triangulation3.getEdges();

    Mat _resultImg = second_image.clone();
//    Mat resultImg_one = Mat(_resultImg.rows,_resultImg.cols,CV_8UC3);
//    cvtColor(_resultImg,resultImg_one,CV_GRAY2BGR);

    for(const auto &e3 : edges3)
    {
        line(feature3, Point(e3.p1.x, e3.p1.y), Point(e3.p2.x, e3.p2.y), Scalar(0, 0, 255), 1);
    }

    imshow("feature3",feature3);
    waitKey(0);

    ///delaunay four
    cout << "\tDT four:" << endl;
    std::vector<Vector2<float> > points4;
    for(const auto &g:good_matches)
    {
        points4.emplace_back(Vector2<float>(mvKeys2[g.trainIdx].pt.x , mvKeys2[g.trainIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation4;
    const std::vector<Triangle<float> > triangles4 = triangulation4.triangulate(points4);  //逐点插入法
    triangulation4.computeEdgeMatrix();
    std::cout << "\t\t" << triangles4.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges4 = triangulation4.getEdges();

//    Mat _resultImg2 = second_image.clone();
//    Mat resultImg_one = Mat(_resultImg2.rows,_resultImg2.cols,CV_8UC3);
//    cvtColor(_resultImg2,resultImg_one,CV_GRAY2BGR);

    for(const auto &e4 : edges4)
    {
        line(feature4, Point(e4.p1.x, e4.p1.y), Point(e4.p2.x, e4.p2.y), Scalar(0, 0, 255), 1);
    }

    imshow("feature4",feature4);
    waitKey(0);

    /****************************************/
//    cout << "\nfinish!" << endl;
//    return 0;



















//    Eigen::MatrixXd::Index maxRow,maxCol;
//    Eigen::Matrix<double, 3, 3> A;
//    A << 1, 2, 3,
//            4, 5, 6,
//            7, 8, 9;
//    cout << "显示矩阵元素：\n" << A << endl;
//    cout << "测试：\n" << A.colwise().sum().maxCoeff(&maxRow,&maxCol) << endl;
//    cout << maxRow << "," << maxCol << endl;



    /*******************  剔除 good_matchs 中的点，对剩余点集进行二次匹配   *********************/
    //   cv::Mat中没有删除某一列或者行的函数
    //   只能构造新的Mat，在删除某一列后，将后边的复制到新的Mat当中去
    //   新的解决方案是：将Mat转换为vector，使用back() pop()等操作处理后，再转换成Mat
    //   注意：由于删除的是列，而转换成vector后操作的是行，因此可以对Mat进行转置后，再进行转换操作，即Mat.t()
    //   在循环外边完成Mat到vector的转换工作，进行循环操作并退出后，再进行转换回来

    //    cout << "\n原始特征点：\n" << endl;
    //    for(const auto &p:mvKeys1)
    //    {
    //        cout << p.pt << endl;
    //    }
    //    cout << "\n原始描述子：\n" << endl;
    //    cout << mDes1 << endl;

    temp = (int)(mnFeaturesPerLevel1[level]-good_matches.size());
    cv::Mat mDes1_new(temp,32,CV_8U);   /// 严格注意type  因为ORB对应的描述子是 8U，使用 32F时，会导致BFMatch出错 (吃了大亏。。。)
    cv::Mat mDes2_new(temp,32,CV_8U);
    temp = 0;

    vector<int> order1,order2;
    cout << "Size of goodmatchs:  " << good_matches.size() << endl;
    // 更新特征点
    for(const auto &g:good_matches)
    {
        order1.emplace_back(g.queryIdx);
        order2.emplace_back(g.trainIdx);
    }
    sort(order1.begin(),order1.end());
    sort(order2.begin(),order2.end());

//    cout << "\nsize of order1: " << order1.size() << endl;
//    for(const auto &p:order1)
//    {
//        cout << p << endl;
//    }
//
//    cout << "\nsize of order2: " << order2.size() << endl;
//    for(const auto &p:order2)
//    {
//        cout << p << endl;
//    }

//    copy(mvKeys1.begin(),mvKeys1.begin(),mvKeys1_new.begin());
//    cout << "Sizes of mvKeys1_new: \t" << mvKeys1_new.size() << endl;
    // 更新描述子
    int dele_temp_1=0;
    int dele_temp_2=0;
    int dele_temp_count1=0;
    int dele_temp_count2=0;
    for (int i = 0; i < mnFeaturesPerLevel1[level]; ++i)
    {
        if(i == *(order1.begin()+dele_temp_count1))
            dele_temp_count1++;
        else
        {
            mvKeys1_new.insert(mvKeys1_new.end(),mvKeys1.begin()+i,mvKeys1.begin()+i+1);
            mDes1.row(i).copyTo(mDes1_new.row(dele_temp_1));
            dele_temp_1++;
        }

        if(i == *(order2.begin()+dele_temp_count2))
            dele_temp_count2++;
        else
        {
            mvKeys2_new.insert(mvKeys2_new.begin()+dele_temp_2,mvKeys2.begin()+i,mvKeys2.begin()+i+1);
            mDes2.row(i).copyTo(mDes2_new.row(dele_temp_2));
            dele_temp_2++;
        }

    }
    cout << "Sizes of mvKeys1_new: \t" << mvKeys1_new.size() << endl;
    cout << "Sizes of mDes1_new:\t\t" << mDes1_new.size << endl;
    cout << "Sizes of mvKeys2_new: \t" << mvKeys2_new.size() << endl;
    cout << "Sizes of mDes2_new:\t\t" << mDes2_new.size << endl;

    //  计算DT网络的边矩阵，并计算差的范数，提取外点
    //  更新keypoints，进行第二次match

//    cout << "\n筛选后的特征点：\n" << endl;
//    for(const auto &p:mvKeys1_new)
//    {
//        cout << p.pt << endl;
//    }
//    cout << "\n筛选后的描述子：\n" << endl;
//    cout << mDes1_new << endl;
    /**************** 特征匹配 ******************/
    vector<DMatch> matches2,good_matches2;
    BFMatcher matcher2 (NORM_HAMMING);
    matcher2.match(mDes1_new,mDes2_new,matches2);

    //计算最大与最小距离
    min_dist = 10000,max_dist = 0;
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
    cv::drawMatches(feature5,mvKeys1_new,feature6,mvKeys2_new,good_matches2,show2);
    imwrite("matches2.png",show2);
    imshow("matches2",show2);
    waitKey(0);
    /****************************************/
    cout << "finish!" << endl;
    return 0;
}