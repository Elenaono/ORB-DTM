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

#define d_max_vaule 35  //35    55
#define d_max_vaule_new 45

#define m_max_value 5   //5
#define n_max_value 5
#define sizeofEdgeMatrix 500

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

    /**************** 克隆图片 ******************/
    Mat feature3 = feature1.clone();
    Mat feature4 = feature2.clone();
    Mat feature5 = feature1.clone();
    Mat feature6 = feature2.clone();
    Mat feature7 = feature1.clone();
    Mat feature8 = feature2.clone();
    Mat debugOne = feature1.clone();
    Mat debugTwo = feature2.clone();
    /**************** 第一次特征匹配 ******************/
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

    sort(good_matches.begin(), good_matches.end(), cmp1);   //排序
    good_matches.erase(unique(good_matches.begin(),good_matches.end(),cmp2),good_matches.end());    //去重

    // 对新的排列重新赋值index
    for(int i =0 ;i < good_matches.size();i++)
    {
        good_matches[i].imgIdx = i;
    }

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
        line(feature1, Point(e.p1.x, e.p1.y), Point(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
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
    /**************** 显示匹配结果与初始DT网络 ******************/
    cout << "\t匹配:" << endl;
    cout << "\t\tmatch:" << good_matches.size()<<endl;
    Mat show;
    cv::drawMatches(feature1,mvKeys1,feature2,mvKeys2,good_matches,show);
    imwrite("matches.png",show);
    imshow("matches",show);
    waitKey(0);

    /*******************  构建边矩阵，并计算相似度(范数)，进行DT网络的优化  *********************/
    cout << "\n计算DTM的相关信息：" << endl;
    Eigen::MatrixXd::Index maxRow,maxCol;
    Eigen::MatrixXd edgeMatrix = Eigen::MatrixXd::Zero(sizeofEdgeMatrix,sizeofEdgeMatrix);  //computeEdgeMatrix() 在此处也修改了 20,20 ，需要同步修改，后期改进此处
    edgeMatrix = triangulation.getEdgeMatrix() - triangulation2.getEdgeMatrix();
    //    double value =0;
    //    value = edgeMatrix.norm();
    //    cout << "\tvalue: " << value <<  endl;      // 相似度

    edgeMatrix.cwiseAbs().colwise().sum().maxCoeff(&maxRow,&maxCol);    // 边矩阵.绝对值.列.和.最大值(行序号,列序号)

//    cout << "提取候选外点：\t"  << maxCol << endl;
//    cout << "显示sum:\n" << edgeMatrix.cwiseAbs().colwise().sum() << endl;
//    cout << "计算列和：\n" << edgeMatrix.cwiseAbs().colwise().sum()<< endl;
//    cout << "显示边矩阵之差：\n"<< edgeMatrix.cwiseAbs().col(maxCol).transpose() << endl;
//    cout << "二者之差：\n" << edgeMatrix.cwiseAbs().colwise().sum() - edgeMatrix.cwiseAbs().col(maxCol).transpose()<< endl;
//    cout << "候选外点：" << mvKeys2[good_matches[maxCol].trainIdx].pt << endl;

    // 通过DT网络的边矩阵之差的范数，删除列和较大的候选外点集
    cout << "\nold size:\t" << good_matches.size()<<endl;
    for(int i = sizeofEdgeMatrix;i != 0 ;i--)
    {
        if((edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) >= m_max_value )
        {
            cout << (edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) << "\t,\t" << mvKeys1[good_matches[i-1].queryIdx].pt <<"\t,\t" << mvKeys2[good_matches[i-1].trainIdx].pt << endl;
            good_matches.erase(good_matches.begin()+i-1);
        }
    }
    cout << "new size:\t" << good_matches.size()<<endl;

    // 显示优化后的DT网络
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
    for(const auto &e3 : edges3)
    {
        line(feature3, Point(e3.p1.x, e3.p1.y), Point(e3.p2.x, e3.p2.y), Scalar(0, 0, 255), 1);
    }

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

    for(const auto &e4 : edges4)
    {
        line(feature4, Point(e4.p1.x, e4.p1.y), Point(e4.p2.x, e4.p2.y), Scalar(0, 0, 255), 1);
    }

    Mat show_temp;
    cv::drawMatches(feature3,mvKeys1,feature4,mvKeys2,good_matches,show_temp);
    imshow("show_temp",show_temp);
    waitKey(0);
    /*******************  剔除 good_matchs 中的点，对剩余点集进行二次匹配   *********************/
    //   cv::Mat中没有删除某一列或者行的函数
    //   只能构造新的Mat，在删除某一列后，将后边的复制到新的Mat当中去
    //   新的解决方案是：将Mat转换为vector，使用back() pop()等操作处理后，再转换成Mat
    //   注意：由于删除的是列，而转换成vector后操作的是行，因此可以对Mat进行转置后，再进行转换操作，即Mat.t()
    //   在循环外边完成Mat到vector的转换工作，进行循环操作并退出后，再进行转换回来

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

    /**************** 第二次特征匹配 ******************/
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

    sort(good_matches2.begin(), good_matches2.end(), cmp1);   //排序
    good_matches2.erase(unique(good_matches2.begin(),good_matches2.end(),cmp2),good_matches2.end());    //去重

    // 对新的排列重新赋值index
    for(int i =0 ;i < good_matches2.size();i++)
    {
        good_matches2[i].imgIdx = i;
    }

    /**********************  构建第二组 DT 网络  ******************************/
    ///delaunay five
    cout << "\tDT five:" << endl;
    std::vector<Vector2<float> > points5;
    for(const auto &g:good_matches2)
    {
        points5.emplace_back(Vector2<float>(mvKeys1_new[g.queryIdx].pt.x , mvKeys1_new[g.queryIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation5;
    const std::vector<Triangle<float> > triangles5 = triangulation5.triangulate(points5);  //逐点插入法
    triangulation5.computeEdgeMatrix();
    std::cout << "\t\t" << triangles5.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges5 = triangulation5.getEdges();
    for(const auto &e5 : edges5)
    {
        line(feature5, Point(e5.p1.x, e5.p1.y), Point(e5.p2.x, e5.p2.y), Scalar(0, 0, 255), 1);
    }

    ///delaunay six
    cout << "\tDT six:" << endl;
    std::vector<Vector2<float> > points6;
    for(const auto &g:good_matches2)
    {
        points6.emplace_back(Vector2<float>(mvKeys2_new[g.trainIdx].pt.x , mvKeys2_new[g.trainIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation6;
    const std::vector<Triangle<float> > triangles6 = triangulation6.triangulate(points6);  //逐点插入法
    triangulation6.computeEdgeMatrix();
    std::cout << "\t\t" << triangles6.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges6 = triangulation6.getEdges();

    for(const auto &e4 : edges6)
    {
        line(feature6, Point(e4.p1.x, e4.p1.y), Point(e4.p2.x, e4.p2.y), Scalar(0, 0, 255), 1);
    }

    /**************** 显示匹配结果与初始DT网络 ******************/
    cout << "match:" << good_matches2.size()<<endl;
    Mat show2;
    cv::drawMatches(feature5,mvKeys1_new,feature6,mvKeys2_new,good_matches2,show2);
    imwrite("matches2.png",show2);
    imshow("matches2",show2);
    waitKey(0);

    /*******************  构建边矩阵，并计算相似度(范数)，进行DT网络的优化  *********************/
    cout << "\n计算DTM的相关信息：" << endl;
//    Eigen::MatrixXd::Index maxRow,maxCol;
//    Eigen::MatrixXd edgeMatrix = Eigen::MatrixXd::Zero(50,50);  //computeEdgeMatrix() 在此处也修改了 20,20 ，需要同步修改，后期改进此处
    edgeMatrix = Eigen::MatrixXd::Zero(sizeofEdgeMatrix,sizeofEdgeMatrix);
    edgeMatrix = triangulation5.getEdgeMatrix() - triangulation6.getEdgeMatrix();
    //    double value =0;
    //    value = edgeMatrix.norm();
    //    cout << "\tvalue: " << value <<  endl;      // 相似度

    edgeMatrix.cwiseAbs().colwise().sum().maxCoeff(&maxRow,&maxCol);    // 边矩阵.绝对值.列.和.最大值(行序号,列序号)

    cout << "提取候选外点：\t"  << maxCol << endl;
    cout << "显示sum:\n" << edgeMatrix.cwiseAbs().colwise().sum() << endl;
    cout << "计算列和：\n" << edgeMatrix.cwiseAbs().colwise().sum()<< endl;
    cout << "显示边矩阵之差：\n"<< edgeMatrix.cwiseAbs().col(maxCol).transpose() << endl;
    cout << "二者之差：\n" << edgeMatrix.cwiseAbs().colwise().sum() - edgeMatrix.cwiseAbs().col(maxCol).transpose()<< endl;
    cout << "候选外点：" << mvKeys2[good_matches[maxCol].trainIdx].pt << endl;

    // 通过DT网络的边矩阵之差的范数，删除列和较大的候选外点集
    cout << "\nold size:\t" << good_matches2.size()<<endl;
    for(int i = sizeofEdgeMatrix;i != 0 ;i--)
    {
        if((edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) >= n_max_value )      //m_max_value
        {
            cout << (edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) << "\t,\t" << mvKeys1_new[good_matches2[i-1].queryIdx].pt <<"\t,\t" << mvKeys2_new[good_matches2[i-1].trainIdx].pt << endl;
            good_matches2.erase(good_matches2.begin()+i-1);
        }
    }
    cout << "new size:\t" << good_matches2.size()<<endl;

    // 显示优化后的DT网络
    ///delaunay seven
    cout << "\tDT seven:" << endl;
    std::vector<Vector2<float> > points7;
    for(const auto &g:good_matches2)
    {
        points7.emplace_back(Vector2<float>(mvKeys1_new[g.queryIdx].pt.x , mvKeys1_new[g.queryIdx].pt.y ,g.imgIdx ));
    }
    Delaunay<float> triangulation7;
    const std::vector<Triangle<float> > triangles7 = triangulation7.triangulate(points7);  //逐点插入法
    triangulation7.computeEdgeMatrix();
    std::cout << "\t\t" << triangles7.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges7 = triangulation7.getEdges();
    for(const auto &e7 : edges7)
    {
        line(feature7, Point(e7.p1.x, e7.p1.y), Point(e7.p2.x, e7.p2.y), Scalar(0, 0, 255), 1);
    }

    ///delaunay eight
    cout << "\tDT eight:" << endl;
    std::vector<Vector2<float> > points8;
    for(const auto &g:good_matches2)
    {
        points8.emplace_back(Vector2<float>(mvKeys2_new[g.trainIdx].pt.x , mvKeys2_new[g.trainIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation8;
    const std::vector<Triangle<float> > triangles8 = triangulation8.triangulate(points8);  //逐点插入法
    triangulation8.computeEdgeMatrix();
    std::cout << "\t\t" << triangles8.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges8 = triangulation8.getEdges();

    for(const auto &e8 : edges8)
    {
        line(feature8, Point(e8.p1.x, e8.p1.y), Point(e8.p2.x, e8.p2.y), Scalar(0, 0, 255), 1);
    }

    Mat show1_temp;
    cv::drawMatches(feature7,mvKeys1_new,feature8,mvKeys2_new,good_matches2,show1_temp);
    imshow("show1_temp",show1_temp);
    waitKey(0);


    /*******************  构建边矩阵，并计算相似度(范数)，进行DT网络的优化  *********************/
//    cout << "\n计算DTM的相关信息：" << endl;
////    Eigen::MatrixXd::Index maxRow,maxCol;
////    Eigen::MatrixXd edgeMatrix = Eigen::MatrixXd::Zero(50,50);  //computeEdgeMatrix() 在此处也修改了 20,20 ，需要同步修改，后期改进此处
//    edgeMatrix = Eigen::MatrixXd::Zero(sizeofEdgeMatrix,sizeofEdgeMatrix);
//    edgeMatrix = triangulation7.getEdgeMatrix() - triangulation8.getEdgeMatrix();
//    //    double value =0;
//    //    value = edgeMatrix.norm();
//    //    cout << "\tvalue: " << value <<  endl;      // 相似度
//
//    edgeMatrix.cwiseAbs().colwise().sum().maxCoeff(&maxRow,&maxCol);    // 边矩阵.绝对值.列.和.最大值(行序号,列序号)
//
//    cout << "提取候选外点：\t"  << maxCol << endl;
//    cout << "显示sum:\n" << edgeMatrix.cwiseAbs().colwise().sum() << endl;
//    cout << "计算列和：\n" << edgeMatrix.cwiseAbs().colwise().sum()<< endl;
//    cout << "显示边矩阵之差：\n"<< edgeMatrix.cwiseAbs().col(maxCol).transpose() << endl;
//    cout << "二者之差：\n" << edgeMatrix.cwiseAbs().colwise().sum() - edgeMatrix.cwiseAbs().col(maxCol).transpose()<< endl;
//    cout << "候选外点：" << mvKeys2[good_matches[maxCol].trainIdx].pt << endl;
//
//    // 通过DT网络的边矩阵之差的范数，删除列和较大的候选外点集
//    cout << "\nold size:\t" << good_matches2.size()<<endl;
//    for(int i = sizeofEdgeMatrix;i != 0 ;i--)
//    {
//        if((edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) >= 1 )      //m_max_value
//        {
//            cout << (edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) << "\t,\t" << mvKeys1_new[good_matches2[i-1].queryIdx].pt <<"\t,\t" << mvKeys2_new[good_matches2[i-1].trainIdx].pt << endl;
//            good_matches2.erase(good_matches2.begin()+i-1);
//        }
//    }
//    cout << "new size:\t" << good_matches2.size()<<endl;

    /****************************************/












    cout << "finish!" << endl;
    return 0;
}