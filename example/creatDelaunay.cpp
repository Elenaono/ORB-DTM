#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <array>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "include/vector2.h"
#include "include/triangle.h"
#include "include/delaunay.h"
#include "include/calculator.h"
#include "include/computeSM.h"

using namespace std;
using namespace cv;

int main()
{
    /************************** create delaunay triangulation one **********************************/
    cout << "构建第一个Delaunay网络:" << endl;
    /// 生成随机坐标point1
    std::vector<Vector2<float> > points_one;
    points_one.emplace_back(Vector2<float>(200,100));
    points_one.emplace_back(Vector2<float>(200,300));
    points_one.emplace_back(Vector2<float>(100,220));

    /// Delaunay 三角生成 net1
    Delaunay<float> triangulation_one;
    std::vector<Triangle<float> > triangles_one = triangulation_one.triangulate(points_one);  //逐点插入法
    std::vector<Edge<float> > edges_one = triangulation_one.getEdges();
    // 显示
    std::cout << "Points_one : \t\t" << points_one.size() << std::endl;
    std::cout << "Triangles_one : \t" << triangles_one.size() << std::endl;
    std::cout << "Edges_one : \t\t" << edges_one.size() << std::endl;

    /// opencv 显示 net1
    cv::Mat first_image(480 , 620 , CV_8UC3);
    for(const auto &e : edges_one)
    {
        line(first_image, Point2f(e.p1.x, e.p1.y), Point2f(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }
    imshow("first",first_image);
//    waitKey(0);

    /************************** create delaunay triangulation one **********************************/
    cout << "\n构建第二个Delaunay网络:" << endl;
    /// 生成随机坐标point2
    std::vector<Vector2<float> > points_two;
    points_two.emplace_back(Vector2<float>(200,100));
    points_two.emplace_back(Vector2<float>(200,300));
    points_two.emplace_back(Vector2<float>(0,0));

    /// Delaunay 三角生成 net1
    Delaunay<float> triangulationnew_two;
    std::vector<Triangle<float> > triangles_two = triangulationnew_two.triangulate(points_two);  //逐点插入法
    std::vector<Edge<float> > edges_two = triangulationnew_two.getEdges();
    // 显示
    std::cout << "Points_two : \t\t" << points_two.size() << std::endl;
    std::cout << "Triangles_two : \t" << triangles_two.size() << std::endl;
    std::cout << "Edges_two : \t\t" << edges_two.size() << std::endl;


    /// opencv 显示 net1
    cv::Mat second_image (480 , 620 , CV_8UC3);
    for(const auto &e : edges_two)
    {
        line(second_image, Point2f(e.p1.x, e.p1.y), Point2f(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }
    imshow("second_image",second_image);
//    waitKey(0);

    /************************** compute similarity matrix of DT nets **********************************/
    cout << "\n计算两个网络的相似度矩阵:" << endl;

    cout << "Size of net1:" << triangles_one.size() << endl;
    cout << "Size of net2:" << triangles_two.size() << endl;

    unsigned long m=triangles_one.size();
    unsigned long n=triangles_two.size();
    Eigen::MatrixXd similarityMatrix = Eigen::MatrixXd::Zero(m,n);
    computesimilarityMatrix(triangulation_one,triangulationnew_two,similarityMatrix);   //相似度矩阵：使用引用作为函数参数

    cout << "\nsimilarityMatrix:\t" << m << "*" << n << endl << similarityMatrix << endl;

    /*****************  test!  ********************/

    /*****************************************/

    cout << "\nfinish!" << endl;
    return 0;
}

