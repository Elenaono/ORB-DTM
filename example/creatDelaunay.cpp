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

#include "include/Vertex.h"
#include "include/triangle.h"
#include "include/delaunay.h"
//#include "include/calculator.h"
#include "include/computeSM.h"

using namespace std;
using namespace cv;
// rename
/**
 * @brief
 * @remark
 *          输入参数：二维vector
 *          Delaunay >> Triangle >> Edge >> Vertex
 * @return
 */



int main()
{
    /************************** create delaunay triangulation one **********************************/
    cout << "构建第一个Delaunay网络:" << endl;
    /// 生成随机坐标point1
    std::vector<Vertex<float> > points_one;
    points_one.emplace_back(Vertex<float>(200, 100, 0));
    points_one.emplace_back(Vertex<float>(200, 300, 1));
    points_one.emplace_back(Vertex<float>(100, 220, 2));
    points_one.emplace_back(Vertex<float>(300, 200, 3));
    points_one.emplace_back(Vertex<float>(80, 80, 4));
    points_one.emplace_back(Vertex<float>(330, 330, 5));

//    points_one.emplace_back(Vertex<float>(280,120));

    /// Delaunay 三角生成 net1
    Delaunay<float> triangulation_one;
    std::vector<Triangle<float> > triangles_one = triangulation_one.Triangulate(points_one);  //逐点插入法
    std::vector<Edge<float> > edges_one = triangulation_one.GetEdges();
    std::vector<Vertex<float> > point_store = triangulation_one.GetVertices();
    triangulation_one.ComputeEdgeMatrix();
    // 显示
    std::cout << "Points_one : \t\t" << points_one.size() << std::endl;
    std::cout << "Triangles_one : \t" << triangles_one.size() << std::endl;
    std::cout << "Edges_one : \t\t" << edges_one.size() << std::endl;

    /// opencv 显示 net1
    cv::Mat first_image(480 , 620 , CV_8UC3);   //480  620
    for(const auto &e : edges_one)
    {
        line(first_image, Point2f(e.p1.x, e.p1.y), Point2f(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }
    imshow("first",first_image);
    waitKey(0);

    /************************** create delaunay triangulation two **********************************/

    cout << "\n构建第二个Delaunay网络:" << endl;
    /// 生成随机坐标point2
    std::vector<Vertex<float> > points_two;
    points_two.emplace_back(Vertex<float>(200, 100, 0));
    points_two.emplace_back(Vertex<float>(200, 300, 1));
    points_two.emplace_back(Vertex<float>(100, 220, 2));
    points_two.emplace_back(Vertex<float>(300, 200, 3));
    points_two.emplace_back(Vertex<float>(80, 80, 4));
    points_two.emplace_back(Vertex<float>(330, 330, 5));
    points_two.emplace_back(Vertex<float>(180, 180, 6));



    /// Delaunay 三角生成 net2
    Delaunay<float> triangulation_two;
    std::vector<Triangle<float> > triangles_two = triangulation_two.Triangulate(points_two);  //逐点插入法
    std::vector<Edge<float> > edges_two = triangulation_two.GetEdges();
    triangulation_two.ComputeEdgeMatrix();
    // 显示
    std::cout << "Points_two : \t\t" << points_two.size() << std::endl;
    std::cout << "Triangles_two : \t" << triangles_two.size() << std::endl;
    std::cout << "Edges_two : \t\t" << edges_two.size() << std::endl;

    /// opencv 显示 net2
    cv::Mat second_image (480 , 620 , CV_8UC3);
    for(const auto &e : edges_two)
    {
        line(second_image, Point2f(e.p1.x, e.p1.y), Point2f(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }
    imshow("second_image",second_image);
    waitKey(0);

    /************************** compute edge matrix of DT nets **********************************/
    Eigen::MatrixXd edgeMatrix = Eigen::MatrixXd::Zero(500,500);  //Zero(20,20);
    edgeMatrix = triangulation_one.GetEdgeMatrix() - triangulation_two.GetEdgeMatrix();
    double value =0;
    value = edgeMatrix.norm();
    cout << "\nvalue: " << value <<  endl;

    /*****************  test!  ********************/

    /*****************************************/
//    waitKey(0);
    cout << "\nmain end, see you...";
    return 0;
}

