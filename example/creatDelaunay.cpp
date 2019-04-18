#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <array>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include "include/vector2.h"
#include "include/triangle.h"
#include "include/delaunay.h"
#include "include/calculator.h"
#include "include/computeSM.h"

using namespace std;
using namespace cv;


// 上传至github
int main()
{
    string file1 = "./data/desk1.png";
    /******************   生成随机坐标   ***************************/
    std::vector<Vector2<float> > points;
    points.emplace_back(Vector2<float>(200,100));
    points.emplace_back(Vector2<float>(200,300));
    points.emplace_back(Vector2<float>(100,220));

//    points.emplace_back(Vector2<float>(300,200));
//    points.emplace_back(Vector2<float>(80,80));
//    points.emplace_back(Vector2<float>(330,330));
    /******************   Delaunay 三角生成   *********************/
    Delaunay<float> triangulation;
    /*
    //    const std::vector<Triangle<float> > triangles = triangulation.triangulate(points);  //逐点插入法
     */
    std::vector<Triangle<float> > triangles = triangulation.triangulate(points);  //逐点插入法
    /*
//    std::cout << triangles.size() << " triangles generated\n";
//    const std::vector<Edge<float> > edges = triangulation.getEdges();
    */
    std::vector<Edge<float> > edges = triangulation.getEdges();

    std::cout << "Points : " << points.size() << std::endl;
    /*
//    for(const auto &p : points)
//        std::cout << p << std::endl;
*/
    std::cout << "Triangles : " << triangles.size() << std::endl;
    /*
//    for(const auto &t : triangles)
//        std::cout << t << std::endl;
*/
    std::cout << "Edges : " << edges.size() << std::endl;
    /*
//    for(const auto &e : edges)
//        std::cout << e << std::endl;
*/
    /*****************  opencv 显示  *****************************/
    /*
//    cv::Mat first_image = cv::imread(file1,0);
     */
    cv::Mat first_image(480 , 620 , CV_8UC3);
    /*
//    imshow("test",first_image);
//    waitKey(0);
//    std::cout << "\nEdges : " << edges.size() << std::endl;
     */
    for(const auto &e : edges)
    {   /*
        cout << e.p1.x<<","<<e.p1.y << ";" << e.p2.x<<","<<e.p2.y << endl;
        */
        line(first_image, Point2f(e.p1.x, e.p1.y), Point2f(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }

    imshow("first",first_image);
    waitKey(0);
    /******************  插入一个新节点  ****************************/
    cout << "*************************"<< endl;

    std::vector<Vector2<float> > new_points;

    new_points.emplace_back(Vector2<float>(200,100));
    new_points.emplace_back(Vector2<float>(200,300));
    new_points.emplace_back(Vector2<float>(0,0));

//    new_points.emplace_back(Vector2<float>(300,200));
//    new_points.emplace_back(Vector2<float>(80,80));
//    new_points.emplace_back(Vector2<float>(330,330));
//    new_points.emplace_back(Vector2<float>(180,180));

//    Delaunay<float> triangulationnew(triangulation);
    Delaunay<float> triangulationnew;
    std::vector<Triangle<float> > trianglesnew = triangulationnew.triangulate(new_points);  //逐点插入法


//    cerr << "Size of new_points:" << new_points.size()<< endl;
//    triangulationnew.insertNewPointsInside(new_points);

    std::vector<Edge<float> > edgesnew = triangulationnew.getEdges();
    edgesnew = triangulationnew.getEdges();
    std::cout << "\nEdges : " << edgesnew.size() << std::endl;
    new_points = triangulationnew.getVertices();
    std::cout << "\nPoints : " << new_points.size() << std::endl;
    /*
//    triangles.assign(triangulation.triangulate(points).begin(),triangulation.triangulate(points).end());
//    std::cout << "Triangles : " << triangles.size() << std::endl;
//    edges.assign(triangulation.getEdges().begin(),triangulation.getEdges().end());
*/
    cv::Mat second_image (480 , 620 , CV_8UC3);

    for(const auto &e : edgesnew)
    {
        //cout << e.p1.x<<","<<e.p1.y << ";" << e.p2.x<<","<<e.p2.y << endl;
        line(second_image, Point2f(e.p1.x, e.p1.y), Point2f(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }
    imshow("second_image",second_image);
    waitKey(0);

    cout << "Size of net1:" << triangles.size() << endl;
    cout << "Size of net2:" << trianglesnew.size() << endl;

    /*****************************************/
//    std::vector<Triangle<float> > triangles = triangulation.triangulate(points);  //逐点插入法




    double SM=0;
    SM = computesimilarityMatrix(triangulation,triangulationnew);
    cout << "SM:" << SM << endl;
//    cerr << "SM value:"<< endl;
//    for (int i = 0; i < 50; ++i)
//    {
//        for (int j = 0; j < 50; ++j)
//        {
//            cout << SM[i][j] << ";";
//        }
//
//        cout <<endl;
//    }

    /*****************************************/
    cout << "finish!" << endl;
    return 0;
}

