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

using namespace std;
using namespace cv;

int main()
{
    string file1 = "./data/desk1.png";
    /******************   生成随机坐标   *********************/
    std::vector<Vector2<float> > points;
    points.emplace_back(Vector2<float>(200,100));
    points.emplace_back(Vector2<float>(200,300));
    points.emplace_back(Vector2<float>(100,200));
    points.emplace_back(Vector2<float>(300,200));
    points.emplace_back(Vector2<float>(80,80));
    points.emplace_back(Vector2<float>(330,330));
    /******************   Delaunay 三角生成   *********************/
    Delaunay<float> triangulation;
//    const std::vector<Triangle<float> > triangles = triangulation.triangulate(points);  //逐点插入法
    std::vector<Triangle<float> > triangles = triangulation.triangulate(points);  //逐点插入法
    std::cout << triangles.size() << " triangles generated\n";
//    const std::vector<Edge<float> > edges = triangulation.getEdges();
    std::vector<Edge<float> > edges = triangulation.getEdges();

    std::cout << " ========= ";
    std::cout << "\nPoints : " << points.size() << std::endl;
//    for(const auto &p : points)
//        std::cout << p << std::endl;

    std::cout << "Triangles : " << triangles.size() << std::endl;
//    for(const auto &t : triangles)
//        std::cout << t << std::endl;

    std::cout << "Edges : " << edges.size() << std::endl;
//    for(const auto &e : edges)
//        std::cout << e << std::endl;

    /*****************  opencv 显示  ****************/
//    cv::Mat first_image = cv::imread(file1,0);
    cv::Mat first_image(480 , 620 , CV_8UC3);
//    imshow("test",first_image);
//    waitKey(0);
//    std::cout << "\nEdges : " << edges.size() << std::endl;
    for(const auto &e : edges)
    {
//        cout << e.p1.x<<","<<e.p1.y << ";" << e.p2.x<<","<<e.p2.y << endl;
        line(first_image, Point2f(e.p1.x, e.p1.y), Point2f(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);

//        imshow("first",first_image);
//        waitKey(0);
    }

    imshow("first",first_image);
//    waitKey(0);

    /*****************************************/
    cout << "*************************************************"<< endl;
//    /*
    std::vector<Vector2<float> > new_points;
    new_points.emplace_back(Vector2<float>(180,180));
//    cerr << "Size of new_points:" << new_points.size()<< endl;

    triangulation.insertNewPointsInside(new_points);

    edges = triangulation.getEdges();
    std::cout << "\nEdges : " << edges.size() << std::endl;
    points = triangulation.getVertices();
    std::cout << "\nPoints : " << points.size() << std::endl;
//    triangles.assign(triangulation.triangulate(points).begin(),triangulation.triangulate(points).end());
//    std::cout << "Triangles : " << triangles.size() << std::endl;
//    edges.assign(triangulation.getEdges().begin(),triangulation.getEdges().end());

    cv::Mat second_image (480 , 620 , CV_8UC3);


    for(const auto &e : edges)
    {
//        cout << e.p1.x<<","<<e.p1.y << ";" << e.p2.x<<","<<e.p2.y << endl;
        line(second_image, Point2f(e.p1.x, e.p1.y), Point2f(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
//        imshow("second_image",second_image);
//        waitKey(0);
    }
    imshow("second_image",second_image);
    imwrite("second_image.png",second_image);
    waitKey(0);

//     */
    /*****************************************/

    cout << "finish!" << endl;
    return 0;
}

