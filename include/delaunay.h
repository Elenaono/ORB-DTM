#ifndef H_DELAUNAY
#define H_DELAUNAY

#include "Vertex.h"
#include "edge.h"
#include "triangle.h"

#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>

template <class T>
class Delaunay
{
public:
    using TriangleType = Triangle<T>;
    using EdgeType = Edge<T>;
    using VertexType = Vertex<T>;

public:
    std::vector<TriangleType>& Triangulate(std::vector<VertexType> &vertices);
    void ComputeEdgeMatrix();
    const std::vector<TriangleType>& GetTriangles() const { return triangles_; }
    const std::vector<EdgeType>& GetEdges() const { return edges_; }
    const std::vector<VertexType>& GetVertices() const { return vertices_; }
    const Eigen::MatrixXd& GetEdgeMatrix() const { return edgeMatrix_; }

private:
    std::vector<TriangleType> triangles_;
    std::vector<EdgeType> edges_;
    std::vector<VertexType> vertices_;
    Eigen::MatrixXd edgeMatrix_;
};

/**
 * @brief Deluanay 三角剖分核心算法  ---  逐点插入法
 * @tparam T
 * @param vertices
 * @return
 */
template<class T>
typename std::vector<typename Delaunay<T>::TriangleType> &Delaunay<T>::Triangulate(std::vector<VertexType> &vertices) {
    // 将点云拷贝一份
    vertices_ = vertices;

    /// 计算超级三角形的一些参数
    T minX = vertices[0].x;
    T minY = vertices[0].y;
    T maxX = minX;
    T maxY = minY;

    //计算超级三角形的上下左右边界
    for(std::size_t i = 0; i < vertices.size(); ++i)
    {
        if (vertices[i].x < minX) minX = vertices[i].x;
        if (vertices[i].y < minY) minY = vertices[i].y;
        if (vertices[i].x > maxX) maxX = vertices[i].x;
        if (vertices[i].y > maxY) maxY = vertices[i].y;
    }

    const T dx = maxX - minX;
    const T dy = maxY - minY;
    const T deltaMax = std::max(dx, dy);
    const T midx = half(minX + maxX);
    const T midy = half(minY + maxY);

    //尽可能扩大超级三角形范围，可以取比20更大的数字
    const VertexType p1(midx - 20 * deltaMax, midy - deltaMax,0);
    const VertexType p2(midx, midy + 20 * deltaMax,0);
    const VertexType p3(midx + 20 * deltaMax, midy - deltaMax,0);

    // Create a list of triangles, and add the supertriangle in it
    // 将超级三角形添加至 triangles_
    // push_back() 此处的参数其实是一个构造函数 Triangle()
    // 使用p1,p2,p3构造三角形后，放进容器中
    triangles_.push_back(TriangleType(p1, p2, p3));

    ///开始依次遍历每个点
//        unsigned long index=0;
    for(auto p = begin(vertices); p != end(vertices); p++)
    {
        //std::cout << "Traitement du point " << *p << std::endl;
        //std::cout << "triangles_ contains " << triangles_.size() << " elements" << std::endl;

        //构造变量用来存储临时新产生的边
        std::vector<EdgeType> polygon;

        ///依次遍历 _triangles  中的每个三角形
        for(auto & t : triangles_)
        {
            //std::cout << "Processing " << std::endl << *t << std::endl;

            if(t.CircumCircleContains(*p))  //如果包含点 p，那么就要产生新的3条边
            {
                //std::cout << "Pushing bad triangle " << *t << std::endl;
                t.isBad = true;  //flag 发生改变，准备接下来在 triangles_  中将其剔除
                polygon.push_back(t.e1);
//                    std::cout << t.e1 << std::endl;
                polygon.push_back(t.e2);
//                    std::cout << t.e2 << std::endl;
                polygon.push_back(t.e3);    // 第一次迭代时，存储的是超级三角形
//                    std::cout << t.e3 << std::endl;
//                    std::cout << "\n\n" << std::endl;

            }
            else
            {
                //std::cout << " does not contains " << *p << " in his circum center" << std::endl;
            }
//                count++;
        }
        ///更新 _triangles
        // std::remove_if只移动不删除，erase将其删除，这里还用到了C++11的 lambda 表达式
        // remove_if(beg, end, op)   //移除区间[beg,end)中每一个“令判断式:op(elem)获得true”的元素；
        // remove_if() 并不会实际移除容器中的元素，而是将需要移除的元素移动到容器尾部，从而得到一个分界容器(前：保留；后：待移除)
        // 借助erase 函数才能真正的移除元素
        triangles_.erase(std::remove_if(begin(triangles_), end(triangles_), [](TriangleType &t){return t.isBad;}), end(triangles_));

        ///这个用来删除重复的边
        for(auto e1 = begin(polygon); e1 != end(polygon); ++e1)
        {
            for(auto e2 = e1 + 1; e2 != end(polygon); ++e2)
            {
                if(almost_equal(*e1, *e2))
                {
                    e1->isBad = true;
                    e2->isBad = true;
                }
            }
        }

        ///更新 polygon   这些边围成了一个凸多边形
        polygon.erase(std::remove_if(begin(polygon), end(polygon), [](EdgeType &e){return e.isBad;})  ,    end(polygon));

//            std::cout <<  "First:" <<std::endl;
        for(const auto e : polygon)
        {
            triangles_.push_back(TriangleType(e.p1, e.p2, *p));     // 为每一条凸多边形的边，与凸多边形内的点，形成一个三角形

//                std::cout <<  e <<std::endl;
        }
//            std::cout <<  "First:end\n" <<std::endl;


    }
//        std::cerr << "count:" << count << std::endl;

    ///删除超级三角形
    triangles_.erase(std::remove_if(begin(triangles_), end(triangles_), [p1, p2, p3](TriangleType &t){
        return t.ContainsVertex(p1) || t.ContainsVertex(p2) || t.ContainsVertex(p3);
    }), end(triangles_));

    for(const auto t : triangles_)
    {
        edges_.push_back(t.e1);
        edges_.push_back(t.e2);
        edges_.push_back(t.e3);
    }

    return triangles_;
}

template<class T>
void Delaunay<T>::ComputeEdgeMatrix() {
    unsigned long count=edges_.size();
    edgeMatrix_ = Eigen::MatrixXd::Zero(500, 500);
    for(const auto &p:edges_)
    {
        unsigned long m=p.p1.index;
        unsigned long n=p.p2.index;
        edgeMatrix_(m, n) = 1;
        edgeMatrix_(n, m) = 1;
    }
}

#endif
