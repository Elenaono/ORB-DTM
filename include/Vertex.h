#ifndef H_VECTOR2
#define H_VECTOR2

#include "numeric.h"

//#include "include/edge.h"

#include <iostream>
#include <cmath>

template <typename T>
class Vertex
{
public:
    Vertex(): x(0), y(0){}  // Constructors  构造函数
    Vertex(T _x, T _y): x(_x), y(_y){}     // 拷贝构造
    Vertex(const Vertex &v): x(v.x), y(v.y), index(v.index){}     // 拷贝构造
    Vertex(T _x, T _y, T _index): x(_x), y(_y), index(_index){}

    // Operations
    // 计算距离
    T dist2(const Vertex &v) const
    {
        T dx = x - v.x;
        T dy = y - v.y;
        return dx * dx + dy * dy;
    }

    T dist(const Vertex &v) const
    {
        return sqrt(dist2(v));
    }

    //计算平方和，此函数在 delaunay.h 中判断一点是否在三角形内部
    T norm2() const
    {
        return x * x + y * y;
    }

//    friend double Edge::ComputeSideLength();

private:
    T x;
    T y;
    T index;
};

//全特化
template <>
inline float Vertex<float>::dist(const Vertex<float> &v) const { return hypotf(x - v.x, y - v.y);}

template <>
inline double Vertex<double>::dist(const Vertex<double> &v) const { return hypotf(x - v.x, y - v.y);}

template<typename T>
std::ostream& operator << (std::ostream &str, Vertex<T> const &point)
{
    return str << "Point x: " << point.x << " y: " << point.y << " index: " << point.index;
}

template<typename T>
bool operator == (const Vertex<T>& v1, const Vertex<T>& v2)
{
    return (v1.x == v2.x) && (v1.y == v2.y);
}

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(const Vertex<T>& v1, const Vertex<T>& v2, int ulp=2)
{
    return almost_equal(v1.x, v2.x, ulp) && almost_equal(v1.y, v2.y, ulp);
}

#endif
