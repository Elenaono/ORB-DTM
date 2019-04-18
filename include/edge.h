#ifndef H_EDGE
#define H_EDGE

#include "vector2.h"
#include <math.h>

template <class T>
class Edge
{
public:

    using VertexType = Vector2<T>;  //相当于 typedef

    //构造函数
    Edge(const VertexType &p1, const VertexType &p2) : p1(p1), p2(p2), isBad(false) {}
    Edge(const Edge &e) : p1(e.p1), p2(e.p2), isBad(false) {}

    inline double computesideLength()
    {
        sideLength = sqrt(  (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y)  );
        return sideLength;
    }

    VertexType p1;
    VertexType p2;
    double sideLength;
    bool isBad;
};

template <class T>
inline std::ostream &operator << (std::ostream &str, Edge<T> const &e)
{
    return str << "Edge " << e.p1 << ", " << e.p2;
}

template <class T>
inline bool operator == (const Edge<T> & e1, const Edge<T> & e2)
{
    return 	(e1.p1 == e2.p1 && e1.p2 == e2.p2) ||
              (e1.p1 == e2.p2 && e1.p2 == e2.p1);
}

template <class T>
inline bool almost_equal (const Edge<T> & e1, const Edge<T> & e2)
{
    return	(almost_equal(e1.p1, e2.p1) && almost_equal(e1.p2, e2.p2)) ||
              (almost_equal(e1.p1, e2.p2) && almost_equal(e1.p2, e2.p1));
}

#endif
