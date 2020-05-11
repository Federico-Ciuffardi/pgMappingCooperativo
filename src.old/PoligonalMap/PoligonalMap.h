#include <boost/polygon/polygon.hpp>

using namespace boost::polygon;

typedef polygon_data<int> Polygon;

class PoligonalMap{
private:
     Polygon polygons[];

public:
    PoligonalMap(/* args */);
    ~PoligonalMap();
};
