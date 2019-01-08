#include "lib/util/InterpLookupTable.h"
#include <limits>
#include <algorithm>

namespace frc973 {

InterpLookupTable::InterpLookupTable()
        : points()
        , min_x(std::numeric_limits<double>::max())
        , max_x(std::numeric_limits<double>::min()) {
}

InterpLookupTable::~InterpLookupTable() {
}

InterpLookupTable &InterpLookupTable::AddPoint(double x, double y) {
    points.push_back({x, y});
    std::sort(points.begin(), points.end());

    if (x > max_x) {
        max_x = x;
    }
    if (x < min_x) {
        min_x = x;
    }

    return *this;
}

double InterpLookupTable::LookupPoint(double x) {
    if (points.size() == 0) {
        return 0.0;
    }
    else if (points.size() == 1) {
        return points[0].y;
    }
    else if (x > max_x) {
        Point a = points[points.size() - 1], b = points[points.size() - 2];
        return InterpolatePoints(a, b, x);
    }
    else if (x < min_x) {
        Point a = points[0], b = points[1];
        return InterpolatePoints(a, b, x);
    }
    else {
        Point lesser = points[0], greater = points[points.size() - 1];

        for (auto it = points.begin(); it < points.end(); it++) {
            if (it->x == x) {
                return it->y;
            }
            if (it->x < x && it->x > lesser.x) {
                lesser = *it;
            }
            if (it->x > x && it->x < greater.x) {
                greater = *it;
            }
        }

        return InterpolatePoints(lesser, greater, x);
    }
}

double InterpLookupTable::InterpolatePoints(const Point &a, const Point &b,
                                            const double x) {
    return a.y + (x - a.x) * (b.y - a.y) / (b.x - a.x);
}
}
