#ifndef LOCALIZATION_UTILS_H
#define LOCALIZATION_UTILS_H

#include <cmath>
#include "geometry_msgs/msg/quaternion.hpp"

struct CartesianPoint {
    double x;
    double y;
};

class GpsConverter {
public:
    GpsConverter(double latInit = 0.0, double lonInit = 0.0);
    CartesianPoint toCartesian(double latCurr, double longCurr);

private:
    double latInit_;
    double lonInit_;
    double deg2rad(double degrees);
    const double a = 6378137.0;
    const double f = 1.0 / 298.2572236;
};

#endif // LOCALIZATION_UTILS_H