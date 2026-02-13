#include "localization_utils.h"

GpsConverter::GpsConverter(double latInit, double lonInit) : latInit_(latInit), lonInit_(lonInit) {}

double GpsConverter::deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

CartesianPoint GpsConverter::toCartesian(double latCurr, double longCurr) {
    double lat_diff = latCurr - latInit_;
    double lon_diff = longCurr - lonInit_;

    double lat_rad = deg2rad(lat_diff);
    double lon_rad = deg2rad(lon_diff);

    // Aproximación equirectangular
    double R = 6371000.0;
    double x = R * lon_rad * std::cos(deg2rad(latInit_));
    double y = R * lat_rad;

    return {-x, -y};
}