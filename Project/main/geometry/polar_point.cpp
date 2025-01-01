#include <geometry/polar_point.h>
#include <geometry/cartesian_point.h>

// Constructor
PolarPoint::PolarPoint(double ang_rad, double dist) : angle_rad(ang_rad), distance(dist) {}

double PolarPoint::getAngleRadians() const {
    return angle_rad;
}

double PolarPoint::getDistance() const {
    return distance;
}

void PolarPoint::setPolar(double ang_rad, double dist) {
    angle_rad = ang_rad;
    distance = dist;
}

void PolarPoint::setCartesian(double x, double y) {
    distance = sqrt(x * x + y * y);
    angle_rad = atan2(y, x);
}

void PolarPoint::convertToCartesian(double &x, double &y) const {
    x = distance * cos(angle_rad);
    y = distance * sin(angle_rad);
}

CartesianPoint PolarPoint::toCartesian() const {
    return CartesianPoint(distance * cos(angle_rad), distance * sin(angle_rad));
}

double PolarPoint::distanceBetweenPoints(const PolarPoint& point1, const PolarPoint& point2)
{
    double x1, y1, x2, y2;
    point1.convertToCartesian(x1, y1);
    point2.convertToCartesian(x2, y2);
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

std::ostream& operator<<(std::ostream& os, const PolarPoint& point) {
    os << "[" << point.angle_rad << "," << point.distance<<"]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<PolarPoint>& points) {
    os << "{";
    for (const auto& point : points) {
        os << point;
    }
    os << "}";
    return os;
}
