#pragma once

#include <iostream>
#include <geometry/cartesian_point.h>
#include <cmath>


class Position {
private:
    CartesianPoint point; //position in the cartesian space
    double orientation;   //orientation in radiants

public:
    

    Position(CartesianPoint p = CartesianPoint(), double o = 0);

    CartesianPoint getPoint() const;

    double getOrientation() const;
	
    void setPoint(CartesianPoint p);

    void setPoint(double x, double y);

    void setOrientation(double o = 0);

    void setPosition(CartesianPoint p, double o = 0);

	void setPosition(double x, double y, double o = 0);
	
	static double sinCosToRad(double sin, double cos);

    friend std::ostream& operator<<(std::ostream& os, const Position& pos);
};
