#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <cmath>
using namespace pros::c;
using namespace pros;

Point::Point() { x = y = 0; }
Point::Point(double x, double y) {
    this->x = x;
    this->y = y;
}
void Point::set(double x, double y) {
    this->x = x;
    this->y = y;
}
Point operator+(const Point& p1, const Point& p2) { return Point(p1.x + p2.x, p1.y + p2.y); }
Point operator-(const Point& p1, const Point& p2) { return Point(p1.x - p2.x, p1.y - p2.y); }
double operator*(const Point& p1, const Point& p2) { return p1.x * p2.x + p1.y * p2.y; }
bool operator<(const Point& p1, const Point& p2) {
    Point p1RotCCW = p1.rotate(1);
    if (p1RotCCW * p2 > 0.001) return true;
    return false;
}
bool operator>(const Point& p1, const Point& p2) {
    Point p1RotCCW = p1.rotate(1);
    if (p1RotCCW * p2 < -0.001) return true;
    return false;
}
double Point::magCross(const Point& p) { return fabs(this->x * p.y - this->y * p.x); }
double Point::mag() const { return sqrt(pow(x, 2.0) + pow(y, 2.0)); }
Point Point::rotate(int dir) const {
    if (dir > 0) {
        return Point(-y, x);
    } else {
        return Point(y, -x);
    }
}
Point Point::abs() {
    Point p(fabs(x), fabs(y));
    return p;
}
Point Point::unit() {
    double m = this->mag();
    return Point(this->x / m, this->y / m);
}

/*
struct P{
    ld x,y;
    P(ld x, ld y):x(x), y(y){}
    P():x(0), y(0){}
    P operator+(const P& p) const { return P(x+p.x, y+p.y); }
    P operator-(const P& p) const { return P(x-p.x, y-p.y); }
    P operator*(ld k) const { return P(x*k, y*k); }
    bool operator <(const P& p) const { return y<p.y || (y==p.y && x<p.x); } //ymin->xmin
};
*/