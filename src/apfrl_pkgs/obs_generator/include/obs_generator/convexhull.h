#include <bits/stdc++.h>
#include <iostream>
//#include <obs_generator/data_structures.h>
#include "data_structures.h"

#include <algorithm>

std::vector<Point> getHull(std::vector<Point>);
bool equalPoint(Point p1, Point p2);
bool comparePoint(const Point &p1, const Point &p2);
int findSide(const Point &p1, const Point &p2, const Point &p);
double lineDist(const Point &p1, const Point &p2, const Point &p);
void quickHull(std::vector<Point> &hull, const std::vector<Point> &a, int n, Point p1, Point p2, int side);
bool onHull(const std::vector<Point> &hull, const std::vector<double> &x, const std::vector<double> &y);
bool onHull(const std::vector<Point> &hull, const std::vector<Point> &points);
// bool equalPoint(Point p1, double p2x, double p2y);
