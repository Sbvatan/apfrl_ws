//#include "obs_generator/data_structures.h"
//#include <obs_generator/convexhull.h>

#include "obs_generator/data_structures.h"
#include "obs_generator/convexhull.h"

// Stores the result (points of convex hull)
#define NUM_MID_POINTS 2
#define MAX_MID_POINT_DIST 3

bool comparePoint(const Point &p1, const Point &p2)
{
  return p1.x * p1.x + p1.y * p1.y > p2.x * p2.x + p2.y * p2.y;
}

bool equalPoint(Point p1, Point p2)
{
  if (p1.x == p2.x && p1.y == p2.y)
    return true;
  return false;
}

// Returns the side of point p with respect to line
// joining points p1 and p2.

int findSide(const Point &p1, const Point &p2, const Point &p)
{
  double val = (p.y - p1.y) * (p2.x - p1.x) - (p2.y - p1.y) * (p.x - p1.x);

  if (val > 0)
    return 1;
  else if (val < 0)
    return -1;
  return 0;
}

// returns a value proportional to the distance
// between the point p and the line joining the
// points p1 and p2
double lineDist(const Point &p1, const Point &p2, const Point &p)
{
  return abs((p.y - p1.y) * (p2.x - p1.x) - (p2.y - p1.y) * (p.x - p1.x));
}

// End points of line L are p1 and p2.  side can have value
// 1 or -1 specifying each of the parts made by the line L
void quickHull(std::vector<Point> &hull, const std::vector<Point> &a, int n,
               Point p1, Point p2, int side)
{
  int ind = -1;
  int max_dist = 0;

  // finding the point with maximum distance
  // from L and also on the specified side of L.
  for (int i = 0; i < n; i++)
  {
    double temp = lineDist(p1, p2, a[i]);
    if (findSide(p1, p2, a[i]) == side && temp > max_dist)
    {
      ind = i;
      max_dist = temp;
    }
  }

  // If no point is found, add the end points
  // of L to the convex hull.
  if (ind == -1)
  {
    hull.push_back(p1);
    hull.push_back(p2);
    return;
  }

  // Recur for the two parts divided by a[ind]
  quickHull(hull, a, n, a[ind], p1, -findSide(a[ind], p1, p2));
  quickHull(hull, a, n, a[ind], p2, -findSide(a[ind], p2, p1));
}
// TODO: Add points around nearest point on hull to not miss it!
std::vector<Point> getHull(std::vector<Point> points)
{
  std::vector<Point> hull;
  unsigned int n = points.size();
  // points[i].second -> y-coordinate of the ith point
  if (n < 3)
  {
    std::cout << "Convex hull not possible\n";
    return hull;
  }

  // Finding the point with minimum and
  // maximum x-coordinate
  int min_x = 0, max_x = 0;
  for (int i = 1; i < n; i++)
  {
    if (points[i].x < points[min_x].x)
      min_x = i;
    if (points[i].x > points[max_x].x)
      max_x = i;
  }

  // Recursively find convex hull points on
  // one side of line joining points[min_x] and
  // points[max_x]
  quickHull(hull, points, n, points[min_x], points[max_x], 1);

  // Recursively find convex hull points on
  // other side of line joining points[min_x] and
  // points[max_x]
  quickHull(hull, points, n, points[min_x], points[max_x], -1);

  sort(hull.begin(), hull.end(), comparePoint);
  auto unique_end = std::unique(hull.begin(), hull.end(), equalPoint);
  hull.erase(unique_end, hull.end());
  // To not miss nearest ellipse point
  //  std::cout <<"===============" << std::endl;
  //  for (auto p: hull){
  //    std::cout << p.x << " " << p.y << std::endl;
  //  }
  auto len = hull.size();
  for (int i = -NUM_MID_POINTS; i < NUM_MID_POINTS; i++)
  {
    auto x = (hull[len - 1].x + hull[len - 2].x) / 2 + i * 0.01;
    auto y = (hull[len - 1].y + hull[len - 2].y) / 2;
    if (x * x + y * y > MAX_MID_POINT_DIST * MAX_MID_POINT_DIST)
    {
      break;
    }
    //hull.emplace_back(x, y);
  }
  return hull;
  /*
while (!hull.empty())
{
  cout << "(" <<( *hull.begin()).x << ", "
       << (*hull.begin()).y << ") ";
  hull.erase(hull.begin());
}
  */
}

bool onHull(const std::vector<Point> &hull, const std::vector<double> &x,
            const std::vector<double> &y)
{
  for (auto &hull_point : hull)
  {
    for (int j = 0; j < x.size(); j++)
    {
      if (hull_point.x == x[j] && hull_point.y == y[j])
      {
        return true;
      }
    }
  }
  return false;
}

bool onHull(const std::vector<Point> &hull, const std::vector<Point> &points)
{
  for (auto hull_point : hull)
  {
    for (auto point : points)
    {
      if (hull_point == point)
      {
        return true;
      }
    }
  }
  return false;
}
