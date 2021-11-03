#pragma once

#include <vector>

namespace euclidean_clustering
{
// Point datastructure for euclidean clustering
struct Point
{
  double x, y;
  int cluster;
  double minDist;

  // Default constructor
  Point() : x(0.0), y(0.0), cluster(-1), minDist(__DBL_MAX__) {}

  // Constructor with coordinates
  Point(double x, double y) : x(x), y(y), cluster(-1), minDist(__DBL_MAX__) {}

  // Method to calculate the distance between this and another point
  double distance_sq(Point p) const { return (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y); }
};

// Convertor function from polar coordinates to a Point instance
Point fromPolar(double range, double angle);

// Clustering with the assumption that subsequent points are next to one another.
void greedyDistanceClustering(
  std::vector<Point> & points, std::vector<Point> & clusters, double max_dist,
  double max_cluster_size);

}  // namespace euclidean_clustering
