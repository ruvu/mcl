#include "euclidean_clustering.hpp"

#include <math.h>
#include <ruvu_mcl_msgs/LandmarkList.h>
#include <sensor_msgs/LaserScan.h>

#include <queue>
#include <vector>

namespace euclidean_clustering
{
Point fromPolar(double range, double angle)
{
  return Point(range * std::cos(angle), range * std::sin(angle));
}

std::vector<Point> fromLaserScan(const sensor_msgs::LaserScan & scan)
{
  std::vector<Point> points;
  points.resize(scan.ranges.size());
  for (int i = 0; i < scan.ranges.size(); i++) {
    if (!(scan.range_min <= scan.ranges[i] && scan.ranges[i] <= scan.range_max)) continue;

    points[i] = fromPolar(scan.ranges[i], scan.angle_min + i * scan.angle_increment);
  }
  return points;
}

void greedyDistanceClustering(
  std::vector<Point> & points, std::vector<Point> & centroids, double max_dist,
  double max_cluster_size)
{
  if (!points.size()) return;

  points[0].cluster = 0;

  if (points.size() == 1) return;

  int num_clusters = 1;
  double max_dist_sq = max_dist * max_dist;
  // Assign a cluster id to every point. Increment the cluster id when distance to last point is too large.
  for (int i = 1; i < points.size(); ++i) {
    if (points[i].distance_sq(points[i - 1]) > max_dist_sq) {
      num_clusters++;
    }
    points[i].cluster = num_clusters - 1;
  }

  // Mark clusters above max_cluster_size for deletion
  // We're not removing the clusters yet, because this would invalidate the cluster indices we just so carefully
  // assigned to every point, and we still need those.
  Point first_cluster_point;
  double max_size_sq = max_cluster_size * max_cluster_size;
  std::vector<int> clusters_to_remove;
  for (auto point : points) {
    if (first_cluster_point.cluster != point.cluster) {
      first_cluster_point = point;
    } else if (clusters_to_remove.size() && clusters_to_remove.back() == point.cluster) {
      continue;
    } else if (first_cluster_point.distance_sq(point) > max_size_sq) {
      clusters_to_remove.push_back(point.cluster);
    }
  }

  // Calculate cluster centroids
  std::vector<int> nPoints(num_clusters, 0);
  std::vector<double> sumX(num_clusters, 0.), sumY(num_clusters, 0.);

  for (auto & point : points) {
    nPoints[point.cluster]++;
    sumX[point.cluster] += point.x;
    sumY[point.cluster] += point.y;
  }

  centroids.resize(num_clusters);
  for (int clusterId = 0; clusterId < num_clusters; ++clusterId) {
    centroids[clusterId].x = sumX[clusterId] / nPoints[clusterId];
    centroids[clusterId].y = sumY[clusterId] / nPoints[clusterId];
  }

  // Remove clusters marked for deletion
  for (auto it = clusters_to_remove.rbegin(); it != clusters_to_remove.rend(); it++) {
    centroids.erase(centroids.begin() + *it);
  }
}

}  // namespace euclidean_clustering
