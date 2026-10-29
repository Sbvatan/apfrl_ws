#ifndef OBS_GENERATOR_H
#define OBS_GENERATOR_H

#include "data_structures.h"
#include "ellipse_fit.h"
#include "convexhull.h"
#include <algorithm>
#include "utils.hpp"

class ObsGenerator
{
public:
    /// @brief Generates obstacles using scan data
    /// @param input Scan data, humans and human coef
    /// @param obstacles Output ellipses
    void generate(const Input &input, std::vector<Obstacle> &obstacles);
    void modelObstaclesAsEllipses(const Input &input, std::vector<Obstacle> &obstacles);

    /// @brief Set laser parameters from the LaserScan message
    void setLaserParams(const LaserParams &params);

    // Laser parameters (set at runtime from LaserScan msg)
    double maxRangeReading = 40.0;        // from range_max
    double kLaserScanIncRad = 0.0;        // from angle_increment
    double kLaserMinAngle = -M_PI;        // from angle_min
    double kLaserPanAngleHalfRad = M_PI;  // (angle_max - angle_min) / 2
    size_t scan_size_ = 0;                // from ranges.size()

    static constexpr double kLaserNoiseStd = 0.02;
private:
    /// @brief Finds clusters in the scan data using connected component analysis
    /// @param scan Input scan data
    /// @param points Input points to return cluster as points
    /// @param clusters Output clusters
    void ClusterScan(const std::vector<double> &scan, const std::vector<Point> &points,
                     const double startPan,
                     const double endPan,
                     std::vector<std::vector<Point>> &clusters);
    /// @brief Finds clusters in the scan data using connected component analysis based on two types of
    /// discontinuities
    /// @param scan Input scan data
    /// @param points Input points to return cluster as points
    /// @param points Laser sensor pan angle start in radians
    /// @param points Laser sensor pan angle end in radioans
    /// @param clusters Output clusters

    void ClusterScanWrtDiscontinuities
                    (const std::vector<double> &scan,
                      const std::vector<Point> &points,
                      const double startPan,
                      const double endPan,
                      std::vector<RangeCluster> &clusters);

    void ClusterScanAlperen
        (const std::vector<double> &scan,
         const std::vector<Point> &points,
         const double startPan,
         const double endPan,
         std::vector<RangeCluster> &clusters);

    RangeCluster mergeClusters(RangeCluster cl1, RangeCluster cl2);

    std::vector<RangeCluster> DouglasPeucker(std::vector<Point> PointList, float epsilon);

    double distanceFromLine(Point point1, double A, double B, double C);

    /// @brief Converts scan data to points
    /// @param scan Input scan data
    /// @param points Output points

    void GeneratePoints(const std::vector<double> &scan,  std::vector<Point> &points);


    void MatchToHuman(Obstacle &obs, std::vector<bool> &humans_added, const Input &input);


    // This part needs to be modifed as to be defined simply by the laser sensor
    static constexpr double kClusterDistThreshold = 0.3;
    static constexpr double maxRangeDerivative = 0.03;

    static constexpr unsigned int kMinClusterSize = 3U;


    static constexpr double kMaxHumanEllipseDist = 0.6;
    static constexpr double kHumanEllipseRA = 0.5;
    static constexpr double kHumanEllipseRB = 0.4;

    static constexpr int  kLaserClusterPar = 4;

    static constexpr double minGapDist = 1; // Robot's passage distance;
    static constexpr double lambda = M_PI/18;  // Lambda for Adaptive Breakpoint Detection.
    static constexpr double epsilon = 0.2;  // Lambda for Adaptive Breakpoint Detection.
};
#endif
