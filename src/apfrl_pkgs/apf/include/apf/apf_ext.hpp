/*
 * Author       : Batuhan Vatan
 * Date         : 11 Oct '24
 * Description  : This package is extended version of the work done by Kemal Bektaş's Thesis
 * APF Formulation is extended to account continuous function generation and each obstacle having a power term(that can be learned)
 */

#ifndef APF_EXT_H
#define APF_EXT_H
#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include "data_structures.h" // Ensure this defines Input, Obstacle, and Velocity

/// @brief Artificial Potential Fields Extended version (2D, relative measurements)
class APF_EXT
{
public:
    APF_EXT(); // Constructor

    /// @brief Computes velocity commands (linear and angular) based on APF.
    ///        All inputs (target, obstacles) are relative to the robot.
    /// @param input  Contains the target, obstacles, and apf_k parameter.
    /// @param output Contains the computed linear and angular velocities.
    void calculateVel(const Input &input, Velocity &output);

    /// @brief Set APF parameters at runtime (from ROS params)
    void setParams(double robot_radius, double max_v, double min_v,
                   double max_w, double min_gamma, double d_th,
                   double neighborhood_radius);

    /// @brief Choose which gradPhi formulation drives the steering angle.
    /// Both describe the same Phi and agree to ~1e-15 rad at the apf_k values
    /// used so far; the stable one is the only one usable at large apf_k.
    void setUseStableGradient(bool on) { kUseStableGradient = on; }

private:
    // Since all measurements are relative, the robot's center is fixed at (0,0)
    const Eigen::Vector2d centerOfRobot{0.0, 0.0};

    // Robot and controller parameters (defaults, overridden by setParams)
    bool kUseStableGradient = true;
    double kRobotRadius = 0.4;
    double kMaxV = 0.8;
    double kMinV = -0.3;
    double kMaxW = M_PI / 3;
    double kMinGamma = 0.02;
    double d_th_goal = 0.02;
    double kNeighborhoodRadius = 3.0;

    // Core functions for potentials and gradients
    double getObstacleTheta(const Obstacle &obstacle);
    double getObstacleRadius(const Obstacle &obstacle);
    double computeBeta(const std::vector<Obstacle> &obstacles, std::vector<double> &b_o);
    double computeGamma(const Input &input);

    std::vector<Obstacle> filterObstacles(const std::vector<Obstacle> &obstacles);

    Eigen::Vector2d computeGradientGamma(const Input &input);
    Eigen::Vector2d computeGradientb_o_i(const Obstacle &obstacle, size_t i);
    // grad(Beta) itself: sum_i (prod_{j!=i} b_o_j) * grad b_o_i. O(N^2), and
    // every term carries the full magnitude of the product. Used only by
    // computeGradPhiAnalytic.
    Eigen::Vector2d computeGradientBeta(const std::vector<Obstacle> &obstacles, const std::vector<double> &b_o);

    // grad(log Beta) = sum_i (grad b_o_i / b_o_i). O(N), and free of the
    // product's magnitude. Used by computeGradPhiStable.
    Eigen::Vector2d computeNormalizedGradBeta(const std::vector<Obstacle> &obstacles, const std::vector<double> &b_o);

    // The navigation-function gradient, two ways. Both describe the same Phi
    // and give the same direction; they differ only in conditioning. See
    // apf_ext.cpp for which is used and why.
    Eigen::Vector2d computeGradPhiAnalytic(double Beta, double Gamma, double k,
                                           const Eigen::Vector2d &gradBeta,
                                           const Eigen::Vector2d &gradGamma);
    Eigen::Vector2d computeGradPhiStable(double Gamma, double k,
                                         const Eigen::Vector2d &normGradBeta,
                                         const Eigen::Vector2d &gradGamma);
};

#endif // APF_EXT_H
