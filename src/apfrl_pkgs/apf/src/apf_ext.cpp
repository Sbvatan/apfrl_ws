/*
 * Author       : Batuhan Vatan
 * Date         : 11 Oct '24
 * Description  : This package is extended version of the work done by Kemal Bektaş's Thesis
 * APF Formulation is extended to account continuous function generation and each obstacle having a power term(that can be learned)
 */

#include "apf/apf_ext.hpp"

// Constructor (nothing special needed)
APF_EXT::APF_EXT() {}

void APF_EXT::setParams(double robot_radius, double max_v, double min_v,
                        double max_w, double min_gamma, double d_th,
                        double neighborhood_radius)
{
    kRobotRadius = robot_radius;
    kMaxV = max_v;
    kMinV = min_v;
    kMaxW = max_w;
    kMinGamma = min_gamma;
    d_th_goal = d_th;
    kNeighborhoodRadius = neighborhood_radius;
}

// -------------------- Obstacle Helper Functions --------------------

// getObstacleTheta: Computes an effective angle from the robot (at 0,0) to the obstacle,
// adjusted by the obstacle’s orientation (obstacle.yaw).
double APF_EXT::getObstacleTheta(const Obstacle &obstacle)
{
    // With robot at (0,0): angle from robot to obstacle is atan2(obstacle.y, obstacle.x)
    return atan2(obstacle.y, obstacle.x) - obstacle.yaw;
}

// getObstacleRadius: Computes the effective obstacle “radius” along the direction defined by getObstacleTheta.
double APF_EXT::getObstacleRadius(const Obstacle &obstacle)
{
    double theta = getObstacleTheta(obstacle);
    double denominator = sqrt(pow(obstacle.r_a, 2) * pow(sin(theta), 2) +
                              pow(obstacle.r_b, 2) * pow(cos(theta), 2));
    const double epsilon = 1e-6;
    denominator = std::max(denominator, epsilon);
    return (obstacle.r_a * obstacle.r_b) / denominator;
}

// -------------------- Potential Functions --------------------

// computeBeta: Computes the product of individual obstacle functions (b_o).
// For the outer obstacle (i == 0) we define:
//   b_o = ((rho - kRobotRadius)² - r²)
// For inner obstacles (i >= 1) we define:
//   b_o = (r² - (kRobotRadius + rho)²)
// where r is the squared distance from the robot (at 0,0) to the obstacle.
double APF_EXT::computeBeta(const std::vector<Obstacle> &obstacles, std::vector<double> &b_o)
{
    size_t numObs = obstacles.size();

    double Beta = 1.0;
    b_o.resize(numObs);
    for (size_t i = 0; i < numObs; i++)
    {
        double rho = getObstacleRadius(obstacles[i]);
        // Relative position (robot is at 0,0, so this is just the obstacle coordinates)
        Eigen::Vector2d rel(obstacles[i].x, obstacles[i].y);
        double r2 = rel.squaredNorm();
        double temp;
        if (i == 0) // Outer obstacle (workspace boundary)
        {
            temp = pow((rho - kRobotRadius), 2) - r2;
        }
        else // Inner obstacle (collision avoidance)
        {
            temp = r2 - pow((kRobotRadius + rho), 2);
        }
        // Clamp to avoid near-zero values.
        b_o[i] = std::max(temp, 0.01);
        Beta *= b_o[i];
    }

    return Beta;
}

// computeGamma: Computes the attractive potential as the squared distance from the robot to the target.
// Since the robot is at (0,0), this is simply target.x² + target.y².
double APF_EXT::computeGamma(const Input &input)
{
    Eigen::Vector2d target(input.target.x, input.target.y);
    double dist = target.norm();
    if (dist >= d_th_goal)
        return pow(dist, 2);
    else
        return 0;
}

// -------------------- Gradient Functions --------------------

// computeGradientGamma: Computes the gradient of the attractive potential.
// With U_att = ||target||², we set ∇U_att = 2·target.
// (Using a positive gradient here ensures that later when the combined
// term is “inverted” to yield a command the robot moves toward the target.)
Eigen::Vector2d APF_EXT::computeGradientGamma(const Input &input)
{
    Eigen::Vector2d target(input.target.x, input.target.y);
    return 2.0 * target;
}

// computeGradientb_o_i: Computes the gradient of the i-th obstacle function.
// With the robot at (0,0), the term (centerOfRobot - o_i) becomes -o_i.
Eigen::Vector2d APF_EXT::computeGradientb_o_i(const Obstacle &obstacle, size_t i)
{
    // In the relative frame the obstacle position is o_i.
    Eigen::Vector2d o_i(obstacle.x, obstacle.y);
    // For the term that comes from differentiating (x^2+y^2),
    // the gradient is 2*o_i (not -2*o_i) because d/dx(x^2) = 2x.
    Eigen::Vector2d gradC = 2.0 * o_i;

    // Compute the effective angle as before.
    double theta = getObstacleTheta(obstacle);

    // Compute the derivative of the effective radius with respect to theta.
    double numerator = obstacle.r_a * obstacle.r_b * (pow(obstacle.r_a, 2) - pow(obstacle.r_b, 2)) *
                       sin(theta) * cos(theta);
    double denominator = pow((pow(obstacle.r_a, 2) * pow(sin(theta), 2) +
                              pow(obstacle.r_b, 2) * pow(cos(theta), 2)),
                             1.5);
    double gradRho_wrt_Theta = -(numerator / denominator);

    // The gradient of theta with respect to (x,y). (Here, we use the fact that the robot is at (0,0).)
    double temp_norm = o_i.squaredNorm();
    // Guard against division by zero (if an obstacle is exactly at the origin—unlikely but good practice)
    if (temp_norm < 1e-6)
        temp_norm = 1e-6;
    Eigen::Vector2d gradTheta;
    gradTheta << -obstacle.y / temp_norm, obstacle.x / temp_norm;

    // Combined, the gradient of the effective radius with respect to (x,y)
    Eigen::Vector2d gradRho = gradRho_wrt_Theta * gradTheta;

    // Now, choose the sign depending on whether the obstacle is "outer" (workspace) or "inner":
    if (i == 0)
    {
        // For the outer obstacle, the original formulation was:
        //    grad = -gradC + 2*(getObstacleRadius(obstacle) - kRobotRadius)*gradRho
        // This causes a force that pushes the robot _inward_ when near the workspace boundary.
        return -gradC + 2.0 * (getObstacleRadius(obstacle) - kRobotRadius) * gradRho;
    }
    else
    {
        // For inner obstacles, we use:
        //    grad = gradC - 2*(kRobotRadius + getObstacleRadius(obstacle))*gradRho
        // This gives a repulsive term that points away from the obstacle.
        return gradC - 2.0 * (kRobotRadius + getObstacleRadius(obstacle)) * gradRho;
    }
}

// computeGradientBeta: grad(Beta), as the sum over obstacles of that
// obstacle's gradient weighted by the product of all the others. This is the
// direct analytical form. Kept because computeGradPhiAnalytic needs it and
// because it is the expression the derivation is written against; it is not on
// the path the planner runs.
Eigen::Vector2d APF_EXT::computeGradientBeta(const std::vector<Obstacle> &obstacles, const std::vector<double> &b_o)
{
    Eigen::Vector2d gradBeta(0, 0);
    size_t numObs = obstacles.size();
    for (size_t i = 0; i < numObs; i++)
    {
        double product_b_o = 1.0;
        for (size_t j = 0; j < numObs; j++)
        {
            if (j != i)
                product_b_o *= b_o[j];
        }
        gradBeta += product_b_o * computeGradientb_o_i(obstacles[i], i);
    }
    return gradBeta;
}

// computeNormalizedGradBeta: sum_i (grad b_o_i / b_o_i).
//
// This is grad(log Beta), not grad(Beta). The two differ by a factor of Beta,
// which cancels out of the steering direction (see the gradPhi assembly below),
// so the planner never needs Beta itself.
//
// The previous form built grad(Beta) directly, as sum_i (prod_{j!=i} b_o_j) *
// grad b_o_i. That is O(N^2), and every term carries the full magnitude of the
// product. With the 50-96 obstacles the laser generator actually produces,
// Beta reaches e^75 to e^144 and the two large terms of the gradPhi numerator
// then cancel: measured against this stable form, the resulting *angle* is more
// than a degree wrong above apf_k ~ 24 at 50 obstacles, and above apf_k ~ 11 at
// 10 obstacles, degrading to floating-point noise beyond that. That is what
// capped apf_k, not the topology.
//
// b_o is clamped to >= 0.01 in computeBeta, so the division is safe.
Eigen::Vector2d APF_EXT::computeNormalizedGradBeta(const std::vector<Obstacle> &obstacles, const std::vector<double> &b_o)
{
    Eigen::Vector2d sum(0, 0);
    size_t numObs = obstacles.size();
    for (size_t i = 0; i < numObs; i++)
    {
        sum += computeGradientb_o_i(obstacles[i], i) / b_o[i];
    }
    return sum;
}

// filterObstacles: Returns only obstacles whose elliptical boundary intersects
// the robot's neighborhood circle (radius kNeighborhoodRadius centered at 0,0).
// The first obstacle (index 0, workspace boundary) is always kept.
std::vector<Obstacle> APF_EXT::filterObstacles(const std::vector<Obstacle> &obstacles)
{
    std::vector<Obstacle> filtered;
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        // Always keep the workspace boundary (index 0)
        if (i == 0)
        {
            filtered.push_back(obstacles[i]);
            continue;
        }

        // Distance from robot (0,0) to obstacle center
        double dist_to_center = std::hypot(obstacles[i].x, obstacles[i].y);

        // Effective ellipse radius along the direction from robot to obstacle
        double eff_radius = getObstacleRadius(obstacles[i]);

        // Closest point on the ellipse boundary (along center-to-center line)
        // is approximately at distance: dist_to_center - eff_radius
        // The ellipse intersects the neighborhood if this distance <= kNeighborhoodRadius
        if (dist_to_center - eff_radius <= kNeighborhoodRadius)
        {
            filtered.push_back(obstacles[i]);
        }
    }
    return filtered;
}

// -------------------- Main APF Calculation --------------------

// calculateVel: Computes the velocity command based on the APF formulation.
// Since every measurement is relative to the robot (assumed at (0,0)),
// we compute a desired heading from the combined potential field and then map it to
// linear and angular velocity commands.
// -------------------- Main APF Calculation --------------------

// calculateVel: Computes the velocity command based on the APF formulation.
// Since every measurement is relative to the robot (assumed at (0,0)),
// we compute a desired heading from the combined potential field and then map it to
// linear and angular velocity commands.
// ── grad Phi, two ways ──────────────────────────────────────────────────────
// Phi = Gamma / (Gamma^k + Beta)^(1/k). Both functions below return a vector
// with the same direction; only the direction is used (ComputeVelocity takes
// atan2 of it and never the magnitude), so they are interchangeable there.

// The analytical expression, written straight from the derivative of Phi.
// Correct as mathematics, and poorly conditioned as arithmetic: it evaluates
// Beta and Gamma^k explicitly, and with the 50-96 obstacles the laser generator
// produces Beta runs from e^75 to e^144. The two terms of the numerator then
// differ from their difference by many orders of magnitude, and the resulting
// angle is more than a degree wrong above apf_k ~ 24 at 50 obstacles, ~11 at 10
// obstacles, decaying to floating-point noise past that. Kept for reference and
// for the equivalence test; not what the planner runs.
Eigen::Vector2d APF_EXT::computeGradPhiAnalytic(double Beta, double Gamma, double k,
                                                const Eigen::Vector2d &gradBeta,
                                                const Eigen::Vector2d &gradGamma)
{
    const double epsilon = 1e-6;
    double A = Beta + pow(Gamma, k);
    Eigen::Vector2d gradBeta_plus_gamma_k = gradBeta + k * pow(Gamma, (k - 1)) * gradGamma;
    Eigen::Vector2d gradBeta_plus_gamma_k_to_one_over_k =
        (1.0 / k) * pow(A, ((1 - k) / k)) * gradBeta_plus_gamma_k;
    double denom = pow(A, (2.0 / k));
    if (fabs(denom) < epsilon)
        denom = epsilon;
    return ((pow(A, (1.0 / k)) * gradGamma) -
            (Gamma * gradBeta_plus_gamma_k_to_one_over_k)) / denom;
}

// The same gradient, rearranged so nothing large is ever formed. Expanding the
// numerator above, the two Gamma^k gradGamma terms cancel exactly and
// A^((1-k)/k) factors out, leaving Beta * gradGamma - (Gamma/k) * gradBeta.
// Substituting gradBeta = Beta * normGradBeta lets Beta factor out as well.
// Dropping those positive scalars changes the magnitude and not the angle, so
// this is exact for the planner's purposes. Neither Beta nor Gamma^k appears,
// so there is nothing to overflow and nothing to cancel, at any apf_k.
Eigen::Vector2d APF_EXT::computeGradPhiStable(double Gamma, double k,
                                              const Eigen::Vector2d &normGradBeta,
                                              const Eigen::Vector2d &gradGamma)
{
    return gradGamma - (Gamma / k) * normGradBeta;
}
void APF_EXT::calculateVel(const Input &input, Velocity &output)
{

    // Compute the obstacle product and attractive potential.
    std::vector<double> b_o;

    // Filter obstacles: only keep those intersecting the robot's neighborhood
    std::vector<Obstacle> nearby_obstacles = filterObstacles(input.obstacles);

    RCLCPP_DEBUG(rclcpp::get_logger("apf"), "Obstacle count %zu (filtered from %zu)",
                 nearby_obstacles.size(), input.obstacles.size());

    double Beta = computeBeta(nearby_obstacles, b_o);
    double Gamma = computeGamma(input);

    // Clamp Gamma to a minimum positive value to avoid division by zero or log(0)
    const double epsilon = 1e-6;
    if (Gamma < epsilon)
        Gamma = epsilon;

    // Compute gradients.
    Eigen::Vector2d gradGamma = computeGradientGamma(input);
    Eigen::Vector2d normGradBeta = computeNormalizedGradBeta(nearby_obstacles, b_o);

    Eigen::Vector2d gradPhi;
    if (kUseStableGradient)
    {
        gradPhi = computeGradPhiStable(Gamma, input.apf_k, normGradBeta, gradGamma);
    }
    else
    {
        // The analytic form loses the steering angle once apf_k climbs past
        // roughly 0.35 * (obstacle count) -- measured against the stable form
        // over 10 to 116 obstacles. Past that it does not fail loudly: it
        // returns a confident-looking direction computed from cancellation
        // noise. Warn rather than switch, so the choice stays with the caller.
        double k_safe = 0.35 * static_cast<double>(nearby_obstacles.size());
        if (input.apf_k > k_safe)
        {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("apf"),
                             "apf_k=%.1f exceeds ~%.1f, the range the analytic gradient "
                             "holds with %zu obstacles; its steering angle is unreliable "
                             "above that. Set use_stable_gradient:=true.",
                             input.apf_k, k_safe, nearby_obstacles.size());
        }
        Eigen::Vector2d gradBeta = computeGradientBeta(nearby_obstacles, b_o);
        gradPhi = computeGradPhiAnalytic(Beta, Gamma, input.apf_k, gradBeta, gradGamma);
    }

    // Compute the desired heading angle from gradPhi.
    double angle_phi = atan2(gradPhi.y(), gradPhi.x());

    RCLCPP_DEBUG(rclcpp::get_logger("apf"), "Beta: %.4f, Gamma: %.4f, gradPhi: (%.4f, %.4f), angle_phi: %.4f", Beta, Gamma, gradPhi.x(), gradPhi.y(), angle_phi);
    // Map this desired direction to velocity commands.
    if (Gamma > kMinGamma)
    {
        output.linear = kMaxV * cos(angle_phi);
        output.angular = kMaxW * sin(angle_phi);
    }
    else
    {
        output.linear = 0.0;
        output.angular = 0.0;
    }

    // Final NaN check - if any computation resulted in NaN, output zero velocity
    if (std::isnan(output.linear) || std::isnan(output.angular))
    {
        RCLCPP_WARN(rclcpp::get_logger("apf"), "NaN detected in velocity output, setting to zero");
        output.linear = 0.0;
        output.angular = 0.0;
    }
}
