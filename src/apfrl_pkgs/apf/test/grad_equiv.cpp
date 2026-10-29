// Sweeps apf_k and compares the two gradPhi formulations on identical inputs,
// through the real calculateVel path. Same experiment as the Python assessment,
// run against the code that actually ships.
#include "apf/apf_ext.hpp"
#include <cstdio>
#include <random>
#include <cmath>

static double angdiff(double a, double b) {
    double d = std::fabs(a - b);
    while (d > M_PI) d = std::fabs(d - 2 * M_PI);
    return d;
}

int main() {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> ux(-2.8, 2.8), ur(0.15, 0.5);
    printf("  %5s %8s   %s\n", "N", "apf_k", "max |angle(analytic) - angle(stable)| over 40 draws");
    for (int N : {10, 25, 50, 96, 116}) {
        for (double k : {1.0, 3.0, 5.0, 10.0, 20.0, 35.0, 50.0}) {
            double worst = 0.0;
            for (int t = 0; t < 40; ++t) {
                Input in; in.apf_k = k;
                in.target.x = ux(rng) * 2.0; in.target.y = ux(rng) * 2.0;
                for (int i = 0; i < N; ++i) {
                    Obstacle o{}; o.x = ux(rng); o.y = ux(rng);
                    o.r_a = ur(rng); o.r_b = ur(rng) * 0.8;
                    in.obstacles.push_back(o);
                }
                Velocity va{}, vs{};
                APF_EXT a; a.setParams(0.4, 0.8, -0.3, 1.047, 0.02, 0.01, 3.0);
                a.setUseStableGradient(false); a.calculateVel(in, va);
                APF_EXT b; b.setParams(0.4, 0.8, -0.3, 1.047, 0.02, 0.01, 3.0);
                b.setUseStableGradient(true);  b.calculateVel(in, vs);
                // recover the steering angle from the velocity pair
                double aa = std::atan2(vs.angular / 1.047, va.linear / 0.8);
                double ang_a = std::atan2(va.angular / 1.047, va.linear / 0.8);
                double ang_s = std::atan2(vs.angular / 1.047, vs.linear / 0.8);
                (void)aa;
                worst = std::max(worst, angdiff(ang_a, ang_s));
            }
            printf("  %5d %8.1f   %.3e rad  (%.4f deg)%s\n", N, k, worst,
                   worst * 180.0 / M_PI, worst > 0.0175 ? "   <-- analytic unreliable" : "");
        }
    }

    // ── how much steering authority apf_k actually has, stable form ────────
    // The stable gradient is gradGamma - (Gamma/k) * sum(grad b_i / b_i), so k
    // divides the repulsive term: small k avoids hard, large k drives straight
    // at the goal. This measures where that transition sits, which is what
    // decides a useful action range -- there is no arithmetic ceiling any more.
    printf("\n  deviation of the steering angle from the straight-to-goal direction\n");
    printf("  (0 deg = ignores obstacles entirely; large = avoidance dominates)\n");
    printf("  %5s %7s %7s %7s %7s %7s %7s %7s %7s\n", "N", "goal m",
           "k=1", "k=10", "k=1e2", "k=1e3", "k=1e4", "k=1e5", "k=1e6");
    for (int N : {10, 50, 96}) {
        for (double gd : {2.0, 5.0, 10.0}) {
            double dev[7] = {0, 0, 0, 0, 0, 0, 0};
            int trials = 30;
            for (int t = 0; t < trials; ++t) {
                Input in;
                in.target.x = gd; in.target.y = 0.0;
                for (int i = 0; i < N; ++i) {
                    Obstacle o{}; o.x = ux(rng); o.y = ux(rng);
                    o.r_a = ur(rng); o.r_b = ur(rng) * 0.8;
                    in.obstacles.push_back(o);
                }
                double ks[7] = {1.0, 10.0, 1e2, 1e3, 1e4, 1e5, 1e6};
                for (int m = 0; m < 7; ++m) {
                    in.apf_k = ks[m];
                    Velocity v{};
                    APF_EXT a; a.setParams(0.4, 0.8, -0.3, 1.047, 0.02, 0.01, 3.0);
                    a.setUseStableGradient(true); a.calculateVel(in, v);
                    double ang = std::atan2(v.angular / 1.047, v.linear / 0.8);
                    dev[m] += std::fabs(ang) * 180.0 / M_PI / trials;  // goal is at angle 0
                }
            }
            printf("  %5d %7.1f %6.1f%s %6.1f%s %6.1f%s %6.1f%s %6.1f%s %6.1f%s %6.1f%s\n",
                   N, gd, dev[0], "d", dev[1], "d", dev[2], "d", dev[3], "d",
                   dev[4], "d", dev[5], "d", dev[6], "d");
        }
    }
    return 0;
}
