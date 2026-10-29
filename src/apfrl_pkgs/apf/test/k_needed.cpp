// For each real frame: how large must apf_k be to bring the steering angle
// within a tolerance of straight-at-target, and how does that scale with the
// number of obstacles that survive filterObstacles (neighborhood_radius)?
//
// Frames can be densified by replicating obstacles with a small jitter, to
// reach counts beyond what a single recording contained. Replication changes
// the geometry, so those rows are marked and used only for the trend.
#include "apf/apf_ext.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

static double dev_at(APF_EXT &a, Input in, double k) {
    in.apf_k = k;
    Velocity v{};
    a.calculateVel(in, v);
    double ang = std::atan2(v.angular / 1.047, v.linear / 0.8);
    double goal = std::atan2(in.target.y, in.target.x);
    double d = std::fabs(ang - goal);
    while (d > M_PI) d = std::fabs(d - 2 * M_PI);
    return d * 180.0 / M_PI;
}

int main(int argc, char **argv) {
    if (argc < 2) { printf("usage: k_needed <csv> [tolerance_deg]\n"); return 1; }
    double tol = (argc > 2) ? atof(argv[2]) : 20.0;
    std::ifstream in(argv[1]);
    std::map<int, Input> frames;
    std::string line;
    while (std::getline(in, line)) {
        std::stringstream ss(line); std::string tok; std::vector<double> f;
        while (std::getline(ss, tok, ',')) f.push_back(atof(tok.c_str()));
        if (f.size() < 7) continue;
        auto &fr = frames[(int)f[0]];
        fr.target.x = f[1]; fr.target.y = f[2];
        Obstacle o{}; o.x = f[3]; o.y = f[4]; o.r_a = f[5]; o.r_b = f[6];
        fr.obstacles.push_back(o);
    }
    std::mt19937 rng(5);
    std::normal_distribution<double> jit(0.0, 0.08);

    printf("  tolerance: steering within %.0f deg of the target\n", tol);
    printf("  %8s %10s %12s %8s\n", "N in 3m", "frames", "median k", "source");
    std::map<int, std::vector<double>> byN;
    for (int mult = 1; mult <= 4; ++mult) {
        for (auto &kv : frames) {
            Input fr = kv.second;
            if (mult > 1) {
                size_t base = fr.obstacles.size();
                for (int m = 1; m < mult; ++m)
                    for (size_t i = 0; i < base; ++i) {
                        Obstacle o = fr.obstacles[i];
                        o.x += jit(rng); o.y += jit(rng);
                        fr.obstacles.push_back(o);
                    }
            }
            int nin = 0;
            for (auto &o : fr.obstacles)
                if (std::hypot(o.x, o.y) <= 3.0) ++nin;
            if (nin == 0) continue;
            APF_EXT a; a.setParams(0.4, 0.8, -0.3, 1.047, 0.02, 0.01, 3.0);
            a.setUseStableGradient(true);
            double lo = 1.0, hi = 1e5;
            if (dev_at(a, fr, hi) > tol) { byN[(nin / 10) * 10].push_back(-1); continue; }
            for (int it = 0; it < 40; ++it) {
                double mid = std::sqrt(lo * hi);
                if (dev_at(a, fr, mid) > tol) lo = mid; else hi = mid;
            }
            byN[(nin / 10) * 10].push_back(hi);
        }
    }
    for (auto &kv : byN) {
        auto v = kv.second;
        std::sort(v.begin(), v.end());
        printf("  %5d-%-3d %10zu %12.1f %8s\n", kv.first, kv.first + 9, v.size(),
               v[v.size() / 2], kv.first > 60 ? "replic." : "real");
    }
    return 0;
}
