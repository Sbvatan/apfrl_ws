// Replays real /obstacles frames through calculateVel and sweeps apf_k.
//
// The synthetic version of this got the answer wrong: sampling obstacle centres
// uniformly around the robot put ~4 per frame inside the b_o clamp (r below
// robot radius + obstacle radius), and a clamped term is ~130x a normal one, so
// the sum was dominated by obstacles the robot was standing inside. Those are
// collisions, not navigation states. Real frames do not contain them.
//
// Input CSV: msg_id, goal_x, goal_y, obs_x, obs_y, r_a, r_b  (odom/base frame)
#include "apf/apf_ext.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>

int main(int argc, char **argv) {
    if (argc < 2) { printf("usage: k_sweep <obstacles.csv>\n"); return 1; }
    std::ifstream in(argv[1]);
    if (!in) { printf("cannot open %s\n", argv[1]); return 1; }

    std::map<int, Input> frames;
    std::string line;
    while (std::getline(in, line)) {
        std::stringstream ss(line); std::string tok;
        std::vector<double> f;
        while (std::getline(ss, tok, ',')) f.push_back(atof(tok.c_str()));
        if (f.size() < 7) continue;
        int id = (int)f[0];
        auto &fr = frames[id];
        fr.target.x = f[1]; fr.target.y = f[2];
        Obstacle o{}; o.x = f[3]; o.y = f[4]; o.r_a = f[5]; o.r_b = f[6];
        fr.obstacles.push_back(o);
    }
    printf("  %zu real frames loaded from %s\n", frames.size(), argv[1]);
    if (frames.empty()) return 1;

    size_t tot = 0, clamped = 0;
    for (auto &kv : frames) {
        for (auto &o : kv.second.obstacles) {
            double rho = (o.r_a + o.r_b) / 2.0;
            double b = o.x * o.x + o.y * o.y - (0.4 + rho) * (0.4 + rho);
            ++tot; if (b < 0.01) ++clamped;
        }
    }
    printf("  %zu obstacles, %zu inside the b_o clamp (%.1f%%)\n\n",
           tot, clamped, 100.0 * clamped / tot);

    const double ks[] = {1, 1.5, 2, 5, 10, 20, 50, 100, 300, 1000, 3000, 10000};
    printf("  %8s %10s %12s %12s\n", "apf_k", "mean dev", "median dev", "frames >5 deg");
    printf("  %8s %10s %12s %12s\n", "", "from goal", "from goal", "off k=1");
    std::vector<double> base;
    for (double k : ks) {
        double sum = 0; std::vector<double> devs; size_t i = 0, moved = 0;
        for (auto &kv : frames) {
            Input in2 = kv.second; in2.apf_k = k;
            Velocity v{};
            APF_EXT a; a.setParams(0.4, 0.8, -0.3, 1.047, 0.02, 0.01, 3.0);
            a.setUseStableGradient(true); a.calculateVel(in2, v);
            double ang = std::atan2(v.angular / 1.047, v.linear / 0.8);
            double goal = std::atan2(in2.target.y, in2.target.x);
            double d = std::fabs(ang - goal);
            while (d > M_PI) d = std::fabs(d - 2 * M_PI);
            d = d * 180.0 / M_PI;
            sum += d; devs.push_back(d);
            if (base.size() > i && std::fabs(d - base[i]) > 5.0) ++moved;
            ++i;
        }
        std::sort(devs.begin(), devs.end());
        if (base.empty()) base = devs;   // note: sorted, used only as a coarse guard
        printf("  %8.0f %9.1fd %11.1fd %12zu\n", k, sum / frames.size(),
               devs[devs.size() / 2], moved);
    }
    return 0;
}
