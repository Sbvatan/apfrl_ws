#include "apf/apf.h"
#include <iostream>

APF::APF(double xScale) : kXscale(xScale) {}
void APF::CalculateVel(const Input &input, Velocity &output)
{
  // TODO: COMPOSITIONAL FUNCTION MUST BE IMPLEMENTED! LOOK SOCIAL APF-RL PAPER FOR REFERENCE
  std::vector<double> Bs{};

  double B{1.0};
  double Bmin{10.0};
  size_t numObs = input.obstacles.size();
  double denomx{}, denomy{}, dx{}, dy{}, oth{}, obs_r{}, dth{}, dtx{}, dty{},
      dbx{}, dby{};

  for (size_t i = 0U; i < numObs; i++)
  {
    dx = -input.obstacles[i].x;
    dy = -input.obstacles[i].y;
    // Skip obstacles with zero relative distance: dtx/dty = -dy/(dx²+dy²) and
    // dx/(dx²+dy²) become 0/0 = NaN, which propagates to atan2 and cmd_vel.
    // Author flagged this as `## Fix` at the dtx/dty assignment.
    if (std::abs(dx) < 1e-9 && std::abs(dy) < 1e-9) {
      // Add a neutral entry so denominator B-product stays well-defined.
      Bs.push_back(Beta_min_);
      B *= Beta_min_;
      continue;
    }
    auto r_a = input.obstacles[i].r_a;
    auto r_b = input.obstacles[i].r_b;
    oth = atan2(dy, dx) - input.obstacles[i].yaw;
    //Using min distance to ellipse formulas
    if (i == 0U)
    { // Don't apply for outer boundary
      r_a /= kOuterSafetyFactor;
      r_b /= kOuterSafetyFactor;
    }
    else
    {
    r_a *= kSafetyFactor;
    r_b *= kSafetyFactor;
     }
    obs_r =
        r_a * r_b /
        sqrt(pow(r_a, 2) * pow(sin(oth), 2) + pow(r_b, 2) * pow(cos(oth), 2));
    dth = r_a * r_b * sin(oth) * cos(oth) * (pow(r_a, 2) - pow(r_b, 2)) /
          pow(pow(r_a, 2) * pow(sin(oth), 2) + pow(r_b, 2) * pow(cos(oth), 2),
              1.5);

    dtx = -dy / (pow(dx, 2) + pow(dy, 2));
    dty = dx / (pow(dx, 2) + pow(dy, 2));

    //!!! dx=0  dy=0  dtx and dty = nan !!!                 ##Fix

    if (i == 0U)
    {
      double Bsi = -pow(dx, 2) - pow(dy, 2) + pow(obs_r - kRobotRadius, 2);
      Bsi = std::max(Bsi, Beta_min_);
      if (std::abs(Bsi) < std::abs(Bmin))
      {
        Bmin = std::abs(Bsi);
      }
      Bs.push_back(Bsi);
      dbx = 2.0 * dx - 2 * (obs_r - kRobotRadius) * dth * dtx;
      dby = 2.0 * dy - 2 * (obs_r - kRobotRadius) * dth * dty; 
      denomx += dbx / Bs[i];
      denomy += dby / Bs[i];
      //std::cout<< "dx" << dx <<" dy: "<<dy<<" dth: "<<dth<<" dtx: " << dtx <<" dty: "<<dty << " dbx: " << dbx <<" dby: "<<dby<<" Bsi: "<<Bs[i]<<std::endl;
    }
    else
    {
    double Bsi = pow(dx, 2) + pow(dy, 2) - pow(obs_r + kRobotRadius, 2);
    Bsi = std::max(Bsi, Beta_min_);
    if (std::abs(Bsi) < std::abs(Bmin))
    {
      Bmin = std::abs(Bsi);
    }
    Bs.push_back(Bsi);
    dbx = 2 * (dx - (obs_r + kRobotRadius) * dth * dtx);
    dby = 2 * (dy - (obs_r + kRobotRadius) * dth * dty);
    denomx += dbx / Bs[i];
    denomy += dby / Bs[i];
    //std::cout<< "dx: " << dx <<" dy: "<<dy<<" dth: "<<dth<<" dtx: " << dtx <<" dty: "<<dty << " dbx: " << dbx <<" dby: "<<dby<<" Bsi: "<<Bs[i]<<std::endl;
    }
    B *= Bs[i];
  }
  minBetaCurrent_ = Bmin;
  // Calculate gamma
  double dgx = input.target.x;
  double dgy = input.target.y;
  double gamma = pow(dgx, 2) + pow(dgy, 2);

  //std::cout << "gamma "<<gamma<<" dgx: "<<dgx<<" dgy: "<<dgy<<std::endl<<std::endl;
  if (gamma > kMinGamma)
  {
    double dgammax = 2.0 * dgx;
    double dgammay = 2.0 * dgy;
     //std::cout << "gamma" << gamma <<" apfk: "<<input.apf_k<<" dgammay: "<<dgammay<<" B: "<<B<<"denomy: "<<denomy<<" denomx: "<<denomx<< std::endl;
    double Fth = atan2(-(input.apf_k * dgammay * B - gamma * B * denomy),
                       -(input.apf_k * dgammax * B - gamma * B * denomx));
    double th = Fth + M_PI;

    auto vel_factor = std::min(fabs(dgx) / kXscale, 1.0);
    vel_factor = std::max(vel_factor, 0.25);
    //std::cout<<"vel_factor: "<<vel_factor<<" theta: "<<th<<std::endl;
    output.linear = std::max(vel_factor * cos(th) * kMaxV, kMinV);
    output.angular = sin(th) * kMaxW;

    // smoothing can be added using vOld, wOld
  }
  else
  {
    output.linear = 0.0;
    output.angular = 0.0;
  }
}
double APF::getMinBetaCurrent()
{
  return minBetaCurrent_;
}
