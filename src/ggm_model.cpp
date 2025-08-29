#include "gazebo_leo_gravity/ggm_model.hpp"
#include "gazebo_leo_gravity/legendre.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace gazebo_leo_gravity
{

// --------------------------------------
// Load function: parse header + coefficients
// --------------------------------------
bool GGMModel::load(const std::string& filename, int nmax)
{
  nmax_ = nmax;
  coeffs_.clear();

  std::ifstream fin(filename);
  if (!fin.is_open())
  {
    std::cerr << "[GGMModel] Failed to open file: " << filename << std::endl;
    return false;
  }

  std::string line;
  bool inHeader = true;

  while (std::getline(fin, line))
  {
    if (line.find("end_of_head") != std::string::npos)
    {
      inHeader = false;
      continue;
    }

    if (inHeader)
    {
      // Parse header information
      if (line.find("earth_gravity_constant") != std::string::npos)
      {
        std::stringstream ss(line);
        std::string key;
        ss >> key >> GM_;
      }
      else if (line.find("radius") != std::string::npos)
      {
        std::stringstream ss(line);
        std::string key;
        ss >> key >> a_;
      }
      else if (line.find("norm") != std::string::npos)
      {
        if (line.find("fully_normalized") != std::string::npos)
          normalized_ = true;
        else
          normalized_ = false;
      }
      continue;
    }

    // Parse coefficient data (gfc lines)
    if (line.rfind("gfc", 0) == 0)
    {
      // Convert D exponent to E for C++ parsing
      for (auto &c : line)
      {
        if (c == 'D') c = 'E';
      }

      std::istringstream iss(line);
      std::string tag;
      int n, m;
      double C, S, sigmaC, sigmaS;
      iss >> tag >> n >> m >> C >> S >> sigmaC >> sigmaS;

      if (n <= nmax_)
      {
        coeffs_.push_back({n, m, C, S});
      }
    }
  }

  if (GM_ == 0 || a_ == 0)
  {
    std::cerr << "[GGMModel] Warning: GM or radius not found in header, using defaults!" << std::endl;
    if (GM_ == 0) GM_ = 3.986004415e14;    // [m^3/s^2]
    if (a_ == 0)  a_ = 6378136.3;          // [m]
  }

  std::cout << "[GGMModel] Loaded GGM file: " << filename
            << " (nmax=" << nmax_ << ", coeffs=" << coeffs_.size() << ")\n";

  return true;
}

// --------------------------------------
// Compute gravity acceleration
// Input: position vector (x,y,z) [m]
// Output: acceleration vector (ax,ay,az) [m/s^2]
// --------------------------------------
ignition::math::Vector3d GGMModel::acceleration(const ignition::math::Vector3d& pos) const
{
  double x = pos.X();
  double y = pos.Y();
  double z = pos.Z();

  // Convert to geocentric spherical coordinates
  double r = sqrt(x*x + y*y + z*z);
  double phi = asin(z / r);               // geocentric latitude
  double lambda = atan2(y, x);            // longitude

  double cosphi = cos(phi);
  double sinphi = sin(phi);

  // Compute fully-normalized associated Legendre functions
  LegendreTable P(nmax_, sinphi, normalized_);

  double dUdr = 0.0;
  double dUdphi = 0.0;
  double dUdlambda = 0.0;

  // Accumulate coefficients
  for (auto &c : coeffs_)
  {
    int n = c.n;
    int m = c.m;
    double C = c.C;
    double S = c.S;

    double cosm = cos(m * lambda);
    double sinm = sin(m * lambda);

    double Pnm = P.get(n, m);

    double factor = pow(a_ / r, n);

    // ∂U/∂r
    dUdr += (n+1) * factor * Pnm * (C * cosm + S * sinm);

    // ∂U/∂phi
    double dPnm_dphi = P.dPhi(n, m, cosphi, sinphi);
    dUdphi += factor * dPnm_dphi * (C * cosm + S * sinm);

    // ∂U/∂lambda
    dUdlambda += factor * m * Pnm * (-C * sinm + S * cosm);
  }

  double common = GM_ / (r * r);

  // Convert from spherical to Cartesian acceleration
  double ar = -common * dUdr;
  double aphi = common * dUdphi;
  double alam = common * dUdlambda;

  double ax = (ar * cosphi * cos(lambda)
              - aphi * sinphi * cos(lambda)
              - alam * sin(lambda));
  double ay = (ar * cosphi * sin(lambda)
              - aphi * sinphi * sin(lambda)
              + alam * cos(lambda));
  double az = (ar * sinphi + aphi * cosphi);

  return ignition::math::Vector3d(ax, ay, az);
}

} // namespace gazebo_leo_gravity
