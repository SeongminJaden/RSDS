#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

// 단일 계수 구조체
struct GGMCoef {
  int n, m;
  double C, S;
};

class GGMModel {
public:
  bool load(const std::string& filename, int nmax);
  Eigen::Vector3d gravity(double lat, double lon, double r) const;

private:
  int nmax_;
  std::vector<GGMCoef> coeffs_;
  double GM_, a_;
};
