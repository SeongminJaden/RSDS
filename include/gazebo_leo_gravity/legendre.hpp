#pragma once
#include <vector>
#include <cmath>

namespace gazebo_leo_gravity
{

// --------------------------------------
// Fully-normalized Associated Legendre function table
// Provides Pnm(sinφ) and derivative w.r.t φ
// --------------------------------------
class LegendreTable
{
public:
    // Constructor: compute Pnm table
    // Inputs:
    //   nmax       : maximum degree
    //   sinphi     : sin(latitude)
    //   normalized : whether fully-normalized (default true)
    LegendreTable(int nmax, double sinphi, bool normalized = true);

    // Get Pnm value
    double get(int n, int m) const;

    // Compute derivative w.r.t φ
    double dPhi(int n, int m, double cosphi, double sinphi) const;

private:
    int nmax_;
    bool normalized_;
    std::vector<std::vector<double>> P_; // table P[n][m]

    // Compute factorial
    static double factorial(int n);
};

} // namespace gazebo_leo_gravity
