#include "gazebo_leo_gravity/legendre.hpp"
#include <stdexcept>

namespace gazebo_leo_gravity
{

// --------------------------------------
// Factorial utility
// --------------------------------------
double LegendreTable::factorial(int n)
{
    if (n <= 1) return 1.0;
    double f = 1.0;
    for (int i = 2; i <= n; ++i) f *= i;
    return f;
}

// --------------------------------------
// Constructor: compute fully-normalized Pnm
// --------------------------------------
LegendreTable::LegendreTable(int nmax, double sinphi, bool normalized)
    : nmax_(nmax), normalized_(normalized)
{
    double cosphi = sqrt(1.0 - sinphi*sinphi);

    P_.resize(nmax_ + 1);
    for (int n = 0; n <= nmax_; ++n)
        P_[n].resize(n + 1, 0.0);

    // Base case P0,0
    P_[0][0] = 1.0;

    for (int n = 1; n <= nmax_; ++n)
    {
        // m=0
        P_[n][0] = ((2*n-1) * sinphi * P_[n-1][0] - (n-1) * P_[n-2][0]) / n;

        for (int m = 1; m <= n; ++m)
        {
            if (n == m)
            {
                // Diagonal: Pmm = (2m-1) * cosφ * P_{m-1,m-1}
                P_[n][m] = (2*n - 1) * cosphi * P_[n-1][m-1];
            }
            else
            {
                // Recursion for m < n
                P_[n][m] = ((2*n-1) * sinphi * P_[n-1][m] - (n+m-1) * P_[n-2][m]) / (n-m);
            }

            // Apply full normalization if required
            if (normalized_)
            {
                double factor = sqrt((2.0 - (m==0 ? 1.0:0.0)) * factorial(n-m)/factorial(n+m));
                P_[n][m] *= factor;
            }
        }
    }
}

// --------------------------------------
// Return Pnm value
// --------------------------------------
double LegendreTable::get(int n, int m) const
{
    if (n < 0 || n > nmax_ || m < 0 || m > n) throw std::out_of_range("Invalid n or m");
    return P_[n][m];
}

// --------------------------------------
// Compute derivative w.r.t φ using recurrence
// Inputs: cosφ, sinφ
// --------------------------------------
double LegendreTable::dPhi(int n, int m, double cosphi, double sinphi) const
{
    if (m > n) return 0.0;
    if (n == m) return 0.0; // Diagonal derivative approx 0
    if (n == 0) return 0.0;

    // dPnm/dφ = (1/cosφ) * ( n*sinφ*Pnm - (n+m)*P_{n-1,m} )
    double dP = 0.0;
    if (cosphi != 0.0)
        dP = (n * sinphi * P_[n][m] - (n + m) * P_[n-1][m]) / cosphi;

    return dP;
}

} // namespace gazebo_leo_gravity
