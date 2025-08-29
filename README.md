# ROS2-Based LEO Satellite Simulation & Gravity Plugin Integration

## Project Overview

This project integrates ROS2 and Gazebo to simulate Low Earth Orbit (LEO) satellite dynamics, attitude control, thrust, and space debris removal, with realistic physics including a position-dependent gravity field based on the GGM05C Earth gravity model.

The simulation combines accurate orbital prediction, satellite dynamic modeling, robotic arm manipulation, and environmental physics for high-fidelity research and experimentation.

## Key Features

* **LEO Gravity Simulation** using GGM05C model (position-dependent forces)
* Orbit prediction using TLE data with SGP4 and HPOP models
* Atmospheric drag and environmental physics
* Delta-V thrust and attitude control simulation
* Robotic arm path planning and control via MoveIt2
* Physical contact and joint formation with space debris (FixedJoint)
* ROS2 topic and service-based communication and data visualization

## Project Components

* `satellite_description/`: URDF/XACRO models for satellites and robotic arm
* `satellite_plugins/`: Custom Gazebo plugins (Drag, Thruster, Gripper, Gravity)
* `satellite_control/`: ros2\_control joint controller configurations
* `satellite_moveit/`: MoveIt2 configuration and robotic arm control
* `satellite_utils/`: TLE parsing, orbit calculation, and trajectory utilities
* `gazebo_leo_gravity/`: LEO Gravity Plugin using GGM05C

## LEO Gravity Plugin (GGM05C)

The plugin calculates the gravitational acceleration on satellites in LEO using the GGM05C model and applies the computed forces in Gazebo.

### 1. Earth Gravity Potential

The gravitational potential considering the Earth's oblateness and mass distribution is expanded using spherical harmonics:

$$
V(r, \phi, \lambda) = \frac{GM}{r} 
\left(
 1 + \sum_{n=2}^{\infty} \left(\frac{a}{r}\right)^n 
 \sum_{m=0}^{n} \bar{P}_{n,m}(\sin \phi)
 \Big[\bar{C}_{n,m}\cos(m\lambda) + \bar{S}_{n,m}\sin(m\lambda)\Big]
\right)
$$


Where:

* \$GM\$: Earth's standard gravitational parameter
* \$a\$: Reference Earth radius
* \$(r, \phi, \lambda)\$: Spherical coordinates (radius, latitude, longitude)
* \$\bar{P}\_{n,m}\$: Fully-normalized associated Legendre functions
* \$\bar{C}*{n,m}, \bar{S}*{n,m}\$: Fully-normalized Stokes coefficients (GGM05C)

### 2. Fully-Normalized Associated Legendre Functions

The fully-normalized associated Legendre function is given by:

$$
\bar{P}_{n,m}(x) = N_{n,m} P_{n,m}(x)
$$

with normalization factor:

$$
N_{n,m} = \sqrt{(2-\delta_{m0})(2n+1)\frac{(n-m)!}{(n+m)!}}
$$

Here, \$P\_{n,m}(x)\$ is the standard (unnormalized) associated Legendre function and \$\delta\_{m0}\$ is the Kronecker delta.

### 3. Gradient to Compute Acceleration

Gravitational acceleration is obtained from the gradient of the potential:

$$
\mathbf{a} = -\nabla V
$$

In spherical coordinates:

* Radial component:

$$
a_r = -\frac{\partial V}{\partial r}
$$

* Latitude component:

$$
a_\phi = -\frac{1}{r}\frac{\partial V}{\partial \phi}
$$

* Longitude component:

$$
a_\lambda = -\frac{1}{r \cos\phi}\frac{\partial V}{\partial \lambda}
$$

### 4. Conversion to Cartesian Coordinates

To apply in Gazebo, convert spherical accelerations \$(a\_r, a\_\phi, a\_\lambda)\$ to Cartesian \$(a\_x, a\_y, a\_z)\$:

$$
a_x = a_r \cos\phi \cos\lambda - a_\phi \sin\phi \cos\lambda - a_\lambda \sin\lambda
$$

$$
a\_y = a\_r \cos\phi \sin\lambda - a\_\phi \sin\phi \sin\lambda + a\_\lambda \cos\lambda\$\$
$$

$$
a_z = a_r \sin\phi + a_\phi \cos\phi
$$

### 5. GGM05C Coefficients
- Provided as $\bar{C}_{n,m}$ and $\bar{S}_{n,m}$
- Derived from GRACE satellite observations
- Typical truncation for simulation: $n_{max} = 20 \sim 50$

### 6. Applying Forces in Gazebo
Each satellite's force is computed as:

$$
\mathbf{F} = m \mathbf{a}
$$

where $m$ is the satellite mass and $\mathbf{a}$ is the gravity acceleration vector. The plugin applies this force at every simulation step.

## Orbit Data and Simulation
- TLE data sourced from [Celestrak](https://celestrak.com) or [Space-Track](https://www.space-track.org)
- Satellite position and velocity calculated using `pyorbital` or `sgp4`
- Orbit simulation integrated with Gazebo for realistic dynamics

## Development Environment
- OS: Ubuntu 22.04
- ROS2: Humble or Iron
- Gazebo: Fortress or Garden
- MoveIt2 (latest stable version)
- C++ (for plugins), Python (for ROS nodes and orbit calculations)

## How to Build & Run
1. Setup ROS2 workspace and build packages:
```bash
colcon build --packages-select satellite_description satellite_plugins satellite_control satellite_moveit satellite_utils gazebo_leo_gravity
source install/setup.bash
```
2. Launch the simulation:
```bash
ros2 launch satellite_sim satellite_sim.launch.py
```
3. Use MoveIt2 to control the robotic arm and visualize orbits.

## Future Plans & Extensions
- Sensor simulation (LiDAR, cameras) for satellites
- Support for other orbits (MEO, GEO)
- Real-time orbit adjustment and collision avoidance
- Modeling sensor noise in space environment

## References
- [SGP4 Algorithm Documentation](https://celestrak.com/NORAD/documentation/spacetracks.pdf)
- [PyOrbital GitHub](https://github.com/pytroll/pyorbital)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [MoveIt2](https://moveit.ros.org/)
- GGM05C Gravity Model: Tapley et al., 2013

$$
