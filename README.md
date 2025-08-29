# Gazebo-LEO-Gravity-Plugin

ROS2 Humble + Gazebo 환경에서 **지구중력모델(GGM05C)** 을 이용하여,  
LEO(저궤도) 위성 환경의 위치-의존적 중력을 시뮬레이션하는 플러그인입니다.

---

## ✨ 목적 (Purpose)

- 기본 Gazebo 중력은 **균일한 일정 값 (예: -9.81 m/s²)** 만 지원합니다.  
- 그러나 LEO 환경에서는 위성 위치에 따라 중력 가속도가 달라지며,  
  이는 궤도 시뮬레이션과 자세 제어 연구에 중요한 영향을 줍니다.  
- 본 프로젝트는 **GGM05C 지구중력모델**을 기반으로 위치마다 변하는  
  중력장을 계산하여 Gazebo에 반영합니다.

## 🔬 원리 (Principle)

1. **구면조화 전개 (Spherical Harmonics)**  
   - 지구중력퍼텐셜은 구면좌표계에서 무한급수로 전개됩니다.  
   - 계수는 GGM05C 모델에서 제공됩니다.  

2. **정규화된 연관 르장드르 다항식 (Fully-normalized Associated Legendre Functions)**  
   - GGM05C 계수는 fully-normalized 형식으로 제공됩니다.  
   - SciPy/C++ 재귀식을 이용해 계산합니다.  

3. **위치 의존적 중력 가속도**  
   - 퍼텐셜의 그라디언트를 구하여  
     카르테시안 좌표계에서 `a = -∇V` 를 계산합니다.  
   - 계산된 중력 가속도를 각 링크 질량에 곱하여 force로 적용합니다.  

## 📦 내용 (Overview)

- `src/leo_gravity_world_plugin.cc`  
  Gazebo WorldPlugin: 각 시뮬레이션 step에서 모든 모델에 중력 force를 적용  
- `include/leo_gravity/ggm_model.hpp`  
  구면조화 계수 및 Legendre 다항식 계산 모듈  
- `data/GGM05C.gfc`  
  GGM05C 계수 파일 (GRACE 기반)  
- `CMakeLists.txt`, `package.xml`  
  ROS2 Humble 빌드 환경 구성  

---

## 📚 이론 및 유도 과정 (Theory & Derivation)

### 1. 지구 중력 퍼텐셜의 기본식

지구의 중력 퍼텐셜은 비구형 효과(적도 불룩함, 질량 분포 불균일 등)를 고려하기 위해  
구면조화 함수(Spherical Harmonics)로 전개됩니다.

퍼텐셜:
V(r,ϕ,λ)=rGM​(1+n=2∑∞​(ra​)nm=0∑n​Pˉn,m​(sinϕ)[Cˉn,m​cos(mλ)+Sˉn,m​sin(mλ)])

- \( GM \): 지구 표준 중력상수 (WGS84 기준 \( 3.986004418 \times 10^{14}\, m^3/s^2 \))  
- \( a \): 기준 지구 반경 (WGS84 기준 \( 6378137.0\, m \))  
- \( (r,\phi,\lambda) \): 구면 좌표계 (반경, 위도, 경도)  
- \( \bar{P}_{n,m} \): **fully-normalized Associated Legendre functions**  
- \( \bar{C}_{n,m}, \bar{S}_{n,m} \): **fully-normalized Stokes 계수 (GGM05C 제공)**  

### 2. 정규화된 연관 르장드르 함수

Legendre 다항식은 다음과 같은 정규화因자를 포함합니다.

\[
\bar{P}_{n,m}(x) = N_{n,m} \, P_{n,m}(x)
\]

여기서 \( P_{n,m}(x) \)는 비정규화 Associated Legendre 함수이고,  
정규화因자는:

\[
N_{n,m} = \sqrt{ (2-\delta_{m0})(2n+1) \frac{(n-m)!}{(n+m)!} }
\]

- \(\delta_{m0}\): 크로네커 델타 (m=0이면 1, 아니면 0)  

이 정의는 GGM05C, EGM2008 등에서 채택한 **fully-normalized 형식**입니다.

### 3. 중력 가속도의 유도

중력 가속도는 퍼텐셜의 기울기(Gradient)로 얻습니다.

\[
\mathbf{a} = - \nabla V
\]

구면좌표계에서 미분하면:

- 반경 방향:
\[
a_r = -\frac{\partial V}{\partial r}
\]

- 위도 방향:
\[
a_\phi = -\frac{1}{r}\frac{\partial V}{\partial \phi}
\]

- 경도 방향:
\[
a_\lambda = -\frac{1}{r \cos\phi}\frac{\partial V}{\partial \lambda}
\]

이를 다시 카르테시안 좌표계 \((x,y,z)\)로 변환하면 Gazebo의 물리 엔진에서 직접 적용 가능한  
\((a_x, a_y, a_z)\) 형태의 가속도가 됩니다.

### 4. 계수 파일 (GGM05C)

- GGM05C는 GRACE 위성 관측 기반의 **지구중력장 모델**  
- 계수는 \( \bar{C}_{n,m}, \bar{S}_{n,m} \)로 제공됨 (fully-normalized 형식)  
- 일반적으로 \( n_{max} = 360 \) 또는 \( 720 \)까지 제공됨  
- Gazebo 시뮬레이션에서는 계산량을 줄이기 위해 \( n_{max} = 20 \sim 50 \) 정도로 제한 가능  


### 5. 요약 (Summary)

- **기본 중력**: 단순 \( g = 9.81 \) m/s² → 일정  
- **본 플러그인 중력**:  
  - 구체 위치별로 달라짐 (고도, 위도, 경도 의존)  
  - GGM05C 계수를 활용한 정밀 구면조화 전개  
  - Gazebo 내 각 객체에 질량 × 가속도(force)로 적용

---

## 🚀 사용 방법 (Usage)

### 1. 빌드
```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR-ID/Gazebo-LEO-Gravity-Plugin.git
cd ~/ros2_ws
colcon build --packages-select gazebo_leo_gravity
source install/setup.bash
```


### 2. Gazebo 실행 시 플러그인 로드

world.sdf 예시:
```bash
<plugin name="leo_gravity" filename="libleo_gravity_world_plugin.so">
  <ggm_coeff_file>$(find-pkg-share gazebo_leo_gravity)/data/GGM05C.gfc</ggm_coeff_file>
  <nmax>50</nmax> <!-- 차수 제한 -->
</plugin>
```
### 3. 실행
```bash
ros2 launch gazebo_ros empty_world.launch.py world:=world.sdf
```

---

## 📖 참고 자료
- GGM05C: GRACE 기반 지구중력모델 (Tapley et al., 2013)
- Kaula, W. M. "Theory of Satellite Geodesy" (1966)
- SHTOOLS / ICGEM 자료
