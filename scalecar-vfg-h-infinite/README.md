# VFG Path Following for LIMO

LIMO 모바일 로봇을 위한 VFG(Vector Field Guidance) 기반 경로추종 패키지.
LPV H∞ 제어기와 PID-FF 제어기를 비교 실험할 수 있다.

## 성능 비교 (Step Curvature Path, R=0.5 m)

| 속도 [m/s] | PID-FF Max [deg] | LPV H∞ Max [deg] | 개선율 |
|:---:|:---:|:---:|:---:|
| 1.0 | 15.8 | 14.2 | 10% |
| 1.5 | 18.1 | 11.5 | 36% |
| 2.0 | 19.4 | 8.7 | **55%** |
| 2.5 | 20.2 | 7.3 | **64%** |

> 동일한 VFG (k_e=3.0) 위에서 비교. 센서 잡음 포함 200회 MC 평균.

## 빠른 시작

```bash
pip install -e .
```

```python
from vfg_pathfollowing import Simulator, StepCurvaturePath
import matplotlib.pyplot as plt

path = StepCurvaturePath(R=0.5, theta_arc=1.57)
sim = Simulator(path, controller='lpv-hinf', speed=1.5)
result = sim.run(T=20.0)
result.plot(path=path)
plt.show()  # Jupyter 노트북에서는 %matplotlib inline 사용
```

## 컨트롤러 비교

```python
results = sim.compare(controllers=['lpv-hinf', 'pid-ff'], T=20.0)
Simulator.plot_comparison(results, path=path)
```

## LPV H∞ 제어기 튜닝

블랙박스로 제공되는 LPV H∞ 제어기에는 성능 튜닝을 위한 파라미터 4개가 있다.

### 튜닝 파라미터

| 파라미터 | 기본값 | 범위 | 설명 |
|---------|--------|------|------|
| `K_ff` | `0.0` | `0.0 ~ 1.0` | 곡률 피드포워드 게인. 경로 곡률 정보를 제어 출력에 직접 더해 곡률 전환 구간의 오차를 선제적으로 보정한다. |
| `rho_scale` | `1.0` | `0.5 ~ 2.0` | 스케줄링 파라미터 배율. 실제 차량 특성이 설계 모델과 다를 때 꼭짓점 선택을 보정한다. |
| `output_gain` | `1.0` | `0.5 ~ 1.5` | 전체 피드백 이득 배율. 진동이 발생하면 줄이고, 수렴이 느리면 늘린다. |
| `delta_max` | `inf` | `0.2 ~ 0.5` | 조향 출력 포화 [rad]. 실차 액추에이터 한계에 맞게 설정한다. 기본값은 무제한(시뮬레이션용). |

### 사용 방법

```python
# Simulator에서 dict로 전달
sim = Simulator(path, controller='lpv-hinf', speed=2.0,
                hinf_params={
                    'K_ff': 0.8,        # 곡률 피드포워드 활성화
                    'rho_scale': 1.2,   # 스케줄링 파라미터 보정
                    'output_gain': 1.1, # 이득 소폭 증가
                    'delta_max': 0.5,   # 실차 액추에이터 한계 [rad]
                })

# 또는 제어기 인스턴스 직접 생성
from vfg_pathfollowing import LPVHinfController
ctrl = LPVHinfController.default(K_ff=0.8, rho_scale=1.2)
sim = Simulator(path, controller=ctrl, speed=2.0)
```

### 조정 순서

1. **`output_gain`**: 진동이 없는 최대 이득을 먼저 탐색한다.
2. **`K_ff`**: 곡률 전환 구간에서 피크 오차가 크면 0.5~1.0 범위에서 올린다.
3. **`rho_scale`**: 시뮬레이션과 실차 응답이 다를 때 미세 조정한다.
4. **`delta_max`**: 마지막으로 실차 한계값에 맞게 고정한다.

> **주의**: `K_ff > 1.0` 또는 `output_gain > 1.5`는 발산 위험이 있다.
> 반드시 시뮬레이션에서 검증 후 실차에 적용하라.

## 패키지 구조

```
vfg_pathfollowing/
├── api.py              # Simulator 고수준 API
├── models/             # KinematicBicycle (LIMO)
├── guidance/           # VectorFieldGuidance, PathProjector
├── controllers/        # LPVHinfController, PIDFeedforward
├── paths/              # StepCurvature, Sinusoidal, Slalom, Bezier
├── simulation/         # ClosedLoopSimulator, NoiseGenerator, metrics
└── data/controllers/   # 사전 계산된 v3 제어기 (JSON)
```

## 노트북

| # | 파일 | 내용 |
|---|------|------|
| 1 | `01_quickstart.ipynb` | 5분 안에 첫 시뮬레이션 |
| 2 | `02_pid_vs_lpvhinf.ipynb` | 속도별 성능 비교 |
| 3 | `03_custom_path.ipynb` | 베지어 곡선으로 경로 설계 |
| 4 | `04_tuning.ipynb` | PID/VFG 파라미터 조정 |

## 경로 종류

- **StepCurvaturePath**: 직선 → 원호 → 직선 (과도 응답 평가)
- **SinusoidalPath**: 사인파 (정상상태 추종)
- **SlalomPath**: 좌우 교대 원호 (곡률 부호 변환)
- **BezierPath**: 웨이포인트 기반 자유 경로 설계

## ROS2 실차 배포

```bash
cd ros2_bridge
colcon build --packages-select limo_path_follower
ros2 run limo_path_follower path_follower_node
```

자세한 내용은 `ros2_bridge/README.md` 참고.

## 배경이론 보고서

```bash
make -C report
```

`report/main.pdf`로 경로추종 문제, VFG 원리, 제어기 비교를 학부생 수준에서 설명.

## 테스트

```bash
pytest tests/ -v
```

## 의존성

- Python ≥ 3.10
- NumPy ≥ 1.24
- SciPy ≥ 1.10
- Matplotlib ≥ 3.7

## 라이선스

MIT License. See [LICENSE](LICENSE).
