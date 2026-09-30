# 자율 추적형 드론 포획 시스템 — 수식·프로그램·발표자료 (논문 근거 통합본)

기존 정리안(수식/MATLAB/Simulink/발표자료)에, 확보한 5편 논문의 방법론·근거를 모듈별로 엮었습니다. 각 절 끝에 **[참고 논문]**으로 어떤 논문의 어떤 부분을 근거·차별점으로 쓸 수 있는지 표시했습니다.

---

## 0. 논문 접근 전략 (경상국립대 도서관 DB 기준)

| 구분 | DB | 접근 방법 | 해당 논문 |
|---|---|---|---|
| **IEEE** | IEEE Xplore (전체) | 학교 도서관 → IEEE | EKF UAV 추적 논문 (Step 3) |
| **Elsevier** | ScienceDirect (1995~, 전체) | 학교 도서관 → ScienceDirect | 대체 논문 검색 가능 |
| **Wiley** | Wiley Online Library (1997~2024) | 학교 도서관 → Wiley | 2024년 이전 논문만 |
| **한국 논문** | RISS / DBpia | RISS.kr (무료) 또는 DBpia | 유민성·김선진·심준형·박찬용·강현상 |
| **ResearchGate** | 무료 오픈액세스 | 직접 검색·저자 요청 | ADRC Pan-tilt 논문 |

### MATLAB 코드 인용 논문별 접근 가능 여부

| 논문 | 출판사 | 접근 방법 |
|---|---|---|
| Hattenberger et al. (2023) — Cd_h/Cd_v 근거 | **Sage** ❌ 미구독 | ResearchGate 또는 Google Scholar에서 무료본 검색 |
| WVU/Siqueira (2018) — 고도 멱함수 풍속 | 무료 기술보고서 | Google Scholar "Siqueira WVU UAV wind" 검색 |
| Springer Neural C&A (2026) — 고도유지 ±2m | **Springer** ❌ 미구독 | ResearchGate 검색 또는 아래 ScienceDirect 대체 논문 |
| IEEE EKF UAV Tracking — 칼만 필터 | **IEEE** ✅ 구독 | 학교 도서관 IEEE Xplore 직접 다운로드 |
| AIAA JGCD Net Capture — 포획 성공률 | **AIAA** ❌ 미구독 | ResearchGate 또는 아래 Elsevier 대체 논문 |

### ScienceDirect에서 찾을 수 있는 대체 논문 (검색 키워드)

- **Step 1 (항력 모델 대체)**: ScienceDirect → `"quadrotor aerodynamic drag" OR "UAV drag coefficient"` → *Aerospace Science and Technology* (Elsevier)
- **Step 2 (고도유지·분류 대체)**: ScienceDirect → `"drone trajectory classification altitude-hold"` → *Expert Systems with Applications* (Elsevier)
- **Step 5 (포획 판정 대체)**: ScienceDirect → `"anti-drone net capture success rate"` OR `"UAV interception simulation"` → *Robotics and Autonomous Systems* (Elsevier)

---

## 1. 표적(침입 드론) 궤적 생성 (3.1)

### 1-1. 운동방정식
```
m * dv/dt = F_thrust - F_drag - m*g*ẑ
F_drag = 0.5 * ρ * Cd * A * |v - v_wind| * (v - v_wind)
```
- v_wind: 풍속 벡터, F_thrust: 직선/회피 기동 프로파일

### 1-2. 회피 기동 모델
```
a_lateral(t) = A_evade * sin(2π f_evade * t)
```

### 1-3. MATLAB 구현
```matlab
function dxdt = drone_dynamics(t, x, wind, params)
    v = x(4:6);
    v_rel = v - wind.velocity(t);
    Fdrag = -0.5*params.rho*params.Cd*params.A*norm(v_rel)*v_rel;
    Fthrust = guidance_law(t, x, params);
    a = (Fthrust + Fdrag)/params.m - [0;0;params.g];
    dxdt = [v; a];
end
[t, x] = ode45(@(t,x) drone_dynamics(t,x,wind,params), tspan, x0);
```
**발표 포인트**: 무풍/약풍/강풍 조건별 "계획 vs 실제 궤적" 이탈을 plot3로 비교.

**[참고 논문]** 김선진 외(2024, 대포병탐지레이더 AI 논문)의 <Figure 3>는 표적(포물선)과 클러터(비형태)의 3차원 궤적을 나란히 시각화해 "정상 표적의 궤적은 매끈한 곡선, 비표적/기만은 불규칙"이라는 점을 보여줍니다. 이는 1-1절 궤적 생성 시 "드론=매끈한 유도 궤적, 클러터/새=불규칙 노이즈 궤적"으로 시뮬레이션 데이터를 설계하는 근거로 인용할 수 있습니다.

---

## 2. 표적 분류(필터링) 모듈 (3.2)

### 2-1~2-3. 특징 벡터·판별 함수 (기존과 동일)
```
f = [고도(h), 속도(v), 신호/크기 지표(s)]
score = w1*g_h(h) + w2*g_v(v) + w3*g_s(s)
classify = drone if score > threshold
```
```matlab
function isDrone = classify_target(feat, thresh, w)
    score = w(1)*feat.altScore + w(2)*feat.speedScore + w(3)*feat.sizeScore;
    isDrone = score > thresh;
end
```

### 2-4. 논문 근거로 업그레이드된 설계안
김선진 외(2024) 논문은 **좌표 변화량(Δu)과 궤적 각도 변화량(α)**을 표적/클러터 분류의 핵심 입력변수로 사용합니다(원논문 식(1)~(4)). 단순 (고도,속도,크기) 3변수보다, **연속된 3개 좌표점 사이의 각도 변화**가 "포물선형(표적) vs 불규칙형(클러터)"을 구분하는 데 더 강력하다는 것이 이 논문의 핵심 기여입니다. 이를 반영해 2-2절 특징벡터를 다음과 같이 보강하는 것을 제안합니다.

```
Δu_i = u_(i+1) - u_i                         (고도 변화량)
α_(i+1) = arccos( (v_i · v_(i+1)) / (|v_i||v_(i+1)|) )   (궤적 각도 변화량)

score = w1*g_h(h) + w2*g_v(v) + w3*g_s(s) + w4*g_α(α) + w5*g_Δu(Δu)
```
- 룰 기반(임계값)을 유지하되, 판별 변수에 "궤적 각도"를 추가하면 논문 근거가 명확해지고 성능도 개선될 가능성이 큽니다.
- 원 논문은 LSTM까지 갔지만(재현율 92.21%, 학과 논문 2장 분량엔 과함), 본 프로젝트에서는 "LSTM 대신 임계값 판별을 사용한 이유"를 계산 경량화·설명가능성으로 서술하면 차별점이 됩니다.

**발표 포인트**: 혼동행렬 + 궤적 각도 히스토그램(표적 vs 클러터)을 함께 제시.

**[참고 논문]** 김선진 외(2024) — 식(1)~(4)의 각도/고도변화량 변수 정의를 그대로 인용 가능. 박찬용 외(2024, RF 통신신호 논문)는 영상/궤적이 아닌 **RF 통신신호(RC 프로토콜) 기반 식별**을 다루므로, 5장(향후 과제)에서 "본 연구의 궤적 기반 분류를 RF 기반 식별과 결합하면 오탐률을 추가로 낮출 수 있다"는 확장 방향 서술에 활용.

---

## 3. 탐지/추적부 — 칼만 필터 (3.3)

### 3-1~3-3. 상태공간 모델·수식·MATLAB 구현 (기존과 동일)
```
A = [ I3  Δt·I3 ; 0  I3 ],  H = [ I3  0 ]
[예측] x̂_k⁻ = A x̂_(k-1),  P_k⁻ = A P_(k-1) Aᵀ + Q
[갱신] K_k = P_k⁻ Hᵀ(H P_k⁻ Hᵀ + R)⁻¹,  x̂_k = x̂_k⁻ + K_k(z_k - H x̂_k⁻),  P_k = (I-K_k H)P_k⁻
```
```matlab
function [x_est, P] = kalman_step(x_est, P, z, A, H, Q, R)
    x_pred = A*x_est;
    P_pred = A*P*A' + Q;
    K = P_pred*H' / (H*P_pred*H' + R);
    x_est = x_pred + K*(z - H*x_pred);
    P = (eye(size(P)) - K*H)*P_pred;
end
```

### 3-4. 논문과의 직접 연결 — 유민성·최영훈(2024)
이 논문은 본 프로젝트와 **거의 동일한 파이프라인**(인식→칼만필터 추적→포획)을 구현했습니다. 차이점과 인용 포인트:
- 원 논문: **영상(카메라) 기반** — YOLOv8로 이미지 내 드론 중심좌표 `z_k=(x,y)`를 뽑고, 이를 칼만필터로 2D 이미지 좌표계에서 추적.
- 본 프로젝트: **레이더/라이다 모사 센서 기반** — 3D 실제 좌표(px,py,pz)를 직접 관측하고 칼만필터로 추정.
- → "선행연구(유민성·최영훈, 2024)는 영상 기반 2D 추적을 사용했으나, 본 연구는 3D 위치 센서를 가정하여 실제 거리·사거리 판정까지 확장했다"는 문장으로 서론/관련연구에 인용하면 자연스럽습니다.
- 원 논문의 포획 조건("추적 드론과 불법 드론 거리가 D_th 이하로 작아지면 포획 단계 수행")은 본 프로젝트 3.5절 포획 판정 조건과 정확히 대응되므로 그대로 인용 가능.

**발표 포인트**: 원궤적 vs 노이즈 관측 vs 칼만 추정 그래프 + 날씨별(R 가변) RMSE 비교.

**[참고 논문]** 유민성·최영훈(2024) — 칼만필터 흐름도(Fig.1: 예측→칼만게인→추정→오차공분산 갱신)가 본 프로젝트 3-2절 수식과 1:1 대응. 김선진 외(2024)는 LSTM으로 칼만필터를 대체하는 대안도 제시하므로, "향후 과제"에서 "칼만필터 대신 LSTM 적용 가능성"을 한 줄 언급하면 완성도가 올라갑니다.

---

## 4. 조준 액추에이터 모델 — 핵심 파트 (3.4)

### 4-1~4-4. 전달함수·PID·리드조준·Simulink 구성 (기존과 동일)
```
Θ(s)/V(s) = K / [s(τs+1)]
u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
t_lead = |R_target - R_turret| / v_net
P_predict = P_target + V_target * t_lead
```
```
[표적 위치 예측] → [Pan/Tilt 목표각 계산] → [PID 제어기] → [서보 플랜트] → [실제 각도]
                                              ↑ [풍력 토크 외란]
```

**참고**: 유민성·최영훈(2024) 논문에서는 짐벌 카메라 각도를 "추적 드론과 표적 거리가 가까워질수록 아래로 회전 → 90°에 도달하면 속도를 순간 상승시켜 포획"하는 방식을 씁니다. 이는 본 프로젝트의 리드 조준(4-3절)과는 다른 접근(각도 기반 트리거)이지만, **포획 순간의 "최종 접근 기동" 로직**으로 4-3절에 보조 규칙으로 추가할 수 있습니다:
```
if gimbal_angle >= 90deg
    v_turret = v_turret_boost   % 포획 성공률을 높이기 위한 순간 가속
end
```

**발표 포인트**: 무풍 vs 강풍 조건 step response, 외란 인가 시 오차 억제 과정. (액추에이터 파트는 이 프로젝트의 핵심이므로 슬라이드 비중을 가장 높게)

**[참고 논문]** 유민성·최영훈(2024) — 포획 단계의 짐벌 회전·속도 부스트 로직을 액추에이터 모델의 보조 규칙으로 인용. 그 외 4편은 액추에이터 제어 자체를 다루지 않으므로, 직접 인용 논문이 부족한 절 — 이전에 안내드린 "지향성 Pan-Tilt 시스템 안정화"(2013) 등 별도 확보 논문으로 보강 필요.

---

## 5. 포획(네트) 판정 로직 (3.5)

```
capture_success = (|θ_error| < θ_th) AND (range_min < R < range_max)
```
```matlab
N = 1000; success = zeros(N,1);
for i = 1:N
    wind = sample_wind_condition();
    [theta_err, R] = run_single_trial(wind, params);
    success(i) = (abs(theta_err) < theta_th) && (R > Rmin && R < Rmax);
end
successRate = mean(success);
```
```
SE = sqrt(p(1-p)/N),  95% CI = p ± 1.96*SE
```

**[참고 논문]** 심준형 외(2023)의 Table 1(안티드론 기술 분류: 수동적 수단/능동적 수단 — 물리적 무력화·전자적 무력화)을 근거로, "왜 그물(net) 방식을 선택했는가"를 서론에 정리할 수 있습니다: 직접파괴(총포·레이저)는 2차 피해 위험, 전파교란은 GPS 미사용·사전입력 경로 비행 드론에 무력화 불가 — 반면 포획(net)은 "위험요인의 정확한 제거는 어렵지만 2차 피해가 적다"는 장단점 비교(원논문 2.3절)를 그대로 인용해 방법 선택의 타당성을 뒷받침할 수 있습니다.

---

## 6. 결과 시각화 (3.6)
- `plot3` 기반 궤적+조준선 애니메이션, 반응시간/추적오차/성공률 비교 그래프
- 유민성·최영훈(2024)의 Fig.3(Simulation/Camera View 2분할 화면 구성)처럼, 시뮬레이션 환경(3D)과 추적 시점(조준 카메라/센서 시점)을 나란히 배치하는 시각화 레이아웃을 그대로 벤치마킹하면 발표 자료가 한층 직관적입니다.

---

## 7. MATLAB/Simulink 프로그램 파일 구조 (기존과 동일)
```
/AIOT-Actuator-Project
├── main_simulation.m
├── drone_dynamics.m
├── wind_model.m
├── target_classifier.m        % ← 궤적 각도(α)·고도변화량(Δu) 변수 추가 반영
├── kalman_filter.m
├── lead_pursuit.m
├── servo_plant.slx            % ← 포획단계 속도 부스트 로직 추가 가능
├── capture_logic.m
├── monte_carlo_run.m
├── visualize_results.m
└── results/
```

---

## 8. 발표자료(PPT) 구성 — 논문 인용 지점 표시

1. **연구 배경 및 필요성** — 강현상(2021) 국내 동향(주요시설 500여 개소, 시장규모 성장), 심준형 외(2023) 탐지-식별-무력화 3단계 정의
2. **시스템 개요도** — 1~5절 모듈 블록다이어그램
3. **표적 궤적 & 외란 모델** — 무풍/강풍 3D 비교
4. **표적 분류 로직** — 김선진 외(2024) 궤적 각도·고도변화량 변수 인용, 혼동행렬
5. **추적 필터(칼만)** — 유민성·최영훈(2024) 파이프라인과 비교, RMSE 그래프
6. **핵심: 액추에이터 제어** — 서보 전달함수, PID 튜닝, 포획단계 속도부스트(유민성·최영훈 인용)
7. **리드 조준 효과** — 리드 유무 성공률 비교
8. **실험 설계 및 결과** — 조건별 성공률·오차·반응시간, 몬테카를로 CI
9. **시연 영상/애니메이션** — Simulation/Camera View 2분할 레이아웃 벤치마킹
10. **결론 및 향후 과제** — 박찬용 외(2024) RF 기반 식별과의 융합, LSTM 기반 분류(김선진 외) 확장 가능성

**Q&A 대비 포인트 (논문 근거 추가)**
- 왜 리드 조준이 필요한가 → 네트 비행시간 때문 (유민성·최영훈은 각도 트리거 방식 사용, 본 연구는 위치 예측 방식 — 차별점 설명 가능)
- 왜 칼만 필터인가 → 저비용 센서 노이즈 가정 + 실시간성 (유민성·최영훈 Fig.1 흐름도 인용)
- 왜 룰 기반 분류인가 → 설명가능성 + 궤적 각도라는 물리적으로 해석 가능한 변수 사용(김선진 외 근거)
- 왜 그물 포획인가 → 2차 피해 최소화 (심준형 외 무력화 기술 비교표 근거)

---

## 참고문헌 매핑 요약

| 논문 | 주로 인용할 절 | 접근 방법 |
|---|---|---|
| 유민성·최영훈(2024), 칼만필터 기반 포획 안티드론 | 3.3(추적), 3.4(포획단계 로직), 3.6(시각화 레이아웃) | RISS.kr 검색 |
| 김선진 외(2024), 대포병탐지레이더 AI 분류모델 | 3.1(궤적 특성), 3.2(분류 변수), 5장(향후 과제) | RISS.kr 검색 |
| 심준형 외(2023), 안티드론 대응 방안 | 서론, 3.5(포획 방식 선택 근거) | RISS.kr 검색 |
| 박찬용 외(2024), RF 통신신호 해석 | 5장(향후 과제 — RF 융합) | RISS.kr 검색 |
| 강현상(2021), 안티드론시스템 국내 동향 | 서론(시장·정책 배경) | RISS.kr 검색 |
| IEEE EKF UAV Tracking | 3.3(칼만 필터 구현 근거) | **학교 IEEE 구독 ✅** 다운로드 가능 |
| Hattenberger et al. (2023) | 3.1(Cd_h/Cd_v 파라미터) | ResearchGate 무료본 검색 |
| WVU/Siqueira (2018) | 3.1(고도 멱함수 풍속 모델) | Google Scholar 무료 기술보고서 |
| Springer Neural C&A (2026) | 3.1(고도유지 ±2m), 3.2(분류) | ResearchGate 또는 ScienceDirect 대체 검색 |
| AIAA JGCD Net Capture | 3.5(포획 성공률 기준) | ResearchGate 또는 ScienceDirect 대체 검색 |
