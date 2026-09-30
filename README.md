# 자율 추적형 드론 포획 시스템
AIoT 액추에이터 과목 프로젝트 — MATLAB/Simulink 시뮬레이션

**마감**: 2026년 12월 19일 (종강) | **담당**: wuerphe39

---

## 프로젝트 개요

침입 드론을 탐지·추적하여 네트(그물)를 발사해 포획하는 자율 요격 시스템을  
MATLAB/Simulink 환경에서 시뮬레이션으로 구현한다.  
팬/틸트 서보 PID 제어가 핵심 액추에이터 파트이며, 바람 외란 하에서의 강건성이 주요 평가 지표이다.

---

## 파일 구조

```
AIOT-Actuator-Project/
├── MATLAB/
│   ├── drone_dynamics.m      ← Step 1: 드론 3D 운동방정식 (Hattenberger 2023 항력 모델)
│   ├── wind_model.m          ← Step 1: 고도 의존 풍속 모델 (WVU/Siqueira 2018)
│   ├── guidance_law.m        ← Step 1: 유도 추력 + PD 고도유지 제어기 (Springer 2026)
│   ├── step1_run.m           ← Step 1: 궤적 생성 검증 스크립트 (5개 그래프)
│   ├── target_classifier.m   (Step 2 예정)
│   ├── kalman_filter.m       (Step 3 예정)
│   ├── lead_pursuit.m        (Step 4 예정)
│   ├── servo_plant.slx       (Step 4 예정)
│   ├── capture_logic.m       (Step 5 예정)
│   ├── monte_carlo_run.m     (Step 5 예정)
│   └── visualize_results.m   (Step 6 예정)
├── 논문/                     ← 참고문헌 PDF
├── project.md                ← 시스템 설계 상세
├── drone_capture_project_summary_v2.md  ← 수식·코드·발표자료 통합본
└── README.md                 ← 이 파일
```

---

## 주차별 진행 현황

| 주차 | Step | 내용 | 상태 |
|:---:|:---:|---|:---:|
| 1주차 | Step 1 | 침입 드론 궤적 생성 모듈 구현 | ✅ 완료 |
| 2주차 | Step 2 | 표적 분류기 (드론 vs 새/클러터) | 🔜 진행 예정 |
| 3~4주차 | Step 3 | 칼만 필터 기반 위치 추적 | 예정 |
| 5~6주차 | Step 4 | 팬/틸트 서보 PID 제어 + 리드 조준 | 예정 |
| 7~8주차 | Step 5 | 포획 판정 + 몬테카를로 성공률 | 예정 |
| 9주차 | Step 6 | 결과 시각화 통합 | 예정 |
| 10주차 | Step 7 | 논문 작성 (학과 양식, 2장) | 예정 |

---

## Step 1 완료 내역 (2026-09-24)

### 구현 내용

**드론 물리 모델** (`drone_dynamics.m`)
- 방향별 항력 분리: 수평 Cd_h = 0.47, 수직 Cd_v = 0.77
- 논문 근거: Hattenberger et al. (2023), Sage/IJAV, SCIE

**풍속 모델** (`wind_model.m`)
- 고도 멱함수 프로파일: v(z) = v_ref × (z/z_ref)^0.14
- 논문 근거: Siqueira, WVU (2018), Hellmann 지수 개방 지형 기준

**유도 추력** (`guidance_law.m`)
- PD 고도유지 제어기: F_z = m × (g + Kp_z×e_z − Kd_z×vz)
- 3D 회피 기동: 수평 측방향 + 수직 방향 사인파 중첩
- 논문 근거: Springer Neural C&A (2026), SCIE

### 시뮬레이션 결과 (3개 시나리오)

| 시나리오 | 종료 위치 (X, Y, Z) | 터미널 속도 |
|---|---|---|
| 직선/무풍 | (719.6, 0.0, 30.0) m | ~25 m/s |
| 직선/약풍 5 m/s | (883.1, 0.0, 30.0) m | ~30 m/s |
| 3D회피/강풍 15 m/s | (1208.9, −4.2, 30.0) m | ~40 m/s |

**검증 지표**
- 고도유지: 최대이탈 ±0.07 m (Springer 2026 기준 ±2 m 대비 우수)
- 최대 풍력 이탈량: 72 m (약풍 5 m/s, 30초)
- 궤적 각도 α: 직선 0.00° / 3D회피 최대 0.38° → Step 2 분류 특징벡터로 활용

### 수정 이력

| 날짜 | 파일 | 내용 |
|---|---|---|
| 2026-09-24 | `step1_run.m` | Figure 5 이탈량 계산 버그 수정: `n_common` 인덱스 비교 → `interp1` 보간으로 교체 (ode45의 비균일 시간축 문제) |

---

## 실행 방법

```matlab
% MATLAB에서 MATLAB/ 폴더를 경로에 추가 후 실행
cd('MATLAB')
step1_run
```

5개 그래프 출력:
1. 3D 궤적 비교
2. 고도 시계열 (±2 m 허용범위)
3. 수평 속도 시계열
4. 궤적 각도 α (Step 2 분류 특징벡터)
5. 풍력에 의한 궤적 이탈량

---

## 핵심 논문 목록

| 논문 | 인용 위치 | 접근 방법 |
|---|---|---|
| Hattenberger et al. (2023), Sage/IJAV | Step 1 항력 모델 | ResearchGate |
| WVU/Siqueira (2018) | Step 1 풍속 모델 | Google Scholar 무료 |
| Springer Neural C&A (2026) | Step 1 고도유지, Step 2 분류 | ResearchGate |
| IEEE EKF UAV Tracking | Step 3 칼만 필터 | **학교 IEEE 구독 ✅** |
| 유민성·최영훈 (2024) | Step 3 추적, Step 4 포획 로직 | RISS.kr |
| 김선진 외 (2024) | Step 2 분류 변수 | RISS.kr |
| 심준형 외 (2023) | 서론 무력화 방식 비교 | RISS.kr |
| 박찬용 외 (2024) | 향후 과제 RF 융합 | RISS.kr |
| 강현상 (2021) | 서론 국내 동향 | RISS.kr |
