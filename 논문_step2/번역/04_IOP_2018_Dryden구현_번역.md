# LSA-02 비행 시험 시뮬레이션을 위한 Simulink 기반 Dryden 연속 난류 모델 구현

**원제**: Implementation of Dryden Continuous Turbulence Model into Simulink for LSA-02 Flight Test Simulation  
**저자**: Teuku Mohd Ichwanul Hakim¹, Ony Arifianto²  
**소속**: ¹LAPAN / 반둥공과대학(ITB) / TU Berlin, ²반둥공과대학(ITB)  
**학술지**: IOP Conference Series: Journal of Physics, Conf. Series 1005 (2018) 012017  
**DOI**: 10.1088/1742-6596/1005/1/012017

---

## 초록

난류(turbulence)는 대기 중 압력 및 온도 분포의 불안정으로 발생하는 소규모 공기 운동이다. 비행 역학 모델에 대기 교란으로 통합되는 난류 모델 중 Dryden 연속 난류 모델을 MATLAB Simulink에 구현하였다. 구현 모델은 군사 규격 MIL-HDBK-1797을 준거하며, 파워 스펙트럼 밀도에서 유도된 필터에 대역 제한 가우시안 백색 잡음을 입력하는 방식으로 구성된다. Aerospace Blockset 내장 모델과의 비교 검증을 통해 횡방향 속도($v_g$, $w_g$) 및 각속도($p_g$, $q_g$, $r_g$) 성분에서 차이가 있음을 확인하였으며, 난류 스케일 길이 조정 후 유사한 결과를 얻었다.

---

## 1. 서론

인도네시아 항공우주청(LAPAN)은 TU Berlin과 협력하여 경량 항공기 LSA-02의 전자식 비행 제어 시스템(EFCS) 개발 프로젝트를 2014년부터 진행 중이다. EFCS의 비행 제어 법칙(FCL) 검증을 위한 소프트웨어 인더루프(SILS) 및 하드웨어 인더루프(HILS) 시뮬레이션에서는 실제 비행 환경과 유사한 대기 교란 모델이 필요하다. 본 연구는 Dryden 연속 난류 모델을 Simulink에 구현하고 검증하는 것을 목표로 한다.

---

## 2. 대기 난류 모델 이론

### 2.1 Dryden 모델 vs. Von Kármán 모델

실용적으로는 Dryden 모델이 더 자주 사용된다. 이는 Dryden 모델의 수식이 더 단순하며 시뮬레이션 구현이 용이하기 때문이다. 두 모델 모두 유사한 결과를 제공하지만, Dryden 모델의 전달 함수는 유리 함수(rational function) 형태로 표현되어 아날로그·디지털 필터 구현에 직접 활용 가능하다.

### 2.2 기본 가정

Dryden 난류 모델링에서 난류의 통계적 특성에 대한 4가지 가정:

1. **정상성(Stationary)**: 시간에 독립적
2. **균질성(Homogeneous)**: 공간 위치에 독립적
3. **등방성(Isotropic)**: 방향에 독립적
4. **항공기보다 큰 스케일**: 항공기 전체가 동일한 난류 영향을 받음

### 2.3 난류 파라미터 정의

Dryden 모델에서 난류 특성은 세 파라미터로 정의된다:
- **난류 스케일 길이 ($L$)**: 종방향($L_u$), 횡방향($L_v$), 수직($L_w$) [ft 또는 m]
- **대기 속도 ($V$)**: 항공기 비행 속도 [ft/s 또는 m/s]
- **난류 강도 ($\sigma$)**: 난류 성분의 RMS 값 [ft/s 또는 m/s]

---

## 3. 파워 스펙트럼 밀도 (PSD)

### 3.1 선형 속도 PSD

시간 주파수의 함수로 표현된 Dryden PSD:

**종방향 ($u_g$)**:
$$\Phi_u(\omega) = \frac{2\sigma_u^2 L_u}{\pi V} \cdot \frac{1}{1 + \left(\frac{L_u \omega}{V}\right)^2}$$

**횡방향 ($v_g$)**:
$$\Phi_v(\omega) = \frac{2\sigma_v^2 L_v}{\pi V} \cdot \frac{1 + 12\left(\frac{L_v \omega}{V}\right)^2}{\left[1 + 4\left(\frac{L_v \omega}{V}\right)^2\right]^2}$$

**수직 ($w_g$)**:
$$\Phi_w(\omega) = \frac{2\sigma_w^2 L_w}{\pi V} \cdot \frac{1 + 12\left(\frac{L_w \omega}{V}\right)^2}{\left[1 + 4\left(\frac{L_w \omega}{V}\right)^2\right]^2}$$

### 3.2 각속도 PSD

$$\Phi_p(\omega) = \frac{\sigma_w^2}{V L_w} \cdot \frac{0.8\left(\frac{\pi}{4b}\right)^{1/3}}{1 + \left(\frac{4b\omega}{\pi V}\right)^2}$$

$$\Phi_q(\omega) = \frac{\pm(\omega/V)^2}{1 + \left(\frac{4b\omega}{\pi V}\right)^2} \cdot \Phi_w(\omega)$$

$$\Phi_r(\omega) = \frac{\pm(\omega/V)^2}{1 + \left(\frac{3b\omega}{\pi V}\right)^2} \cdot \Phi_v(\omega)$$

---

## 4. 연속 Dryden 필터

### 4.1 선형 속도 전달 함수

$$H_u(s) = \sigma_u \sqrt{\frac{2L_u}{\pi V}} \cdot \frac{1}{1 + \frac{L_u}{V}s}$$

$$H_v(s) = \sigma_v \sqrt{\frac{2L_v}{\pi V}} \cdot \frac{1 + \frac{2\sqrt{3}L_v}{V}s}{\left(1 + \frac{2L_v}{V}s\right)^2}$$

$$H_w(s) = \sigma_w \sqrt{\frac{2L_w}{\pi V}} \cdot \frac{1 + \frac{2\sqrt{3}L_w}{V}s}{\left(1 + \frac{2L_w}{V}s\right)^2}$$

### 4.2 각속도 전달 함수

$$H_p(s) = \sigma_w \sqrt{\frac{0.8}{V}} \cdot \frac{\left(\frac{\pi}{4b}\right)^{1/6}}{(2L_w)^{1/3}\left(1 + \frac{4b}{\pi V}s\right)}$$

$$H_r(s) = \frac{\mp s/V}{1 + \frac{3b}{\pi V}s} \cdot H_v(s)$$

$$H_q(s) = \frac{\mp s/V}{1 + \frac{4b}{\pi V}s} \cdot H_w(s)$$

연속 Dryden 필터는 저역 통과 필터(low-pass filter)로, 차단 주파수는 난류 스케일 길이와 대기 속도의 비율로 결정된다.

---

## 5. 고도별 파라미터

### 5.1 저고도 (1,000 ft 이하)

난류 스케일 길이는 고도 $h$의 함수:

$$2L_w = h$$
$$L_u = 2L_v = \frac{h}{(0.177 + 0.000823h)^{1.2}}$$

난류 강도:
$$\sigma_w = 0.1 W_{20}$$
$$\sigma_u = \sigma_v = \frac{\sigma_w}{(0.177 + 0.000823h)^{0.4}}$$

표 1. $W_{20}$ 기준값 (20 ft 고도 풍속)

| 난류 등급 | $W_{20}$ |
|----------|----------|
| 경도(Light) | 15 노트 |
| 중등도(Moderate) | 30 노트 |
| 심한(Severe) | 45 노트 |

### 5.2 중·고고도 (2,000 ft 이상)

등방성 가정, 스케일 길이 상수:
- $L_u = L_v = L_w = 1,750$ ft

---

## 6. Simulink 구현 및 검증

### 6.1 구현 방법

1. 백색 잡음 발생기 (Band-Limited White Noise 블록)
2. Dryden 성형 필터 (Transfer Function 블록)
3. 좌표계 변환 (항공기 동체 좌표계)

### 6.2 Aerospace Blockset과의 비교

Aerospace Blockset 내장 Dryden 모델과 비교 검증 결과:
- 종방향 속도 $u_g$: 거의 동일
- 횡방향 속도 $v_g$, 수직 속도 $w_g$: 차이 발생
- 각속도 $p_g$, $q_g$, $r_g$: 차이 발생

차이의 원인: Aerospace Blockset에서 사용하는 난류 스케일 길이 결정 방법의 차이. 스케일 길이 조정 후 두 모델 결과가 수렴함을 확인.

---

## 7. 결론

본 연구는 MIL-HDBK-1797을 준거한 Dryden 연속 난류 모델을 MATLAB Simulink에 성공적으로 구현하였다. 선형 속도 3성분과 각속도 3성분으로 구성된 6자유도 난류 모델이 구현되었으며, Aerospace Blockset 비교를 통해 검증하였다. 난류 스케일 길이 파라미터의 정확한 설정이 모델 정확도에 핵심임을 확인하였다.

---

**Step 2 활용 방향**: 본 논문의 Simulink 구현 방법은 `wind_model.m`의 MATLAB 구현에 직접 참조. 저고도 파라미터 수식(스케일 길이, 강도)을 Step 2 드론 시뮬레이션에 적용. 비교 검증 방법론을 Step 2 모델 검증에 활용.

---

*번역일: 2026-10-03*
