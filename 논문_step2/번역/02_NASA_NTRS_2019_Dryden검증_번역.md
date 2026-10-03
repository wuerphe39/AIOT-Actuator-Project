# Dryden 난류 모델 및 MIL-F-8785 돌풍 구배 구현 검증

**원제**: Verifying Implementation of the Dryden Turbulence Model and MIL-F-8785 Gust Gradient  
**저자**: Michael M. Madden  
**기관**: NASA Langley Research Center  
**보고서 번호**: NASA/TM-2019-220275  
**발행연도**: 2019  
**URL**: https://ntrs.nasa.gov/api/citations/20190000875/downloads/20190000875.pdf

---

## 초록

본 보고서는 Dryden 연속 난류 모델과 MIL-F-8785C 규격에 정의된 돌풍 구배(gust gradient) 입력의 수치 구현을 검증하는 방법론을 제시한다. 이산 시간 시뮬레이션에서의 영차 홀드(Zero-Order Hold, ZOH) 및 일차 홀드(First-Order Hold, FOH) 이산화 방법을 비교하고, 각 방법이 구현된 Dryden 모델의 파워 스펙트럼 밀도(PSD) 특성에 미치는 영향을 분석한다.

---

## 1. 서론

항공기 시뮬레이션에서 난류 환경의 정확한 모델링은 비행 제어 시스템 설계 및 검증의 핵심 요소이다. 미군 규격 MIL-F-8785C 및 MIL-HDBK-1797은 항공기 설계에 사용할 표준 대기 난류 모델로 Dryden 모델을 채택하고 있다. 그러나 이 모델을 이산 시간 디지털 시뮬레이션에 구현할 때 발생하는 수치적 오류는 충분히 연구되지 않았다.

본 보고서의 목적은:
1. Dryden PSD 수식의 정확한 도출 과정을 문서화
2. ZOH 및 FOH 이산화 방법의 정확도를 PSD 기준으로 검증
3. 구현 시 발생하는 일반적인 오류를 식별하고 수정 방법 제시

---

## 2. Dryden 파워 스펙트럼 밀도

### 2.1 선형 속도 성분 PSD

Dryden 모델은 세 축 방향 선형 속도 난류 성분($u_g$, $v_g$, $w_g$)의 PSD를 다음과 같이 정의한다:

**종방향 성분 ($u_g$)**:
$$\Phi_u(\Omega) = \sigma_u^2 \frac{2L_u}{\pi} \cdot \frac{1}{1 + (L_u \Omega)^2}$$

**횡방향 성분 ($v_g$)**:
$$\Phi_v(\Omega) = \sigma_v^2 \frac{L_v}{\pi} \cdot \frac{1 + 3(L_v \Omega)^2}{\left[1 + (L_v \Omega)^2\right]^2}$$

**수직 성분 ($w_g$)**:
$$\Phi_w(\Omega) = \sigma_w^2 \frac{L_w}{\pi} \cdot \frac{1 + 3(L_w \Omega)^2}{\left[1 + (L_w \Omega)^2\right]^2}$$

여기서:
- $\Omega = \omega/V$ [rad/m]: 공간 주파수
- $L_u$, $L_v$, $L_w$: 난류 스케일 길이 [m 또는 ft]
- $\sigma_u$, $\sigma_v$, $\sigma_w$: 난류 강도 [m/s 또는 ft/s]
- $V$: 항공기 대기 속도 [m/s 또는 ft/s]

### 2.2 각속도 성분 PSD

각속도 난류 성분($p_g$, $q_g$, $r_g$)의 PSD:

**롤률 ($p_g$)**:
$$\Phi_p(\Omega) = \sigma_w^2 \frac{0.8}{V} \cdot \frac{\left(\frac{\pi}{4b}\right)^{1/3}}{1 + \left(\frac{4b\Omega}{\pi}\right)^2}$$

**피치율 ($q_g$)**:
$$\Phi_q(\Omega) = \frac{\pm(\Omega/V)^2}{1 + \left(\frac{4b\Omega}{\pi}\right)^2} \cdot \Phi_w(\Omega)$$

**요율 ($r_g$)**:
$$\Phi_r(\Omega) = \frac{\pm(\Omega/V)^2}{1 + \left(\frac{3b\Omega}{\pi}\right)^2} \cdot \Phi_v(\Omega)$$

여기서 $b$는 항공기 날개 폭[ft]이다.

---

## 3. 연속 시간 성형 필터

PSD 방정식의 제곱근으로부터 연속 시간 전달 함수를 유도한다.

### 3.1 선형 속도 필터

$$H_u(s) = \sigma_u \sqrt{\frac{2L_u}{\pi V}} \cdot \frac{1}{1 + \frac{L_u}{V}s}$$

$$H_v(s) = \sigma_v \sqrt{\frac{L_v}{\pi V}} \cdot \frac{1 + \frac{\sqrt{3}L_v}{V}s}{\left(1 + \frac{L_v}{V}s\right)^2}$$

$$H_w(s) = \sigma_w \sqrt{\frac{L_w}{\pi V}} \cdot \frac{1 + \frac{\sqrt{3}L_w}{V}s}{\left(1 + \frac{L_w}{V}s\right)^2}$$

이 필터에 대역 제한 가우시안 백색 잡음을 입력하면 Dryden PSD와 일치하는 난류 신호를 생성할 수 있다.

---

## 4. 이산화 방법 비교

### 4.1 영차 홀드 (ZOH)

ZOH는 샘플 간격 동안 입력이 일정하게 유지된다고 가정하는 가장 단순한 이산화 방법이다. 고주파 성분에서 ZOH 이산화는 연속 시간 PSD와 차이를 보이는 경향이 있다.

### 4.2 일차 홀드 (FOH)

FOH는 샘플 간격 동안 입력이 선형 보간된다고 가정하며, 일반적으로 ZOH보다 더 정확한 이산화를 제공한다. 그러나 FOH는 실제 물리적 구현이 더 복잡하다.

### 4.3 검증 방법론

구현된 이산 필터의 PSD를 이론 Dryden PSD와 비교하여 검증:

$$\hat{\Phi}(\omega) = \frac{1}{N \cdot f_s} \left| \sum_{k=0}^{N-1} x_k e^{-j2\pi k/N} \right|^2$$

- $N$: 샘플 수
- $f_s$: 샘플링 주파수
- $x_k$: 이산 필터 출력

---

## 5. 고도별 파라미터

### 5.1 저고도 (1,000 ft 이하)

난류 스케일 길이는 고도 $h$의 함수:

$$2L_w = h$$
$$L_u = 2L_v = \frac{h}{(0.177 + 0.000823h)^{1.2}}$$

난류 강도:
$$\sigma_w = 0.1 W_{20}$$
$$\sigma_u = \sigma_v = \frac{\sigma_w}{(0.177 + 0.000823h)^{0.4}}$$

여기서 $W_{20}$은 20 ft 고도에서의 풍속:
- 경도 난류: $W_{20} = 15$ 노트
- 중등도: $W_{20} = 30$ 노트
- 심한 난류: $W_{20} = 45$ 노트

### 5.2 중·고고도 (2,000 ft 이상)

난류가 등방성(isotropic)이라 가정:
- $L_u = L_v = L_w = 1,750$ ft (상수)
- $\sigma_u = \sigma_v = \sigma_w$ (일정)

---

## 6. MIL-F-8785C 돌풍 구배

이산 돌풍(discrete gust)은 "1-cosine" 형상으로 정의된다:

$$w_g(x) = \frac{W_m}{2}\left(1 - \cos\frac{\pi x}{d_m}\right), \quad 0 \leq x \leq 2d_m$$

여기서:
- $W_m$: 최대 돌풍 속도
- $d_m$: 돌풍 길이 (항공기 날개 폭의 1.5 ~ 25배)

---

## 7. 주요 결론

1. **ZOH 이산화의 한계**: 낮은 샘플링 주파수에서 ZOH는 Dryden PSD를 부정확하게 재현하며, 특히 고주파 영역에서 에너지 손실이 발생한다.
2. **FOH의 우수성**: FOH 이산화는 ZOH보다 더 정확한 PSD 일치를 보이나, 추가적인 구현 복잡도를 수반한다.
3. **검증 방법론**: 파워 스펙트럼 비교 방법이 Dryden 모델 구현의 정확도를 검증하는 효과적인 도구임을 확인하였다.
4. **일반적인 오류**: 스케일 길이와 강도 파라미터의 단위 혼동(ft vs. m), 샘플링 주파수 설정 오류가 주요 구현 오류 원인이다.

---

**Step 2 활용 방향**: 본 논문의 Dryden PSD 수식과 성형 필터 전달 함수는 Step 2 `wind_model.m` 구현의 이론적 근거이다. ZOH vs FOH 비교는 MATLAB/Simulink 이산 시뮬레이션에서 적절한 샘플링 주파수 선택의 지침을 제공한다.

---

*번역일: 2026-10-03*
