% STEP1_RUN  드론 궤적 생성 검증 스크립트 (논문 파라미터 적용판)
%
%   시나리오 3종:
%     S1 — 직선 비행 / 무풍
%     S2 — 직선 비행 / 약풍 (5 m/s)
%     S3 — 3D 회피 기동 / 강풍 (15 m/s)
%
%   참고 논문:
%     [1] Hattenberger et al. (2023), Sage/Int. J. Micro Air Vehicles (SCIE)
%         "Evaluation of drag coefficient for a quadrotor model"
%         → 방향별 항력 분리 구조 근거 (Cd_h=0.47, Cd_v=0.77, A=0.073 m²는 문헌 일반값 기반 가정)
%     [2] Siqueira, WVU (2017)
%         "Modeling of Wind Phenomena and Analysis of Their Effects on UAV"
%         → 고도 멱함수 프로파일 구조 근거 (alpha_w=0.14는 개방 지형 문헌 일반값 가정)
%     [3] Springer Neural Comp. & App. (2026), SCIE
%         "Improving visual differentiation of drones and birds using trajectory features"
%         → 드론 고도 유지 범위 ±2 m, 속도 5~20 m/s

clear; clc; close all;

%% ── 물리 파라미터 (가정값 — 문헌 일반값 기반, 원문 직접 제시 아님) ──────
p.m     = 1.35;   % kg  — 250mm 급 쿼드로터 + 페이로드 (실측 범위 1.2~1.5 kg)
p.rho   = 1.225;  % kg/m³ — 해면 공기밀도
p.Cd_h  = 0.47;   % 수평 항력계수 (가정값)
p.Cd_v  = 0.77;   % 수직 항력계수 (가정값)
p.A     = 0.073;  % m²  — 유효 전면적 (가정값)
p.g     = 9.81;   % m/s²

% ── 비행 파라미터 ────────────────────────────────────────────────────
p.thrust     = 5;     % N   — 수평 접근 추력 (13 N → 터미널 속도 ~25 m/s로 논문 유효범위 초과; 5 N → ~15 m/s)
p.target_dir = [1; 0; 0];

% ── 고도유지 PD 게인 (Springer 2026: ±2 m 이내 유지) ─────────────────
% F_z = m*(g + Kp_z*e - Kd_z*vz)  →  z̈ + Kd_z*ż + Kp_z*z = 0  (m 약분)
% ωn = sqrt(Kp_z) ≈ 4.47 rad/s,  ζ = Kd_z/(2*ωn) ≈ 0.89 (준임계 감쇠)
p.cruise_alt = 30;    % m
p.Kp_z       = 20;    % 1/s²  (F_z 내에서 m이 곱해져 단위가 N/m이 아님)
p.Kd_z       = 8;     % 1/s

% ── 회피 기동 파라미터 ───────────────────────────────────────────────
p.A_evade   = 3.0;   % m/s²  수평 회피 진폭
p.A_evade_z = 1.5;   % m/s²  수직 회피 진폭
p.f_evade   = 0.3;   % Hz

% ── 시뮬레이션 설정 ──────────────────────────────────────────────────
tspan = [0, 30];
x0    = [0; 0; 30; 8; 0; 0];   % 초기: (0,0,30) m, 속도 8 m/s in X

%% ── 시나리오 실행 ──────────────────────────────────────────────────────
p.flight_mode = 'straight';
[t1, x1] = ode45(@(t,x) drone_dynamics(t, x, 'calm',   p), tspan, x0);

p.flight_mode = 'straight';
[t2, x2] = ode45(@(t,x) drone_dynamics(t, x, 'light',  p), tspan, x0);

p.flight_mode = 'evasive';
[t3, x3] = ode45(@(t,x) drone_dynamics(t, x, 'strong', p), tspan, x0);

%% ── 그림 1: 3D 궤적 ─────────────────────────────────────────────────
figure('Name','Step1: 3D Trajectory','Position',[50 50 860 620]);
plot3(x1(:,1), x1(:,2), x1(:,3), 'b-',  'LineWidth', 2); hold on;
plot3(x2(:,1), x2(:,2), x2(:,3), 'g--', 'LineWidth', 2);
plot3(x3(:,1), x3(:,2), x3(:,3), 'r-.', 'LineWidth', 2);
plot3(x0(1), x0(2), x0(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('직선/무풍','직선/약풍','3D회피/강풍','출발점','Location','best');
title('[Step 1] 침입 드론 3D 궤적 비교');

%% ── 그림 2: 고도 시계열 — ±2 m 유지 여부 확인 (Springer 2026) ──────
figure('Name','Step1: Altitude','Position',[50 700 860 280]);
plot(t1, x1(:,3), 'b-', t2, x2(:,3), 'g--', t3, x3(:,3), 'r-.', 'LineWidth', 2);
hold on;
yline(p.cruise_alt + 2, 'k:', 'LineWidth', 1.5, 'Label', '+2 m');
yline(p.cruise_alt - 2, 'k:', 'LineWidth', 1.5, 'Label', '-2 m');
ylim([p.cruise_alt - 3, p.cruise_alt + 3]);  % ±2m 선이 축 안에 보이도록
grid on;
xlabel('Time (s)'); ylabel('Altitude Z (m)');
legend('직선/무풍','직선/약풍','3D회피/강풍','±2 m 허용범위','Location','best');
title('[Step 1] 고도 시계열 — 드론 고도유지 특성 (Springer 2026)');

%% ── 그림 3: 수평 속도 시계열 ────────────────────────────────────────
spd1 = vecnorm(x1(:,4:5), 2, 2);
spd2 = vecnorm(x2(:,4:5), 2, 2);
spd3 = vecnorm(x3(:,4:5), 2, 2);

figure('Name','Step1: Speed','Position',[930 50 860 280]);
plot(t1, spd1, 'b-', t2, spd2, 'g--', t3, spd3, 'r-.', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Horizontal Speed (m/s)');
legend('직선/무풍','직선/약풍','3D회피/강풍');
title('[Step 1] 수평 속도 시계열');

%% ── 그림 4: 궤적 각도 변화량 α (Step 2 분류 특징벡터 미리보기) ────────
% 1 Hz 고정 샘플링으로 α 계산 — ode45 비균일 시간점 사용 시 Δt가 너무 작아
% 이웃 벡터 사이 각도가 수치적으로 미세해지는 문제를 방지
t_samp  = (0:1:29)';                          % 1 Hz 샘플 시각 (0~29 s)
x1_s    = interp1(t1, x1, t_samp);
x3_s    = interp1(t3, x3, t_samp);

calc_alpha = @(traj) arrayfun(@(i) ...
    acosd(max(-1, min(1, dot(traj(i,4:6), traj(i+1,4:6)) / ...
    (norm(traj(i,4:6))*norm(traj(i+1,4:6)) + 1e-9)))), ...
    1:size(traj,1)-1);

a1 = calc_alpha(x1_s);
a3 = calc_alpha(x3_s);

figure('Name','Step1: TrajAngle','Position',[930 380 860 280]);
plot(t_samp(1:end-1), a1, 'b-o', t_samp(1:end-1), a3, 'r-.^', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('\alpha (deg)');
legend('직선/무풍','3D회피/강풍');
title('[Step 1→2] 궤적 각도 변화량 \alpha — 1 Hz 샘플링 (Springer 2026)');

%% ── 그림 5: 풍력 궤적 이탈량 ───────────────────────────────────────
% ode45는 각 시나리오마다 서로 다른 시간 지점을 출력하므로,
% 인덱스로 직접 비교하면 다른 시각의 위치를 빼게 됨 → interp1 보정 필요
t_common = linspace(0, min(t1(end), t2(end)), 500);
x1_i = interp1(t1, x1(:,1:3), t_common);
x2_i = interp1(t2, x2(:,1:3), t_common);
deviation = vecnorm(x2_i - x1_i, 2, 2);

figure('Name','Step1: Deviation','Position',[930 700 860 280]);
plot(t_common, deviation, 'm-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Position Deviation (m)');
title('[Step 1] 풍력에 의한 궤적 이탈량 (무풍 대비, 약풍 조건)');

%% ── 수치 요약 출력 ──────────────────────────────────────────────────
fprintf('\n=== Step 1 결과 요약 ===\n');
fprintf('가정 파라미터 (문헌 일반값): Cd_h=%.2f, Cd_v=%.2f, A=%.3f m²\n', ...
        p.Cd_h, p.Cd_v, p.A);
fprintf('종료 위치:\n');
fprintf('  직선/무풍    : (%.1f, %.1f, %.1f) m\n', x1(end,1), x1(end,2), x1(end,3));
fprintf('  직선/약풍    : (%.1f, %.1f, %.1f) m\n', x2(end,1), x2(end,2), x2(end,3));
fprintf('  3D회피/강풍  : (%.1f, %.1f, %.1f) m\n', x3(end,1), x3(end,2), x3(end,3));
fprintf('최대 풍력 이탈량 : %.2f m\n', max(deviation));
fprintf('고도 유지 (직선/무풍): 최대이탈 %.2f m (Springer 2026 기준 ±2 m)\n', ...
        max(abs(x1(:,3) - p.cruise_alt)));
fprintf('평균 궤적 각도 α (직선/무풍): %.2f °\n', mean(a1));
fprintf('평균 궤적 각도 α (회피/강풍): %.2f °\n', mean(a3));
fprintf('========================\n\n');
