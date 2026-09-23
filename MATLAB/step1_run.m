% STEP1_RUN  드론 궤적 생성 검증 스크립트 (논문 파라미터 적용판)
%
%   시나리오 3종:
%     S1 — 직선 비행 / 무풍
%     S2 — 직선 비행 / 약풍 (5 m/s)
%     S3 — 3D 회피 기동 / 강풍 (15 m/s)
%
%   참고 논문:
%     [1] Hattenberger et al. (2023), Sage/IJAV (SCIE)
%         "Evaluation of drag coefficient for a quadrotor model"
%         → Cd_h = 0.47, Cd_v = 0.77
%     [2] Siqueira, WVU (2018)
%         "Modeling of Wind Phenomena and Analysis of Their Effects on UAV"
%         → 고도 멱함수 프로파일 (alpha_w = 0.14)
%     [3] Springer Neural Comp. & App. (2026), SCIE
%         "Improving visual differentiation of drones and birds using trajectory features"
%         → 드론 고도 유지 범위 ±2 m, 속도 5~20 m/s

clear; clc; close all;

%% ── 물리 파라미터 (Hattenberger 2023 기반) ───────────────────────────
p.m     = 1.35;   % kg  — 250mm 급 쿼드로터 + 페이로드 (실측 범위 1.2~1.5 kg)
p.rho   = 1.225;  % kg/m³ — 해면 공기밀도
p.Cd_h  = 0.47;   % 수평 항력계수 [1] Table 2
p.Cd_v  = 0.77;   % 수직 항력계수 [1] Table 2
p.A     = 0.073;  % m²  — 유효 전면적 (250mm 쿼드로터, [1] 측정치 기반)
p.g     = 9.81;   % m/s²

% ── 비행 파라미터 ────────────────────────────────────────────────────
p.thrust     = 13;    % N   — 수평 접근 추력
p.target_dir = [1; 0; 0];

% ── 고도유지 PD 게인 (Springer 2026: ±2 m 이내 유지) ─────────────────
% ωn = sqrt(Kp_z/m) ≈ 3.85 rad/s,  ζ ≈ 0.77 (준임계 감쇠)
p.cruise_alt = 30;    % m
p.Kp_z       = 20;    % N/m
p.Kd_z       = 8;     % N·s/m

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
yline(p.cruise_alt + 2, 'k:', 'LineWidth', 1);
yline(p.cruise_alt - 2, 'k:', 'LineWidth', 1);
grid on;
xlabel('Time (s)'); ylabel('Altitude Z (m)');
legend('직선/무풍','직선/약풍','3D회피/강풍','±2 m 허용범위');
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
% α_i = arccos(v_i · v_{i+1} / (|v_i|*|v_{i+1}|))
% 논문: Springer 2026 — 드론은 α 변화 완만, 새/랜덤 클러터는 α 급변
calc_alpha = @(traj) arrayfun(@(i) ...
    acosd(max(-1, min(1, dot(traj(i,4:6), traj(i+1,4:6)) / ...
    (norm(traj(i,4:6))*norm(traj(i+1,4:6)) + 1e-9)))), ...
    1:size(traj,1)-1);

a1 = calc_alpha(x1);
a3 = calc_alpha(x3);

figure('Name','Step1: TrajAngle','Position',[930 380 860 280]);
plot(t1(1:end-1), a1, 'b-', t3(1:end-1), a3, 'r-.', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('\alpha (deg)');
legend('직선/무풍','3D회피/강풍');
title('[Step 1→2] 궤적 각도 변화량 \alpha — 분류 특징벡터 (Springer 2026)');

%% ── 그림 5: 풍력 궤적 이탈량 ───────────────────────────────────────
n_common = min(length(t1), length(t2));
deviation = vecnorm(x2(1:n_common,1:3) - x1(1:n_common,1:3), 2, 2);

figure('Name','Step1: Deviation','Position',[930 700 860 280]);
plot(t2(1:n_common), deviation, 'm-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Position Deviation (m)');
title('[Step 1] 풍력에 의한 궤적 이탈량 (무풍 대비, 약풍 조건)');

%% ── 수치 요약 출력 ──────────────────────────────────────────────────
fprintf('\n=== Step 1 결과 요약 ===\n');
fprintf('파라미터 (Hattenberger 2023): Cd_h=%.2f, Cd_v=%.2f, A=%.3f m²\n', ...
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
