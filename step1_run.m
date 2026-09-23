% STEP1_RUN  드론 궤적 생성 검증 스크립트
%
%   시나리오 3종 비교:
%     S1 — 직선 비행 / 무풍
%     S2 — 직선 비행 / 약풍 (5 m/s)
%     S3 — 회피 기동 / 강풍 (15 m/s)
%
%   결과: 3D 궤적, 고도 시계열, 속도 시계열

clear; clc; close all;

%% ── 공통 파라미터 ──────────────────────────────────────────────────────
p.m      = 1.5;      % kg  드론 질량
p.rho    = 1.225;    % kg/m³  해면 공기밀도
p.Cd     = 0.8;      % 항력계수
p.A      = 0.10;     % m²  전면적
p.g      = 9.81;     % m/s²
p.thrust = 15;       % N   수평 접근 추력

p.target_dir = [1; 0; 0];   % X축 방향으로 접근

% 회피 기동 파라미터
p.A_evade = 3.0;   % m/s²
p.f_evade = 0.3;   % Hz

% 시뮬레이션 설정
tspan = [0, 30];                   % 시뮬레이션 시간 (s)
x0    = [0; 0; 30; 8; 0; 0];      % 초기 상태: (0,0,30)m, 속도 8 m/s in X

%% ── 시나리오 실행 ──────────────────────────────────────────────────────
p.flight_mode = 'straight';
[t1, x1] = ode45(@(t,x) drone_dynamics(t, x, 'calm',   p), tspan, x0);

p.flight_mode = 'straight';
[t2, x2] = ode45(@(t,x) drone_dynamics(t, x, 'light',  p), tspan, x0);

p.flight_mode = 'evasive';
[t3, x3] = ode45(@(t,x) drone_dynamics(t, x, 'strong', p), tspan, x0);

%% ── 3D 궤적 ────────────────────────────────────────────────────────────
figure('Name','Step1: 3D Trajectory','Position',[100 100 800 600]);
plot3(x1(:,1), x1(:,2), x1(:,3), 'b-',  'LineWidth', 2); hold on;
plot3(x2(:,1), x2(:,2), x2(:,3), 'g--', 'LineWidth', 2);
plot3(x3(:,1), x3(:,2), x3(:,3), 'r-.', 'LineWidth', 2);
plot3(x0(1), x0(2), x0(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('직선 / 무풍', '직선 / 약풍', '회피 / 강풍', '출발점', ...
       'Location', 'best');
title('침입 드론 3D 궤적 — 조건별 비교');

%% ── 고도 시계열 ─────────────────────────────────────────────────────────
figure('Name','Step1: Altitude','Position',[100 750 800 300]);
plot(t1, x1(:,3), 'b-', t2, x2(:,3), 'g--', t3, x3(:,3), 'r-.', ...
     'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Altitude Z (m)');
legend('직선/무풍', '직선/약풍', '회피/강풍');
title('고도 시계열');

%% ── 수평 속도 크기 시계열 ────────────────────────────────────────────────
spd1 = vecnorm(x1(:,4:5), 2, 2);
spd2 = vecnorm(x2(:,4:5), 2, 2);
spd3 = vecnorm(x3(:,4:5), 2, 2);

figure('Name','Step1: Speed','Position',[950 100 800 300]);
plot(t1, spd1, 'b-', t2, spd2, 'g--', t3, spd3, 'r-.', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Horizontal Speed (m/s)');
legend('직선/무풍', '직선/약풍', '회피/강풍');
title('수평 속도 시계열');

%% ── 계획 vs 실제 궤적 이탈량 (약풍 기준) ────────────────────────────────
deviation = vecnorm(x2(:,1:3) - x1(1:length(t2), 1:3), 2, 2);

figure('Name','Step1: Wind Deviation','Position',[950 450 800 300]);
plot(t2, deviation, 'm-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Position Deviation (m)');
title('풍력에 의한 궤적 이탈량 (무풍 기준)');

%% ── 주요 수치 출력 ──────────────────────────────────────────────────────
fprintf('\n=== Step 1 결과 요약 ===\n');
fprintf('시뮬레이션 종료 위치:\n');
fprintf('  직선/무풍  : (%.1f, %.1f, %.1f) m\n', x1(end,1), x1(end,2), x1(end,3));
fprintf('  직선/약풍  : (%.1f, %.1f, %.1f) m\n', x2(end,1), x2(end,2), x2(end,3));
fprintf('  회피/강풍  : (%.1f, %.1f, %.1f) m\n', x3(end,1), x3(end,2), x3(end,3));
fprintf('최대 풍력 이탈량 : %.2f m\n', max(deviation));
fprintf('========================\n\n');
