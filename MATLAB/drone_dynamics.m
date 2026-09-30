function dxdt = drone_dynamics(t, x, wind_cond, params)
% DRONE_DYNAMICS  ODE function for intruder drone 3D motion.
%
%   State  x = [px; py; pz; vx; vy; vz]  (6x1, m / m/s)
%   Output dxdt = [vx; vy; vz; ax; ay; az]
%
%   운동방정식:
%       m * dv/dt = F_thrust + F_drag - m*g*ẑ
%
%   [논문 근거 — Hattenberger et al., 2023, Sage/Int. J. Micro Air Vehicles, SCIE]
%   쿼드로터 항력은 비행 방향에 따라 다름 (방향별 항력 분리 근거):
%       수평 항력: F_drag_h = -0.5*ρ*Cd_h*A*|v_rel_h|*v_rel_h
%       수직 항력: F_drag_v = -0.5*ρ*Cd_v*A*|v_rel_v|*v_rel_v
%   Cd_h=0.47, Cd_v=0.77, A=0.073 m²는 해당 논문 직접 제시값이 아닌 문헌 일반값 기반 가정값.
%   단일 Cd를 쓰는 기존 모델 대비 수직 감속 특성을 더 정확히 재현.

vel = x(4:6);

% ── 풍속 벡터 (고도 의존, 결정론적) ─────────────────────────────────────
v_wind = wind_model(t, wind_cond, x(3));

% ── 상대 속도 분해 ────────────────────────────────────────────────────
v_rel   = vel - v_wind;
v_rel_h = [v_rel(1); v_rel(2); 0];   % 수평 성분
v_rel_v =  v_rel(3);                  % 수직 성분

% ── 방향별 항력 계산 (Hattenberger 2023) ─────────────────────────────
half_rho_A = 0.5 * params.rho * params.A;

mag_h = norm(v_rel_h);
if mag_h > 1e-6
    F_drag_h = -half_rho_A * params.Cd_h * mag_h * v_rel_h;
else
    F_drag_h = zeros(3,1);
end

if abs(v_rel_v) > 1e-6
    F_drag_v = -half_rho_A * params.Cd_v * abs(v_rel_v) * [0; 0; v_rel_v];
else
    F_drag_v = zeros(3,1);
end

F_drag = F_drag_h + F_drag_v;

% ── 유도 추력 ─────────────────────────────────────────────────────────
F_thrust = guidance_law(t, x, params);

% ── 가속도 ───────────────────────────────────────────────────────────
a = (F_thrust + F_drag) / params.m - [0; 0; params.g];

dxdt = [vel; a];
end
