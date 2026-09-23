function F_thrust = guidance_law(t, x, params)
% GUIDANCE_LAW  Thrust force vector [Fx; Fy; Fz] (N) for the intruder drone.
%
%   params.flight_mode : 'straight' | 'evasive'
%   params.target_dir  : 3x1 approach direction (unit vector, horizontal)
%   params.thrust      : horizontal thrust magnitude (N)
%   params.cruise_alt  : target cruise altitude (m)
%   params.Kp_z        : altitude-hold proportional gain (N/m)
%   params.Kd_z        : altitude-hold derivative gain   (N·s/m)
%   params.A_evade     : lateral evasion accel amplitude (m/s²)  [evasive]
%   params.A_evade_z   : vertical evasion accel amplitude (m/s²) [evasive]
%   params.f_evade     : evasion frequency (Hz)                   [evasive]
%
%   [논문 근거 — Springer Neural Computing & Applications, 2026, SCIE]
%   드론은 고도를 ±2 m 이내로 유지하는 경향이 있음 (새·민간기와 구별되는 특성).
%   단순 중력보상(F=mg) 대신 PD 고도유지 제어기를 사용하여 이 특성을 재현:
%       F_z = m*(g + Kp_z*(z_target - z) - Kd_z*vz)
%   → 고도오차 발생 시 능동 복원, 수직 항력 변화에도 안정적 고도 유지.

z_pos = x(3);
vz    = x(6);

% ── 수평 접근 추력 ────────────────────────────────────────────────────
hdir = params.target_dir;
hdir(3) = 0;
if norm(hdir) > 1e-6
    hdir = hdir / norm(hdir);
end
F_approach = params.thrust * hdir;

% ── 고도유지 PD 제어기 (Springer 2026) ───────────────────────────────
e_z = params.cruise_alt - z_pos;
F_z = params.m * (params.g + params.Kp_z * e_z - params.Kd_z * vz);
F_alt = [0; 0; F_z];

% ── 비행 모드별 추력 합산 ─────────────────────────────────────────────
switch params.flight_mode

    case 'straight'
        F_thrust = F_approach + F_alt;

    case 'evasive'
        % 수평 측방향 회피 (접근 방향에 수직)
        lat = [-hdir(2); hdir(1); 0];
        a_lat = params.A_evade * sin(2*pi * params.f_evade * t);
        F_lat = params.m * a_lat * lat;

        % 수직 방향 회피 (위상 오프셋으로 수평과 분리)
        a_vert = params.A_evade_z * sin(2*pi * params.f_evade * 1.5*t + pi/4);
        F_vert = [0; 0; params.m * a_vert];

        F_thrust = F_approach + F_alt + F_lat + F_vert;

    otherwise
        error('guidance_law: unknown flight_mode "%s".', params.flight_mode);
end
end
