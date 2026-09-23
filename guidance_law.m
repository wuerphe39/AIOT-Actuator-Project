function F_thrust = guidance_law(t, x, params)
% GUIDANCE_LAW  Thrust force vector [Fx; Fy; Fz] (N) for the intruder drone.
%
%   params.flight_mode : 'straight' | 'evasive'
%   params.target_dir  : 3x1 unit vector of horizontal approach direction
%   params.thrust      : horizontal thrust magnitude (N)
%   params.A_evade     : evasion lateral accel amplitude (m/s²)  [evasive only]
%   params.f_evade     : evasion frequency (Hz)                  [evasive only]

% ── 중력 보상 (고도 유지) ──────────────────────────────────────────────
F_grav = [0; 0; params.m * params.g];

% ── 수평 접근 추력 ────────────────────────────────────────────────────
hdir = params.target_dir;
hdir(3) = 0;
if norm(hdir) > 1e-6
    hdir = hdir / norm(hdir);
end
F_approach = params.thrust * hdir;

% ── 비행 모드별 추력 합산 ─────────────────────────────────────────────
switch params.flight_mode

    case 'straight'
        F_thrust = F_approach + F_grav;

    case 'evasive'
        % 접근 방향에 수직인 측방향 단위벡터
        lat = [-hdir(2); hdir(1); 0];
        a_lat = params.A_evade * sin(2*pi * params.f_evade * t);
        F_evade = params.m * a_lat * lat;
        F_thrust = F_approach + F_grav + F_evade;

    otherwise
        error('guidance_law: unknown flight_mode "%s".', params.flight_mode);
end
end
