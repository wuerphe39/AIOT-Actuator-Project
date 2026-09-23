function dxdt = drone_dynamics(t, x, wind_cond, params)
% DRONE_DYNAMICS  ODE function for intruder drone 3D motion.
%
%   State  x = [px; py; pz; vx; vy; vz]  (6x1, m / m/s)
%   Output dxdt = [vx; vy; vz; ax; ay; az]
%
%   운동방정식:
%       m * dv/dt = F_thrust - F_drag - m*g*ẑ
%       F_drag = 0.5 * rho * Cd * A * |v_rel| * v_rel
%       v_rel  = v_drone - v_wind

pos = x(1:3);  %#ok<NASGU>
vel = x(4:6);

% 풍속 벡터 (결정론적)
v_wind = wind_model(t, wind_cond);

% 상대 속도 및 항력
v_rel     = vel - v_wind;
v_rel_mag = norm(v_rel);

if v_rel_mag > 1e-6
    F_drag = -0.5 * params.rho * params.Cd * params.A * v_rel_mag * v_rel;
else
    F_drag = zeros(3,1);
end

% 유도 추력
F_thrust = guidance_law(t, x, params);

% 가속도  (중력은 guidance_law 내 보상항으로 상쇄)
a = (F_thrust + F_drag) / params.m - [0; 0; params.g];

dxdt = [vel; a];
end
