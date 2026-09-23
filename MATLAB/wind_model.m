function v_wind = wind_model(t, condition, z)
% WIND_MODEL  Wind velocity vector [vx; vy; vz] (m/s) at time t.
%
%   condition : 'calm' | 'light' | 'strong'
%   z         : drone altitude (m), optional — default 30 m
%
%   [논문 근거]
%   Siqueira, WVU (2018): 대기경계층 멱함수 프로파일
%       v(z) = v_ref * (z / z_ref)^alpha_w
%       z_ref = 10 m, alpha_w = 0.14 (개방 지형 Hellmann 지수)
%   → 고도가 높을수록 풍속이 커지는 효과를 반영.
%
%   결정론적(deterministic) 모델 — ode45 재평가 시 일관성 보장.
%   실제 난류는 Step 3 칼만 필터의 측정 노이즈 R로 반영한다.

if nargin < 3
    z = 30;   % 기본 순항 고도 (m)
end

% ── 고도 보정 계수 (멱함수 프로파일) ──────────────────────────────────
z_ref     = 10;    % m, 기준 고도
alpha_w   = 0.14;  % Hellmann 지수 (개방 지형)
z_eff     = max(z, 1);                   % 0 이하 방지
alt_scale = (z_eff / z_ref)^alpha_w;    % 고도 보정 배율

% ── 조건별 기저 풍속 + 돌풍 ───────────────────────────────────────────
switch condition
    case 'calm'
        v_base = [0; 0; 0];

    case 'light'
        % 5 m/s 기저풍 + 0.5 m/s 사인파 돌풍
        v_base = [5 + 0.5*sin(0.5*t);
                  0;
                  0];

    case 'strong'
        % 15 m/s 기저풍 + 2 m/s 돌풍 + 측풍 성분
        v_base = [15 + 2*sin(0.3*t);
                   2*cos(0.2*t);
                   0];

    otherwise
        error('wind_model: unknown condition "%s". Use calm/light/strong.', condition);
end

% 고도 보정 적용 (수평 성분에만)
v_wind    = v_base;
v_wind(1) = v_base(1) * alt_scale;
v_wind(2) = v_base(2) * alt_scale;
end
