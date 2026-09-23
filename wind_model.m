function v_wind = wind_model(t, condition)
% WIND_MODEL  Wind velocity vector [vx; vy; vz] (m/s) at time t.
%
%   condition : 'calm' | 'light' | 'strong'
%
%   결정론적(deterministic) 모델 — ode45 재평가 시 일관성 보장.
%   실제 난류는 Step 3(칼만 필터)의 측정 노이즈 R로 반영한다.

switch condition
    case 'calm'
        v_wind = [0; 0; 0];

    case 'light'
        % 5 m/s 기저풍 + 0.5 m/s 사인파 돌풍
        v_wind = [5 + 0.5*sin(0.5*t);
                  0;
                  0];

    case 'strong'
        % 15 m/s 기저풍 + 2 m/s 돌풍 + 측풍 성분
        v_wind = [15 + 2*sin(0.3*t);
                   2*cos(0.2*t);
                   0];

    otherwise
        error('wind_model: unknown condition "%s". Use calm/light/strong.', condition);
end
end
