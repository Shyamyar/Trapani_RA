%% Equations of motion
g = knots2nms(convvel(9.81,'m/s','kts')); % acceleration due to gravity, nm/s^2
shi_dot = @(V,phi) g*tan(phi)/V;  
V_x = @(V,shi) V * sin(shi);
V_y = @(V,shi) V * cos(shi);
R = @(UAV) UAV.V^2 / (g * tan(UAV.phi));
time = @(UAV) (abs(UAV.del_shi) * UAV.V)/ (g * tan(UAV.phi)); % time elasped during turn
del_shi = @(t,UAV) t*g*tan(UAV.phi)/UAV.V; % del_shi as a function of time, phi, and V

%% Maneuvering positions
man.shi = @(UAV) UAV.shi0 + UAV.del_shi; % heading range post turn for A
% man.p_A = @(del_shi) [1-cos(del_shi);sin(del_shi)] * A.R .* sign(del_shi);
man.p = @(UAV) UAV.p0 + [(cos(UAV.shi0) - cos(UAV.shi0+UAV.del_shi));(-sin(UAV.shi0) + sin(UAV.shi0+UAV.del_shi))] * UAV.R .* sign(UAV.del_shi);

% Straight Line position
str.shi = @(t,UAV) ones(1,length(t)) * UAV.shi0; % heading post turn
% str.p_A = @(t) [zeros(1,length(turn)); t * A.V]; % B maneuvering
str.p = @(t,UAV) UAV.p0 + [sin(UAV.shi0);cos(UAV.shi0)] * t .* UAV.V; % A maneuvering