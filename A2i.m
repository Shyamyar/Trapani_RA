% Converts state of an aircraft from intial coordinates of A with heading =
% 0 to inertial coordinates
function state = A2i(P,shi,A)
Rot = [cos(A.shi0i) -sin(A.shi0i);
    sin(A.shi0i) cos(A.shi0i)]; % coordinate transformation to A's frame

state.P = Rot' * P + A.p0i; % position transformation
state.shi = shi + A.shi0i; % heading transformation