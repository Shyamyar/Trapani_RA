% Designations
% shi is heading angle w.r.t vertical axis
% phi is bank angle
% g is acceleration due to gravity
% V is airspeed
% x and y are position coordinates in rectangular coordinates
% R is turn radius
% Right turn is +ve (del_shi>0) & Left turn is -ve (del_shi<0)
% A and B are structures for two UAVs in cosideration
% t is elasped time during turn; t_1 is at the end of turn
% t_s is elasped time after t_1 (beginning straight-line segment after turn

clc; clear;
%% Initialization Position, Heading and Velocity
eom; % run equations of motion; maneuver, and straight motion equations
sep_thres = 5; % 5nmi
A.V = knots2nms(400); % knots, ground speed of aircraft A
B.V = knots2nms(480); % knots, ground speed of aircraft B
A.shi0i = deg2rad(0); % heading of A w.r.t inertial frame
B.shi0i = deg2rad(-90); % heading of B w.r.t inertial frame
A.p0i = [0;0]; %[x_A0i; y_A0i]; % position of A w.r.t inertial frame
B.p0i = [12;12.5]; %[x_B0i; y_B0i]; % position of B w.r.t inertial frame
% bank = [15,20,25,30]; % bank angles range
bank = 20;% bank angle
turn_lim = 135; % turn limit for turn range
% con_all = ["A","B","AB+","AB-"];
con_all = "A";

% For animation
play = 0;
myVideo = VideoWriter('Paired Aircraft CA');

%% Initial Position after coordinate transformation (to A's initial position)
state_A0 = i2A(A.p0i,A.shi0i,A);
A.p0 = state_A0.P; A.shi0 = state_A0.shi; % initial position of B relative to A (i.e. coordinate transformation)
state_B0 = i2A(B.p0i,B.shi0i,A);
B.p0 = state_B0.P; B.shi0 = state_B0.shi; % initial position of B relative to A (i.e. coordinate transformation)

%% Right left Indices
for k = 1:length(con_all)
con = con_all(k);
if strcmp(con,'AB-')
    L_R = -1;
elseif strcmp(con,'AB+')
    L_R = 1; 
else
    L_R = 0;
end % Any one straight=0, B turns left=-1, B turns right=1

for j=1:length(bank)
%% Change of heading and corresponding parameters
turn = deg2rad(-turn_lim:turn_lim);
no_turn = zeros(1,length(turn));
if strcmp(con,'AB+') || strcmp(con,'AB-')
    A.del_shi = turn; % turn angles range for A
%     B.del_shi = L_R*turn; % turn angles range for B
    B.del_shi = L_R*(A.V/B.V)*abs(A.del_shi); % when both maneuvering for equal time, +ve right turn, -ve left turn
    A.phi = deg2rad(bank(j)); % bank angle, 20deg
    B.phi = deg2rad(bank(j)); % bank angle, 20deg
    A.R = R(A); % Turn radius for A
    B.R = R(B); % Turn radius for B
elseif strcmp(con,'B')
    A.del_shi = no_turn; % turn angles range for A
    B.del_shi = turn; % turn angles range for B
    A.phi = deg2rad(0); % bank angle, 20deg
    B.phi = deg2rad(bank(j)); % bank angle, 20deg
    B.R = R(B); % Turn radius for B    
else
    A.del_shi = turn; % turn angles range for A
    B.del_shi = no_turn; % turn angles range for B
    A.phi = deg2rad(bank(j)); % bank angle, 20deg
    B.phi = deg2rad(0); % bank angle, 20deg
    A.R = R(A); % Turn radius for A
end

%% Position of UAV over turn phase

switch con
    case {'AB+','AB-'}
        t = time(A); % time elasped at the end of turn considering A
        A.shi = man.shi(A);
        B.shi = man.shi(B);
        A.p = man.p(A);
        B.p = man.p(B);
        A.p_orig = str.p(t,A); B.p_orig = str.p(t,B);

    case 'B'
        t = time(B); % time elasped at the end of turn considering B
        A.shi = str.shi(t,A);
        B.shi = man.shi(B);
        A.p = str.p(t,A);
        B.p = man.p(B);
        A.p_orig = str.p(t,A); B.p_orig = str.p(t,B);        

    case 'A'
        t = time(A); % time elasped at the end of turn considering A
        A.shi = man.shi(A);
        B.shi = str.shi(t,B);
        A.p = man.p(A);
        B.p = str.p(t,B);
        A.p_orig = str.p(t,A); B.p_orig = str.p(t,B);

end

%% Separation of UAVs at turns
sep = separation(A.p,B.p);
del_xy = sep.del_xy; % separation in both x and y coordinates
D_AB = sep.D_AB; % separation magnitude

%% Separation along straight line segments/after turn
V_Rx = V_x(B.V,B.shi) - V_x(A.V,A.shi); % x coordinate of V_R
V_Ry = V_y(B.V,B.shi) - V_y(A.V,A.shi); % y coordinate of V_R
V_R = [V_Rx; V_Ry]; % relative velocity of B w.r.t A in A frame
for i = 1:length(turn)
    t_smin(i) = -(del_xy(1,i)*V_Rx(i) + del_xy(2,i)*V_Ry(i))/norm(V_R(:,i))^2;
    if t_smin(i) > 0
        d_smin(i) = sqrt((del_xy(1,i)+V_Rx(i)*t_smin(i))^2 + (del_xy(2,i)+V_Ry(i)*t_smin(i))^2);
        t_tot(i) = (t(i)+t_smin(i))/60; % in min
    else
        d_smin(i) = D_AB(i);
        t_tot(i) = t(i)/60; % in min
    end
end

%% Visualize
state_APi = A2i(A.p,A.shi,A); A.pi = state_APi.P; % position of A in inertial frame
state_BPi = A2i(B.p,B.shi,A); B.pi = state_BPi.P; % position of B in inertial frame
state_APi_orig = A2i(A.p_orig,A.shi0,A); A.pi_orig = state_APi_orig.P; % position of A in inertial frame
state_BPi_orig = A2i(B.p_orig,B.shi0,A); B.pi_orig = state_BPi_orig.P; % position of B in inertial frame

if L_R == 0
    titl1 = sprintf('Top View of Turns (%s Turn)', con);
    titl2 = sprintf('Sepration between Aircrafts (%s Turn)', con);
elseif L_R == 1
    titl1 = sprintf('Top View of Turns (%s Turn), B turns right', con);
    titl2 = sprintf('Sepration between Aircrafts (%s Turn), B turns right', con);
else
    titl1 = sprintf('Top View of Turns (%s Turn), B turns left', con);
    titl2 = sprintf('Sepration between Aircrafts (%s Turn), B turns left', con);
end

figure(1)
if length(con_all) > 1, subplot(2,2,k); end
p1 = plot(A.pi_orig(1,:),A.pi_orig(2,:),'b:');
hold on;
M1 = "Aircraft A Original";
p2 = plot(B.pi_orig(1,:),B.pi_orig(2,:),'r:');
M2 = "Aircraft B Original";
p3 = plot(A.pi(1,:),A.pi(2,:),'b');
pos_x = A.pi(1,1); pos_y = A.pi(2,1);
if con~="B" text(pos_x, pos_y, num2str(bank(j))); end
M3 = "Aircraft A Maneuver";
p4 = plot(B.pi(1,:),B.pi(2,:),'r');
pos_x = B.pi(1,1); pos_y = B.pi(2,1);
if con~="A" text(pos_x, pos_y, num2str(bank(j))); end
M4 = "Aircraft B Maneuver";
ylabel('Y (nm)');
xlabel('X (nm)');
% xlim([-30,30]); ylim([-15,40]);
title(titl1)
axis equal;
grid on;
if bank(j) == 30 || length(bank) == 1
    legend([p1,p2,p3,p4],[M1,M2,M3,M4]);
end

figure(2)
if length(con_all) > 1, subplot(2,2,k); end
p1 = plot(rad2deg(turn),D_AB,'b-');
M1 = "Separation at end of Turn";
pos_x = min(rad2deg(turn)); pos_y = D_AB(rad2deg(turn) == pos_x);
text(pos_x-10, pos_y, num2str(bank(j)));
ylabel('Separation (nm), Time (min)');
xlabel('Turning Aircraft Turn angle (deg)');
xlim([-150,150]); ylim([0,20]); 
title(titl2)
hold on;
grid on;

p2 = plot(rad2deg(turn),d_smin,'b.','MarkerSize',2);
M2 = "Minimum Separation after Turn";
pos_x = max(rad2deg(turn)); pos_y = d_smin(rad2deg(turn) == pos_x);
text(pos_x, pos_y, num2str(bank(j)));
hold on;
grid on;

if bank(j) == 30 || length(bank) == 1
    p3 = plot(rad2deg(turn),t_tot,'r--'); 
    M3 = "Time to Min Sep";
    pos_x = min(rad2deg(turn)); pos_y = t_tot(rad2deg(turn) == pos_x);
    text(pos_x-10, pos_y, num2str(bank(j)));
    p4 = plot(rad2deg(-180:180),sep_thres*ones(361),'k-'); % 5 nmi threshold line for CPA
    text(120, sep_thres+1, "5 nmi");
    legend([p1,p2,p3],[M1,M2,M3]);
end

%% Animation for (-180,180) Turn
if play == 1 && length(con_all) == 1
mov_cnt = 1;
open(myVideo)
figure(3)
axis equal;
hold on; grid on;
h1=animatedline('Color','b','Linewidth',1); % A Left
h2=animatedline('Color','b','Linewidth',1); % A Right
h3=animatedline('Color','r','Linewidth',1); % B Left
h4=animatedline('Color','r','Linewidth',1); % B Right
h5=animatedline('Color','b','LineStyle',':'); % B Right
h6=animatedline('Color','r','LineStyle',':'); % B Right

range=1:2:turn_lim+1;
for i=range
    xlim([-30 30]);
    ylim([-15 40]);
    title('Motion of Aircrafts')
    addpoints(h5,A.pi_orig(1,i+turn_lim),A.pi_orig(2,i+turn_lim));
    addpoints(h6,B.pi_orig(1,i+turn_lim),B.pi_orig(2,i+turn_lim));    
    head1 = scatter(A.pi(1,turn_lim+2-i),A.pi(2,turn_lim+2-i),'b','Filled'); % A Left
    text1 = text(A.pi(1,turn_lim+2-i),A.pi(2,turn_lim+2-i)+2,"A");
    addpoints(h1,A.pi(1,turn_lim+2-i),A.pi(2,turn_lim+2-i));
    head2 = scatter(A.pi(1,i+turn_lim),A.pi(2,i+turn_lim),'b','Filled'); % A Right
    addpoints(h2,A.pi(1,i+turn_lim),A.pi(2,i+turn_lim));
    text2 = text(A.pi(1,i+turn_lim),A.pi(2,i+turn_lim)+2,"A");
    head3 = scatter(B.pi(1,turn_lim+2-i),B.pi(2,turn_lim+2-i),'r','Filled'); % B Left
    addpoints(h3,B.pi(1,turn_lim+2-i),B.pi(2,turn_lim+2-i));
    text3 = text(B.pi(1,turn_lim+2-i),B.pi(2,turn_lim+2-i)+2,"B");
    head4 = scatter(B.pi(1,i+turn_lim),B.pi(2,i+turn_lim),'r','Filled'); % B Right
    addpoints(h4,B.pi(1,i+turn_lim),B.pi(2,i+turn_lim));
    text4 = text(B.pi(1,i+turn_lim),B.pi(2,i+turn_lim)+2,"B");
    bank_text = sprintf('Bank Angle %d', bank(j));
    text5 = text(25,35,bank_text);
    drawnow;
    pause(0.001);
    MM(mov_cnt)=getframe;
    frame=getframe(gcf);
    writeVideo(myVideo,frame);
    mov_cnt=mov_cnt+1;
    if i<range(end)
        delete(head1);delete(text1);
        delete(head2);delete(text2);
        delete(head3);delete(text3);
        delete(head4);delete(text4);
    end
    delete(text5);
end
% legend([h1,h3,h5,h6],["A\_Maneuver","B\_Maneuver","A\_Original","B\_Original"]);
end
end
end
close(myVideo);