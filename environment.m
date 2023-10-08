%% Prerquisites
clc; clear;
eom; % run equations of motion; maneuver, and straight motion equations

%% Limits for States of UAVs 
%Collision Thresholds
Airspace = 2.5; % Total Airspace Visible
xlm = Airspace * [-1,1]; % x limits
ylm = Airspace * [-1,1]; % y limits
Dbound = Airspace*0.2; % CPA in nmi / Minimum Distance
tau = 20; % sec
Dcollision = 0.01; % Collision Distance

NumUAV = 12;
simul_time = 201; % Considering Index 1 as Time = 0 sec i.e. initial position
t = 0:simul_time-1;

red = 0.2; % reduction for operable airspace
range_pos = xlm * (1-red); % reduced region where UAVs can be
range_shi = [-pi,pi]; % rad
range_vel_kn = [45 80]; % knots
range_vel = knots2nms(range_vel_kn); % nmi/sec
pos_UAV = NaN(2,simul_time,NumUAV); % position of UAVs over time
shi_UAV = NaN(NumUAV,simul_time); % headings of UAVs over time
vel_UAV = NaN(NumUAV,simul_time); % velocities of UAVs over time
del_shi_UAV = zeros(NumUAV,simul_time); % heading changes of UAVs over time w.r.t initial time
dist_UAV = NaN(NumUAV,NumUAV,simul_time); % distance between two UAVs over time [UAV1,UAV2,t]
% dist_wall = NaN(2,simul_time,NumUAV); % x,y separation from wall for each UAV over time i.e. [x;y,UAV,t]
col_stat_UAV = zeros(NumUAV,NumUAV,simul_time); % status of collision between two UAVs over time (1 or 0)
col_stat_wall = zeros(NumUAV,simul_time); % status of collision between a UAV and walls over time (1 or 0)

%% Initialization at t=0
pos_UAV(:,1,:) = range_pos(1) + (range_pos(2)-range_pos(1)).*rand(2,NumUAV); % intial [x;y] position
shi_UAV(:,1) = range_shi(1) + (range_shi(2)-range_shi(1)).*rand(NumUAV,1); % initial heading position
% vel_UAV = repmat(range_vel(1) + (range_vel(2)-range_vel(1)).*rand(NumUAV,1),[1,simul_time]); % velocity distribution over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(NumUAV,1); % velocities for each UAV

for i = 1:NumUAV
    % UAV is a structure created for dealing with each UAV
    UAV.p0 = pos_UAV(:,1,i);
    UAV.V = vel_UAV(i,1);
    UAV.shi0 = shi_UAV(i,1);
    UAV.del_shi = shi_UAV(i,:);
    pos_UAV(:,2:end,i) = str.p(1:simul_time-1,UAV); % updating position for remaining time
    shi_UAV(i,2:end) = str.shi(1:simul_time-1,UAV); % updating heading for remaining time
end

for k = 1:simul_time
%% Collision Status Update
col_stat_wall(:,k:end) = (squeeze((Airspace-abs(pos_UAV(1,k:end,:)))<=Dbound | (Airspace-abs(pos_UAV(2,k:end,:)))<=Dbound))';
for i = 1:NumUAV
    for j = 1:NumUAV
        del_xy = pos_UAV(:,k:end,i)-pos_UAV(:,k:end,j);
        dist_UAV(i,j,k:end) = sqrt(dot(squeeze(del_xy),squeeze(del_xy)));
        if i~=j
            col_stat_UAV(i,j,k:end) = abs(dist_UAV(i,j,k:end))<=Dbound | abs(dist_UAV(i,j,k:end))<=Dbound;
        end
    end
end


%% Decision making
col_avoid = 1; % On = 1
if col_avoid == 1
for j = 1:NumUAV
    wall_ind = find(col_stat_wall(j,:) == 1);
    if wall_ind>0
        pos_UAV(1,wall_ind(1):end,j) = pos_UAV(1,wall_ind(1),j);
        pos_UAV(2,wall_ind(1):end,j) = pos_UAV(2,wall_ind(1),j);
    end
end

for i = 1:NumUAV
    for j = 1:NumUAV
        uav_ind = find(col_stat_UAV(i,j,:) == 1);
        if uav_ind>0
            pos_UAV(1,uav_ind(1):end,j) = pos_UAV(1,uav_ind(1),j);
            pos_UAV(2,uav_ind(1):end,j) = pos_UAV(2,uav_ind(1),j);
        end
    end
end
end
end
%% Plot
% myVideo = VideoWriter('RA Environment');
% mov_cnt = 1;
% open(myVideo)
figure(1)
head = scatter(squeeze(pos_UAV(1,1,:)),squeeze(pos_UAV(2,1,:)));
% paths = plot(squeeze(pos_UAV(1,:,:)),squeeze(pos_UAV(2,:,:)),'color',[0.9,0.9,0.9]);
a = [1:NumUAV]'; b = num2str(a); c = cellstr(b);
dx = 0.01; dy = 0.01; % displacement so the text does not overlay the data points
text(pos_UAV(1,1,:)+dx, pos_UAV(2,1,:)+dy, c);

tail = 20;
for i = 1:simul_time
    title('Motion of Aircrafts')
    delete(head)
    if i < tail+1
        paths = plot(squeeze(pos_UAV(1,:,:)),squeeze(pos_UAV(2,:,:)),'color',[0.9,0.9,0.9]);
        hold on
        plot(squeeze(pos_UAV(1,1:i,:)),squeeze(pos_UAV(2,1:i,:)))        
    else
        paths = plot(squeeze(pos_UAV(1,:,:)),squeeze(pos_UAV(2,:,:)),'color',[0.9,0.9,0.9]);
        hold on
        plot(squeeze(pos_UAV(1,i-tail:i,:)),squeeze(pos_UAV(2,i-tail:i,:)))
    end
    head = scatter(squeeze(pos_UAV(1,i,:)),squeeze(pos_UAV(2,i,:)));
    text(pos_UAV(1,i,:)+dx, pos_UAV(2,i,:)+dy, c);
    xlm=1.5*[min(pos_UAV(1,i,:)),max(pos_UAV(1,i,:))];
    ylm=1.5*[min(pos_UAV(2,i,:)),max(pos_UAV(2,i,:))];
    xlim(xlm); ylim(ylm);
    drawnow
    hold off
%     MM(mov_cnt)=getframe;
%     frame=getframe(gcf);
%     writeVideo(myVideo,frame);
%     mov_cnt=mov_cnt+1;
end
xlim(xlm); ylim(ylm);
% close(myVideo);