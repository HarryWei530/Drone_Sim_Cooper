clc;clear all ;close all;

waypoints = [0,0,0; 
            0,1,0.5; 
            1,1,0.7; 
            1,0,0.9; 
            0,0,1;
            0,0,0.6;
            0,0,0]';
N = 100;
total_time = 10; % total time in seconds
% dt = time/N;
% t = (0:N-1)*dt;
t = (0:length(waypoints)-1);
dt = total_time/max(t);
t = t*dt;
finess = 0.01; % The smaller the value, the finer the trajectory;

time_traj = max(t);
x = waypoints(1,:);
y = waypoints(2,:);
z = waypoints(3,:);
xq = 0:finess:max(t);

x_traj = spline(t,x,xq);

y_traj = spline(t,y,xq);
z_traj = spline(t,z,xq);

x_fin = x_traj(end);
y_fin = y_traj(end);



%%
% u_traj = diff(x_traj);
% u_traj = [u_traj(1) u_traj] / dt;
% %
% v_traj = diff(y_traj);
% v_traj = [v_traj(1) v_traj] / dt;
% %
% w_traj = diff(z_traj);
% w_traj = [w_traj(1) w_traj] / dt;
% 
% 
% x_traj = cat(1,x_traj, u_traj)';
% y_traj = cat(1,y_traj, v_traj)';
% z_traj = cat(1,z_traj, w_traj)';

%%

figure;
scatter3(x,y,z);
hold on;
plot3(x_traj,y_traj,z_traj)
hold off;
title("Specified Trajectory of Drone")
grid on;
xlabel("X [m]");
ylabel("Y [m]");
zlabel("Z [m]");

x_traj = cat(1,xq,x_traj)';
y_traj = cat(1,xq,y_traj)';
z_traj = cat(1,xq,z_traj)';

clearvars dt finess N t total_time x y z xq

figure, plot(y_traj(:,1)/max(y_traj(:,1))), hold on, plot(y_traj(:,2)/max(y_traj(:,2)))

