clc;
clear all;
close all;

% Robot parameters
robotPose.x = 2.0;
robotPose.y = 3.0;
robotPose.theta = deg2rad(45);

% Populate wheel velocities vector
vl_mps = 0.7;
vr_radps = deg2rad(20);
% Linear and Angular Velocity Measurements
u = [vl_mps, vr_radps; vl_mps, vr_radps; vl_mps, vr_radps; vl_mps, vr_radps];
% time between iterations in seconds
dt_s = 0.5;

hold on;
drawRobot(robotPose.x, robotPose.y, rad2deg(robotPose.theta), 0.1);
xpath = [robotPose.x]; ypath =[robotPose.y];

for ii = 1:length(u(:,1))
    robotPose.x = robotPose.x + u(ii,1)*cos(robotPose.theta)*dt_s;
    robotPose.y = robotPose.y + u(ii,1)*sin(robotPose.theta)*dt_s;
    robotPose.theta = robotPose.theta + vr_radps*dt_s;
    
    drawRobot(robotPose.x, robotPose.y, rad2deg(robotPose.theta), 0.1);
    xpath = [xpath; robotPose.x]; ypath =[ypath; robotPose.y];
end

plot(xpath, ypath);
xlim([1.75 3.0]); ylim([2.5 4.5]);
xlabel('meters'); ylabel('meters');